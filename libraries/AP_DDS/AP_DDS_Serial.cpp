#include "AP_DDS_Client.h"

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include <errno.h>

static constexpr uint16_t SERIAL_CLOSE_DRAIN_TIMEOUT_MS = 100;
static constexpr uint16_t SERIAL_ERROR_REPORT_INTERVAL_MS = 5000;

/*
  open connection on a serial port
 */
bool AP_DDS_Client::serial_transport_open(uxrCustomTransport *t)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (serial_manager == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Serial manager unavailable", msg_prefix);
        return false;
    }

    const int8_t port_num = serial_manager->find_portnum(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (port_num < 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s No serial port with protocol 45", msg_prefix);
        return false;
    }

    auto *dds_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (dds_port == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s SERIAL%d unavailable", msg_prefix, (int)port_num);
        return false;
    }

    const uint32_t dds_baud = serial_manager->find_baudrate(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (dds_baud == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s SERIAL%d invalid baud", msg_prefix, (int)port_num);
        return false;
    }

    // AP_SerialManager normally configures the UART before the DDS thread starts.
    // Reconnecting only needs to transfer ownership to this thread; avoid
    // reconfiguring active UART/DMA hardware unless initialisation was lost.
    if (dds_port->is_initialized()) {
        dds_port->begin(0);
    } else {
        dds_port->begin(dds_baud);
    }
    if (!dds_port->is_initialized()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s SERIAL%d init failed", msg_prefix, (int)port_num);
        return false;
    }

    // Drop stale bytes from a previous session before starting framing again.
    if (!dds_port->discard_input()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s SERIAL%d ownership failed", msg_prefix, (int)port_num);
        return false;
    }

    dds->serial.port = dds_port;
    dds->serial.port_num = port_num;
    dds->serial.baud = dds_baud;
    return true;
}

/*
  close serial transport
 */
bool AP_DDS_Client::serial_transport_close(uxrCustomTransport *t)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    if (dds->serial.port != nullptr) {
        // flush() wakes the UART TX thread but does not wait for queued bytes.
        // Give a session-delete or final frame a bounded opportunity to drain.
        dds->serial.port->flush();
        const uint32_t start_ms = AP_HAL::millis();
        while (dds->serial.port->tx_pending() &&
               AP_HAL::millis() - start_ms < SERIAL_CLOSE_DRAIN_TIMEOUT_MS) {
            hal.scheduler->delay_microseconds(100);
        }
        return dds->serial.port->discard_input();
    }
    return true;
}

/*
  write on serial transport
 */
size_t AP_DDS_Client::serial_transport_write(uxrCustomTransport *t, const uint8_t* buf, size_t len, uint8_t* error)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    if (dds->serial.port == nullptr) {
        *error = EINVAL;
        return 0;
    }

    // Try hard to queue the whole framed message. Returning partial writes too
    // often can leave truncated frames on the wire and trigger agent-side
    // deserialization errors.
    size_t total_written = 0;
    const uint32_t start_ms = AP_HAL::millis();
    const uint32_t write_timeout_ms = 100;

    while (total_written < len) {
        const uint32_t txspace = dds->serial.port->txspace();
        if (txspace == 0) {
            if ((AP_HAL::millis() - start_ms) >= write_timeout_ms) {
                break;
            }
            hal.scheduler->delay_microseconds(100);
            continue;
        }

        const size_t remaining = len - total_written;
        const size_t chunk_len = ((size_t)txspace < remaining) ? (size_t)txspace : remaining;
        const ssize_t n = dds->serial.port->write(buf + total_written, chunk_len);
        if (n <= 0) {
            if ((AP_HAL::millis() - start_ms) >= write_timeout_ms) {
                break;
            }
            hal.scheduler->delay_microseconds(100);
            continue;
        }

        total_written += (size_t)n;
    }

    if (total_written != len) {
        *error = EIO;
        const uint32_t now_ms = AP_HAL::millis();
        if (dds->serial.last_write_error_report_ms == 0 ||
            now_ms - dds->serial.last_write_error_report_ms >= SERIAL_ERROR_REPORT_INTERVAL_MS) {
            dds->serial.last_write_error_report_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Serial write failed", msg_prefix);
        }
        return total_written;
    }

    *error = 0;
    return total_written;
}

/*
  read from a serial transport
 */
size_t AP_DDS_Client::serial_transport_read(uxrCustomTransport *t, uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    if (dds->serial.port == nullptr) {
        *error = EINVAL;
        return 0;
    }

    const uint8_t no_reconnect = static_cast<uint8_t>(ReconnectReason::NONE);
    if (dds->reconnect_reason.load(std::memory_order_relaxed) != no_reconnect) {
        dds->serial.read_cancelled = true;
        *error = ECANCELED;
        return 0;
    }

    // A transport read is normally bounded by the XRCE timeout.  Also observe
    // the supervisor request so a stalled owner can leave the UART wait and
    // perform session cleanup itself.  Returning no bytes preserves the XRCE
    // partial-frame state for the subsequent transport reset.
    const uint32_t bounded_timeout_ms = timeout_ms > 0 ? uint32_t(timeout_ms) : 0U;
    const uint32_t tstart = AP_HAL::millis();
    while (AP_HAL::millis() - tstart < bounded_timeout_ms &&
           dds->serial.port->available() < len &&
           dds->reconnect_reason.load(std::memory_order_relaxed) == no_reconnect) {
        hal.scheduler->delay_microseconds(100); // TODO select or poll this is limiting speed (100us)
    }
    if (dds->reconnect_reason.load(std::memory_order_relaxed) != no_reconnect) {
        dds->serial.read_cancelled = true;
        *error = ECANCELED;
        return 0;
    }

    const ssize_t bytes_read = dds->serial.port->read(buf, len);
    if (bytes_read <= 0) {
        *error = 1;
        return 0;
    }

    *error = 0;
    dds->note_transport_rx((size_t)bytes_read);
    return (size_t)bytes_read;
}

/*
  initialise serial connection
 */
bool AP_DDS_Client::ddsSerialInit()
{
    // setup a framed transport for serial
    uxr_set_custom_transport_callbacks(&serial.transport, true,
                                       serial_transport_open,
                                       serial_transport_close,
                                       serial_transport_write,
                                       serial_transport_read);

    if (!uxr_init_custom_transport(&serial.transport, (void*)this)) {
        return false;
    }
    comm = &serial.transport.comm;
    return true;
}
