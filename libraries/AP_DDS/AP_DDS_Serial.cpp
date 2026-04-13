#include "AP_DDS_Client.h"

#include <AP_SerialManager/AP_SerialManager.h>

#include <errno.h>

/*
  open connection on a serial port
 */
bool AP_DDS_Client::serial_transport_open(uxrCustomTransport *t)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    auto *dds_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (dds_port == nullptr) {
        return false;
    }

    // Ensure the UART is actively configured with the SERIALx_BAUD value used
    // for DDS (not only ownership transfer), so the transport matches agent baud.
    const uint32_t dds_baud = serial_manager->find_baudrate(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (dds_baud == 0) {
        return false;
    }
    dds_port->begin(dds_baud);
    // Drop stale bytes from a previous session before starting framing again.
    dds_port->discard_input();
    dds->serial.port = dds_port;
    return true;
}

/*
  close serial transport
 */
bool AP_DDS_Client::serial_transport_close(uxrCustomTransport *t)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    if (dds->serial.port != nullptr) {
        // Drain pending TX as much as possible and clear stale RX bytes to reduce
        // framing corruption after reconnect.
        dds->serial.port->flush();
        dds->serial.port->discard_input();
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

    *error = (total_written == len) ? 0 : 1;
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

    const uint32_t tstart = AP_HAL::millis();
    while (AP_HAL::millis() - tstart < uint32_t(timeout_ms) &&
           dds->serial.port->available() < len) {
        hal.scheduler->delay_microseconds(100); // TODO select or poll this is limiting speed (100us)
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
