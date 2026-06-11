/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  base class for CAN simulated devices
*/
#include "SIM_DroneCANDevice.h"
#if AP_TEST_DRONECAN_DRIVERS

#include <canard/publisher.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <dronecan_msgs.h>
#include <SITL/SITL.h>
#include <AP_DroneCAN/AP_Canard_iface.h>


using namespace SITL;

void DroneCANDevice::_setup_eliptical_correcion(uint8_t i)
{
    Vector3f diag = AP::sitl()->mag_diag[i].get();
    if (diag.is_zero()) {
        diag = {1,1,1};
    }
    const Vector3f &diagonals = diag;
    const Vector3f &offdiagonals = AP::sitl()->mag_offdiag[i];
    
    if (diagonals == _last_dia && offdiagonals == _last_odi) {
        return;
    }
    
    _eliptical_corr = Matrix3f(diagonals.x,    offdiagonals.x, offdiagonals.y,
                               offdiagonals.x, diagonals.y,    offdiagonals.z,
                               offdiagonals.y, offdiagonals.z, diagonals.z);
    if (!_eliptical_corr.invert()) {
        _eliptical_corr.identity();
    }
    _last_dia = diag;
    _last_odi = offdiagonals;
}

void DroneCANDevice::update_compass() {

    // Sampled at 100Hz
    const uint32_t now = AP_HAL::micros64();
    if ((now - _compass_last_update_us < 10000) && (_compass_last_update_us != 0)) {
        return;
    }
    _compass_last_update_us = now;

    // calculate sensor noise and add to 'truth' field in body frame
    // units are milli-Gauss
    Vector3f noise = rand_vec3f() * AP::sitl()->mag_noise;
    Vector3f new_mag_data = AP::sitl()->state.bodyMagField + noise;

    _setup_eliptical_correcion(0);
    Vector3f f = (_eliptical_corr * new_mag_data) - AP::sitl()->mag_ofs[0].get();
    // rotate compass
    f.rotate_inverse((enum Rotation)AP::sitl()->mag_orient[0].get());
    f.rotate(AP::compass().get_board_orientation());
    // scale the compass to simulate sensor scale factor errors
    f *= AP::sitl()->mag_scaling[0];

    static Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength> mag_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_ahrs_MagneticFieldStrength mag_msg {};
    mag_msg.magnetic_field_ga[0] = f.x/1000.0f;
    mag_msg.magnetic_field_ga[1] = f.y/1000.0f;
    mag_msg.magnetic_field_ga[2] = f.z/1000.0f;
    mag_msg.magnetic_field_covariance.len = 0;
    mag_pub.broadcast(mag_msg);
    static Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength2> mag2_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_ahrs_MagneticFieldStrength2 mag2_msg;
    mag2_msg.magnetic_field_ga[0] = f.x/1000.0f;
    mag2_msg.magnetic_field_ga[1] = f.y/1000.0f;
    mag2_msg.magnetic_field_ga[2] = f.z/1000.0f;
    mag2_msg.sensor_id = 0;
    mag2_msg.magnetic_field_covariance.len = 0;
    mag2_pub.broadcast(mag2_msg);
}

void DroneCANDevice::update_rangefinder() {

    // Sampled at 100Hz
    const uint32_t now = AP_HAL::micros64();
    if ((now - _rangefinder_last_update_us < 10000) && (_rangefinder_last_update_us != 0)) {
        return;
    }
    _rangefinder_last_update_us = now;
    static Canard::Publisher<uavcan_equipment_range_sensor_Measurement> pub{CanardInterface::get_test_iface()};
    uavcan_equipment_range_sensor_Measurement msg;
    msg.timestamp.usec = AP_HAL::micros64();
    msg.sensor_id = 0;
    msg.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
    const float dist = AP::sitl()->get_rangefinder(0);
    if (!isnan(dist)) {
        msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
        msg.range = MAX(0, dist);
    } else {
        msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
        msg.range = 0;
    }
    pub.broadcast(msg);
}

void DroneCANDevice::update()
{
    update_compass();
    update_rangefinder();
}

#endif // AP_TEST_DRONECAN_DRIVERS
