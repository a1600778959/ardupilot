#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_DAL/AP_DAL.h>
#include <GCS_MAVLink/GCS.h>

bool NavEKF3_core::healthy(void) const
{
    return healthy(true);
}

bool NavEKF3_core::healthy_for_horizontal_nav(void) const
{
    return healthy(false);
}

// Check basic filter health metrics and return a consolidated health status.
// When require_height is false, vertical innovation consistency is reported
// through vertical status flags but does not invalidate horizontal navigation.
bool NavEKF3_core::healthy(bool require_height) const
{
    uint16_t faultInt;
    getFilterFaults(faultInt);
    if (faultInt > 0) {
        return false;
    }
    const bool bad_horizontal_consistency = (velTestRatio > 1) && (posTestRatio > 1);
    if (bad_horizontal_consistency && (!require_height || (hgtTestRatio > 1))) {
        // In full 3D mode all three metrics must be bad before the filter is
        // considered extremely unhealthy.  For horizontal navigation, height
        // consistency is deliberately excluded from the blocking decision.
        return false;
    }
    // Give the filter a second to settle before use
    if ((imuSampleTime_ms - ekfStartTime_ms) < 1000 ) {
        return false;
    }
    // position innovations must be within limits when on-ground and in a static mode of operation
    float horizErrSq = sq(innovVelPos[3]) + sq(innovVelPos[4]);
    if (onGround && (PV_AidingMode == AID_NONE) &&
        ((horizErrSq > 1.0f) || (require_height && (fabsF(hgtInnovFiltState) > 1.0f)))) {
        return false;
    }

    // all OK
    return true;
}

/*
  per-core pre-arm checks. returns false if we fail arming checks, in
  which case the buffer will be populated with a failure message
  requires_position should be true if horizontal position configuration should be checked
*/
bool NavEKF3_core::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (requires_position) {
        // additional checks when position is required, used by pre-arm checks
        const float max_vel_innovation = 2.0;
        const float hvel_innovation = sqrtf(sq(innovVelPos[0])+sq(innovVelPos[1]));
        if (onGround && PV_AidingMode == AID_ABSOLUTE &&
            frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS) &&
            hvel_innovation > max_vel_innovation) {
            // more than 2 m/s horizontal velocity innovation on the ground
            dal.snprintf(failure_msg, failure_msg_len,
                         "EKF3[%u] vel error %.1f", unsigned(core_index)+1, hvel_innovation);
            return false;
        }
    }

    // all OK
    return true;
}

// Return a consolidated error score where higher numbers represent larger errors
// Intended to be used by the front-end to determine which is the primary EKF
float NavEKF3_core::errorScore() const
{
    float score = 0.0f;
    if (tiltAlignComplete && yawAlignComplete) {
        // Check GPS fusion performance
        score = MAX(score, 0.5f * (velTestRatio + posTestRatio));
        // Check altimeter fusion performance
        score = MAX(score, hgtTestRatio);
        // Check magnetometer fusion performance - need this when magnetometer affinity is enabled to override the inherent compass
        // switching mechanism, and instead be able to move to a better lane
        if (frontend->_affinity & EKF_AFFINITY_MAG) {
            score = MAX(score, 0.3f * (magTestRatio.x + magTestRatio.y + magTestRatio.z));
        }
    }
    return score;
}

// provides the height limit to be observed by the control loops
// returns false if no height limiting is required
bool NavEKF3_core::getHeightControlLimit(float &) const
{
    return false;
}


// return the Euler roll, pitch and yaw angle in radians
void NavEKF3_core::getEulerAngles(Vector3f &euler) const
{
    outputDataNew.quat.to_euler(euler);
    euler = euler - dal.get_trim();
}

// return body axis gyro bias estimates in rad/sec
void NavEKF3_core::getGyroBias(Vector3f &gyroBias) const
{
    if (dtEkfAvg < 1e-6f) {
        gyroBias.zero();
        return;
    }
    gyroBias = (stateStruct.gyro_bias / dtEkfAvg).tofloat();
}

// return accelerometer bias in m/s/s
void NavEKF3_core::getAccelBias(Vector3f &accelBias) const
{
    if (!statesInitialised) {
        accelBias.zero();
        return;
    }
    accelBias = (stateStruct.accel_bias / dtEkfAvg).tofloat();
}

// return the transformation matrix from XYZ (body) to NED axes
void NavEKF3_core::getRotationBodyToNED(Matrix3f &mat) const
{
    outputDataNew.quat.rotation_matrix(mat);
    mat = mat * dal.get_rotation_vehicle_body_to_autopilot_body();
}

// return the quaternions defining the rotation from NED to XYZ (body) axes
void NavEKF3_core::getQuaternion(Quaternion& ret) const
{
    ret = outputDataNew.quat.tofloat();
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t NavEKF3_core::getLastYawResetAngle(float &yawAng) const
{
    yawAng = yawResetAngle;
    return lastYawReset_ms;
}

// return the amount of NE position change due to the last position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF3_core::getLastPosNorthEastReset(Vector2f &pos) const
{
    pos = posResetNE.tofloat();
    return lastPosReset_ms;
}

// return the amount of vertical position change due to the last vertical position reset in metres
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF3_core::getLastPosDownReset(float &posD) const
{
    posD = posResetD;
    return lastPosResetD_ms;
}

// return the amount of NE velocity change due to the last velocity reset in metres/sec
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t NavEKF3_core::getLastVelNorthEastReset(Vector2f &vel) const
{
    vel = velResetNE.tofloat();
    return lastVelReset_ms;
}

// return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
// returns true if wind state estimation is active
bool NavEKF3_core::getWind(Vector3f &wind) const
{
    wind.x = stateStruct.wind_vel.x;
    wind.y = stateStruct.wind_vel.y;
    wind.z = 0.0f; // currently don't estimate this
    return !inhibitWindStates;
}

// return the NED velocity of the body frame origin in m/s
//
void NavEKF3_core::getVelNED(Vector3f &vel) const
{
    // correct for the IMU position offset (EKF calculations are at the IMU)
    vel = (outputDataNew.velocity + velOffsetNED).tofloat();
}

// Return the rate of change of vertical position in the down direction (dPosD/dt) of the body frame origin in m/s
float NavEKF3_core::getPosDownDerivative(void) const
{
    // return the value calculated from a complementary filter applied to the EKF height and vertical acceleration
    // correct for the IMU offset (EKF calculations are at the IMU)
    return vertCompFiltState.vel + velOffsetNED.z;
}

// Write the last estimated NE position of the body frame origin relative to the reference point (m).
// Return true if the estimate is valid
bool NavEKF3_core::getPosNE(Vector2f &posNE) const
{
    // There are three modes of operation: absolute aiding, relative odometry aiding and constant position.
    if (PV_AidingMode != AID_NONE) {
        // This is the normal mode of operation where we can use the EKF position states
        // correct for the IMU offset (EKF calculations are at the IMU)
        posNE = (outputDataNew.position.xy() + posOffsetNED.xy() + public_origin.get_distance_NE_ftype(EKF_origin)).tofloat();
        return true;

    } else {
        // In constant position mode the EKF position states are at the origin, so we cannot use them as a position estimate
        if(validOrigin) {
            auto &gps = dal.gps();
            if ((gps.status(selected_gps) >= AP_DAL_GPS::GPS_OK_FIX_2D)) {
                // If the origin has been set and we have GPS, then return the GPS position relative to the origin
                const Location &gpsloc = gps.location(selected_gps);
                posNE = public_origin.get_distance_NE_ftype(gpsloc).tofloat();
                return false;
            } else {
                // If no GPS fix is available, all we can do is provide the last known position
                posNE = outputDataNew.position.xy().tofloat();
                return false;
            }
        } else {
            // If the origin has not been set, then we have no means of providing a relative position
            posNE.zero();
            return false;
        }
    }
    return false;
}

// Write the last calculated D position of the body frame origin relative to the EKF local origin
// Return true if the estimate is valid
bool NavEKF3_core::getPosD_local(float &posD) const
{
    posD = outputDataNew.position.z + posOffsetNED.z;

    // Return the current height solution status
    return filterStatus.flags.vert_pos;

}

// Write the last calculated D position of the body frame origin relative to the public origin
// Return true if the estimate is valid
bool NavEKF3_core::getPosD(float &posD) const
{
    bool ret = getPosD_local(posD);

    // adjust posD for difference between our origin and the public_origin
    Location local_origin;
    if (getOriginLLH(local_origin)) {
        posD += (public_origin.alt - local_origin.alt) * 0.01;
    }

    return ret;
}

// return the estimated height of body frame origin above ground level
bool NavEKF3_core::getHAGL(float &HAGL) const
{
    HAGL = terrainState - outputDataNew.position.z - posOffsetNED.z;
    // If we know the terrain offset and altitude, then we have a valid height above ground estimate
    return !hgtTimeout && gndOffsetValid && healthy();
}

// Return the last calculated latitude, longitude and height in WGS-84
// If a calculated location isn't available, return a raw GPS measurement
// The status will return true if a calculation or raw measurement is available
// The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
bool NavEKF3_core::getLLH(Location &loc) const
{
    Location origin;
    if (getOriginLLH(origin)) {
        float posD;
        if (getPosD_local(posD) && PV_AidingMode != AID_NONE) {
            // Altitude returned is an absolute altitude relative to the WGS-84 spherioid
            loc.set_alt_cm(origin.alt - posD*100.0, Location::AltFrame::ABSOLUTE);
            if (filterStatus.flags.horiz_pos_abs || filterStatus.flags.horiz_pos_rel) {
                // The EKF is able to provide a position estimate
                loc.lat = EKF_origin.lat;
                loc.lng = EKF_origin.lng;
                loc.offset(outputDataNew.position.x + posOffsetNED.x,
                           outputDataNew.position.y + posOffsetNED.y);
                return true;
            } else {
                // We have been be doing inertial dead reckoning for too long so use raw GPS if available
                if (getGPSLLH(loc)) {
                    return true;
                } else {
                    // Return the EKF estimate but mark it as invalid
                    loc.lat = EKF_origin.lat;
                    loc.lng = EKF_origin.lng;
                    loc.offset(outputDataNew.position.x + posOffsetNED.x,
                               outputDataNew.position.y + posOffsetNED.y);
                    return false;
                }
            }
        } else {
            // Return a raw GPS reading if available and the last recorded positon if not
            if (getGPSLLH(loc)) {
                return true;
            } else {
                loc.lat = EKF_origin.lat;
                loc.lng = EKF_origin.lng;
                loc.offset(lastKnownPositionNE.x + posOffsetNED.x,
                           lastKnownPositionNE.y + posOffsetNED.y);
                loc.alt = EKF_origin.alt - lastKnownPositionD*100.0;
                return false;
            }
        }
    } else {
        // The EKF is not navigating so use raw GPS if available
        return getGPSLLH(loc);
    }
}

bool NavEKF3_core::getGPSLLH(Location &loc) const
{
    const auto &gps = dal.gps();
    if ((gps.status(selected_gps) >= AP_DAL_GPS::GPS_OK_FIX_3D)) {
        loc = gps.location(selected_gps);
        return true;
    }
    return false;
}

// return neutral navigation limits when no sensor-specific limit is active
void NavEKF3_core::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    ekfGndSpdLimit = 400.0f; //return 80% of max filter speed
    ekfNavVelGainScaler = 1.0f;
}


// return the LLH location of the filters NED origin
bool NavEKF3_core::getOriginLLH(Location &loc) const
{
    if (validOrigin) {
        loc = public_origin;
        // report internally corrected reference height if enabled
        if ((frontend->_originHgtMode & (1<<2)) == 0) {
            loc.alt = (int32_t)(100.0f * (float)ekfGpsRefHgt);
        }
    }
    return validOrigin;
}

// return earth magnetic field estimates in measurement units / 1000
void NavEKF3_core::getMagNED(Vector3f &magNED) const
{
    magNED = (stateStruct.earth_magfield * 1000.0f).tofloat();
}

// return body magnetic field estimates in measurement units / 1000
void NavEKF3_core::getMagXYZ(Vector3f &magXYZ) const
{
    magXYZ = (stateStruct.body_magfield*1000.0f).tofloat();
}

// return magnetometer offsets
// return true if offsets are valid
bool NavEKF3_core::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    const auto &compass = dal.compass();
    if (!compass.available()) {
        return false;
    }

    // compass offsets are valid if we have finalised magnetic field initialisation, magnetic field learning is not prohibited,
    // primary compass is valid and state variances have converged
    const float maxMagVar = 5E-6f;
    bool variancesConverged = (P[19][19] < maxMagVar) && (P[20][20] < maxMagVar) && (P[21][21] < maxMagVar);
    if ((mag_idx == magSelectIndex) &&
            finalInflightMagInit &&
            !inhibitMagStates &&
            compass.healthy(magSelectIndex) &&
            variancesConverged) {
        magOffsets = compass.get_offsets(magSelectIndex) - stateStruct.body_magfield.tofloat()*1000.0;
        return true;
    } else {
        magOffsets = compass.get_offsets(magSelectIndex);
        return false;
    }
}

// return the innovations for the NED Pos, NED Vel, XYZ Mag and yaw measurements
bool NavEKF3_core::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &reservedInnov, float &yawInnov) const
{
    velInnov.x = innovVelPos[0];
    velInnov.y = innovVelPos[1];
    velInnov.z = innovVelPos[2];
    posInnov.x = innovVelPos[3];
    posInnov.y = innovVelPos[4];
    posInnov.z = innovVelPos[5];
    magInnov.x = 1e3f*innovMag[0]; // Convert back to sensor units
    magInnov.y = 1e3f*innovMag[1]; // Convert back to sensor units
    magInnov.z = 1e3f*innovMag[2]; // Convert back to sensor units
    reservedInnov = 0.0f;
    yawInnov   = innovYaw;
    return true;
}

// return the drag and sideslip innovations
void NavEKF3_core::getDragSideslipInnovations(Vector2f &dragInnov, float &betaInnov) const
{
#if EK3_FEATURE_DRAG_FUSION
    dragInnov.x = innovDrag[0];
    dragInnov.y = innovDrag[1];
    betaInnov   = innovBeta;
#endif
}

// return the innovation consistency test ratios for the velocity, position and magnetometer measurements
// this indicates the amount of margin available when tuning the various error traps
// also return the delta in position due to the last position reset
bool NavEKF3_core::getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &reservedVar, Vector2f &offset) const
{
    velVar   = sqrtF(velTestRatio);
    posVar   = sqrtF(posTestRatio);
    hgtVar   = sqrtF(hgtTestRatio);
    // If we are using simple compass yaw fusion, populate all three components with the yaw test ratio to provide an equivalent output
    magVar.x = sqrtF(MAX(magTestRatio.x,yawTestRatio));
    magVar.y = sqrtF(MAX(magTestRatio.y,yawTestRatio));
    magVar.z = sqrtF(MAX(magTestRatio.z,yawTestRatio));
    reservedVar = 0.0f;
    offset   = posResetNE.tofloat();

    return true;
}

bool NavEKF3_core::getOrientationCovariance(Matrix3f &covariance) const
{
    covariance = Matrix3f();

    if (!healthy()) {
        return false;
    }

    constexpr float quat_delta = 1.0e-4f;
    float jacobian[3][4] {};
    Vector3f euler_plus;
    Vector3f euler_minus;

    for (uint8_t index = 0; index < 4; index++) {
        QuaternionF quat_plus = stateStruct.quat;
        quat_plus[index] += quat_delta;
        quat_plus.normalize();
        quat_plus.to_euler(euler_plus);

        QuaternionF quat_minus = stateStruct.quat;
        quat_minus[index] -= quat_delta;
        quat_minus.normalize();
        quat_minus.to_euler(euler_minus);

        jacobian[0][index] = wrap_PI(euler_plus.x - euler_minus.x) / (2.0f * quat_delta);
        jacobian[1][index] = wrap_PI(euler_plus.y - euler_minus.y) / (2.0f * quat_delta);
        jacobian[2][index] = wrap_PI(euler_plus.z - euler_minus.z) / (2.0f * quat_delta);
    }

    for (uint8_t row = 0; row < 3; row++) {
        for (uint8_t col = 0; col < 3; col++) {
            float value = 0.0f;
            for (uint8_t i = 0; i < 4; i++) {
                for (uint8_t j = 0; j < 4; j++) {
                    value += jacobian[row][i] * P[i][j] * jacobian[col][j];
                }
            }
            covariance[row][col] = value;
        }
    }

    return is_positive(covariance[0][0]) && is_positive(covariance[1][1]) && is_positive(covariance[2][2]);
}

// get a particular source's velocity innovations
// returns true on success and results are placed in innovations and variances arguments
bool NavEKF3_core::getVelInnovationsAndVariancesForSource(AP_NavEKF_Source::SourceXY source, Vector3f &innovations, Vector3f &variances) const
{
    switch (source) {
    case AP_NavEKF_Source::SourceXY::GPS:
        // check for timeouts
        if (dal.millis() - gpsRetrieveTime_ms > 500) {
            return false;
        }
        innovations = gpsVelInnov.tofloat();
        variances = gpsVelVarInnov.tofloat();
        return true;
#if EK3_FEATURE_EXTERNAL_NAV
    case AP_NavEKF_Source::SourceXY::EXTNAV:
        // check for timeouts
        if (dal.millis() - extNavVelInnovTime_ms > 500) {
            return false;
        }
        innovations = extNavVelInnov.tofloat();
        variances = extNavVelVarInnov.tofloat();
        return true;
#endif // EK3_FEATURE_EXTERNAL_NAV
    default:
        // variances are not available for this source
        return false;
    }

    // should never get here but just in case
    return false;
}

/*
return the filter fault status as a bitmasked integer
 0 = quaternions are NaN
 1 = velocities are NaN
 2 = badly conditioned X magnetometer fusion
 3 = badly conditioned Y magnetometer fusion
 4 = badly conditioned Z magnetometer fusion
 5 = badly conditioned synthetic sideslip fusion
 6 = filter is not initialised
*/
void  NavEKF3_core::getFilterFaults(uint16_t &faults) const
{
    faults = (stateStruct.quat.is_nan()<<0 |
              stateStruct.velocity.is_nan()<<1 |
              faultStatus.bad_xmag<<2 |
              faultStatus.bad_ymag<<3 |
              faultStatus.bad_zmag<<4 |
              faultStatus.bad_sideslip<<5 |
              !statesInitialised<<6);
}

// Return the navigation filter status message
void  NavEKF3_core::getFilterStatus(nav_filter_status &status) const
{
    status = filterStatus;
}

#if HAL_GCS_ENABLED
// send an EKF_STATUS message to GCS
void NavEKF3_core::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }
    if (filterStatus.flags.gps_glitching) {
        flags |= (1<<15);
    }

    // get variances
    float velVar = 0, posVar = 0, hgtVar = 0, reservedVar = 0;
    Vector3f magVar;
    Vector2f offset;
    getVariances(velVar, posVar, hgtVar, magVar, reservedVar, offset);


    // Only report range finder normalised innovation levels if the EKF uses it
    // for primary height estimation. This prevents false alarms when a range
    // finder is fitted for another application.
    float temp = 0;
    if ((frontend->_useRngSwHgt > 0) && activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER) {
        temp = sqrtF(auxRngTestRatio);
    }

    const mavlink_ekf_status_report_t packet{
        velVar,
        posVar,
        hgtVar,
        fmaxf(fmaxf(magVar.x,magVar.y),magVar.z),
        temp,
        flags,
        0.0f
    };

    // send message
    mavlink_msg_ekf_status_report_send_struct(link.get_chan(), &packet);
}
#endif  // HAL_GCS_ENABLED

// report the reason for why the backend is refusing to initialise
const char *NavEKF3_core::prearm_failure_reason(void) const
{
    if (gpsGoodToAlign) {
        // we are not failing
        return nullptr;
    }
    return prearm_fail_string;
}


// report the number of frames lapsed since the last state prediction
// this is used by other instances to level load
uint8_t NavEKF3_core::getFramesSincePredict(void) const
{
    return framesSincePredict;
}
