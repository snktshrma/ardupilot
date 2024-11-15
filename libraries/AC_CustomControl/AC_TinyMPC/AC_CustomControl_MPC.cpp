#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_MPC_ENABLED

#include "AC_CustomControl_MPC.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
// const AP_Param::GroupInfo AC_CustomControl_MPC::var_info[] = {
//     // @Param: PARAM1
//     // @DisplayName: param1
//     // @Description: Dummy parameter for MPC custom controller backend
//     // @User: Advanced
//     AP_GROUPINFO("PARAM1", 1, AC_CustomControl_MPC, param1, 0.0f),

//     AP_GROUPEND
// };

// initialize in the constructor
AC_CustomControl_MPC::AC_CustomControl_MPC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_MPC::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // arducopter main attitude controller already ran
    // we don't need to do anything else

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MPC custom controller working");

    // return what arducopter main controller outputted
    return Vector3f(_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw());
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_MPC::reset(void)
{
}

#endif  // AP_CUSTOMCONTROL_MPC_ENABLED
