/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/serial.h"
#include "drivers/sensor.h"
#include "drivers/pwm_mapping.h"
#include "drivers/accgyro.h"

#include "io/serial_cli.h"
#include "io/rc_controls.h"
#include "io/beeper.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#include "mw.h"
#include "scheduler.h"

#include "statusindicator.h"

static uint32_t warningLedTimer = 0;

typedef enum {
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
    WARNING_ARM_PREV_CLI,       // 2 flashes - CLI active in the configurator
    WARNING_ARM_PREV_FAILSAFE,  // 3 flashes - Failsafe mode
    WARNING_ARM_PREV_ANGLE,     // 4 flashes - Maximum arming angle exceeded
    WARNING_ARM_PREV_CALIB,     // 5 flashes - Calibration active
    WARNING_ARM_PREV_OVERLOAD   // 6 flashes - System overload
} warningLedState_e;

static warningLedState_e warningLedState = WARNING_LED_OFF;
static uint8_t flashsLeft = 0;

warningLedState_e getWarningState(void)
{
    if (isCalibrating()) {
        return WARNING_ARM_PREV_CALIB;
    }
    if (rcModeIsActive(BOXFAILSAFE) || failsafePhase() == FAILSAFE_LANDED) {
        return WARNING_ARM_PREV_FAILSAFE;
    }
    if (!imuIsAircraftArmable(armingConfig()->max_arm_angle)) {
        return WARNING_ARM_PREV_ANGLE;
    }
    if (cliMode == 1) {
        return WARNING_ARM_PREV_CLI;
    }
    if (isSystemOverloaded()) {
        return WARNING_ARM_PREV_OVERLOAD;
    }
    if (beeperIsOn == 1) {
        return WARNING_LED_ON;
    }

    return WARNING_LED_OFF;
}

void warningLedRefresh(void)
{
    if (flashsLeft == 0) {
        warningLedState = getWarningState();
    }

    switch (warningLedState) {
        case WARNING_LED_OFF:
            LED0_OFF;
            break;
        case WARNING_LED_ON:
            LED0_ON;
            break;
        default:
            if (flashsLeft == 0) {
                flashsLeft = 2 * warningLedState;
                LED0_OFF;
            } else {
                flashsLeft--;
                LED0_TOGGLE;
            }
            break;
    }

    uint32_t now = micros();
    warningLedTimer = now + ((flashsLeft > 0) ? WARNING_LED_FAST_SPEED : WARNING_LED_SLOW_SPEED);
}

void warningLedUpdate(void)
{
    uint32_t now = micros();

    if ((int32_t)(now - warningLedTimer) > 0 || (beeperIsOn && warningLedState == WARNING_LED_OFF)
            || (!beeperIsOn && warningLedState == WARNING_LED_ON)) {
        warningLedRefresh();
    }
}


