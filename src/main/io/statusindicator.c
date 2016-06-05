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

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/failsafe.h"
#include "flight/imu.h"

#include "mw.h"
#include "scheduler.h"

#include "statusindicator.h"

static uint32_t warningLedTimer = 0;

typedef enum {
    ARM_PREV_NONE = 0,
    ARM_PREV_CLI,       // 2 flashes - CLI active in the configurator
    ARM_PREV_FAILSAFE,  // 3 flashes - Failsafe mode
    ARM_PREV_ANGLE,     // 4 flashes - Maximum arming angle exceeded
    ARM_PREV_CALIB,     // 5 flashes - Calibration active
    ARM_PREV_OVERLOAD   // 6 flashes - System overload
} armingPreventedReason_e;

static armingPreventedReason_e armingPreventionReason = ARM_PREV_NONE;
static uint8_t flashesLeft = 0;

armingPreventedReason_e getArmingPreventionReason(void)
{
    if (isCalibrating()) {
        return ARM_PREV_CALIB;
    }
    if (rcModeIsActive(BOXFAILSAFE) || failsafePhase() == FAILSAFE_LANDED) {
        return ARM_PREV_FAILSAFE;
    }
    if (!imuIsAircraftArmable(armingConfig()->max_arm_angle)) {
        return ARM_PREV_ANGLE;
    }
    if (cliMode) {
        return ARM_PREV_CLI;
    }
    if (isSystemOverloaded()) {
        return ARM_PREV_OVERLOAD;
    }
    return ARM_PREV_NONE;
}

void warningLedRefresh(void)
{
    if (flashesLeft == 0) {
        armingPreventionReason = getArmingPreventionReason();
    }

    if (armingPreventionReason > ARM_PREV_NONE) {
        if (flashesLeft == 0) {
            flashesLeft = 2 * (armingPreventionReason + 1);
            LED0_OFF;
        } else {
            flashesLeft--;
            LED0_TOGGLE;
        }
    }

    uint32_t now = micros();
    warningLedTimer = now + ((flashesLeft > 0) ? WARNING_LED_FAST_SPEED : WARNING_LED_SLOW_SPEED);
}

void warningLedBeeper(bool on)
{
    if (armingPreventionReason == ARM_PREV_NONE) {
        if (on) {
            LED0_ON;
        } else {
            LED0_OFF;
        }
    }
}

void warningLedUpdate(void)
{
    uint32_t now = micros();
    if ((int32_t)(now - warningLedTimer) > 0) {
        warningLedRefresh();
    }
}


