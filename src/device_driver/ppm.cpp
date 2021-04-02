/*
 * servo.cpp
 *
 *  Created on: 28 Jan 2019
 *      Author: lch19
 */

#if 0

#include "device_driver/ppm.h"
#include "driver/eflexpwm.h"
#include <assert.h>

#ifdef SERVO_USED

namespace DeviceDriver {

    namespace ServoPins {
        System::Pinout::Name servo_pins[SERVO_USED] = SERVO_PINS;
    }

    PPM::PPM(const Config &config) : min_pos((config.min_pos_width / (1.0 / config.frequency * 1000000)) * 1000), convert_ratio(
                                                                                                                      ((config.max_pos_width / (1.0 / config.frequency * 1000000)) * 1000 - min_pos) / 1800), interpolation_maximum(config.max_value) {
        assert(config.id < SERVO_USED);
        width = config.initial_value;
        System::Pinout::Name pwm_pin = ServoPins::servo_pins[config.id];
        if (Driver::eFlexPWM::CheckPin(pwm_pin)) {
            Driver::eFlexPWM::Config eflexpwm_config;
            eflexpwm_config.pwm_pin = pwm_pin;
            eflexpwm_config.duty_cycle = width * convert_ratio + min_pos;
            eflexpwm_config.frequency = config.frequency;
            eflexpwm_config.alignment = Driver::eFlexPWM::Config::Alignment::kEdgeAligned;
            eflexpwm_config.reload_logic = Driver::eFlexPWM::Config::ReloadLogic::kReloadFullCycle;
            pwm = new Driver::eFlexPWM(eflexpwm_config);
        } else {
            assert(false);
        }
    }

} // namespace DeviceDriver

#endif


#endif