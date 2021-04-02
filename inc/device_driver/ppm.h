/*
 * servo.h
 *
 *  Created on: 28 Jan 2019
 *      Author: lch19
 */

#if 0

#ifndef DEVICE_DRIVER_SERVO_H_
#define DEVICE_DRIVER_SERVO_H_

#include "../config/config.h"
#include "driver/timer.h"

#ifdef SERVO_USED

namespace DeviceDriver {

    /*!
 * @addtogroup servo Servo
 * @ingroup device_driver
 * @{
 */
    /**
 * @brief Servo driver
 * 
 * ## Example
 * ~~~~~~~~~~~~~~~{.cpp}
 * ~~~~~~~~~~~~~~~
 */
    class PPM {
    public:
        struct Config {
            uint8_t id;
            uint16_t initial_value = 0;
            uint32_t frequency;     /*!<In Hz */
            uint32_t min_pos_width; /*!<In μs */
            uint32_t max_pos_width; /*!<In μs */
            uint32_t max_value;
        };
        explicit PPM(const Config &config);

        /**
	 * @brief Set the servo degree
	 * 
	 * @param degree: degree that want the servo to turn \n
	 *                range 0 to interpolation_maximum, means servo degree 0.0° to 180.0° with resolution of interpolation_maximum
	 */
        inline void SetWidth(uint16_t width) {
            this->width = (width > interpolation_maximum ? interpolation_maximum : width);
            pwm->SetDutyCycle(this->width * convert_ratio + min_pos);
        }

        /**
	 * @brief Get the servo degree that set before
	 * 
	 * @return uint16_t degree of the servo, range 0 to interpolation_maximum, means servo degree 0.0° to 180.0° with resolution of interpolation_maximum
	 */
        inline uint16_t GetWidth() const {
            return width;
        }

    private:
        Driver::Timer *pwm;
        const float min_pos;
        const float convert_ratio;
        uint16_t width;
        const uint32_t interpolation_maximum;
    };
    /** @}*/

} // namespace DeviceDriver

#endif

#endif /* DEVICE_DRIVER_SERVO_H_ */

#endif