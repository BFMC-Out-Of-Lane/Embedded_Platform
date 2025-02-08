#include "drivers/lights.hpp"

namespace drivers
{
   /**
    * @brief Class constructor lights
    *
    */
    CLights::CLights(
        PinName f_pwm_pin,
        int f_inf_limit,
        int f_sup_limit
    )
    : m_pwm_pin(f_pwm_pin)
    , m_inf_limit(f_inf_limit)
    , m_sup_limit(f_sup_limit)
    {
        /* constructor behaviour */
        // Set the ms_period on the pwm_pin
        m_pwm_pin.period_ms(ms_period);
        // Set position to zero
        m_pwm_pin.pulsewidth_us(zero_default);
    }

    /** @brief  CLights class destructor
     */
    CLights::~CLights()
    {
    }

    void CLights::setBrightness(int f_brightness)
    {
        // Check if the brightness is within the limits
        if (f_brightness < m_inf_limit)
        {
            f_brightness = m_inf_limit;
        }
        else if (f_brightness > m_sup_limit)
        {
            f_brightness = m_sup_limit;
        }
        // Set the brightness
//        m_pwm_pin.pulsewidth_us(f_brightness);
        // m_pwm_pin.write((float)f_brightness/100);
        printf("brightness in setBrightness: %d\n", f_brightness);
        m_pwm_pin.pulsewidth_us(conversion(f_brightness));
    }

    int CLights::conversion(int f_brightness) {
        return ((step_value * f_brightness) / 100) + zero_default;
    }

    bool CLights::inRange(int f_brightness){
        return (m_inf_limit<=f_brightness) && (f_brightness<=m_sup_limit);
    };
}; // namespace drivers