#ifndef LIGHTS_HPP
#define LIGHTS_HPP
/* The mbed library */
#include <mbed.h>

namespace drivers
{
    /**
     * @brief Interface to control the lights' brightness.
     *
     */
    class ILightsCommand
    {
        public:
            virtual void setBrightness(int f_brightness) = 0 ;
            virtual bool inRange(int f_brightness) = 0 ;
    };

   /**
    * @brief Class lights
    *
    */
    class CLights: public ILightsCommand
    {
        public:
            /* Constructor */
            CLights(
                PinName     f_pwm_pin,
                int       f_inf_limit,
                int       f_sup_limit
            );
            /* Destructor */
            ~CLights();

            /* Set brightness */
            void setBrightness(int f_brightness);
            /* Check brightness is in range */
            bool inRange(int f_brightness);

        private:
            /** @brief PWM output pin */
            PwmOut m_pwm_pin;
            /** @brief 0 default */
            uint16_t zero_default = 0; //0.074568(7.4% duty cycle) * 20000µs(ms_period)
            /** @brief 0 default */
            uint16_t ms_period = 1000; // 20000µs
            /** @brief step_value */
            int16_t step_value = 102;  // 0.00051 * 20000µs(ms_period) * 10(scale factor)
            /** @brief Inferior limit */
            const int m_inf_limit;
            /** @brief Superior limit */
            const int m_sup_limit;

    }; // class CLights
}; // namespace drivers

#endif // LIGHTS_HPP
