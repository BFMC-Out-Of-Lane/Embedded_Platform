#ifndef HCSR04_HPP
#define HCSR04_HPP

/* The mbed library */
#include <mbed.h>

namespace drivers
{
   /**
    * @brief Class hcsr04
    *
    */
    class CHcsr04
    {
        public:
            /* Constructor */
            CHcsr04(
                PinName  f_pinTrg,
                PinName  f_pinEcho
            );
            /* Destructor */
            ~CHcsr04();

            /* @brief Measure distance */
            uint16_t measureDistance();

            uint16_t averageDistance();

        private:
            /* private variables & method member */

            /* @brief Pin trigger and echo */
            mbed::DigitalOut m_pinTrg;
            mbed::DigitalIn m_pinEcho;


    }; // class CHcsr04
}; // namespace drivers

#endif // HCSR04_HPP
