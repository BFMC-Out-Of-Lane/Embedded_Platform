#ifndef HCSR04_HPP
#define HCSR04_HPP

/* The mbed library */
#include <mbed.h>
#include <chrono>

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

            void sendTriggerPulse();

            /* Método público para obtener la distancia */
            uint16_t getDistance();


            void setEchoRiseCallbacks(Callback<void()> riseCallback);
            void setEchoFallCallbacks(Callback<void()> fallCallback);

            /* Callbacks for echo pin */
            void onEchoRise();
            void onEchoFall();


        private:
            /* private variables & method member */

            /* @brief Pin trigger and echo */
            mbed::DigitalOut m_pinTrg;
            mbed::InterruptIn m_pinEcho;

            mbed::Timer m_timerEcho;/* Timer to measure echo pulse duration */

            volatile uint16_t m_distance;/* Last measured distance */



    }; // class CHcsr04
}; // namespace drivers

#endif // HCSR04_HPP
