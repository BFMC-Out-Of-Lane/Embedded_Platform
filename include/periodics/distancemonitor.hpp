#ifndef DISTANCEMONITOR_HPP
#define DISTANCEMONITOR_HPP

#include <chrono>
#include <utils/task.hpp>
#include <drivers/hcsr04.hpp>
#include <drivers/speedingmotor.hpp>

namespace periodics
{
   /**
    * @brief Class distancemonitor
    *
    */
    class CDistancemonitor: public utils::CTask
    {
        public:
            /* Constructor */
            CDistancemonitor(
                std::chrono::milliseconds f_period,
                PinName f_pinTrg1,
                PinName f_pinEcho1,
                PinName f_pinTrg2,
                PinName f_pinEcho2,
                PinName f_pinTrg3,
                PinName f_pinEcho3,
                drivers::ISpeedingCommand&    f_speedingControl,
                UnbufferedSerial& f_serial
            );
            /* Destructor */
            ~CDistancemonitor();

            void serialCallbackDISTANCEMONcommand(char const * a, char * b);

            /* Serial callback method for Speed */ 
            void serialCallbackSPEEDcommand(char const * a, char * b);
            void serialCallbackBRAKEcommand(char const * a, char * b);

        private:
            /* private variables & method member */

            /* Run method */
            virtual void        _run();

            /* @brief Pin trigger and echo */
            drivers::CHcsr04 m_ultrasonicSensor1;
            drivers::CHcsr04 m_ultrasonicSensor2;
            drivers::CHcsr04 m_ultrasonicSensor3;

            /* @brief Serial communication obj.  */
            UnbufferedSerial&          m_serial;

            drivers::ISpeedingCommand&    m_speedingControl;

            /** @brief Active flag  */
            bool m_isActive;

    }; // class CDistancemonitor
}; // namespace periodics

#endif // DISTANCEMONITOR_HPP
