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
                drivers::CHcsr04& f_ultrasonicSensor1,
                drivers::CHcsr04& f_ultrasonicSensor2,
                drivers::CHcsr04& f_ultrasonicSensor3,
                drivers::ISpeedingCommand&    f_speedingControl,
                UnbufferedSerial& f_serial
            );
            /* Destructor */
            ~CDistancemonitor();

            void serialCallbackDISTANCEMONcommand(char const * a, char * b);

        private:
            /* private variables & method member */

            /* Run method */
            virtual void        _run();

            /* @brief Pin trigger and echo */
            drivers::CHcsr04& m_ultrasonicSensor1;
            drivers::CHcsr04& m_ultrasonicSensor2;
            drivers::CHcsr04& m_ultrasonicSensor3;

            uint16_t distance_mm1;
            uint16_t distance_mm2;
            uint16_t distance_mm3;
            uint16_t m_samples;

            drivers::ISpeedingCommand&    m_speedingControl;

            /* @brief Serial communication obj.  */
            UnbufferedSerial&          m_serial;

            /** @brief Active flag  */
            bool m_isActive;

    }; // class CDistancemonitor
}; // namespace periodics

#endif // DISTANCEMONITOR_HPP
