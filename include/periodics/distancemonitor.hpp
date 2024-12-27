#ifndef DISTANCEMONITOR_HPP
#define DISTANCEMONITOR_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>

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
                UnbufferedSerial& f_serial,
                mbed::DigitalOut f_pinTrg, 
                mbed::DigitalIn f_pinEcho, 
            );
            /* Destructor */
            ~CDistancemonitor();
            void serialCallbackDISTANCEMcommand(char const * a, char * b);

        private:
            /* private variables & method member */


            /* Run method */
            virtual void        _run();
            UnbufferedSerial&   m_serial;
            mbed::DigitalOut    m_pinTrg;
            mbed::DigitalIn     m_pinEcho;

            /** @brief Active flag  */
            bool m_isActive;

    }; // class CDistancemonitor
}; // namespace periodics

#endif // DISTANCEMONITOR_HPP
