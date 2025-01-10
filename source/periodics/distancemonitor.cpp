#include "periodics/distancemonitor.hpp"

#define _18_chars 256
#define DISTANCE_SAMPLES 10

namespace periodics
{
   /**
    * @brief Class constructor distancemonitor
    *
    */
    CDistancemonitor::CDistancemonitor(
        std::chrono::milliseconds f_period,
        drivers::CHcsr04& f_ultrasonicSensor1,
        drivers::CHcsr04& f_ultrasonicSensor2,
        drivers::CHcsr04& f_ultrasonicSensor3,
        drivers::ISpeedingCommand&    f_speedingControl,
        UnbufferedSerial& f_serial
    )
    : utils::CTask(f_period)
    , m_ultrasonicSensor1(f_ultrasonicSensor1)
    , m_ultrasonicSensor2(f_ultrasonicSensor2)
    , m_ultrasonicSensor3(f_ultrasonicSensor3)
    , distance_mm1(0)
    , distance_mm2(0)
    , distance_mm3(0)
    , m_samples(0)
    , m_speedingControl(f_speedingControl)
    , m_serial(f_serial)
    , m_isActive(false)
    {
        /* constructor behaviour */
    }

    /** @brief  CDistancemonitor class destructor
     */
    CDistancemonitor::~CDistancemonitor()
    {
    }

    void CDistancemonitor::serialCallbackDISTANCEMONcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
                m_isActive=(l_isActivate>=1);
                //bool_globalsV_x_isActive = (l_isActivate>=1);
                sprintf(b,"1");
        }else{
            sprintf(b,"syntax error");
        }
    }

    /* Run method */
    void CDistancemonitor::_run()
    {
        /* Run method behaviour */
        //if(!m_isActive) return;

        m_ultrasonicSensor1.sendTriggerPulse();
        //m_ultrasonicSensor2.sendTriggerPulse();
        //m_ultrasonicSensor3.sendTriggerPulse();

        m_ultrasonicSensor1.setEchoRiseCallbacks(mbed::callback(this, &drivers::CHcsr04::onEchoRise);
        m_ultrasonicSensor1.setEchoFallCallbacks(mbed::callback(this, &drivers::CHcsr04::onEchoFall);

        if (m_samples < DISTANCE_SAMPLES){
            distance_mm1 += m_ultrasonicSensor1.getDistance();
            distance_mm2 += m_ultrasonicSensor2.getDistance();
            //distance_mm3 += m_ultrasonicSensor3.getDistance();
            m_samples++;
        }
        else {

            char buffer[_18_chars];
            snprintf(buffer, sizeof(buffer), "@ultrasonic:%d;%d;;\r\n", distance_mm1/DISTANCE_SAMPLES, distance_mm2/DISTANCE_SAMPLES);
            m_serial.write(buffer,strlen(buffer));

            distance_mm1 = 0;
            distance_mm2 = 0;
            //distance_mm3 = 0;
            m_samples = 0;
        }

        /*
        if (distance1 < 5) {
            m_speedingControl.setBrake();
        }
        */
        
    }
}; // namespace periodics