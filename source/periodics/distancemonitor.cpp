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

        if (f_ultrasonicSensor1.getEchoState() == false) {
            m_ultrasonicSensor1.sendTriggerPulse();
        }
/*
        if (f_ultrasonicSensor2.getEchoState() == false) {
            m_ultrasonicSensor2.sendTriggerPulse();
        }

        m_ultrasonicSensor1.setEchoRiseCallbacks();
        m_ultrasonicSensor1.setEchoFallCallbacks();
*/
        distance_mm1 += m_ultrasonicSensor1.calculateAverageDistance();
        distance_mm2 += m_ultrasonicSensor2.calculateAverageDistance();

        char buffer[_18_chars];
        snprintf(buffer, sizeof(buffer), "@ultrasonic:%d;%d;;\r\n", distance_mm1, distance_mm2);
        m_serial.wr   ite(buffer,strlen(buffer));

        /*
        if (distance1 < 5) {
            m_speedingControl.setBrake();
        }
        */
        
    }
}; // namespace periodics