#include "periodics/distancemonitor.hpp"

#define _18_chars 256

namespace periodics
{
   /**
    * @brief Class constructor distancemonitor
    *
    */
    CDistancemonitor::CDistancemonitor(
        std::chrono::milliseconds f_period,
        PinName f_pinTrg1,
        PinName f_pinEcho1,
        PinName f_pinTrg2,
        PinName f_pinEcho2,
        PinName f_pinTrg3,
        PinName f_pinEcho3,
        drivers::ISpeedingCommand&    f_speedingControl,
        UnbufferedSerial& f_serial
    )
    : utils::CTask(f_period)
    , m_ultrasonicSensor1(f_pinTrg1, f_pinEcho1)
    , m_ultrasonicSensor2(f_pinTrg2, f_pinEcho2)
    , m_ultrasonicSensor3(f_pinTrg3, f_pinEcho3)
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

        uint16_t distance_mm1 = m_ultrasonicSensor1.averageDistance() * 10;
        uint16_t distance_mm2 = m_ultrasonicSensor2.averageDistance() * 10;
        //uint16_t distance3 = m_ultrasonicSensor3.averageDistance();

        char buffer[_18_chars];

        snprintf(buffer, sizeof(buffer), "@ultrasonic:%d;%d;;\r\n", distance_mm1, distance_mm2);
        m_serial.write(buffer,strlen(buffer));

        /*
        if (distance1 < 5) {
            m_speedingControl.setBrake();
        }
        */
        
    }
}; // namespace periodics