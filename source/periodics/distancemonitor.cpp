#include "periodics/distancemonitor.hpp"

#define _18_chars 256
#define SENSOR_SAMPLES 6

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
    , m_isBraking(false)
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
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_distanceMon_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
        }else{
            sprintf(b,"syntax error");
        }
    }

    /* Run method */
    void CDistancemonitor::_run()
    {
        /* Run method behaviour */
        if(!m_isActive) return;

        m_ultrasonicSensor1.sendTriggerPulse();
        m_ultrasonicSensor2.sendTriggerPulse();

        m_ultrasonicSensor1.setEchoRiseCallbacks();
        m_ultrasonicSensor1.setEchoFallCallbacks();
        m_ultrasonicSensor2.setEchoRiseCallbacks();
        m_ultrasonicSensor2.setEchoFallCallbacks();

        distance_mm1 = m_ultrasonicSensor1.getDistance();
        distance_mm2 = m_ultrasonicSensor2.getDistance();

        if (m_samples < SENSOR_SAMPLES) {
            m_samples++;
        }
        else {
            m_samples = 0;
//            distance_mm1 = m_ultrasonicSensor1.calculateAverageDistance();
//            distance_mm2 = m_ultrasonicSensor2.calculateAverageDistance();

//            char buffer[_18_chars];
//            snprintf(buffer, sizeof(buffer), "@ultrasonic:%d;%d;;\r\n", distance_mm1*10, distance_mm2*10);
//            m_serial.write(buffer,strlen(buffer));
            printf("@ultrasonic:%d;%d;;\r\n", distance_mm1, distance_mm2);
        }

//		if (distance_mm1 < 200 ) {
////				if (distance_mm1 < 200 || distance_mm2 < 200) {
//			if (!m_isBraking) {
//				m_speedingControl.setBrake();
//				m_isBraking = true;
//				m_brakeEndTime = std::chrono::steady_clock::now() + std::chrono::seconds(2);
//				printf("FRENO DE EMERGENCIA!");
//			}
//		}
//
//		if (m_isBraking && std::chrono::steady_clock::now() >= m_brakeEndTime) {
//			m_isBraking = false;
//		}

    }
}; // namespace periodics