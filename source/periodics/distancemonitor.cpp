#include "periodics/distancemonitor.hpp"

#define buffer_256 256

// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor distancemonitor
    *
    */
    CDistancemonitor::CDistancemonitor(
        std::chrono::milliseconds f_period,
        UnbufferedSerial& f_serial,
        mbed::DigitalOut f_pinTrg;
        mbed::DigitalIn  f_pinEcho;
    )
    : utils::CTask(f_period)
    , m_serial(f_serial) 
    , m_pinTrg(f_pinTrg)
    , m_pinEcho(f_pinTrg)
    {
    }

    /** @brief  CDistancemonitor class destructor
     */
    CDistancemonitor::~CDistancemonitor()
    {
    }

    void CDistancemonitor::serialCallbackDISTANCEMcommand(char const * a, char const * b){
        uint8_t l_isActivate;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);
        if(1 == l_res){
            m_isActive=(l_isActivate>=1);
            sprintf(b,"1");
        }else{
            sprintf(b,"syntax error");
        }
    }

    /* Run method */
    void CDistancemonitor::_run()
    {
        /* Run method behaviour */
        if(!m_isActive) return;

        char buffer[buffer_256];
        snprintf(buffer, sizeof(buffer), "@ultrasonic:%d:%d;;\r\n", 123,123);
        m_serial.write(buffer, strlen(buffer));

    }

}; // namespace periodics