#include "drivers/hcsr04.hpp"
#define _18_chars 256

namespace drivers
{
   /**
    * @brief Class constructor hcsr04
    *
    */
    CHcsr04::CHcsr04(
        PinName  f_pinTrg,
        PinName  f_pinEcho
    )
    : m_pinTrg(f_pinTrg)
    , m_pinEcho(f_pinEcho)
    {
        /* constructor behaviour */
        m_pinTrg = 0;
    }

    /** @brief  CHcsr04 class destructor
     */
    CHcsr04::~CHcsr04()
    {
    }
    uint16_t CHcsr04::averageDistance()
    {   
        uint16_t accumulated_distance = 0;
        
        for (uint8_t i = 0 ; i < 30; i++){
                accumulated_distance += CHcsr04::measureDistance();
                }

        return   accumulated_distance/30;
    }
    uint16_t CHcsr04::measureDistance()
    {
        /* Measure distance behaviour */
        uint32_t time;
        uint16_t distance;

        Timer timer;
        timer.start();

        wait_us(2);
        m_pinTrg = 1;
        wait_us(10);
        m_pinTrg = 0;

        // wait for the echo pin to go high
        Timer timeout;
        timeout.start();
        while (m_pinEcho == 0) {
            if (timeout.elapsed_time().count() > 30000) { // Timeout 30 ms
                return -1;
            }
        }

        // measure how long the echo pin was high
        timer.reset();
        while (m_pinEcho == 1) {
            if (timeout.elapsed_time().count() > 30000) { // Timeout 30 ms
                return -1;
            }
        }
        time = timer.elapsed_time().count(); // time in microseconds
        timer.stop();

        distance = time * 0.034f / 2.0f;

        return distance;

    };
}; // namespace drivers