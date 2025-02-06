#include "drivers/hcsr04.hpp"
#define _18_chars 256
#define DISTANCE_SAMPLES 5

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
    , m_distance(0)
    , m_distanceAverage(0)
    , m_echoDetected(false)
    , m_distanceIndex(0)

    {
        /* constructor behaviour */
        m_pinTrg = 0; // Make sure the trigger pin is initially low
        m_pinEcho.mode(PullDown); // Make sure the echo pin is initially low
        m_timerEcho.stop(); // Make sure the timer is initially stopped

        //memset(m_lastDistances, 0, sizeof(m_lastDistances)); // Initialize the array with zeros
    }

    /** @brief  CHcsr04 class destructor
     */
    CHcsr04::~CHcsr04(){}

    void CHcsr04::setEchoRiseCallbacks() {
        m_pinEcho.rise(mbed::callback(this, &CHcsr04::onEchoRise));
    }

    void CHcsr04::setEchoFallCallbacks() {
        m_pinEcho.fall(mbed::callback(this, &CHcsr04::onEchoFall));
    }


    // Trigger the pulse for ultrasonic sensor
    void CHcsr04::sendTriggerPulse() {
        m_pinTrg = 1;
        ThisThread::sleep_for(chrono::microseconds(10).count() / 1000); // Convert microseconds to milliseconds
        m_pinTrg = 0;
    }

    // Called when echo pin rises (start of echo)
    void CHcsr04::onEchoRise() {
        //m_echoDetected = true;  // Indicar que el eco fue detectado
        m_timerEcho.reset(); // Reset the timer
        m_timerEcho.start(); // Start the timer
    }

    // Called when echo pin falls (end of echo)
    void CHcsr04::onEchoFall() {
        m_timerEcho.stop(); // Stop the timer
        uint32_t echoDuration = m_timerEcho.elapsed_time().count();// Measure the time when the echo pulse falls

        // Calculate the distance using the formula: Distance = (time * speed_of_sound) / 2
        // Speed of sound ≈ 0.034 cm/μs
//        m_distance = (echoDuration * 0.034 / 2) *10 ; // Convert cm to mm
        m_distance = (echoDuration * 0.17);
//        updateDistanceArray(m_distance);
    }

    uint16_t CHcsr04::getDistance() {
        return m_distance;
    }

    bool CHcsr04::getEchoState() {
        return m_echoDetected;
    }

    void CHcsr04::handleTimeout() {
        if (!m_echoDetected) {
            // Si no se detectó eco, volver a enviar un pulso trigger
            sendTriggerPulse();
        }
    }

    void CHcsr04::updateDistanceArray(uint16_t newDistance) {
        m_lastDistances[m_distanceIndex] = newDistance;
        m_distanceIndex = (m_distanceIndex + 1) % DISTANCE_SAMPLES; // Circular buffer logic
    }

    uint16_t CHcsr04::calculateAverageDistance() {
        for (uint8_t i = 0; i < DISTANCE_SAMPLES; ++i) {
            if (m_lastDistances[i] == 0) {
                return 0; // Return 0 if any value in the array is 0
            }
        }

        uint32_t sum = 0;
        for (uint8_t i = 0; i < DISTANCE_SAMPLES; ++i) {
//            printf("Distance %d: %d\n", i, m_lastDistances[i]);
            sum += m_lastDistances[i];
//            printf("Sum: %d\n", sum);
        }
//        printf("%d, %d, %d, %d, %d\n", m_lastDistances[0], m_lastDistances[1], m_lastDistances[2], m_lastDistances[3], m_lastDistances[4]);

        m_distanceAverage = sum / DISTANCE_SAMPLES;
        return m_distanceAverage;
    }
}; // namespace drivers