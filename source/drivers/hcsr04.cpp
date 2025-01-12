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
    , m_distance(0)
    {
        /* constructor behaviour */
        m_pinTrg = 0; // Make sure the trigger pin is initially low
        m_pinEcho.mode(PullDown); // Make sure the echo pin is initially low
        m_timerEcho.stop(); // Make sure the timer is initially stopped
        // Configure interrupts for echo pin
        //m_pinEcho.rise(callback(this, &CHcsr04::onEchoRise));
        //m_pinEcho.fall(callback(this, &CHcsr04::onEchoFall));
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
        m_timerEcho.reset(); // Reset the timer
        m_timerEcho.start(); // Start the timer
    }

    // Called when echo pin falls (end of echo)
    void CHcsr04::onEchoFall() {
        m_timerEcho.stop(); // Stop the timer
        uint32_t echoDuration = m_timerEcho.elapsed_time().count();// Measure the time when the echo pulse falls

        // Calculate the distance using the formula: Distance = (time * speed_of_sound) / 2
        // Speed of sound ≈ 0.034 cm/μs
        m_distance = echoDuration * 0.034 / 2;
    }

    uint16_t CHcsr04::getDistance() {
        return m_distance;
    }
}; // namespace drivers