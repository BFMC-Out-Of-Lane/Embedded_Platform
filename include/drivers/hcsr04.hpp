#ifndef HCSR04_HPP
#define HCSR04_HPP
#define DISTANCE_SAMPLES 5

/* The mbed library */
#include <mbed.h>
#include <chrono>

namespace drivers
{
   /**
    * @brief Class hcsr04
    *
    */
    class CHcsr04
    {
        public:
            /* Constructor */
            CHcsr04(
                PinName  f_pinTrg,
                PinName  f_pinEcho
            );
            /* Destructor */
            ~CHcsr04();

            void sendTriggerPulse();

            /* Método público para obtener la distancia */
            uint16_t getDistance();
            bool getEchoState();

            void setEchoRiseCallbacks();
            void setEchoFallCallbacks();

            /* Callbacks for echo pin */
            void onEchoRise();
            void onEchoFall();

            uint16_t calculateAverageDistance(); // Método para calcular el promedio de las distancias

        private:
            /* private variables & method member */

            /* @brief Pin trigger and echo */
            mbed::DigitalOut m_pinTrg;
            mbed::InterruptIn m_pinEcho;

            mbed::Timer m_timerEcho;/* Timer to measure echo pulse duration */
            Timeout m_timeout;       // Temporizador para manejar el timeout

            volatile uint16_t m_distance;/* Last measured distance */
            volatile uint32_t m_distanceAverage; // Average distance
            bool m_echoDetected;     // Bandera para saber si se recibió un pulso de eco
            uint16_t m_lastDistances[DISTANCE_SAMPLES]; // Array to store the last 5 distances
            uint8_t m_distanceIndex; // Index to keep track of the current position in the array

            void handleTimeout();   // Método para manejar el timeout
            void updateDistanceArray(uint16_t newDistance); // Método para actualizar el arreglo de distancias


    }; // class CHcsr04
}; // namespace drivers

#endif // HCSR04_HPP
