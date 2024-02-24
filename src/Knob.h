#ifndef KNOB_HPP
#define KNOB_HPP

#include <string>
#include <STM32FreeRTOS.h>

class Knob {
public:
    Knob();  // Constructor
    ~Knob(); // Destructor

    // Other member functions
    uint8_t getRotationISR();
    uint8_t getRotation();
    void updateRotation(std::string BA_curr);

private:
    // Member variables
    SemaphoreHandle_t mutex;
    uint8_t rotation;
    std::string BA_prev;
    bool incrementLast;
};

#endif // KNOB_HPP
