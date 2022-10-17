#include <Arduino.h>
#include <Servo.h>
#include <ATmega128_pins.h>

// Servo
Servo servo[4][3];
const uint8_t servo_pin[4][3] = {
    // Coxa,    Femur,    Tibia
    {PORT_F_7, PORT_F_6, PORT_F_5},  // Depan Kiri
    {PORT_A_0, PORT_A_1, PORT_A_2},  // Depan Kanan
    {PORT_D_7, PORT_D_6, PORT_D_5},  // Belakang Kanan
    {PORT_B_7, PORT_B_6, PORT_B_5}}; // Belakang Kiri

void setup()
{
    Serial.begin(115200);
    Serial.println("Setup");

    for (uint8_t i = 0; i < 4; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            servo[i][j].attach(servo_pin[i][j]);
            servo[i][j].write(90);
        }
    }
}

void loop()
{
   
}

