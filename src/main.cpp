#include <Arduino.h>
#include <Servo.h>
#include <ATmega128_pins.h>
#include <CommandHandler.h>

// Servo
Servo servo[4][3];
const uint8_t servo_pin[4][3] = {
    // Coxa,    Femur,    Tibia
    {PORT_F_7, PORT_F_6, PORT_F_5},  // Depan Kiri
    {PORT_A_0, PORT_A_1, PORT_A_2},  // Depan Kanan
    {PORT_D_7, PORT_D_6, PORT_D_5},  // Belakang Kanan
    {PORT_B_7, PORT_B_6, PORT_B_5}}; // Belakang Kiri

CommandHandler<> command(Serial, '[', ']');

// Function
void cmd_unknown();
void cmd_set_servo(CommandParameter &parameters);

void setup()
{
    Serial.begin(115200);
    Serial.println("Setup");

    command.AddCommand(F("servo"), cmd_set_servo);
    command.SetDefaultHandler(cmd_unknown);

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
    command.Process();
}


void cmd_unknown(){
    Serial.println(F("Unknown Command"));
}

void cmd_set_servo(CommandParameter &params)
{
    int leg = params.NextParameterAsInteger();

    if (leg < 0 || leg > 3) return;

    Serial.print(F("Leg "));
    Serial.print(leg);

    const int angle[3] = {
        params.NextParameterAsInteger(),
        params.NextParameterAsInteger(),
        params.NextParameterAsInteger(),
    };

    for (uint8_t i = 0; i < 3; i++){
        if (angle[i] < 0) return;
        if (angle[i] < 180) {
            servo[leg][i].write(angle[i]);
            Serial.print(F(" "));
            Serial.print(angle[i]);
        } else {
            Serial.print(F(" KEEP"));
        }
    }

    Serial.println();
}

