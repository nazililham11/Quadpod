#include <ATmega128_pins.h>
#include <Arduino.h>
#include <CommandHandler.h>
#include <Servo.h>

// Servo
Servo servo[4][3];
const uint8_t servo_pin[4][3] = {
    // Coxa,    Femur,    Tibia
    {PORT_F_7, PORT_F_6, PORT_F_5},   // Depan Kiri
    {PORT_A_0, PORT_A_1, PORT_A_2},   // Depan Kanan
    {PORT_D_7, PORT_D_6, PORT_D_5},   // Belakang Kanan
    {PORT_B_7, PORT_B_6, PORT_B_5}};  // Belakang Kiri

// Command
CommandHandler<> command(Serial, '[', ']');

// Posisi kaki
float site_now[4][3];
float site_expect[4][3];
bool is_stand;

// Gerakan kaki
const float z_base = -50;
const float z_stand = -85;
const float x_base = 60;
const float y_base = 0;

// Kecepatan Gerakan
float move_speed = 1;
float stand_seat_speed = 1;
float temp_speed[4][3];

// Dimensi
const float coxa_len = 25;   // Coxa (mm)
const float femur_len = 42;  // Femur (mm)
const float tibia_len = 72;  // Coxa (mm)

// Konstan
const float pi = 3.1415926;
const float KEEP = 255;

// Task Timer
uint16_t site_timer_ms = 50;
unsigned long site_timer;
uint16_t job_timer_ms = 5000;
unsigned long job_timer;

// Function
void servo_init();
void servo_write(uint8_t leg, float coxa, float femur, float tibia);

void cmd_init();
void cmd_unknown();
void cmd_set_servo(CommandParameter &params);

void site_init();
void site_set(uint8_t leg, float x, float y, float z);
void site_services();

void ik_polar_to_servo(uint8_t leg, float &alpha, float &beta, float &gamma);
void ik_cartesian_to_polar(float &alpha, float &beta, float &gamma, float x,
                           float y, float z);

void gait_sit();
void gait_stand();

void setup() {
    Serial.begin(115200);
    Serial.println("Setup");

    cmd_init();
    servo_init();
    site_init();

    Serial.println("Setup Done");
}

void loop() {
    unsigned long currentTime = millis();

    if (currentTime - site_timer > site_timer_ms) {
        site_timer = currentTime;
        site_services();
    }

    if (currentTime - job_timer > job_timer_ms) {
        job_timer = currentTime;
        if (is_stand)
            gait_sit();
        else
            gait_stand();
        is_stand = !is_stand;
    }

    command.Process();
}

void servo_init() {
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            servo[i][j].attach(servo_pin[i][j]);
            servo[i][j].write(90);
        }
    }
}

void servo_write(uint8_t leg, float coxa, float femur, float tibia) {
    servo[leg][0].write(coxa);
    servo[leg][1].write(femur);
    servo[leg][2].write(tibia);
}

void cmd_init() {
    command.AddCommand(F("servo"), cmd_set_servo);
    command.SetDefaultHandler(cmd_unknown);
}

void cmd_unknown() { Serial.println(F("Unknown Command")); }

void cmd_set_servo(CommandParameter &params) {
    int leg = params.NextParameterAsInteger();

    if (leg < 0 || leg > 3) return;

    Serial.print(F("Leg "));
    Serial.print(leg);

    const int angle[3] = {
        params.NextParameterAsInteger(),
        params.NextParameterAsInteger(),
        params.NextParameterAsInteger(),
    };

    for (uint8_t i = 0; i < 3; i++) {
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

void site_init() {
    site_set(0, x_base, y_base, z_base);  // Depan kiri
    site_set(1, x_base, y_base, z_base);  // Depan kanan
    site_set(2, x_base, y_base, z_base);  // Belakang kanan
    site_set(3, x_base, y_base, z_base);  // Belakang kiri

    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            site_now[i][j] = site_expect[i][j];
        }
    }
}

void site_set(uint8_t leg, float x, float y, float z) {
    float length_x = 0, length_y = 0, length_z = 0;

    if (x != KEEP) length_x = x - site_now[leg][0];
    if (y != KEEP) length_y = y - site_now[leg][1];
    if (z != KEEP) length_z = z - site_now[leg][2];

    float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

    temp_speed[leg][0] = length_x / length * move_speed;
    temp_speed[leg][1] = length_y / length * move_speed;
    temp_speed[leg][2] = length_z / length * move_speed;

    if (x != KEEP) site_expect[leg][0] = x;
    if (y != KEEP) site_expect[leg][1] = y;
    if (z != KEEP) site_expect[leg][2] = z;
}

void site_services() {
    float coxa, femur, tibia;

    for (uint8_t i = 0; i < 4; i++) {
        bool is_expect = true;

        for (uint8_t j = 0; j < 3; j++) {
            if (site_now[i][j] != site_expect[i][j]) is_expect = false;

            if (abs(site_now[i][j] - site_expect[i][j]) >=
                abs(temp_speed[i][j]))
                site_now[i][j] += temp_speed[i][j];
            else
                site_now[i][j] = site_expect[i][j];
        }

        if (!is_expect) {
            ik_cartesian_to_polar(femur, tibia, coxa, site_now[i][0],
                                  site_now[i][1], site_now[i][2]);
            ik_polar_to_servo(i, femur, tibia, coxa);
            servo_write(i, coxa, femur, tibia);
        }
    }
}

void ik_cartesian_to_polar(float &alpha, float &beta, float &gamma, float x,
                           float y, float z) {
    float v, w;

    w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
    v = w - coxa_len;

    alpha =
        atan2(z, v) +
        acos((pow(femur_len, 2) - pow(tibia_len, 2) + pow(v, 2) + pow(z, 2)) /
             2 / femur_len / sqrt(pow(v, 2) + pow(z, 2)));
    beta =
        acos((pow(femur_len, 2) + pow(tibia_len, 2) - pow(v, 2) - pow(z, 2)) /
             2 / femur_len / tibia_len);

    gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);

    alpha = alpha / pi * 180;
    beta = beta / pi * 180;
    gamma = gamma / pi * 180;
}

void ik_polar_to_servo(uint8_t leg, float &alpha, float &beta, float &gamma) {
    switch (leg) {
        case 2:  // Belakang Kanan
        case 0:  // Depan Kiri
            alpha = 90 - alpha;
            beta = beta;
            gamma = 90 - gamma;
            break;

        case 1:  // Depan Kanan
        case 3:  // Belakang Kiri
            alpha = 90 + alpha;
            beta = 180 - beta;
            gamma += 90;
            break;
    }
}

void gait_sit() {
    Serial.println("Sit");
    move_speed = stand_seat_speed;
    site_set(0, x_base, y_base, z_base);  // Depan kiri
    site_set(1, x_base, y_base, z_base);  // Depan kanan
    site_set(2, x_base, y_base, z_base);  // Belakang kanan
    site_set(3, x_base, y_base, z_base);  // Belakang kiri
}

void gait_stand() {
    Serial.println("Stand");
    move_speed = stand_seat_speed;
    site_set(0, x_base, y_base, z_stand);  // Depan kiri
    site_set(1, x_base, y_base, z_stand);  // Depan kanan
    site_set(2, x_base, y_base, z_stand);  // Belakang kanan
    site_set(3, x_base, y_base, z_stand);  // Belakang kiri
}