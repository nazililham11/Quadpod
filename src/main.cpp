#include <ATmega128_pins.h>
#include <Arduino.h>
#include <CommandHandler.h>
#include <Servo.h>
#include <NewPing.h>

// ID kaki
#define FRONT_R 0  // Depan Kanan
#define REAR_R 1   // Belakang Kanan
#define FRONT_L 2  // Depan Kiri
#define REAR_L 3   // Belakang Kiri

// ID gait action
#define SIT 0
#define STAND 1
#define MOVE_FORWARD 2
#define MOVE_BACKWARD 3
#define TURN_LEFT 4
#define TURN_RIGHT 5

// ID Ultrasonic
#define US_FRONT 0    // Depan
#define US_FRONT_R 1  // Depan Kanan
#define US_REAR_R 2   // Belakang Kanan
#define US_FRONT_L 3  // Depan Kiri
#define US_REAR_L 4   // Belakang Kiri

#define PING_MEDIAN 5    // Banyak penbacaan sensor untuk diambil nilai median
#define MAX_DISTANCE 400 // Jarak max ultrasonic (cm)

// ID Command info
# define VAL_ANGLE 0
# define VAL_SITE 1
# define VAL_CALIB 2

// Enable Services
#define ENABLE_SERVICES_PIN PORT_F_3
bool enable_services = true;

// Ultrasonic
uint16_t us_distance[5];
NewPing sonar[5] = {
    NewPing(PORT_E_2, PORT_E_2, MAX_DISTANCE),  // Depan
    NewPing(PORT_C_7, PORT_C_7, MAX_DISTANCE),  // Depan Kanan
    NewPing(PORT_C_6, PORT_C_6, MAX_DISTANCE),  // Belakang Kanan
    NewPing(PORT_E_3, PORT_E_3, MAX_DISTANCE),  // Depan Kiri
    NewPing(PORT_E_4, PORT_E_4, MAX_DISTANCE)   // Belakang Kiri
};

// Servo
Servo servo[4][3];
const uint8_t servo_pin[4][3] = {
    // Coxa,    Femur,    Tibia
    {PORT_A_0, PORT_A_1, PORT_A_2},   // Depan Kanan
    {PORT_D_7, PORT_D_6, PORT_D_5},   // Belakang Kanan
    {PORT_F_7, PORT_F_6, PORT_F_5},   // Depan Kiri
    {PORT_B_7, PORT_B_6, PORT_B_5}};  // Belakang Kiri

// Kalibrasi sudut servo
uint8_t enable_servo_calib = 1;
int8_t servo_calib[4][3] = {
    {-5, 0, 7},    // Depan Kanan
    {4, -7, -4},   // Belakang Kanan
    {-4, -2, -5},  // Depan Kiri
    {0, 0, 10}};   // Belakang Kiri

// Command
CommandHandler<10, 30, 20> command(Serial, '[', ']');

// Posisi kaki
float site_now[4][3];
float site_expect[4][3];

// Gerakan kaki
float z_base = -50;
float z_stand = -85;
float z_up = -30;
float x_base = 60;
float y_base = 0;
float y_step = 40;

// Kecepatan Gerakan
float move_speed = 1;
float stand_seat_speed = 1;
float leg_move_speed = 8;
float body_move_speed = 1.5;
float spot_turn_speed = 4;
float temp_speed[4][3];

// Dimensi
float coxa_len = 25;     // Coxa (mm)
float femur_len = 42;    // Femur (mm)
float tibia_len = 72;    // Coxa (mm)
float length_side = 77;  // Jarak antar kaki (mm)

// Konstan
const float pi = 3.1415926;
const float KEEP = 255;

// Konstan untuk putar
const float temp_a = sqrt(pow(2 * x_base + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_base + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_base + length_side, 2) +
                          pow(2 * y_base + y_step + length_side, 2));
const float temp_alpha = acos(
    (pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_base + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

// Task Timer
uint32_t site_timer_ms = 20;  // Site Timer
unsigned long site_timer;
uint32_t us_timer_ms = 1000;  // Ultrasonic Timer
unsigned long us_timer;

// Function
void servo_init();
void servo_write(uint8_t, float, float, float);

void us_refresh(uint8_t);
void us_refresh_all();

void cmd_init();
void cmd_unknown();
void cmd_set_leg(CommandParameter &);
void cmd_gait_action(CommandParameter &);
void cmd_get_config(CommandParameter &);
void cmd_get_leg(CommandParameter &);
void cmd_get_us(CommandParameter &);

void site_init();
void site_set(uint8_t, float, float, float);
void site_services();
void site_wait(uint8_t);
void site_wait_all();

void ik_polar_to_servo(uint8_t, float &, float &, float &);
void ik_cartesian_to_polar(float &, float &, float &, float, float, float);

void gait_sit();
void gait_stand();
void gait_move_forward(uint8_t);
void gait_move_backward(uint8_t);
void gait_turn_left(uint8_t);
void gait_turn_right(uint8_t);

void setup() {
    Serial.begin(115200);
    Serial.println(F("Setup"));

    pinMode(ENABLE_SERVICES_PIN, INPUT_PULLUP);

    cmd_init();
    servo_init();
    site_init();

    enable_services = digitalRead(ENABLE_SERVICES_PIN);

    Serial.println(F("Setup Done"));

    Serial.print(F("Services "));
    if (enable_services) Serial.println(F("Enabled"));
    else Serial.println(F("Disabled"));
}

void loop() {
    if (enable_services) {
        unsigned long currentTime = millis();
        if (currentTime - site_timer > site_timer_ms) {
            site_timer = currentTime;
            site_services();
        }
        if (currentTime - us_timer > us_timer_ms) {
            site_timer = currentTime;
            us_refresh_all();
        }
    }

    command.Process();
}

void us_refresh(uint8_t us_id) {
    delay(50);
    us_distance[us_id] = sonar[us_id].ping_median(PING_MEDIAN);
    us_distance[us_id] = sonar[us_id].convert_cm(us_distance[us_id]);
}

void us_refresh_all() {
    for (uint8_t i = 0; i < 5; i++) {
        us_refresh(i);
    }
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
    if (enable_servo_calib) {
        coxa = coxa + servo_calib[leg][0];
        femur = femur + servo_calib[leg][1];
        tibia = tibia + servo_calib[leg][2];
    }
    servo[leg][0].write(coxa);
    servo[leg][1].write(femur);
    servo[leg][2].write(tibia);
}

void cmd_init() {
    command.AddCommand(F("set_leg"), cmd_set_leg);
    command.AddCommand(F("gait_act"), cmd_gait_action);
    command.AddCommand(F("get_cfg"), cmd_get_config);
    command.AddCommand(F("get_leg"), cmd_get_leg);
    command.AddCommand(F("get_us"), cmd_get_us);
    command.SetDefaultHandler(cmd_unknown);

    command.AddVariable(F("enable_servo_calib"), enable_servo_calib);

    command.AddVariable(F("site_timer_ms"), site_timer_ms);
    command.AddVariable(F("us_timer_ms"), us_timer_ms);

    command.AddVariable(F("coxa_len"), coxa_len);
    command.AddVariable(F("femur_len"), femur_len);
    command.AddVariable(F("tibia_len"), tibia_len);

    command.AddVariable(F("z_base"), z_base);
    command.AddVariable(F("z_stand"), z_stand);
    command.AddVariable(F("z_up"), z_up);
    command.AddVariable(F("x_base"), x_base);
    command.AddVariable(F("y_base"), y_base);
    command.AddVariable(F("y_step"), y_step);

    command.AddVariable(F("stand_seat_speed"), stand_seat_speed);
    command.AddVariable(F("leg_move_speed"), leg_move_speed);
    command.AddVariable(F("body_move_speed"), body_move_speed);
    command.AddVariable(F("spot_turn_speed"), spot_turn_speed);
}

void cmd_unknown() { Serial.println(F("Unknown Command")); }

void cmd_set_leg(CommandParameter &params) {
    const int values_id = params.NextParameterAsInteger();
    const int leg_id = params.NextParameterAsInteger();

    if (values_id < 0 || values_id > 2) return;
    if (leg_id < 0 || leg_id > 3) return;

    int values[3] = {
        params.NextParameterAsInteger(),
        params.NextParameterAsInteger(),
        params.NextParameterAsInteger(),
    };

    bool is_valid[3];
    for (uint8_t i = 0; i < 3; i++) {
        if (values_id == VAL_ANGLE) {
            is_valid[i] = values[i] >= 0 && values[i] < 180;
            values[i] = is_valid[i] ? values[i] : servo[leg_id][i].read();
        } else if (values_id == VAL_SITE) {
            is_valid[i] = values[i] > -100 && values[i] < 100;
            values[i] = is_valid[i] ? values[i] : KEEP;
        } else if (values_id == VAL_CALIB) {
            is_valid[i] = values[i] > -180 && values[i] < 180;
            values[i] = is_valid[i] ? values[i] : servo_calib[leg_id][i];
        }
    }

    Serial.print(F("Leg "));
    Serial.print(leg_id);
    
    switch (values_id) {
        case VAL_ANGLE: Serial.print(F(" Angle ")); break;
        case VAL_SITE: Serial.print(F(" Site ")); break;
        case VAL_CALIB: Serial.print(F(" Calib ")); break;
    }
    
    for (uint8_t i = 0; i < 3; i++) {
        Serial.print(F(" "));
        if (is_valid[i]) Serial.print(values[i]);
        else Serial.print(F(" KEEP"));
    }

    Serial.println();

    if (values_id == VAL_SITE) {
        float coxa, femur, tibia;
        ik_cartesian_to_polar(femur, tibia, coxa, values[0], values[1], values[2]);
        ik_polar_to_servo(leg_id, femur, tibia, coxa);
        servo_write(leg_id, coxa, femur, tibia);
    } else {
        for (uint8_t i = 0; i < 3; i++) {
            if (values_id == VAL_ANGLE) {
                servo[leg_id][i].write(values[i]);
            } else if (values_id == VAL_CALIB) {
                servo_calib[leg_id][i] = values[i];   
            }
        }
    }
}

void cmd_gait_action(CommandParameter &params) {
    int action_id = params.NextParameterAsInteger();
    if (action_id < 0 || action_id > 5) return;

    Serial.print(F("Gait Action "));

    switch (action_id) {
        case 0: Serial.print(F("sit")); break;
        case 1: Serial.print(F("stand")); break;
        case 2: Serial.print(F("move_forward")); break;
        case 3: Serial.print(F("move_backward")); break;
        case 4: Serial.print(F("turn_left")); break;
        case 5: Serial.print(F("turn_right")); break;
    }

    int step = 0;
    if (action_id >= 2) {
        step = params.NextParameterAsInteger();
        if (step < 0) return;
        Serial.print(F(" step "));
    }
    Serial.println();

    switch (action_id) {
        case 0: gait_sit(); break;
        case 1: gait_stand(); break;
        case 2: gait_move_forward(step); break;
        case 3: gait_move_backward(step); break;
        case 4: gait_turn_left(step); break;
        case 5: gait_turn_right(step); break;
    }
}

void cmd_get_config(CommandParameter &params) {
    Serial.print(F("{\"coxa_len\":"));
    Serial.print(round(coxa_len));
    Serial.print(F(",\"femur_len\":"));
    Serial.print(round(femur_len));
    Serial.print(F(",\"tibia_len\":"));
    Serial.print(round(tibia_len));
    Serial.print(F(",\"length_side\":"));
    Serial.print(round(length_side));

    Serial.print(F(",\"z_base\":"));
    Serial.print(round(z_base));
    Serial.print(F(",\"z_stand\":"));
    Serial.print(round(z_stand));
    Serial.print(F(",\"z_up\":"));
    Serial.print(round(z_up));
    Serial.print(F(",\"x_base\":"));
    Serial.print(round(x_base));
    Serial.print(F(",\"y_base\":"));
    Serial.print(round(y_base));
    Serial.print(F(",\"y_step\":"));
    Serial.print(round(y_step));

    Serial.print(F(",\"move_speed\":"));
    Serial.print(round(move_speed));
    Serial.print(F(",\"stand_seat_speed\":"));
    Serial.print(round(stand_seat_speed));
    Serial.print(F(",\"leg_move_speed\":"));
    Serial.print(round(leg_move_speed));
    Serial.print(F(",\"body_move_speed\":"));
    Serial.print(round(body_move_speed));
    Serial.print(F(",\"spot_turn_speed\":"));
    Serial.print(round(spot_turn_speed));

    Serial.println(F("}"));
}

void cmd_get_us(CommandParameter &params) {
    int is_refresh = params.NextParameterAsInteger(0);
    if (is_refresh != 0) 
        us_refresh_all();
    
    Serial.print("{\"us\":[");
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(us_distance[i]);
        if (i < 4) Serial.print(",");
    }
    Serial.println("]}");
}

void cmd_get_leg(CommandParameter &params) {
    const int values_id = params.NextParameterAsInteger();
    if (values_id < 0 || values_id > 2) return;

    Serial.print(F("{\""));
    switch (values_id) {
        case VAL_ANGLE: Serial.print(F("angle")); break;
        case VAL_SITE: Serial.print(F("site")); break;
        case VAL_CALIB: Serial.print(F("calib")); break;
    }
    Serial.print(F("\":["));

    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("["));
        for (uint8_t j = 0; j < 3; j++) {
            switch (values_id) {
                case VAL_ANGLE: Serial.print(servo[i][j].read()); break;
                case VAL_SITE: Serial.print(round(site_now[i][j])); break;
                case VAL_CALIB: Serial.print(servo_calib[i][j]); break;
            }
            if (j < 2) Serial.print(F(","));
        }
        Serial.print(F("]"));
        if (i < 3) Serial.print(F(","));
    }
    Serial.println(F("]}"));
}

void site_init() {
    site_set(FRONT_L, x_base, y_base, z_base);  // Depan kiri
    site_set(FRONT_R, x_base, y_base, z_base);  // Depan kanan
    site_set(REAR_R, x_base, y_base, z_base);   // Belakang kanan
    site_set(REAR_L, x_base, y_base, z_base);   // Belakang kiri

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

void site_wait(uint8_t leg) {
    while (1) {
        unsigned long currentTime = millis();
        if (currentTime - site_timer > site_timer_ms) {
            site_timer = currentTime;
            site_services();
        }
        if (site_now[leg][0] != site_expect[leg][0]) continue;
        if (site_now[leg][1] != site_expect[leg][1]) continue;
        if (site_now[leg][2] != site_expect[leg][2]) continue;
        break;
    }
}

void site_wait_all() {
    for (uint8_t i = 0; i < 4; i++) {
        site_wait(i);
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
        case REAR_R:
        case FRONT_L:
            alpha = 90 - alpha;
            beta = beta;
            gamma = 90 - gamma;
            break;
        case FRONT_R:
        case REAR_L:
            alpha = 90 + alpha;
            beta = 180 - beta;
            gamma = 90 + gamma;
            break;
    }
}

void gait_sit() {
    Serial.println("Sit");
    move_speed = stand_seat_speed;
    site_set(FRONT_L, x_base, y_base + y_step, z_base);  // Depan kiri
    site_set(FRONT_R, x_base, y_base + y_step, z_base);  // Depan kanan
    site_set(REAR_R, x_base, y_base + y_step, z_base);   // Belakang kanan
    site_set(REAR_L, x_base, y_base + y_step, z_base);   // Belakang kiri
    site_wait_all();
}

void gait_stand() {
    Serial.println("Stand");
    move_speed = stand_seat_speed;
    site_set(FRONT_L, x_base, y_base + y_step, z_stand);  // Depan kiri
    site_set(FRONT_R, x_base, y_base + y_step, z_stand);  // Depan kanan
    site_set(REAR_R, x_base, y_base + y_step, z_stand);   // Belakang kanan
    site_set(REAR_L, x_base, y_base + y_step, z_stand);   // Belakang kiri
    site_wait_all();
}

void gait_move_forward(uint8_t step) {
    Serial.print("Move Forward for ");
    Serial.print(step);
    Serial.println(" step");

    move_speed = leg_move_speed;
    while (step-- > 0) {
        if (site_now[FRONT_L][1] == y_base) {
            // Gerak kaki depan kiri dan belakang kanan
            site_set(FRONT_L, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_L, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(FRONT_L, x_base, y_base + 2 * y_step, z_base);
            site_wait_all();
            move_speed = body_move_speed;
            site_set(FRONT_R, x_base, y_base, z_base);
            site_set(REAR_R, x_base, y_base + 2 * y_step, z_base);
            site_set(FRONT_L, x_base, y_base + y_step, z_base);
            site_set(REAR_L, x_base, y_base + y_step, z_base);
            site_wait_all();
            move_speed = leg_move_speed;
            site_set(REAR_R, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(REAR_R, x_base, y_base, z_up);
            site_wait_all();
            site_set(REAR_R, x_base, y_base, z_base);
            site_wait_all();
        } else {
            // Gerak kaki depan kanan dan belakang kiri
            site_set(FRONT_R, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base + 2 * y_step, z_base);
            site_wait_all();
            move_speed = body_move_speed;
            site_set(FRONT_R, x_base, y_base + y_step, z_base);
            site_set(REAR_R, x_base, y_base + y_step, z_base);
            site_set(FRONT_L, x_base, y_base, z_base);
            site_set(REAR_L, x_base, y_base + 2 * y_step, z_base);
            site_wait_all();
            move_speed = leg_move_speed;
            site_set(REAR_L, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(REAR_L, x_base, y_base, z_up);
            site_wait_all();
            site_set(REAR_L, x_base, y_base, z_base);
            site_wait_all();
        }
    }
}

void gait_move_backward(uint8_t step) {
    Serial.print("Move Backward for ");
    Serial.print(step);
    Serial.println(" step");

    move_speed = leg_move_speed;
    while (step-- > 0) {
        if (site_now[REAR_L][1] == y_base) {
            // Gerak kaki belakang kiri dan depan kanan
            site_set(REAR_L, x_base, y_base, z_up);
            site_wait_all();
            site_set(REAR_L, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(REAR_L, x_base, y_base + 2 * y_step, z_base);
            site_wait_all();
            move_speed = body_move_speed;
            site_set(FRONT_R, x_base, y_base + 2 * y_step, z_base);
            site_set(REAR_R, x_base, y_base, z_base);
            site_set(FRONT_L, x_base, y_base + y_step, z_base);
            site_set(REAR_L, x_base, y_base + y_step, z_base);
            site_wait_all();
            move_speed = leg_move_speed;
            site_set(FRONT_R, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base, z_base);
            site_wait_all();
        } else {
            // Gerak kaki belakang kanan dan depan kiri
            site_set(REAR_R, x_base, y_base, z_up);
            site_wait_all();
            site_set(REAR_R, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(REAR_R, x_base, y_base + 2 * y_step, z_base);
            site_wait_all();
            move_speed = body_move_speed;
            site_set(FRONT_R, x_base, y_base + y_step, z_base);
            site_set(REAR_R, x_base, y_base + y_step, z_base);
            site_set(FRONT_L, x_base, y_base + 2 * y_step, z_base);
            site_set(REAR_L, x_base, y_base, z_base);
            site_wait_all();
            move_speed = leg_move_speed;
            site_set(FRONT_L, x_base, y_base + 2 * y_step, z_up);
            site_wait_all();
            site_set(FRONT_L, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_L, x_base, y_base, z_base);
            site_wait_all();
        }
    }
}

void gait_turn_left(uint8_t step) {
    Serial.print("Turn Left for ");
    Serial.print(step);
    Serial.println(" step");

    move_speed = spot_turn_speed;
    while (step-- > 0) {
        if (site_now[REAR_L][1] == y_base) {
            // Gerak kaki belakang kiri dan belakang kanan
            site_set(REAR_L, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_R, turn_x1, turn_y1, z_base);
            site_set(REAR_R, turn_x0, turn_y0, z_base);
            site_set(FRONT_L, turn_x1, turn_y1, z_base);
            site_set(REAR_L, turn_x0, turn_y0, z_up);
            site_wait_all();
            site_set(REAR_L, turn_x0, turn_y0, z_base);
            site_wait_all();
            site_set(FRONT_R, turn_x1, turn_y1, z_base);
            site_set(REAR_R, turn_x0, turn_y0, z_base);
            site_set(FRONT_L, turn_x1, turn_y1, z_base);
            site_set(REAR_L, turn_x0, turn_y0, z_base);
            site_wait_all();
            site_set(REAR_R, turn_x0, turn_y0, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base, z_base);
            site_set(REAR_R, x_base, y_base, z_up);
            site_set(FRONT_L, x_base, y_base + y_step, z_base);
            site_set(REAR_L, x_base, y_base + y_step, z_base);
            site_wait_all();
            site_set(REAR_R, x_base, y_base, z_base);
            site_wait_all();
        } else {
            // Gerak kaki depan kanan dan depan kiri
            site_set(FRONT_R, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_R, turn_x0, turn_y0, z_up);
            site_set(REAR_R, turn_x1, turn_y1, z_base);
            site_set(FRONT_L, turn_x0, turn_y0, z_base);
            site_set(REAR_L, turn_x1, turn_y1, z_base);
            site_wait_all();
            site_set(FRONT_R, turn_x0, turn_y0, z_base);
            site_wait_all();
            site_set(FRONT_R, turn_x0, turn_y0, z_base);
            site_set(REAR_R, turn_x1, turn_y1, z_base);
            site_set(FRONT_L, turn_x0, turn_y0, z_base);
            site_set(REAR_L, turn_x1, turn_y1, z_base);
            site_wait_all();
            site_set(FRONT_L, turn_x0, turn_y0, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base + y_step, z_base);
            site_set(REAR_R, x_base, y_base + y_step, z_base);
            site_set(FRONT_L, x_base, y_base, z_up);
            site_set(REAR_L, x_base, y_base, z_base);
            site_wait_all();
            site_set(FRONT_L, x_base, y_base, z_base);
            site_wait_all();
        }
    }
}

void gait_turn_right(uint8_t step) {
    Serial.print("Turn Right for ");
    Serial.print(step);
    Serial.println(" step");

    move_speed = spot_turn_speed;
    while (step-- > 0) {
        if (site_now[FRONT_L][1] == y_base) {
            // Gerak kaki depan kiri dan depan kanan
            site_set(FRONT_L, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_R, turn_x0, turn_y0, z_base);
            site_set(REAR_R, turn_x1, turn_y1, z_base);
            site_set(FRONT_L, turn_x0, turn_y0, z_up);
            site_set(REAR_L, turn_x1, turn_y1, z_base);
            site_wait_all();
            site_set(FRONT_L, turn_x0, turn_y0, z_base);
            site_wait_all();
            site_set(FRONT_R, turn_x0, turn_y0, z_base);
            site_set(REAR_R, turn_x1, turn_y1, z_base);
            site_set(FRONT_L, turn_x0, turn_y0, z_base);
            site_set(REAR_L, turn_x1, turn_y1, z_base);
            site_wait_all();
            site_set(FRONT_R, turn_x0, turn_y0, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base, z_up);
            site_set(REAR_R, x_base, y_base, z_base);
            site_set(FRONT_L, x_base, y_base + y_step, z_base);
            site_set(REAR_L, x_base, y_base + y_step, z_base);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base, z_base);
            site_wait_all();
        } else {
            // Gerak kaki belakang kanan dan belakang kiri
            site_set(REAR_R, x_base, y_base, z_up);
            site_wait_all();
            site_set(FRONT_R, turn_x1, turn_y1, z_base);
            site_set(REAR_R, turn_x0, turn_y0, z_up);
            site_set(FRONT_L, turn_x1, turn_y1, z_base);
            site_set(REAR_L, turn_x0, turn_y0, z_base);
            site_wait_all();
            site_set(REAR_R, turn_x0, turn_y0, z_base);
            site_wait_all();
            site_set(FRONT_R, turn_x1, turn_y1, z_base);
            site_set(REAR_R, turn_x0, turn_y0, z_base);
            site_set(FRONT_L, turn_x1, turn_y1, z_base);
            site_set(REAR_L, turn_x0, turn_y0, z_base);
            site_wait_all();
            site_set(REAR_L, turn_x0, turn_y0, z_up);
            site_wait_all();
            site_set(FRONT_R, x_base, y_base + y_step, z_base);
            site_set(REAR_R, x_base, y_base + y_step, z_base);
            site_set(FRONT_L, x_base, y_base, z_base);
            site_set(REAR_L, x_base, y_base, z_up);
            site_wait_all();
            site_set(REAR_L, x_base, y_base, z_base);
            site_wait_all();
        }
    }
}