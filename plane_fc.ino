#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// ============================================================
//  STANDALONE CRSF PARSER
//  Extracted from madflight crsf.h - zero external dependencies
//  CRC table and decode logic are verbatim from the source
// ============================================================
#define CRSF_BAUD              420000
#define CRSF_ADDRESS_FC        0xC8
#define CRSF_FRAMETYPE_RC      0x16
#define CRSF_FRAME_SIZE_MAX    64

class CRSFParser {
public:
    uint16_t channel[16];   // Output: 988-2012us, ready to use directly
    uint32_t channel_ts = 0;
    uint32_t timeout_ms = 1000;

    CRSFParser() {
        buf_i = 0;
        buf_len = 0;
        // Safe defaults: throttle low, everything else centred
        for (int i = 0; i < 16; i++) channel[i] = 1500;
        channel[2] = 988; // throttle channel default = minimum
    }

    // Feed one byte at a time. Returns true when a valid RC frame is decoded.
    bool update(uint8_t c) {
        if (buf_i == 0) {
            if (c == CRSF_ADDRESS_FC) buf[buf_i++] = c;
        } else if (buf_i == 1) {
            if (c >= 2 && c <= (CRSF_FRAME_SIZE_MAX - 2)) {
                buf[buf_i++] = c;
                buf_len = c + 2;
            } else {
                buf_i = 0;
            }
        } else if (buf_i < buf_len - 1) {
            buf[buf_i++] = c;
        } else if (buf_i == buf_len - 1) {
            buf[buf_i++] = c;
            uint8_t crc = crsf_crc8(buf + 2, buf[1] - 1);
            bool ok = (crc == buf[buf_len - 1]);
            buf_i = 0;
            if (ok) return decode();
        } else {
            buf_i = 0;
        }
        return false;
    }

    bool is_connected() {
        return (millis() - channel_ts) < timeout_ms;
    }

private:
    uint8_t buf[CRSF_FRAME_SIZE_MAX];
    uint8_t buf_i, buf_len;

    bool decode() {
        if (buf[2] != CRSF_FRAMETYPE_RC) return false;
        if (buf_len != 22 + 4) return false;
        channel_ts = millis();
        channel[0]  = cv((buf[3]  | buf[4]  << 8) & 0x7FF);
        channel[1]  = cv((buf[4]  >> 3 | buf[5]  << 5) & 0x7FF);
        channel[2]  = cv((buf[5]  >> 6 | buf[6]  << 2 | buf[7]  << 10) & 0x7FF);
        channel[3]  = cv((buf[7]  >> 1 | buf[8]  << 7) & 0x7FF);
        channel[4]  = cv((buf[8]  >> 4 | buf[9]  << 4) & 0x7FF);
        channel[5]  = cv((buf[9]  >> 7 | buf[10] << 1 | buf[11] << 9) & 0x7FF);
        channel[6]  = cv((buf[11] >> 2 | buf[12] << 6) & 0x7FF);
        channel[7]  = cv((buf[12] >> 5 | buf[13] << 3) & 0x7FF);
        channel[8]  = cv((buf[14] | buf[15] << 8) & 0x7FF);
        channel[9]  = cv((buf[15] >> 3 | buf[16] << 5) & 0x7FF);
        channel[10] = cv((buf[16] >> 6 | buf[17] << 2 | buf[18] << 10) & 0x7FF);
        channel[11] = cv((buf[18] >> 1 | buf[19] << 7) & 0x7FF);
        channel[12] = cv((buf[19] >> 4 | buf[20] << 4) & 0x7FF);
        channel[13] = cv((buf[20] >> 7 | buf[21] << 1 | buf[22] << 9) & 0x7FF);
        channel[14] = cv((buf[22] >> 2 | buf[23] << 6) & 0x7FF);
        channel[15] = cv((buf[23] >> 5 | buf[24] << 3) & 0x7FF);
        return true;
    }

    // Verbatim CRC table from madflight crsf.h
    uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
        static const uint8_t tab[256] = {
            0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
            0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
            0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
            0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
            0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
            0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
            0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
            0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
            0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
            0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
            0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
            0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
            0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
            0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
            0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
            0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9
        };
        uint8_t crc = 0;
        for (uint8_t i = 0; i < len; i++) crc = tab[crc ^ *ptr++];
        return crc;
    }

    // Verbatim from madflight crsf.h convert_channel_value()
    // 172 -> 988us, 992 -> 1500us, 1811 -> 2012us
    uint16_t cv(unsigned v) {
        static constexpr float scale  = (2012.f - 988.f) / (1811.f - 172.f);
        static constexpr float offset = 988.f - 172.f * scale;
        return (uint16_t)(scale * v + offset);
    }
};

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define CRSF_RX_PIN   16    // ELRS Nano TX --> ESP32 GPIO16
#define CRSF_TX_PIN   17    // ELRS Nano RX --> ESP32 GPIO17 (telemetry, optional)

#define PIN_ESC       5     // Motor ESC
#define PIN_AILERON_L 18    // Left aileron servo
#define PIN_AILERON_R 19    // Right aileron servo
#define PIN_ELEVATOR  15    // Elevator servo
#define PIN_RUDDER    4     // Rudder servo

#define BATTERY_PIN   25
#define LED_PIN       2

// ============================================================
//  RC CHANNEL MAPPING
//  Default C8 ELRS Mode 2 layout:
//  CH1=Roll  CH2=Pitch  CH3=Throttle  CH4=Yaw  CH5=Arm  CH6=Mode
// ============================================================
#define CH_ROLL      0
#define CH_PITCH     1
#define CH_THROTTLE  2
#define CH_YAW       3
#define CH_ARM       4
#define CH_MODE      5      // >1500 = stabilised, <1500 = manual

// ============================================================
//  SERVO DIRECTION FLAGS
//  Set to true if that surface moves the WRONG way, to flip it
//  Test in MANUAL mode on the bench before first flight
// ============================================================
#define REVERSE_AILERON_L  false
#define REVERSE_AILERON_R  true   // right aileron is always opposite to left
#define REVERSE_ELEVATOR   false
#define REVERSE_RUDDER     false

// ============================================================
//  BATTERY
// ============================================================
const float REF_VOLTAGE = 3.3f;
const float R1          = 77600.0f;
const float R2          = 29400.0f;
float battery_voltage   = 10.0f;

// ============================================================
//  GLOBALS
// ============================================================
HardwareSerial crsfSerial(2);
CRSFParser crsf;

int  rcIn[16];
bool isArmed        = false;
bool stabEnabled    = false;
bool crsfSignalLost = true;

Servo escMotor;
Servo servoAilL;
Servo servoAilR;
Servo servoElev;
Servo servoRudd;

// Servo trim offsets in microseconds - adjust after bench test
int trimAilL = 0;
int trimAilR = 0;
int trimElev = 0;
int trimRudd = 0;

// ============================================================
//  IMU
// ============================================================
Adafruit_ICM20948 icm;

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateCalRoll, RateCalPitch, RateCalYaw;

float KalmanAngleRoll  = 0, KalmanUncRoll  = 4.0f;
float KalmanAnglePitch = 0, KalmanUncPitch = 4.0f;

unsigned long LoopTimer = 0;

// ============================================================
//  PID
// ============================================================
struct PIDState { float prevError = 0, prevIterm = 0; };
PIDState pidRoll, pidPitch, pidYaw;

// Gains - all normalized: error in degrees, output in degrees (then scaled to us)
// P: fraction of max correction per degree of error
// D: uses raw gyro rate directly (deg/s), not numerical differentiation
const float P_ROLL  = 0.8f,  I_ROLL  = 0.0f,  D_ROLL  = 0.03f;
const float P_PITCH = 0.8f,  I_PITCH = 0.0f,  D_PITCH = 0.03f;
const float P_YAW   = 0.5f,  I_YAW   = 0.0f,  D_YAW   = 0.0f;

// Max PID correction in microseconds applied on top of stick input
const float PID_LIMIT = 300.0f;

// Max angles in stabilised mode
const float MAX_ROLL_ANGLE  = 45.0f;  // degrees
const float MAX_PITCH_ANGLE = 30.0f;  // degrees
const float PITCH_OFFSET    = 3.0f;   // degrees nose-up at neutral stick

// ============================================================
//  KALMAN FILTER
// ============================================================
void kalman_1d(float &state, float &unc, float rate_input, float angle_meas) {
    state += 0.004f * rate_input;
    unc   += 0.004f * 0.004f * 16.0f;
    float gain = unc / (unc + 9.0f);
    state += gain * (angle_meas - state);
    unc    = (1.0f - gain) * unc;
}

// ============================================================
//  PID - D term uses raw gyro rate to avoid noise from
//  differentiating the error signal (same approach as madflight)
// ============================================================
float pid_equation(float error, float gyro_rate,
                   float P, float I, float D,
                   PIDState &s) {
    float Pterm = P * error;
    float Iterm = s.prevIterm + I * (error + s.prevError) * 0.004f / 2.0f;
    Iterm = constrain(Iterm, -PID_LIMIT, PID_LIMIT);
    float Dterm = -D * gyro_rate; // negative: gyro opposes correction overshoot
    float out   = constrain(Pterm + Iterm + Dterm, -PID_LIMIT, PID_LIMIT);
    s.prevError = error;
    s.prevIterm = Iterm;
    return out;
}

void reset_pid() {
    pidRoll  = {0, 0};
    pidPitch = {0, 0};
    pidYaw   = {0, 0};
}

// ============================================================
//  SERVO WRITE HELPER
//  Applies direction reversal, trim, and clamp in one place
// ============================================================
void servoWrite(Servo &srv, int us, bool reversed, int trim) {
    if (reversed) us = 3000 - us;  // mirror around 1500
    srv.writeMicroseconds(constrain(us + trim, 988, 2012));
}

// ============================================================
//  IMU READ
// ============================================================
void gyro_signals() {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    RateRoll  = gyro.gyro.x * (180.0f / M_PI);
    RatePitch = gyro.gyro.y * (180.0f / M_PI);
    RateYaw   = gyro.gyro.z * (180.0f / M_PI);

    AccX = accel.acceleration.x / 9.81f;
    AccY = accel.acceleration.y / 9.81f;
    AccZ = accel.acceleration.z / 9.81f;

    AngleRoll  =  atan2f(AccY, sqrtf(AccX*AccX + AccZ*AccZ)) * (180.0f / M_PI);
    AnglePitch = -atan2f(AccX, sqrtf(AccY*AccY + AccZ*AccZ)) * (180.0f / M_PI);
}

// ============================================================
//  GYRO CALIBRATION
// ============================================================
void gyroscope_calibration() {
    Serial.println("Calibrating gyro - keep plane perfectly still...");
    RateCalRoll = RateCalPitch = RateCalYaw = 0;
    for (int n = 0; n < 2000; n++) {
        gyro_signals();
        RateCalRoll  += RateRoll;
        RateCalPitch += RatePitch;
        RateCalYaw   += RateYaw;
        delay(1);
    }
    RateCalRoll  /= 2000.0f;
    RateCalPitch /= 2000.0f;
    RateCalYaw   /= 2000.0f;
    Serial.printf("Cal offsets -> Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n",
                  RateCalRoll, RateCalPitch, RateCalYaw);
}

// ============================================================
//  CRSF RECEIVE
// ============================================================
void crsfloop() {
    while (crsfSerial.available()) {
        crsf.update((uint8_t)crsfSerial.read());
        // channel[] is updated internally whenever a valid frame arrives
    }

    if (!crsf.is_connected()) {
        crsfSignalLost = true;
        // Failsafe values
        for (int i = 0; i < 16; i++) rcIn[i] = 1500;
        rcIn[CH_THROTTLE] = 988;
        rcIn[CH_ARM]      = 988;
        reset_pid();
    } else {
        crsfSignalLost = false;
        for (int i = 0; i < 16; i++) rcIn[i] = crsf.channel[i];
    }

    // Arm: switch high AND throttle low
    if (!isArmed && rcIn[CH_ARM] > 1500 && rcIn[CH_THROTTLE] < 1050) {
        isArmed = true;
        Serial.println("ARMED");
    }
    if (isArmed && rcIn[CH_ARM] < 1500) {
        isArmed = false;
        Serial.println("DISARMED");
    }

    stabEnabled = (rcIn[CH_MODE] > 1500);
}

// ============================================================
//  BATTERY MONITOR TASK
// ============================================================
void batteryMonitorTask(void *pvParameters) {
    while (1) {
        int raw = analogRead(BATTERY_PIN);
        float v = (raw / 4095.0f) * REF_VOLTAGE;
        battery_voltage = v * ((R1 + R2) / R2);
        if (battery_voltage < 9.0f) {
            Serial.printf("WARNING: Battery low! %.2fV\n", battery_voltage);
            digitalWrite(LED_PIN, HIGH); vTaskDelay(200 / portTICK_PERIOD_MS);
            digitalWrite(LED_PIN, LOW);  vTaskDelay(200 / portTICK_PERIOD_MS);
        } else {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

// ============================================================
//  FLIGHT CONTROL TASK
// ============================================================
void flightControlTask(void *pvParameters) {
    LoopTimer = micros();
    while (1) {

        // 1. IMU
        gyro_signals();
        RateRoll  -= RateCalRoll;
        RatePitch -= RateCalPitch;
        RateYaw   -= RateCalYaw;

        // 2. Kalman filter
        kalman_1d(KalmanAngleRoll,  KalmanUncRoll,  RateRoll,  AngleRoll);
        kalman_1d(KalmanAnglePitch, KalmanUncPitch, RatePitch, AnglePitch);

        // 3. RC input
        crsfloop();

        // 4. Servo outputs start as direct stick passthrough
        int outThrottle = rcIn[CH_THROTTLE];
        int outAileron  = rcIn[CH_ROLL];
        int outElevator = rcIn[CH_PITCH];
        int outRudder   = rcIn[CH_YAW];

        // 5. Stabilisation
        if (stabEnabled) {
            // Scale stick to desired angle (clamped to limits)
            // 500us from centre = max angle
            float desiredRoll  = constrain(0.06f * (rcIn[CH_ROLL]  - 1500),
                                           -MAX_ROLL_ANGLE,  MAX_ROLL_ANGLE);
            float desiredPitch = constrain(0.06f * (rcIn[CH_PITCH] - 1500),
                                           -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE)
                                 + PITCH_OFFSET; // slight nose-up at neutral

            // Yaw: rate control (not angle) - rudder stays mostly manual
            float desiredYawRate = 0.15f * (rcIn[CH_YAW] - 1500);

            float errorRoll  = desiredRoll  - KalmanAngleRoll;
            float errorPitch = desiredPitch - KalmanAnglePitch;
            float errorYaw   = desiredYawRate - RateYaw;

            // D term fed raw gyro rate directly (madflight approach)
            float corrRoll  = pid_equation(errorRoll,  RateRoll,  P_ROLL,  I_ROLL,  D_ROLL,  pidRoll);
            float corrPitch = pid_equation(errorPitch, RatePitch, P_PITCH, I_PITCH, D_PITCH, pidPitch);
            float corrYaw   = pid_equation(errorYaw,   RateYaw,   P_YAW,   I_YAW,   D_YAW,   pidYaw);

            outAileron  = constrain(rcIn[CH_ROLL]  + (int)corrRoll,  988, 2012);
            outElevator = constrain(rcIn[CH_PITCH] + (int)corrPitch, 988, 2012);
            outRudder   = constrain(rcIn[CH_YAW]   + (int)corrYaw,   988, 2012);
        } else {
            reset_pid(); // prevent windup accumulating in manual mode
        }

        // 6. Safety cutoff
        if (!isArmed || crsfSignalLost) {
            outThrottle = 988;
            outAileron  = 1500;
            outElevator = 1500;
            outRudder   = 1500;
        }

        // 7. Write outputs
        // Throttle has no reversal option - if ESC spins wrong, swap motor wires
        escMotor.writeMicroseconds(constrain(outThrottle, 988, 2012));

        servoWrite(servoAilL, outAileron,  REVERSE_AILERON_L, trimAilL);
        servoWrite(servoAilR, outAileron,  REVERSE_AILERON_R, trimAilR);
        servoWrite(servoElev, outElevator, REVERSE_ELEVATOR,  trimElev);
        servoWrite(servoRudd, outRudder,   REVERSE_RUDDER,    trimRudd);

        // 8. Debug - comment out after bench testing to save CPU
        Serial.printf(
            "Roll:%5.1f° Pitch:%5.1f° | Thr:%4d Ail:%4d Elev:%4d Rudd:%4d | %s %s Batt:%.1fV\n",
            KalmanAngleRoll, KalmanAnglePitch,
            outThrottle, outAileron, outElevator, outRudder,
            stabEnabled ? "STAB" : "MANUAL",
            isArmed    ? "ARMED" : "DISARMED",
            battery_voltage
        );

        // 9. Hold 250Hz (4ms loop)
        while ((micros() - LoopTimer) < 4000);
        LoopTimer = micros();
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    analogReadResolution(12);
    pinMode(BATTERY_PIN, INPUT);

    // --- ICM-20948 ---
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    if (!icm.begin_I2C()) {
        Serial.println("ERROR: ICM-20948 not found! Check wiring and I2C address.");
        // Fast blink = hardware error
        while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
    }
    icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
    icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
    icm.setGyroRateDivisor(9);
    Serial.println("ICM-20948 OK");

    // --- CRSF / ELRS ---
    // No init function needed - CRSFParser constructor handles everything
    crsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    Serial.println("CRSF UART started at 420000 baud");

    // --- Servos and ESC ---
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    escMotor.setPeriodHertz(50);
    servoAilL.setPeriodHertz(50);
    servoAilR.setPeriodHertz(50);
    servoElev.setPeriodHertz(50);
    servoRudd.setPeriodHertz(50);

    escMotor.attach (PIN_ESC,       988, 2012);
    servoAilL.attach(PIN_AILERON_L, 988, 2012);
    servoAilR.attach(PIN_AILERON_R, 988, 2012);
    servoElev.attach(PIN_ELEVATOR,  988, 2012);
    servoRudd.attach(PIN_RUDDER,    988, 2012);

    // Safe starting position
    escMotor.writeMicroseconds(988);
    servoAilL.writeMicroseconds(1500);
    servoAilR.writeMicroseconds(1500);
    servoElev.writeMicroseconds(1500);
    servoRudd.writeMicroseconds(1500);
    Serial.println("Servos centred");

    // --- Gyro calibration ---
    gyroscope_calibration();

    Serial.println("Ready. Arm: flip CH5 switch with throttle low.");
    Serial.println("Mode: CH6 low=MANUAL, CH6 high=STABILISED");

    // --- Start RTOS tasks ---
    xTaskCreate(batteryMonitorTask, "Battery",       4096, NULL, 1, NULL);
    xTaskCreate(flightControlTask,  "FlightControl", 8192, NULL, 2, NULL);
}

void loop() {
    // Empty - all work is done in RTOS tasks
}
