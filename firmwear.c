/**
 * @file firmwear.c / main.cpp
 * @brief MPU-6050 Self-Test & Diagnostic for Direct Socket Header Connection
 * @target ESP32 (PlatformIO / Arduino)
 * 
 * Direct Header Pin Mapping:
 *   ESP32 VIN / 5V -> MPU VCC (Onboard 3.3V LDO powered from 5V rail)
 *   ESP32 GND      -> MPU GND
 *   ESP32 GPIO 13  -> MPU SCL (I2C Clock)
 *   ESP32 GPIO 12  -> MPU SDA (I2C Data)
 *   ESP32 GPIO 14  -> MPU XDA (High-Z Input / Floating)
 *   ESP32 GPIO 27  -> MPU XCL (High-Z Input / Floating)
 *   ESP32 GPIO 26  -> MPU AD0 (Driven LOW to lock address to 0x68)
 *   ESP32 GPIO 25  -> MPU INT (Input)
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Pin Definitions */
#define PIN_SDA              12  /* D12 */
#define PIN_SCL              13  /* D13 */
#define PIN_XDA              14  /* D14 (MPU Aux I2C Data - keep High-Z) */
#define PIN_XCL              27  /* D27 (MPU Aux I2C Clock - keep High-Z) */
#define PIN_AD0              26  /* D26 (Driven LOW in software for addr 0x68) */
#define PIN_INT              25  /* D25 (Data-Ready / Motion Interrupt) */

/* MPU Registers */
#define REG_SELF_TEST_X      0x0D
#define REG_SELF_TEST_Y      0x0E
#define REG_SELF_TEST_Z      0x0F
#define REG_SELF_TEST_A      0x10
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_XOUT_H     0x3B
#define REG_TEMP_OUT_H       0x41
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75

#define NUM_SAMPLES          200

/* State Machine */
enum SensorState {
    STATE_DISCONNECTED,
    STATE_DEBOUNCING,
    STATE_RUN_SELFTEST,
    STATE_LIVE_STREAM
};

static SensorState current_state = STATE_DISCONNECTED;
static uint8_t active_addr = 0x68;
static uint32_t last_poll_ms = 0;
static uint32_t last_stream_ms = 0;
static uint8_t debounce_hits = 0;

/* Diagnostic Result Container */
typedef struct {
    float accel_str[3];
    float gyro_str[3];
    float accel_ft[3];
    float gyro_ft[3];
    float accel_diff_pct[3];
    float gyro_diff_pct[3];
    bool  accel_pass[3];
    bool  gyro_pass[3];
    float die_temp_c;
    bool  temp_pass;
    bool  overall_pass;
} mpu6050_test_report_t;

/* --- Robust I2C Communication Layer --- */

static void reset_i2c_bus() {
    Wire.end();
    delay(10);
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);  /* 100 kHz for high stability */
    Wire.setTimeOut(30);    /* 30ms timeout prevents hangs */
}

static bool mpu_write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

static bool mpu_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    
    uint8_t count = Wire.requestFrom((int)addr, (int)length);
    if (count != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

/* Quick probe to check chip acknowledgement */
static bool probe_mpu(uint8_t *found_addr) {
    uint8_t candidates[2] = {0x68, 0x69};
    uint8_t whoami = 0;

    for (uint8_t i = 0; i < 2; i++) {
        uint8_t a = candidates[i];
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            if (mpu_read_bytes(a, REG_WHO_AM_I, &whoami, 1)) {
                if (whoami == 0x68 || whoami == 0x70 || whoami == 0x72) {
                    *found_addr = a;
                    return true;
                }
            }
        }
    }
    return false;
}

/* Sensor averaging for self-test */
static bool collect_sensor_averages(uint8_t addr, float accel_avg[3], float gyro_avg[3]) {
    int32_t a_sum[3] = {0, 0, 0};
    int32_t g_sum[3] = {0, 0, 0};
    uint8_t buf[14];

    for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
        if (!mpu_read_bytes(addr, REG_ACCEL_XOUT_H, buf, 14)) {
            return false;
        }

        a_sum[0] += (int16_t)((buf[0] << 8) | buf[1]);
        a_sum[1] += (int16_t)((buf[2] << 8) | buf[3]);
        a_sum[2] += (int16_t)((buf[4] << 8) | buf[5]);

        g_sum[0] += (int16_t)((buf[8] << 8) | buf[9]);
        g_sum[1] += (int16_t)((buf[10] << 8) | buf[11]);
        g_sum[2] += (int16_t)((buf[12] << 8) | buf[13]);

        delay(2);
    }

    for (uint8_t i = 0; i < 3; i++) {
        accel_avg[i] = (float)a_sum[i] / (float)NUM_SAMPLES;
        gyro_avg[i]  = (float)g_sum[i]  / (float)NUM_SAMPLES;
    }
    return true;
}

/* Full self-test routine */
static bool run_mpu6050_selftest(uint8_t addr, mpu6050_test_report_t *report) {
    if (!report) return false;
    memset(report, 0, sizeof(mpu6050_test_report_t));

    /* 1. Wake device */
    if (!mpu_write_byte(addr, REG_PWR_MGMT_1, 0x01)) return false;
    delay(50);

    /* 2. Configure baseline: Gyro +/-250dps (0x00), Accel +/-8g (0x10), DLPF ~44Hz */
    mpu_write_byte(addr, REG_SMPLRT_DIV, 0x00);
    mpu_write_byte(addr, REG_CONFIG, 0x03);
    mpu_write_byte(addr, REG_GYRO_CONFIG, 0x00);
    mpu_write_byte(addr, REG_ACCEL_CONFIG, 0x10);
    delay(30);

    /* 3. Read baseline */
    float norm_a[3] = {0}, norm_g[3] = {0};
    if (!collect_sensor_averages(addr, norm_a, norm_g)) return false;

    /* 4. Actuate self-test bits */
    mpu_write_byte(addr, REG_GYRO_CONFIG, 0xE0);
    mpu_write_byte(addr, REG_ACCEL_CONFIG, 0xF0);
    delay(40);

    /* 5. Read actuated response */
    float st_a[3] = {0}, st_g[3] = {0};
    if (!collect_sensor_averages(addr, st_a, st_g)) return false;

    /* 6. Compute STR */
    for (uint8_t i = 0; i < 3; i++) {
        report->accel_str[i] = st_a[i] - norm_a[i];
        report->gyro_str[i]  = st_g[i]  - norm_g[i];
    }

    /* 7. Read factory trim registers */
    uint8_t trim_raw[4] = {0};
    if (!mpu_read_bytes(addr, REG_SELF_TEST_X, trim_raw, 4)) return false;

    uint8_t xg_test = trim_raw[0] & 0x1F;
    uint8_t yg_test = trim_raw[1] & 0x1F;
    uint8_t zg_test = trim_raw[2] & 0x1F;
    uint8_t xa_test = ((trim_raw[0] >> 3) & 0x1C) | ((trim_raw[3] >> 4) & 0x03);
    uint8_t ya_test = ((trim_raw[1] >> 3) & 0x1C) | ((trim_raw[3] >> 2) & 0x03);
    uint8_t za_test = ((trim_raw[2] >> 3) & 0x1C) | (trim_raw[3] & 0x03);

    report->gyro_ft[0] = (xg_test != 0) ? ( 25.0f * 131.0f * powf(1.046f, (float)(xg_test - 1))) : 0.0f;
    report->gyro_ft[1] = (yg_test != 0) ? (-25.0f * 131.0f * powf(1.046f, (float)(yg_test - 1))) : 0.0f;
    report->gyro_ft[2] = (zg_test != 0) ? ( 25.0f * 131.0f * powf(1.046f, (float)(zg_test - 1))) : 0.0f;

    const float a_mult = 4096.0f * 0.34f;
    const float a_base = 0.92f / 0.34f;
    report->accel_ft[0] = (xa_test != 0) ? (a_mult * powf(a_base, ((float)xa_test - 1.0f) / 30.0f)) : 0.0f;
    report->accel_ft[1] = (ya_test != 0) ? (a_mult * powf(a_base, ((float)ya_test - 1.0f) / 30.0f)) : 0.0f;
    report->accel_ft[2] = (za_test != 0) ? (a_mult * powf(a_base, ((float)za_test - 1.0f) / 30.0f)) : 0.0f;

    /* 8. Restore normal ranges */
    mpu_write_byte(addr, REG_GYRO_CONFIG, 0x00);
    mpu_write_byte(addr, REG_ACCEL_CONFIG, 0x00);

    /* 9. Evaluate Pass/Fail (+/- 14%) */
    report->overall_pass = true;
    for (uint8_t i = 0; i < 3; i++) {
        if (report->accel_ft[i] != 0.0f) {
            report->accel_diff_pct[i] = ((report->accel_str[i] - report->accel_ft[i]) / report->accel_ft[i]) * 100.0f;
            report->accel_pass[i] = (fabsf(report->accel_diff_pct[i]) <= 14.0f);
        } else {
            report->accel_diff_pct[i] = 0.0f;
            report->accel_pass[i] = (fabsf(report->accel_str[i]) >= 1200.0f && fabsf(report->accel_str[i]) <= 3900.0f);
        }
        if (!report->accel_pass[i]) report->overall_pass = false;

        if (report->gyro_ft[i] != 0.0f) {
            report->gyro_diff_pct[i] = ((report->gyro_str[i] - report->gyro_ft[i]) / report->gyro_ft[i]) * 100.0f;
            report->gyro_pass[i] = (fabsf(report->gyro_diff_pct[i]) <= 14.0f);
        } else {
            report->gyro_diff_pct[i] = 0.0f;
            report->gyro_pass[i] = (fabsf(report->gyro_str[i]) >= 1310.0f && fabsf(report->gyro_str[i]) <= 13755.0f);
        }
        if (!report->gyro_pass[i]) report->overall_pass = false;
    }

    /* 10. Temperature check */
    uint8_t t_buf[2];
    if (mpu_read_bytes(addr, REG_TEMP_OUT_H, t_buf, 2)) {
        int16_t raw_temp = (int16_t)((t_buf[0] << 8) | t_buf[1]);
        report->die_temp_c = ((float)raw_temp / 340.0f) + 36.53f;
        report->temp_pass = (report->die_temp_c >= 15.0f && report->die_temp_c <= 50.0f);
    } else {
        report->temp_pass = false;
    }
    if (!report->temp_pass) report->overall_pass = false;

    return true;
}

static void print_report(const mpu6050_test_report_t *r) {
    const char *axes[3] = {"X", "Y", "Z"};
    Serial.println("\n=======================================================");
    Serial.println("         MPU-6050 SELF-TEST & DIAGNOSTIC REPORT        ");
    Serial.println("=======================================================");
    
    Serial.println("[ACCELEROMETER]");
    for (uint8_t i = 0; i < 3; i++) {
        Serial.printf("  Accel %s: STR=%+7.1f | FT=%+7.1f | Dev=%+6.2f%% -> [%s]\n",
                      axes[i], r->accel_str[i], r->accel_ft[i], r->accel_diff_pct[i],
                      r->accel_pass[i] ? "PASS" : "FAIL");
    }

    Serial.println("\n[GYROSCOPE]");
    for (uint8_t i = 0; i < 3; i++) {
        Serial.printf("  Gyro  %s: STR=%+7.1f | FT=%+7.1f | Dev=%+6.2f%% -> [%s]\n",
                      axes[i], r->gyro_str[i], r->gyro_ft[i], r->gyro_diff_pct[i],
                      r->gyro_pass[i] ? "PASS" : "FAIL");
    }

    Serial.println("\n[TEMPERATURE]");
    Serial.printf("  Die Temp: %.2f degC -> [%s]\n", r->die_temp_c, r->temp_pass ? "PASS" : "FAIL");
    Serial.println("-------------------------------------------------------");
    Serial.printf("OVERALL DIAGNOSTIC: [%s]\n", r->overall_pass ? "PASSED" : "FAILED / OUT OF TOLERANCE");
    Serial.println("=======================================================\n");
    Serial.println("[STREAM] Switching to LIVE 10 Hz motion stream...");
    Serial.println("Tilt the ESP32 board to observe real-time telemetry:\n");
}

/* --- Setup & Loop --- */

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 1500);

    Serial.println("\n=================================================");
    Serial.println("   MPU-6050 DIRECT-HEADER PIN DIAGNOSTIC         ");
    Serial.println("=================================================");
    Serial.println("Pinout configured:");
    Serial.println("  SDA -> GPIO 12");
    Serial.println("  SCL -> GPIO 13");
    Serial.println("  AD0 -> GPIO 26 (driving LOW -> 0x68)");
    Serial.println("  XDA -> GPIO 14 (High-Z)");
    Serial.println("  XCL -> GPIO 27 (High-Z)");
    Serial.println("  INT -> GPIO 25 (Input)");

    /* 1. Drive AD0 LOW to firmly lock I2C address to 0x68 */
    pinMode(PIN_AD0, OUTPUT);
    digitalWrite(PIN_AD0, LOW);

    /* 2. Configure MPU Aux pins as high-impedance inputs so they don't load the bus */
    pinMode(PIN_XDA, INPUT);
    pinMode(PIN_XCL, INPUT);
    pinMode(PIN_INT, INPUT);

    /* 3. Initialize I2C on SDA=12, SCL=13 */
    reset_i2c_bus();
}

void loop() {
    uint32_t now = millis();

    /* On-demand trigger: Type 't' in Serial Monitor to run Self-Test without pressing EN! */
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 't' || c == 'T') {
            Serial.println("\n\n>>> [TRIGGER] Manual Self-Test Requested! <<<");
            Serial.println("Keep the sensor completely stationary on your desk...");
            delay(1000);
            mpu6050_test_report_t report;
            if (run_mpu6050_selftest(active_addr, &report)) {
                print_report(&report);
            } else {
                Serial.println("[ERROR] I2C communication failed during self-test.");
            }
            return;
        }
    }

    switch (current_state) {
        case STATE_DISCONNECTED: {
            if (now - last_poll_ms >= 150) {
                last_poll_ms = now;
                uint8_t found = 0;
                if (probe_mpu(&found)) {
                    active_addr = found;
                    debounce_hits = 1;
                    current_state = STATE_DEBOUNCING;
                    Serial.printf("\n[CONTACT DETECTED] MPU recognized at 0x%02X! Debouncing...", active_addr);
                } else {
                    Serial.print(".");
                }
            }
            break;
        }

        case STATE_DEBOUNCING: {
            if (now - last_poll_ms >= 50) {
                last_poll_ms = now;
                uint8_t dummy = 0;
                if (probe_mpu(&dummy)) {
                    debounce_hits++;
                    if (debounce_hits >= 3) {
                        Serial.println(" Solid!");
                        Serial.println("[ACTION] Keep ESP32 still! Running self-test in 1 second...");
                        delay(1000);
                        current_state = STATE_RUN_SELFTEST;
                    }
                } else {
                    Serial.println(" [FLICKER] Contact lost during debounce. Re-scanning...");
                    reset_i2c_bus();
                    current_state = STATE_DISCONNECTED;
                }
            }
            break;
        }

        case STATE_RUN_SELFTEST: {
            mpu6050_test_report_t report;
            if (run_mpu6050_selftest(active_addr, &report)) {
                print_report(&report);
                current_state = STATE_LIVE_STREAM;
            } else {
                Serial.println("\n[ERROR] I2C bus dropped out during self-test! Re-checking pins...");
                reset_i2c_bus();
                current_state = STATE_DISCONNECTED;
            }
            break;
        }

        case STATE_LIVE_STREAM: {
            if (now - last_stream_ms >= 100) { /* 10 Hz refresh */
                last_stream_ms = now;
                uint8_t buf[14];
                if (mpu_read_bytes(active_addr, REG_ACCEL_XOUT_H, buf, 14)) {
                    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
                    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
                    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
                    int16_t raw_temp = (int16_t)((buf[6] << 8) | buf[7]);
                    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
                    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
                    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

                    float a_x = ax / 16384.0f;
                    float a_y = ay / 16384.0f;
                    float a_z = az / 16384.0f;
                    float g_x = gx / 131.0f;
                    float g_y = gy / 131.0f;
                    float g_z = gz / 131.0f;

                    /* Arduino Serial Plotter universal format: Label:Value separated by spaces */
                    Serial.printf("Accel_X:%.2f Accel_Y:%.2f Accel_Z:%.2f Gyro_X:%.2f Gyro_Y:%.2f Gyro_Z:%.2f\r\n",
                                  a_x, a_y, a_z, g_x, g_y, g_z);
                } else {
                    Serial.println("\n\n[WARNING] Contact lost! Re-scanning bus...\n");
                    reset_i2c_bus();
                    current_state = STATE_DISCONNECTED;
                }
            }
            break;
        }
    }
}
