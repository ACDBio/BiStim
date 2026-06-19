#include <SPI.h>
#include <math.h>
#include <WiFi.h>

// ===== Пины =====
#define PIN_CS 25
#define PIN_START 22
#define MOSI_PIN 21
#define SCK_PIN 27
#define PIN_DRDY 17
#define MISO_PIN 32
#define PIN_RESET 4

// --- DRV8871 + INA (код стимуляции изменён) ---
#define PIN_DRV_IN1 18      // IN1 DRV8871
#define PIN_DRV_IN2 19      // IN2 DRV8871
#define PIN_ISENSE 33       // Выход INA с шунта

#define PWM_FREQ 20000
#define PWM_RES 8
// ---------------------------------------------------------------

// ===== Команды =====
#define CMD_RESET 0x06
#define CMD_START1 0x08
#define CMD_STOP1 0x0A
#define CMD_RDATA1 0x12
#define CMD_RREG 0x20
#define CMD_WREG 0x40
// ===== Регистры =====
#define REG_ID 0x00
#define REG_POWER 0x01
#define REG_INTERFACE 0x02
#define REG_MODE0 0x03
#define REG_MODE1 0x04
#define REG_MODE2 0x05
#define REG_INPMUX 0x06
#define REG_REFMUX 0x0F
// ===== PGA =====
#define PGA_1 (0 << 4)
#define PGA_2 (1 << 4)
#define PGA_4 (2 << 4)
#define PGA_8 (3 << 4)
#define PGA_16 (4 << 4)
#define PGA_32 (5 << 4)
#define PGA_BYPASS 0x80
// ===== Data rate =====
#define DR_2_5 0x00
#define DR_5 0x01
#define DR_10 0x02
#define DR_16_6 0x03
#define DR_20 0x04
#define DR_50 0x05
#define DR_60 0x06
#define DR_100 0x07
#define DR_400 0x08
#define DR_1200 0x09
#define DR_2400 0x0A
#define DR_4800 0x0B
#define DR_7200 0x0C
#define DR_14400 0x0D
#define DR_19200 0x0E
#define DR_38400 0x0F
// ===== Filters =====
#define FILTER_SINC1 (0 << 5)
#define FILTER_FIR (4 << 5)
#define FILTER_SINC2 (1 << 5)
#define FILTER_SINC3 (2 << 5)
#define FILTER_SINC4 (3 << 5)
// ===== INPMUX =====
#define MUX(p,n) (((p) << 4) | (n))
#define INPMUX_AIN2_AIN3 MUX(0x2, 0x3)
#define INPMUX_AIN0_AIN3 MUX(0x0, 0x3)
// ===== Тест / Стимуляция =====
#define TEST_ENABLED 1

// ===== ЗАЩИТЫ =====
#define HARD_CURRENT_LIMIT_MA       3.0f   // аварийная отсечка по току
#define NO_CURRENT_THRESHOLD_MA     0.05f  // ниже этого считаем "тока нет"
#define NO_CURRENT_TIMEOUT_MS       50     // через сколько мс реагировать на обрыв
#define MAX_DUTY_WHEN_NO_CURRENT    100    // потолок duty при подозрении на обрыв
#define FAULT_COOLDOWN_MS           500    // пауза после срабатывания аварии

// --- ПАРАМЕТРЫ СТИМУЛЯЦИИ (ЗАДАЧА ФОРМЫ, ЧАСТОТЫ И АМПЛИТУДЫ) ---
static float g_stim_freq_hz = 10.0f;          // Частота волны, Гц
static float g_target_amplitude_ua = 1000.0f; // Желаемая амплитуда тока в МИКРОамперах (1000 мкА = 1 мА)
enum StimWaveForm { WAVE_SINE, WAVE_SQUARE, WAVE_TRIANGLE };
static StimWaveForm g_stim_form = WAVE_SINE; // Форма волны

// --- НАСТРОЙКИ ТОКА ---
#define ISENSE_SHUNT_OHMS   10.0f     // Твой шунт, Ом
#define ISENSE_INA_GAIN     20.0f     // INA240 gain
#define INA_REF_MV          1650.0f    // Твой реф ИНЫ на середине шины 3.3 В

// Параметры преобразования мощности повышайки
#define V_BAT               4.8f      // Батарея, В
#define V_OUT               30.0f     // Повышайка, В
#define DCDC_EFFICIENCY     0.85f     // Расчетный КПД (85%)

#define STIM_MAX_DUTY       245
#define ISENSE_CAL_SAMPLES  64

#define STIM_KP 0.45f
#define STIM_KI 0.04f
#define STIM_INTEGRAL_MAX 250.0f
#define STIM_INTEGRAL_MIN -250.0f
#define DEADZONE_MA 0.02f

float g_pga_gain = 32.0f;
SPIClass spi(HSPI);
uint32_t g_test_start_ms = 0;
uint8_t g_test_mode = 255;

// Переменные логов и контроля
volatile float g_measured_current_mA = 0.0f;
static float g_static_current_loss_mA = 0.0f; // Ток холостого хода DCDC от батареи

// Переменные для сглаженного Serial вывода
volatile float log_target_mA = 0.0f;
volatile float log_measured_mA = 0.0f;
volatile float log_shunt_mv = 0.0f;
volatile int log_duty = 0;


// ===== Калибровка хранится в МИЛЬТИВОЛЬТАХ (явно) =====
static float g_isense_zero_mv = 1650.0f;  // напряжение покоя INA, мВ

// ===== Усреднённое чтение ISENSE (антиалиасинг PWM) =====
#define ISENSE_OVERSAMPLE 16
float readIsenseAvg_mV() {
    uint32_t sum = 0;
    for (int i = 0; i < ISENSE_OVERSAMPLE; i++) {
        sum += analogReadMilliVolts(PIN_ISENSE);
    }
    return (float)sum / ISENSE_OVERSAMPLE;
}

#define CURRENT_CALIBRATION 2.0f  // эмпирическая поправка измерения

float calculateStimCurrent_mA(float ina_mv) {
    float delta_mv = fabsf(ina_mv - g_isense_zero_mv);
    if (delta_mv < 1.0f) delta_mv = 0.0f;
    
    float i_bat_mA = delta_mv / (ISENSE_SHUNT_OHMS * ISENSE_INA_GAIN);
    float i_stim_mA = i_bat_mA * (V_BAT / V_OUT) * DCDC_EFFICIENCY * CURRENT_CALIBRATION;
    
    return i_stim_mA;
}

// ===== Калибровка =====
void calibrateIsense() {
    ledcWrite(PIN_DRV_IN1, 0);
    ledcWrite(PIN_DRV_IN2, 0);
    delay(500);  // дать DCDC уйти в холостой ход

    // Длинное усреднение для точного нуля
    double sum = 0;
    const int N = 512;
    for (int i = 0; i < N; i++) {
        sum += analogReadMilliVolts(PIN_ISENSE);
        delay(2);
    }
    g_isense_zero_mv = (float)(sum / N);
    Serial.printf(">>> ISENSE zero = %.2f mV\n", g_isense_zero_mv);
}


// =========================================================
void writeReg(uint8_t reg, uint8_t val) {
    spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(5);
    spi.transfer(CMD_WREG | reg);
    spi.transfer(0x00);
    spi.transfer(val);
    delayMicroseconds(5);
    digitalWrite(PIN_CS, HIGH);
    spi.endTransaction();
    delay(2);
}
uint8_t readReg(uint8_t reg) {
    spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(5);
    spi.transfer(CMD_RREG | reg);
    spi.transfer(0x00);
    uint8_t v = spi.transfer(0x00);
    delayMicroseconds(5);
    digitalWrite(PIN_CS, HIGH);
    spi.endTransaction();
    return v;
}
void sendCmd(uint8_t cmd) {
    spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(5);
    spi.transfer(cmd);
    delayMicroseconds(5);
    digitalWrite(PIN_CS, HIGH);
    spi.endTransaction();
}
int32_t readData(uint8_t *statusOut) {
    spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(5);
    spi.transfer(CMD_RDATA1);
    uint8_t st = spi.transfer(0x00);
    uint8_t b3 = spi.transfer(0x00);
    uint8_t b2 = spi.transfer(0x00);
    uint8_t b1 = spi.transfer(0x00);
    uint8_t b0 = spi.transfer(0x00);
    digitalWrite(PIN_CS, HIGH);
    spi.endTransaction();
    if (statusOut) *statusOut = st;
    return ((int32_t)b3 << 24) | ((int32_t)b2 << 16) |
           ((int32_t)b1 << 8) | (int32_t)b0;
}

void configADC(uint8_t pga, uint8_t dr, uint8_t filter, uint8_t inpmux) {
    writeReg(REG_POWER, 0x03);
    writeReg(REG_INTERFACE, 0x00);
    writeReg(REG_MODE0, 0x10);
    writeReg(REG_MODE1, filter);
    writeReg(REG_MODE2, pga | dr);
    writeReg(REG_INPMUX, inpmux);
    writeReg(REG_REFMUX, 0x00);
    switch (pga) {
        case PGA_1: g_pga_gain = 1; break;
        case PGA_2: g_pga_gain = 2; break;
        case PGA_4: g_pga_gain = 4; break;
        case PGA_8: g_pga_gain = 8; break;
        case PGA_16: g_pga_gain = 16; break;
        case PGA_32: g_pga_gain = 32; break;
    }
    sendCmd(CMD_START1);
    Serial.println("=== REGISTERS ===");
    Serial.printf("MODE0=0x%02X MODE1=0x%02X MODE2=0x%02X INPMUX=0x%02X REFMUX=0x%02X\n",
        readReg(0x03), readReg(0x04), readReg(0x05), readReg(0x06), readReg(0x0F));
}

// ===== Код стимуляции: closed-loop, амплитуда, вывод тока =====
static float g_stim_phase = 0.0f;
static float g_integral = 0.0f;
// ===== Состояние защит =====
static uint32_t g_low_current_count = 0;     // счётчик "нет тока" (в шагах по 1 мс)
static uint32_t g_fault_until_ms = 0;        // до какого времени держим аварию
static bool     g_open_load_suspected = false; // подозрение на обрыв
static float    g_prev_target_mA = 0.0f;     // для детекта смены полярности

int readIsenseAveraged() {
    return analogReadMilliVolts(PIN_ISENSE);
}


// ===== Параметры feedforward =====
#define LOAD_R_OHMS_NOMINAL 10000.0f  // ожидаемая нагрузка (электроды + кожа)
// duty_FF = (I_target_mA * R / 1000) / V_out * 255
// при I=1мА, R=10к, V=30: duty = (10/30)*255 = 85

#define STIM_KP 8.0f      // увеличиваем агрессивность коррекции
#define STIM_KI 0.5f      // и интеграл тоже
#define STIM_INTEGRAL_MAX 80.0f
#define STIM_INTEGRAL_MIN -80.0f

void updateTestSignal() {
#if TEST_ENABLED
    // 1) фаза
    float delta_phase = 2.0f * M_PI * g_stim_freq_hz * 0.001f;
    g_stim_phase += delta_phase;
    if (g_stim_phase > (2.0f * M_PI)) g_stim_phase -= (2.0f * M_PI);

    // 2) форма
    float wave = 0.0f;
    if (g_stim_form == WAVE_SINE) {
        wave = sinf(g_stim_phase);
    } else if (g_stim_form == WAVE_SQUARE) {
        wave = (g_stim_phase < M_PI) ? 1.0f : -1.0f;
    } else if (g_stim_form == WAVE_TRIANGLE) {
        wave = (g_stim_phase < M_PI)
               ? (2.0f * g_stim_phase / M_PI - 1.0f)
               : (3.0f - 2.0f * g_stim_phase / M_PI);
    }

    // 3) целевой ток (мА)
    float target_mA = wave * (g_target_amplitude_ua / 1000.0f);

    // 3.1) СБРОС ИНТЕГРАЛА ПРИ СМЕНЕ ПОЛЯРНОСТИ
    if ((g_prev_target_mA > 0 && target_mA < 0) ||
        (g_prev_target_mA < 0 && target_mA > 0)) {
        g_integral = 0.0f;
    }
    g_prev_target_mA = target_mA;

    // 4) Мёртвая зона
    if (fabsf(target_mA) < DEADZONE_MA) {
        ledcWrite(PIN_DRV_IN1, 0);
        ledcWrite(PIN_DRV_IN2, 0);
        g_integral = 0.0f;
        g_low_current_count = 0;
        log_target_mA = target_mA;
        log_measured_mA = 0.0f;
        log_shunt_mv = readIsenseAvg_mV();
        log_duty = 0;
        return;
    }

    // 5) измерение
    float raw_ina_mv = readIsenseAvg_mV();
    float abs_measured_mA = calculateStimCurrent_mA(raw_ina_mv);

    // 5.1) HARD CURRENT LIMIT — аварийная отсечка
    if (abs_measured_mA > HARD_CURRENT_LIMIT_MA) {
        ledcWrite(PIN_DRV_IN1, 0);
        ledcWrite(PIN_DRV_IN2, 0);
        g_integral = 0.0f;
        g_fault_until_ms = millis() + FAULT_COOLDOWN_MS;
        Serial.printf("!!! OVERCURRENT %.2f mA — fault\n", abs_measured_mA);
        log_target_mA = target_mA;
        log_measured_mA = abs_measured_mA;
        log_shunt_mv = raw_ina_mv;
        log_duty = 0;
        return;
    }

    // 5.2) Если в авварии — молчим до конца cooldown
    if (millis() < g_fault_until_ms) {
        ledcWrite(PIN_DRV_IN1, 0);
        ledcWrite(PIN_DRV_IN2, 0);
        g_integral = 0.0f;
        log_target_mA = target_mA;
        log_measured_mA = abs_measured_mA;
        log_shunt_mv = raw_ina_mv;
        log_duty = 0;
        return;
    }

    // 6) FEEDFORWARD
    float abs_target_mA = fabsf(target_mA);
    float u_load_v = (abs_target_mA / 1000.0f) * LOAD_R_OHMS_NOMINAL;
    float duty_ff = (u_load_v / V_OUT) * 255.0f;

    // 7) PI как коррекция
    float error = abs_target_mA - abs_measured_mA;
    g_integral += error * STIM_KI;
    if (g_integral > STIM_INTEGRAL_MAX) g_integral = STIM_INTEGRAL_MAX;
    if (g_integral < STIM_INTEGRAL_MIN) g_integral = STIM_INTEGRAL_MIN;

    float duty_correction = error * STIM_KP + g_integral;

    // 8) итоговый duty
    int duty = (int)(duty_ff + duty_correction);
    if (duty < 0) duty = 0;
    if (duty > STIM_MAX_DUTY) duty = STIM_MAX_DUTY;

    // 8.1) ЗАЩИТА ОТ ОБРЫВА НАГРУЗКИ
    // если задание есть, а тока нет — копим счётчик
    if (abs_measured_mA < NO_CURRENT_THRESHOLD_MA && abs_target_mA > 0.1f) {
        g_low_current_count++;
        if (g_low_current_count > NO_CURRENT_TIMEOUT_MS) {
            // подозрение на обрыв: режем duty, сбрасываем интеграл
            g_open_load_suspected = true;
            if (duty > MAX_DUTY_WHEN_NO_CURRENT) duty = MAX_DUTY_WHEN_NO_CURRENT;
            g_integral = 0.0f;
        }
    } else {
        if (g_open_load_suspected && abs_measured_mA > NO_CURRENT_THRESHOLD_MA) {
            Serial.println("# Load reconnected");
        }
        g_low_current_count = 0;
        g_open_load_suspected = false;
    }

    // 9) направление
    if (target_mA >= 0.0f) {
        ledcWrite(PIN_DRV_IN2, 0);
        ledcWrite(PIN_DRV_IN1, duty);
    } else {
        ledcWrite(PIN_DRV_IN1, 0);
        ledcWrite(PIN_DRV_IN2, duty);
    }

    float signed_measured_mA = (target_mA >= 0.0f) ? abs_measured_mA : -abs_measured_mA;
    g_measured_current_mA = signed_measured_mA;

    log_target_mA = target_mA;
    log_measured_mA = signed_measured_mA;
    log_shunt_mv = raw_ina_mv;
    log_duty = duty;
#endif
}
// ==================================================================

void setup() {
    analogReadResolution(12);

    WiFi.mode(WIFI_OFF);
    btStop();
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== ADS1263 + DRV8871 CLOSED-LOOP tACS/tVNS ===");

    analogReadResolution(12);

    pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_START, OUTPUT); digitalWrite(PIN_START, LOW);
    pinMode(PIN_RESET, OUTPUT); 
    digitalWrite(PIN_RESET, HIGH);
    pinMode(PIN_DRDY, INPUT);

    spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, PIN_CS);

    digitalWrite(PIN_RESET, LOW); 
    delay(10);
    digitalWrite(PIN_RESET, HIGH); 
    delay(100);
    sendCmd(CMD_RESET);
    delay(50);

    uint8_t id = readReg(REG_ID);
    Serial.printf("ID = 0x%02X\n", id);

    configADC(PGA_16, DR_20, FILTER_FIR, INPMUX_AIN0_AIN3);
    delay(20);
    Serial.printf("MODE1=0x%02X MODE2=0x%02X INPMUX=0x%02X\n", readReg(REG_MODE1), readReg(REG_MODE2), readReg(REG_INPMUX));
    delay(100);
    digitalWrite(PIN_START, HIGH);
    delay(50);

#if TEST_ENABLED
    pinMode(PIN_DRV_IN1, OUTPUT);
    pinMode(PIN_DRV_IN2, OUTPUT);
    pinMode(PIN_ISENSE, INPUT);
    analogSetPinAttenuation(PIN_ISENSE, ADC_11db); // диапазон до 3.3V под милливольты

    ledcAttach(PIN_DRV_IN1, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_DRV_IN2, PWM_FREQ, PWM_RES);
    ledcWrite(PIN_DRV_IN1, 0);
    ledcWrite(PIN_DRV_IN2, 0);

    calibrateIsense();

    g_test_start_ms = millis();
    Serial.println(">>> DRV8871 CLOSED-LOOP ready");
#endif

    Serial.println("=== Стрим ===");
}

// ===== Оценка шума =====
#define NOISE_WINDOW_MS 2000
#define NOISE_BUF_SIZE 4096
static int32_t noise_buf[NOISE_BUF_SIZE];
static uint16_t noise_idx = 0;
static uint32_t noise_t_start = 0;

void compute_and_print_noise() {
    if (noise_idx < 16) { 
        noise_idx = 0; 
        noise_t_start = millis(); 
        return; 
    }
    double sum = 0;
    int32_t mn = INT32_MAX, mx = INT32_MIN;
    for (uint16_t i = 0; i < noise_idx; i++) {
        sum += noise_buf[i];
        if (noise_buf[i] < mn) mn = noise_buf[i];
        if (noise_buf[i] > mx) mx = noise_buf[i];
    }
    double mean = sum / noise_idx;
    double sq = 0;
    for (uint16_t i = 0; i < noise_idx; i++) {
        double d = noise_buf[i] - mean;
        sq += d * d;
    }
    double rms = sqrt(sq / noise_idx);
    double pp = (double)(mx - mn);
    uint16_t q = noise_idx / 4;
    double m1 = 0, m2 = 0;
    for (uint16_t i = 0; i < q; i++) m1 += noise_buf[i];
    for (uint16_t i = noise_idx - q; i < noise_idx; i++) m2 += noise_buf[i];
    m1 /= q; m2 /= q;
    double drift = m2 - m1;
    uint32_t dt_ms = millis() - noise_t_start;
    float fs = (dt_ms > 0) ? (1000.0f * noise_idx / dt_ms) : 0.0f;
    Serial.printf("# NOISE N=%u fs=%.1f mean=%.1f RMS=%.2f PP=%.0f drift=%.1f\n", noise_idx, fs, mean, rms, pp, drift);
    noise_idx = 0;
    noise_t_start = millis();
}

void loop() {
#if TEST_ENABLED
    static uint32_t lastStimMicros = 0;
    uint32_t now = micros();
    if (now - lastStimMicros >= 1000) {
        lastStimMicros += 1000;
        updateTestSignal();
    }
#endif

    if (digitalRead(PIN_DRDY) == LOW) {
        uint8_t st;
        int32_t raw = readData(&st);
        Serial.println(raw);
    }

    // Оптимальный вывод параметров без задержек основного цикла (50 Гц)
    static uint32_t lastPrintMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs >= 20) {
        lastPrintMs = nowMs;
        // Печать: Сырое напряжение шунта (мВ), Наблюдаемый ток (мА), Ожидаемый ток (мА), Текущий ШИМ
        Serial.printf("Shunt_mV:%.1f,Meas_mA:%.3f,Exp_mA:%.3f,Duty:%d\n", log_shunt_mv, log_measured_mA, log_target_mA, log_duty);
    }
}
