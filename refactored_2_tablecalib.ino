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
static bool g_log_enabled = true;
// ===== ЗАЩИТЫ =====
#define HARD_CURRENT_LIMIT_MA       5.0f   // аварийная отсечка по току
#define NO_CURRENT_THRESHOLD_MA     0.05f  // ниже этого считаем "тока нет"
#define NO_CURRENT_TIMEOUT_MS       50     // через сколько мс реагировать на обрыв
#define MAX_DUTY_WHEN_NO_CURRENT    100    // потолок duty при подозрении на обрыв
#define FAULT_COOLDOWN_MS           500    // пауза после срабатывания аварии

// --- ПАРАМЕТРЫ СТИМУЛЯЦИИ (ЗАДАЧА ФОРМЫ, ЧАСТОТЫ И АМПЛИТУДЫ) ---
static float g_stim_freq_hz = 20.0f;          // Частота волны, Гц
static float g_target_amplitude_ua = 3000.0f; // Желаемая амплитуда тока в МИКРОамперах (1000 мкА = 1 мА)
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


// --- ГЛОБАЛЬНЫЕ СОСТОЯНИЯ И ПЕРЕМЕННЫЕ ДЛЯ БЭЙЗЛАЙНА ---
enum CalState {
    CAL_IDLE,
    CAL_BASELINE,
    CAL_WAIT_RES,
    CAL_KNOWN_RES,
    CAL_DONE
};
static CalState g_cal_state = CAL_IDLE;
static float g_cal_known_R = 0.0f;
static bool g_check_active = false;
static float g_check_R = 0.0f;

int   g_baseline_duties[5]   = {0, 10, 50, 100, 200};
float g_baseline_currents[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
int   g_last_duty            = 0; // Для запоминания текущего ШИМа

static float g_baseline_current_mA = 0.0f;   // Бэйзлайн ТОК (холостой ход) для логов
static bool  g_baseline_ready  = false;
// -------------------------------------------------------


// ===== Усреднённое чтение ISENSE (антиалиасинг PWM) =====
#define ISENSE_OVERSAMPLE 16
float readIsenseAvg_mV() {
    uint32_t sum = 0;
    for (int i = 0; i < ISENSE_OVERSAMPLE; i++) {
        sum += analogReadMilliVolts(PIN_ISENSE);
    }
    return (float)sum / ISENSE_OVERSAMPLE;
}

#define CURRENT_CALIBRATION 1.0f  // Вернули на 1.0, чтобы смотреть реальные данные шунта

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

// // ===== Параметры feedforward =====
// #define LOAD_R_OHMS_NOMINAL 100000.0f  // ожидаемая нагрузка (электроды + кожа)

// #define STIM_KP 8.0f      // увеличиваем агрессивность коррекции
// #define STIM_KI 0.5f      // и интеграл тоже
// #define STIM_INTEGRAL_MAX 80.0f
// #define STIM_INTEGRAL_MIN -80.0f
// ===== Параметры feedforward =====
#define LOAD_R_OHMS_NOMINAL 100000.0f  // ожидаемая нагрузка (электроды + кожа)

#define STIM_KP 8.0f      
#define STIM_KI 0.5f      
#define STIM_INTEGRAL_MAX 245.0f   // РАЗБЛОКИРОВАНО! ПИД может вытянуть ШИМ до упора
#define STIM_INTEGRAL_MIN -245.0f


// ===== Огибающие шунта =====
static float g_env_top = 0.0f;
static float g_env_bot = 0.0f;
static float g_env_bot_smooth = 0.0f;
static bool  g_env_init = false;

#define ENV_ATTACK 0.30f
#define ENV_DECAY  0.002f
#define BOT_SMOOTH_ALPHA 0.01f

void updateEnvelopes(float raw_mv) {
    if (!g_env_init) {
        g_env_top = g_env_bot = g_env_bot_smooth = raw_mv;
        g_env_init = true;
        return;
    }
    if (raw_mv > g_env_top) g_env_top += ENV_ATTACK * (raw_mv - g_env_top);
    else                    g_env_top += ENV_DECAY  * (raw_mv - g_env_top);
    if (raw_mv < g_env_bot) g_env_bot += ENV_ATTACK * (raw_mv - g_env_bot);
    else                    g_env_bot += ENV_DECAY  * (raw_mv - g_env_bot);

    // сглаживание нижней огибающей
    g_env_bot_smooth += BOT_SMOOTH_ALPHA * (g_env_bot - g_env_bot_smooth);
}

// =========================================================================
// ПРЯМОЙ РАСЧЕТ ТОКА: Наблюдаемый ток - Бэйзлайн ток
// =========================================================================

// Получить абсолютный ток (включая холостой ход) на основе разницы 1650 - текущее напряжение
float getAbsoluteCurrent_mA(float bot_mv) {
    float delta_mv = INA_REF_MV - bot_mv; 
    if (delta_mv < 0.0f) delta_mv = 0.0f; // защита от выбросов
    
    // Ток = Напряжение / (Сопротивление * Усиление)
    return delta_mv / (ISENSE_SHUNT_OHMS * ISENSE_INA_GAIN);
}

// Вычисление динамического бэйзлайна на основе текущего ШИМа
float getDynamicBaseline_mA(int current_duty) {
    if (!g_baseline_ready) return 0.0f;
    
    int abs_duty = abs(current_duty);
    
    // Выход за пределы таблицы
    if (abs_duty <= g_baseline_duties[0]) return g_baseline_currents[0];
    if (abs_duty >= g_baseline_duties[4]) return g_baseline_currents[4];
    
    // Линейная интерполяция между точками
    for (int i = 0; i < 4; i++) {
        if (abs_duty >= g_baseline_duties[i] && abs_duty <= g_baseline_duties[i+1]) {
            float t = (float)(abs_duty - g_baseline_duties[i]) / (g_baseline_duties[i+1] - g_baseline_duties[i]);
            return g_baseline_currents[i] + t * (g_baseline_currents[i+1] - g_baseline_currents[i]);
        }
    }
    return g_baseline_currents[4];
}

float currentFromBot_mA() {
    if (!g_baseline_ready) return 0.0f;
    
    float active_mv = (g_cal_state == CAL_KNOWN_RES || g_check_active) ? g_env_bot : g_env_bot_smooth;
    float total_bat_mA = getAbsoluteCurrent_mA(active_mv);
    
    // Берем точный бэйзлайн для текущего ШИМа
    float dynamic_baseline_mA = getDynamicBaseline_mA(g_last_duty);
    
    float active_bat_mA = total_bat_mA - dynamic_baseline_mA;
    if (active_bat_mA < 0.0f) active_bat_mA = 0.0f;
    
    // Чистый баланс мощностей: DCDC работает как трансформатор.
    float load_current_mA = active_bat_mA * (V_BAT / V_OUT) * DCDC_EFFICIENCY * CURRENT_CALIBRATION;
    
    return load_current_mA;
}

void updateTestSignal() {
#if TEST_ENABLED
    // 1) фаза
    float delta_phase = 2.0f * M_PI * g_stim_freq_hz * 0.001f;
    g_stim_phase += delta_phase;
    if (g_stim_phase > (2.0f * M_PI)) g_stim_phase -= (2.0f * M_PI);

    // 2) форма
    float wave = 0.0f;
    if (g_cal_state == CAL_KNOWN_RES || g_check_active) {
        // !!! ФИКС: При калибровке и проверке даем чистый DC (постоянный ток), 
        // чтобы ПИД не боролся с синусоидой и огибающей!
        wave = 1.0f; 
    } else {
        if (g_stim_form == WAVE_SINE) {
            wave = sinf(g_stim_phase);
        } else if (g_stim_form == WAVE_SQUARE) {
            wave = (g_stim_phase < M_PI) ? 1.0f : -1.0f;
        } else if (g_stim_form == WAVE_TRIANGLE) {
            wave = (g_stim_phase < M_PI)
                   ? (2.0f * g_stim_phase / M_PI - 1.0f)
                   : (3.0f - 2.0f * g_stim_phase / M_PI);
        }
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

    updateEnvelopes(raw_ina_mv);

    float abs_measured_mA = currentFromBot_mA();   
    
    // 5.1) HARD CURRENT LIMIT — аварийная отсечка
    if (abs_measured_mA > HARD_CURRENT_LIMIT_MA) {
        ledcWrite(PIN_DRV_IN1, 0);
        ledcWrite(PIN_DRV_IN2, 0);
        g_integral = 0.0f;
        g_fault_until_ms = millis() + FAULT_COOLDOWN_MS;
        if (g_log_enabled) Serial.printf("!!! OVERCURRENT %.2f mA — fault\n", abs_measured_mA);
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

    // 6) FEEDFORWARD (подставляем известное R при калибровке, чтобы ШИМ не прыгал)
    float expected_R = LOAD_R_OHMS_NOMINAL;
    if (g_cal_state == CAL_KNOWN_RES) expected_R = g_cal_known_R;
    else if (g_check_active)          expected_R = g_check_R;

    float abs_target_mA = fabsf(target_mA);
    float u_load_v = (abs_target_mA / 1000.0f) * expected_R;
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

    // ЗАПОМИНАЕМ ШИМ ДЛЯ ДИНАМИЧЕСКОГО БЭЙЗЛАЙНА
    g_last_duty = duty;

    // 8.1) ЗАЩИТА ОТ ОБРЫВА НАГРУЗКИ
    if (abs_measured_mA < NO_CURRENT_THRESHOLD_MA && abs_target_mA > 0.1f) {
        g_low_current_count++;
        if (g_low_current_count > NO_CURRENT_TIMEOUT_MS) {
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

// =========================================================================
//                  КАЛИБРОВКА / ПРОВЕРКА / ХАРДКОД
// =========================================================================

// ---------- параметры ----------
#define CAL_CURRENT_UA        500.0f   // ток калибровки/проверки = 1 мА
#define CHECK_TOLERANCE_PCT   10.0f     // допустимое отклонение при проверке, %
#define CHECK_DURATION_MS     2000      // длительность сбора при проверке

// ---------- ЗАХАРДКОЖЕННАЯ КАЛИБРОВКА (вписать из лога CAL_*) ----------
#define HARDCODE_CAL                  0         
#define HARDCODE_BASELINE_CURRENT_MA  0.25f     // <-- Захардкодь сюда ток холостого хода (baseline ток) из лога!

static double   g_cal_current_sum = 0.0;
static double   g_cal_duty_sum    = 0.0;
static uint32_t g_cal_count       = 0;

static bool  g_cal_valid     = false;
static float g_saved_amplitude_ua = 0.0f;      // общий для калибровки и проверки

// =========================== СОСТОЯНИЕ ПРОВЕРКИ ===========================
static uint32_t g_check_t0          = 0;
static double   g_check_current_sum = 0.0;
static double   g_check_duty_sum    = 0.0;
static uint32_t g_check_count       = 0;

// =========================== ВСПОМОГАТЕЛЬНОЕ ===========================
void resetCalAccum() {
    g_cal_current_sum = 0.0;
    g_cal_duty_sum    = 0.0;
    g_cal_count       = 0;
}

// загрузка захардкоженной калибровки — вызвать в конце setup()
void loadHardcodeCal() {
#if HARDCODE_CAL
    g_baseline_current_mA = HARDCODE_BASELINE_CURRENT_MA;
    g_baseline_ready      = true;
    g_cal_valid           = true;
    Serial.printf(">>> HARDCODE CAL loaded: baseline_current=%.3f mA\n", g_baseline_current_mA);
#endif
}

// ============================ КАЛИБРОВКА: STEP ============================
void calibrationStep() {
    switch (g_cal_state) {
        case CAL_BASELINE:
            g_cal_current_sum += getAbsoluteCurrent_mA(g_env_bot_smooth); // Для бэйзлайна нужен чистый ток батареи
            g_cal_duty_sum    += log_duty;
            g_cal_count++;
            break;
        case CAL_KNOWN_RES:
            g_cal_current_sum += currentFromBot_mA(); // Для проверки R копим реальный вычисленный ток
            g_cal_duty_sum    += log_duty;
            g_cal_count++;
            break;
        default: break;
    }
}

// ============================ ПРОВЕРКА: START ============================
void startCheck(float R) {
    if (!g_cal_valid && !g_baseline_ready) {
        Serial.println("!!! CHECK: нет калибровки. Сначала откалибруй (c b r f) или захардкодь.");
        return;
    }
    if (R <= 0.0f) { Serial.println("!!! пример: k1000"); return; }

    g_check_R           = R;
    g_check_current_sum = 0.0;
    g_check_duty_sum    = 0.0;
    g_check_count       = 0;
    g_check_t0          = millis();
    g_check_active      = true;

    g_saved_amplitude_ua  = g_target_amplitude_ua;
    g_target_amplitude_ua = CAL_CURRENT_UA;   // тот же ток, что и при калибровке

    Serial.printf(">>> CHECK: R=%.1f Ом, ток=%.2f мА, сбор %d мс...\n",
                  g_check_R, CAL_CURRENT_UA / 1000.0f, CHECK_DURATION_MS);
}

// ============================ ПРОВЕРКА: STEP ============================
void checkStep() {
    if (!g_check_active) return;

    g_check_current_sum += currentFromBot_mA(); // Сразу копим пересчитанный ток нагрузки
    g_check_duty_sum    += log_duty;
    g_check_count++;

    if (millis() - g_check_t0 < CHECK_DURATION_MS) return;

    g_check_active = false;
    g_target_amplitude_ua = g_saved_amplitude_ua;

    if (g_check_count == 0) { Serial.println("!!! CHECK: нет данных"); return; }

    float i_meas_mA = (float)(g_check_current_sum / g_check_count); 
    float duty_avg  = (float)(g_check_duty_sum    / g_check_count);

    float i_target_mA = CAL_CURRENT_UA / 1000.0f; // Ожидаем то, что заказывали
    float err_I = (i_target_mA > 0.0001f) ? (i_meas_mA - i_target_mA) / i_target_mA * 100.0f : 0.0f;

    Serial.println(">>> CHECK RESULT:");
    Serial.printf("    duty       = %.1f / %d", duty_avg, STIM_MAX_DUTY);
    if (duty_avg >= (STIM_MAX_DUTY - 2)) Serial.print("  (!) НАСЫЩЕНИЕ");
    Serial.println();
    
    Serial.printf("    Target I   = %.3f мА\n", i_target_mA);
    Serial.printf("    Meas I     = %.3f мА | err = %+.1f%%\n", i_meas_mA, err_I);

    bool ok = (fabsf(err_I) <= CHECK_TOLERANCE_PCT) && (duty_avg < (STIM_MAX_DUTY - 2));
    if (ok)
        Serial.printf(">>> CHECK: OK (в пределах %.0f%%). Можно начинать сеанс.\n", CHECK_TOLERANCE_PCT);
    else
        Serial.printf(">>> CHECK: FAIL (откл. > %.0f%%).\n", CHECK_TOLERANCE_PCT);
}

// ============================ ОБРАБОТКА КОМАНД ============================
void handleSerialCommand(char c, String arg) {
    switch (c) {
        case 'c': { // старт baseline — прогон по массиву ШИМ без нагрузки
            g_log_enabled = false;
            Serial.println(">>> CAL: Ищем бэйзлайн ТОК. Нагрузку СНЯТЬ. Идет сбор 5 точек...");
            g_saved_amplitude_ua  = g_target_amplitude_ua;
            
            // for (int i = 0; i < 5; i++) {
            //     int d = g_baseline_duties[i];
            //     ledcWrite(PIN_DRV_IN1, d);
            //     ledcWrite(PIN_DRV_IN2, 0);
            //     delay(300); // даем повышайке стабилизироваться
                
            //     double sum_mA = 0;
            //     int count = 0;
            //     for (int j = 0; j < 64; j++) {
            //         float raw_mv = readIsenseAvg_mV();
            //         updateEnvelopes(raw_mv);
            //         sum_mA += getAbsoluteCurrent_mA(g_env_bot_smooth);
            //         count++;
            //         delay(2);
            //     }
            //     g_baseline_currents[i] = (float)(sum_mA / count);
            //     Serial.printf("    ШИМ = %3d -> Ток покоя = %.3f мА\n", d, g_baseline_currents[i]);
            // }
            for (int i = 0; i < 5; i++) {
                int d = g_baseline_duties[i];
                ledcWrite(PIN_DRV_IN1, d);
                ledcWrite(PIN_DRV_IN2, 0);
                
                // Увеличили время ожидания до 1 секунды
                delay(1000); 
                
                double sum_mA = 0;
                int count = 0;
                
                // Собираем 200 точек с шагом 5 мс (еще 1 полная секунда сбора)
                for (int j = 0; j < 200; j++) {
                    float raw_mv = readIsenseAvg_mV();
                    updateEnvelopes(raw_mv);
                    sum_mA += getAbsoluteCurrent_mA(g_env_bot_smooth);
                    count++;
                    delay(5); 
                }
                g_baseline_currents[i] = (float)(sum_mA / count);
                Serial.printf("    ШИМ = %3d -> Ток покоя = %.3f мА\n", d, g_baseline_currents[i]);
            }
                        
            ledcWrite(PIN_DRV_IN1, 0); 
            g_baseline_ready = true;
            g_baseline_current_mA = g_baseline_currents[1]; // для отображения в логах
            
            resetCalAccum();
            g_cal_state = CAL_WAIT_RES; 
            Serial.println(">>> CAL: Сбор завершен. (Команда 'b' нажата автоматически)");
            Serial.println(">>> Вставь известный R: r<Ом>, напр. r1000");
            break;
        }

        case 'b': // стоп baseline (оставлено для совместимости)
            if (g_baseline_ready) {
                Serial.println(">>> CAL: baseline уже собран. Переходи к вводу R (например, r1000)");
                g_cal_state = CAL_WAIT_RES;
            } else {
                Serial.println("!!! baseline не набран (сначала c)");
            }
            break;

        case 'r': // известный резистор + сбор
            g_log_enabled=false;
            if (g_cal_state == CAL_WAIT_RES || g_cal_state == CAL_DONE) {
                g_cal_known_R = arg.toFloat();
                if (g_cal_known_R <= 0) { Serial.println("!!! пример: r1000"); break; }
                g_target_amplitude_ua = CAL_CURRENT_UA;
                Serial.printf(">>> CAL: R=%.1f Ом, ток=%.2f мА. Сбор...\n",
                              g_cal_known_R, CAL_CURRENT_UA / 1000.0f);
                resetCalAccum();
                g_cal_state = CAL_KNOWN_RES;
            } else {
                Serial.println("!!! сначала baseline (c)");
            }
            break;

        case 'f': { // завершить: показать верификацию 
            if (g_cal_state != CAL_KNOWN_RES || g_cal_count == 0) {
                Serial.println("!!! нет данных по известному R");
                break;
            }
            float i_load_mA = (float)(g_cal_current_sum  / g_cal_count); // УЖЕ вычислен ток нагрузки
            float duty_avg  = (float)(g_cal_duty_sum / g_cal_count);

            if (duty_avg >= (STIM_MAX_DUTY - 2)) {
                Serial.printf("!!! DUTY В НАСЫЩЕНИИ (%.1f >= %d). Петля не держит ток.\n",
                              duty_avg, STIM_MAX_DUTY);
                Serial.println("!!! Уменьши R или ток.");
                g_cal_state = CAL_DONE;
                g_target_amplitude_ua = g_saved_amplitude_ua;
                break;
            }

            float i_target_mA = CAL_CURRENT_UA / 1000.0f; // Ожидаем то, что заказывали (1 мА)

            if (i_load_mA > 0.001f) {
                g_cal_valid = true;
                Serial.println(">>> CAL DONE (ВЕРИФИКАЦИЯ):");
                Serial.printf("    baseline_mA = %.3f  <-- захардкодь как HARDCODE_BASELINE_CURRENT_MA\n", g_baseline_current_mA);
                Serial.printf("    duty(avg)   = %.1f / %d  (запас есть)\n", duty_avg, STIM_MAX_DUTY);
                Serial.printf("    Target I    = %.3f мА\n", i_target_mA);
                Serial.printf("    Meas I      = %.3f мА (наблюдаемый по физике)\n", i_load_mA);
            } else {
                Serial.println("!!! ток слишком мал, проверь подключение");
            }
            g_cal_state = CAL_DONE;
            g_target_amplitude_ua = g_saved_amplitude_ua;
            break;
        }
        case 'x':
            g_log_enabled = true;
            break;

        case 'k':
            g_log_enabled = false;
            startCheck(arg.toFloat());
            break;

        case '?':
            Serial.println("--- КАЛИБРОВКА (редко) ---");
            Serial.println("c=baseline start, b=baseline stop, r<Ом>=known R, f=finish");
            Serial.println("--- ПРОВЕРКА (перед сеансом) ---");
            Serial.println("k<Ом>=check with known R");
            break;

        default:
            Serial.println("!!! неизвестная команда, '?' для справки");
            break;
    }
}
// Serial calibration - end
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
    loadHardcodeCal();
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
    // ===== ЭХО-ТЕСТ: печатаем каждый принятый байт =====
    static char    cmdBuf[32];
    static uint8_t cmdLen = 0;

    while (Serial.available()) {
        char ch = (char)Serial.read();

        // диагностика — видно КАЖДЫЙ байт и его код
        Serial.printf("[RX byte=%d '%c']\n", (int)ch, (ch >= 32 ? ch : '?'));

        if (ch == '\n' || ch == '\r') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                Serial.printf("[CMD='%s']\n", cmdBuf);
                char cmd = cmdBuf[0];
                String arg = (cmdLen > 1) ? String(&cmdBuf[1]) : String("");
                arg.trim();
                handleSerialCommand(cmd, arg);
                cmdLen = 0;
            }
        } else {
            if (cmdLen < sizeof(cmdBuf) - 1) {
                cmdBuf[cmdLen++] = ch;
            }
        }
    }

#if TEST_ENABLED
    static uint32_t lastStimMicros = 0;
    uint32_t now = micros();
    if (now - lastStimMicros >= 1000) {
        lastStimMicros += 1000;
        updateTestSignal();

        calibrationStep();
        checkStep();
    }
#endif
    if (g_log_enabled) {
        if (digitalRead(PIN_DRDY) == LOW) {
            uint8_t st;
            int32_t raw = readData(&st);
            Serial.println(raw);
        }
    }
    static uint32_t lastPrintMs = 0;
    uint32_t nowMs = millis();
    
    if (g_log_enabled && nowMs - lastPrintMs >= 20) {
        lastPrintMs = nowMs;
        float span_mv = g_env_top - g_env_bot;
        if (span_mv < 0.0f) span_mv = 0.0f;

        Serial.printf("Shunt_mV:%.1f,Top_mV:%.1f,Bot_mV:%.1f,BotSmooth_mV:%.1f,"
                "Span_mV:%.1f,Meas_mA:%.3f,Exp_mA:%.3f,Duty:%d,"
                "Base_mA:%.3f,CAL_valid:%d\n",
                log_shunt_mv, g_env_top, g_env_bot, g_env_bot_smooth,
                span_mv,
                log_measured_mA, log_target_mA, log_duty,
                g_baseline_current_mA, (int)g_cal_valid);
    }
}