#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "i2c_lcd.h"

/* ================= IR SENSOR PINS ================ */
#define S1 1
#define S2 2
#define S3 4
#define S4 5
#define S5 6

/* ================= MOTOR PINS ==================== */
#define ENA 12
#define IN1 13
#define IN2 14
#define ENB 15
#define IN3 16
#define IN4 17

/* ================= PID PARAMETERS ================ */
#define KP 100.0f  // proportional
#define KI 20.0f   // integral
#define KD 12.0f   // derivative
#define MAX_CORRECTION 80 // maximum correction to reduce jerks

/* ================= SPEEDS ======================== */
#define BASE_SPEED 130
#define TURN_SPEED 180
#define RECOVERY 170

/* ================= TURN CONTROL ================== */
#define TURN_ERROR_THRESHOLD 1.2f
#define TURN_TIME_MS 140

/* ================= PWM =========================== */
#define PWM_FREQ 5000
#define PWM_RES 8
#define CH_A 0
#define CH_B 1

/* ================= GLOBALS ======================= */
volatile int s[5];              // 1 = WHITE, 0 = BLACK
volatile float linePos = 0;
volatile int lastDir = 1;       // -1 left, +1 right
volatile int turning = 0;       // -1 left, +1 right
volatile int turnActive = 0;    // 0 idle, 1 locked turn
float prevError = 0;
float integral = 0;
SemaphoreHandle_t mutex;
static const char *TAG = "LINE_FOLLOW";

/* ================= UTILS ========================= */
static inline int clamp(int v, int mn, int mx)
{
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

/* ================= MOTOR ========================= */
void driveMotors(int left, int right)
{
    left = clamp(left, -255, 255);
    right = clamp(right, -255, 255);

    gpio_set_level(IN1, left > 0);
    gpio_set_level(IN2, left < 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, abs(left));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_A);

    gpio_set_level(IN3, right > 0);
    gpio_set_level(IN4, right < 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, abs(right));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_B);
}

/* ================= SENSOR TASK =================== */
void sensor_task(void *arg)
{
    while (1)
    {
        int raw[5] = {
            gpio_get_level(S1),
            gpio_get_level(S2),
            gpio_get_level(S3),
            gpio_get_level(S4),
            gpio_get_level(S5)
        };
        
        int active = 0;
        int weighted = 0;
        
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (int i = 0; i < 5; i++)
        {
            s[i] = raw[i];
            if (raw[i] == 0) // BLACK
            {
                active++;
                weighted += (i - 2);
            }
        }
        
        if (active > 0)
        {
            linePos = (float) weighted / active;
            if (linePos > 0.2) lastDir = +1;
            if (linePos < -0.2) lastDir = -1;
        }
        xSemaphoreGive(mutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================= CONTROL TASK ================== */
void control_task(void *arg)
{
    int64_t lastTime = esp_timer_get_time();
    int64_t turnStartTime = 0;
    
    while (1)
    {
        float error;
        int ss[5];
        
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (int i = 0; i < 5; i++) ss[i] = s[i];
        error = -linePos; // steering fix
        xSemaphoreGive(mutex);

        int blackCount = 0;
        for (int i = 0; i < 5; i++) {
            if (ss[i] == 0) blackCount++;
        }

        /* ===== TURN ENTRY ===== */
        if (!turnActive && fabs(error) > TURN_ERROR_THRESHOLD)
        {
            turnActive = 1;
            turning = (error > 0) ? +1 : -1;
            turnStartTime = esp_timer_get_time();
        }

        /* ===== LOCKED TURN (NO SENSOR TRUST) ===== */
        if (turnActive)
        {
            driveMotors(turning * TURN_SPEED, -turning * TURN_SPEED);
            int64_t now = esp_timer_get_time();
            
            /* Minimum guaranteed turn time */
            if ((now - turnStartTime) > TURN_TIME_MS * 1000)
            {
                if (ss[2] == 0) // center sees BLACK
                {
                    turnActive = 0;
                    turning = 0;
                    prevError = 0;
                    integral = 0;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* ===== LINE LOST (ONLY WHEN NOT TURNING) ===== */
        if (blackCount == 0)
        {
            driveMotors(-lastDir * RECOVERY, lastDir * RECOVERY);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* ===== PID CONTROL ===== */
        int64_t now = esp_timer_get_time();
        float dt = (now - lastTime) / 1e6;
        if (dt <= 0) dt = 0.01;
        
        integral += error * dt; // integral term
        
        float P = KP * error;
        float I = KI * integral;
        float D = KD * (error - prevError) / dt;
        
        float correction = clamp(P + I + D, -MAX_CORRECTION, MAX_CORRECTION);
        
        driveMotors(BASE_SPEED + correction, BASE_SPEED - correction);
        
        prevError = error;
        lastTime = now;
        
        vTaskDelay(pdMS_TO_TICKS(20)); // slightly slower update for smoother response
    }
}

/* ================= MAIN ========================== */
void app_main(void)
{
    gpio_config_t in = {
        .pin_bit_mask = (1ULL<<S1) | (1ULL<<S2) | (1ULL<<S3) | (1ULL<<S4) | (1ULL<<S5),
        .mode = GPIO_MODE_INPUT
    };
    gpio_config(&in);

    gpio_config_t out = {
        .pin_bit_mask = (1ULL<<IN1) | (1ULL<<IN2) | (1ULL<<IN3) | (1ULL<<IN4),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&out);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t A = {
        .gpio_num = ENA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = CH_A,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    
    ledc_channel_config_t B = {
        .gpio_num = ENB,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = CH_B,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    
    ledc_channel_config(&A);
    ledc_channel_config(&B);

    lcd_init();
    lcd_clear();
    lcd_put_cursor(0,0);
    lcd_send_string("Line Follower");

    mutex = xSemaphoreCreateMutex();
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 2, NULL);
    xTaskCreate(control_task, "control", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "TURN-LOCK LOGIC ACTIVE + PID SMOOTHED");
}
