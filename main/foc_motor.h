#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include <driver/pulse_cnt.h>
#include <driver/pcnt_types_legacy.h>

#define emalloc(size) heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM)

/*
#define MOTOR_PWMA 42
#define MOTOR_AIN2 41
#define MOTOR_AIN1 40
*/



struct Motor {
    gpio_num_t pin_pwm_1;
    gpio_num_t pin_pwm_2;
    gpio_num_t pin_pwm_3;
    gpio_num_t pin_en;

    ledc_channel_t channel1;
    ledc_channel_t channel2;
    ledc_channel_t channel3;
    int loop;

    // FOC参数
    float voltage_power_supply;
    float shaft_angle;
    float open_loop_timestamp;
    float zero_electric_angle;
    float Ualpha;
    float Ubeta;
    float Ua;
    float Ub;
    float Uc;
    float dc_a;
    float dc_b;
    float dc_c;

    int64_t start_ts;
};

void foc_motor_init();
struct Motor* foc_init_motor(gpio_num_t pin_pwm1, gpio_num_t pin_pwm2, gpio_num_t pin_pwm3, gpio_num_t pin_en,
                        ledc_channel_t channel1,
                        ledc_channel_t channel2,
                        ledc_channel_t channel3,
                        ledc_timer_t timer);

struct Motor* get_left_motor();
struct Motor* get_right_motor();
void motor_enable(struct Motor* motor, int enable);
void motor_run(struct Motor* motor);

// as5600
void foc_motor_i2c_init();
esp_err_t as5600_read_angle(uint16_t *angle);
esp_err_t as5600_i2c_read_bytes(uint8_t reg, uint8_t *data, size_t len);
esp_err_t as5600_i2c_read_reg(uint8_t reg, uint8_t *data);
esp_err_t as5600_i2c_write_bytes(uint8_t reg, uint8_t *data, size_t length);
esp_err_t as5600_i2c_write_reg(uint8_t reg, uint8_t value);

// foc
void motor_set_pwm(struct Motor* motor, float ua, float ub, float uc);
float velocityOpenloop(struct Motor* motor, float target_velocity);
#endif