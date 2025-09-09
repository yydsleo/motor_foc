#include "motor_foc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <driver/ledc.h>
#include <driver/i2c_master.h>
#include <string.h>
#include <math.h>
#include <driver/pulse_cnt.h>
#include <driver/pcnt_types_legacy.h>

#define MOTOR_PWM1 4
#define MOTOR_PWM2 5
#define MOTOR_PWM3 6
#define MOTOR_EN 7
#define MOTOR_PWM_FREQ 20000

#define AS5600_I2C_SDA 15
#define AS5600_I2C_SCL 16
#define AS5600_ADDRESS 0x36
// 原始角度值
#define AS5600_REG_RAW_ANGLE 0x0C
// 滤波后的角度值
#define AS5600_REG_ANGLE 0x0E
// 状态寄存器
#define AS5600_REG_STATUS 0x0B
// 配置寄存器
#define AS5600_REG_CONFIG 0x09

static const char *TAG = "motor";

struct Motor *motor_left, *motor_right;
void foc_motor_init() {
    // init pwm
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&pwm_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PWM timer");
        return;
    }
    motor_left = foc_init_motor(MOTOR_PWM1, MOTOR_PWM2, MOTOR_PWM3, MOTOR_EN,
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
        LEDC_TIMER_0);

    motor_left->i2c_dev_handle = foc_motor_i2c_init();
}

struct Motor* get_left_motor() {
    return motor_left;
}

struct Motor* get_right_motor() {
    return motor_right;
}

struct Motor* foc_init_motor(gpio_num_t pin_in1, gpio_num_t pin_in2, gpio_num_t pin_in3, gpio_num_t pin_en, 
                            ledc_channel_t channel1,
                            ledc_channel_t channel2,
                            ledc_channel_t channel3,
                            ledc_timer_t timer) { 
    struct Motor* motor = emalloc(sizeof(struct Motor));
    if (motor == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for motor");
        return NULL;
    }
    memset(motor, 0, sizeof(struct Motor));
    motor->pin_pwm_1 = pin_in1;
    motor->pin_pwm_2 = pin_in2;
    motor->pin_pwm_3 = pin_in3;
    motor->pin_en = pin_en;

    gpio_set_direction(pin_in1, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_in2, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_in3, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_en, GPIO_MODE_OUTPUT);

    gpio_set_level(pin_in1, 0);
    gpio_set_level(pin_in2, 0);
    gpio_set_level(pin_in3, 0);
    gpio_set_level(pin_en, 0);

    // 配置两个PWM通道(对应H桥的两个输入)
    ledc_channel_config_t pwm_channel1 = {
        .gpio_num = pin_in1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel1,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    esp_err_t ret = ledc_channel_config(&pwm_channel1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel");
        return NULL;
    }
    ledc_channel_config_t pwm_channel2 = {
        .gpio_num = pin_in2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel2,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&pwm_channel2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel");
        return NULL;
    }
    ledc_channel_config_t pwm_channel3 = {
        .gpio_num = pin_in3,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel3,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&pwm_channel3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel");
        return NULL;
    }

    /*
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = pin_in1,
        .mcpwm0b_out_num = pin_in2,
        .mcpwm1a_out_num = pin_in3
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
    */
    motor->channel1 = channel1;
    motor->channel2 = channel2;
    motor->channel3 = channel3;

    // init foc param
    motor->voltage_power_supply = 12;
    motor->start_ts = esp_timer_get_time();

    return motor;
}


i2c_master_dev_handle_t foc_motor_i2c_init() {
    i2c_master_dev_handle_t i2c_dev_handle;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = AS5600_I2C_SCL,
        .sda_io_num = AS5600_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDRESS,
        .scl_speed_hz = 800*1000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c_dev_handle));
    gpio_set_level((gpio_num_t)AS5600_I2C_SDA, 1);
    gpio_set_level((gpio_num_t)AS5600_I2C_SCL, 1);

    uint8_t status = 0;
    ESP_ERROR_CHECK(as5600_i2c_read_reg(i2c_dev_handle, AS5600_REG_STATUS, &status));

    ESP_LOGI(TAG, "I2C bus initialized");
    return i2c_dev_handle;
    /*
    for (int i = 0; i < 128; i += 16) {
        for (int j = 0; j < 16; j++) {
            int addr = i + j;
            if (addr <= 0x03 || addr > 0x77) {
                continue;
            }
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 400*1000,
            };
            i2c_master_dev_handle_t dev_handle;
            esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
            if (ret == ESP_OK) {
                uint8_t status = 0;
                ret = as5600_i2c_read_reg(AS5600_REG_STATUS, &status);
                if (ret == ESP_OK) {
                    printf("%02x: %02x\n", addr, status);
                }
            }
            i2c_master_bus_rm_device(dev_handle);
        }
        printf("\n");
    }
    */
}

esp_err_t as5600_i2c_read_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_receive(i2c_dev_handle, data, len, -1);
}

esp_err_t as5600_i2c_read_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_receive(i2c_dev_handle, data, 1, -1);
}

esp_err_t as5600_i2c_write_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t length)
{
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, &reg, 1, -1);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_transmit(i2c_dev_handle, data, length, -1);
    
    return ret;
}

esp_err_t as5600_i2c_write_reg(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t value)
{
    uint8_t cmd[2] = {reg, value};
    return i2c_master_transmit(i2c_dev_handle, cmd, 2, -1);
}

esp_err_t as5600_read_raw_angle(struct Motor* motor, uint16_t *angle) {
    uint8_t data[2] = {0};
    esp_err_t ret = as5600_i2c_read_bytes(motor->i2c_dev_handle, AS5600_REG_RAW_ANGLE, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    *angle = ((data[0] << 8) | data[1]) & 0x0FFF;;
    return ESP_OK;
}

esp_err_t as5600_read_angle_without_track(struct Motor* motor, float *angle) {
    uint16_t raw_angle = 0;
    esp_err_t ret = as5600_read_raw_angle(motor, &raw_angle);
    if (ret != ESP_OK) {
        return ret;
    }
    // 0.087890625其实就是360/4096(as5600编码器的角度范围是0-4096，所以要先转换成角度值，然后再转换成弧度)
    *angle = (float)raw_angle * 0.087890625 * M_PI / 180;
    return ESP_OK;
}
float angle_without_track;
esp_err_t as5600_read_angle(struct Motor* motor, float *angle) {
    static float prev_angle = 0;
    static int full_ratations = 0;

    // float angle_without_track = 0;
    esp_err_t ret = as5600_read_angle_without_track(motor, &angle_without_track);
    if (ret != ESP_OK) {
        return ret;
    }
    float d_angle = angle_without_track - prev_angle;
    // 判断圈数是否超过80%的一圈
    if (fabs(d_angle) > (0.8f * 2 * M_PI)) {
        full_ratations += (d_angle > 0 ? -1 : 1);
    }
    prev_angle = angle_without_track;
    *angle = (float) full_ratations * 2 * M_PI + angle_without_track;
    return ESP_OK;
}

void motor_run(struct Motor* motor) {
    motor_enable(motor, 1);
    // velocityOpenloop(motor, 70);
    // velocityOpenloop(motor, 70);
    /*
    static float target_angle = 0.0;
    target_angle += 0.020;
    positionClosedloop(motor, target_angle);s
    */

    velocityClosedloop(motor, 50);
}

void motor_enable(struct Motor* motor, int enable) {
    gpio_set_level(motor->pin_en, enable);
}

// foc

void motor_set_pwm(struct Motor* motor, float ua, float ub, float uc) { 
    // printf("%f %f %f\n", ua, ub, uc);
    motor->dc_a = _constrain(ua / motor->voltage_power_supply, 0, 1);
    motor->dc_b = _constrain(ub / motor->voltage_power_supply, 0, 1);
    motor->dc_c = _constrain(uc / motor->voltage_power_supply, 0, 1);

    // printf("%f %f %f\n", motor->dc_a, motor->dc_b, motor->dc_c);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel1, motor->dc_a * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel1);
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel2, motor->dc_b * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel3, motor->dc_c * 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel3);
}

float _normalizeAngle(float angle){
  float a = fmod(angle, 2*M_PI);
  return a >= 0 ? a : (a + 2*M_PI);  
}

float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

void setPhaseVoltage(struct Motor* motor, float Uq,float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el + motor->zero_electric_angle);
    // 帕克逆变换
    motor->Ualpha =  -Uq*sin(angle_el); 
    motor->Ubeta =   Uq*cos(angle_el); 

    // 克拉克逆变换
    motor->Ua = motor->Ualpha + motor->voltage_power_supply/2;
    motor->Ub = (sqrt(3)*motor->Ubeta-motor->Ualpha)/2 + motor->voltage_power_supply/2;
    motor->Uc = (-motor->Ualpha-sqrt(3)*motor->Ubeta)/2 + motor->voltage_power_supply/2;
    motor_set_pwm(motor, motor->Ua, motor->Ub, motor->Uc);
}

void setTorque(struct Motor* motor, float Uq, float angle_el) {
    Uq = _constrain(Uq, -motor->voltage_power_supply/2, motor->voltage_power_supply/2);
    // float Ud = 0;
    angle_el = _normalizeAngle(angle_el);
    // 帕克逆变换
    motor->Ualpha = -Uq*sin(angle_el); 
    motor->Ubeta = Uq*cos(angle_el); 

    // 克拉克逆变换
    motor->Ua = motor->Ualpha + motor->voltage_power_supply/2;
    motor->Ub = (sqrt(3)*motor->Ubeta-motor->Ualpha)/2 + motor->voltage_power_supply/2;
    motor->Uc = (-motor->Ualpha-sqrt(3)*motor->Ubeta)/2 + motor->voltage_power_supply/2;
    motor_set_pwm(motor, motor->Ua, motor->Ub, motor->Uc);
}


// 开环控制
float velocityOpenloop(struct Motor* motor, float target_velocity){
    int64_t now_us = esp_timer_get_time();
    float ts = (now_us - motor->start_ts) * 1e-6f;
    motor->shaft_angle = _normalizeAngle(motor->shaft_angle + target_velocity*ts);
    float Uq = motor->voltage_power_supply/3;
    float eletricasl_angle = _electricalAngle(motor->shaft_angle, 7);
    // printf("Up: %f, electrical angle: %f\n", Uq, eletricasl_angle);
    setPhaseVoltage(motor, Uq,  0, eletricasl_angle);
    motor->start_ts = now_us;  //用于计算下一个时间间隔
    return Uq;
}

// 闭环控制
float positionClosedloop(struct Motor* motor, float target_angle) { 
    float angle = 0;
    as5600_read_angle(motor, &angle);
    motor->shaft_angle = angle_without_track;

    // show speed
    static int64_t prev_us = 0;
    static float prev_angle = 0;
    static float speed = 0;
    static int cnt = 0;
    int64_t cur_us = esp_timer_get_time();
    float cur_speed = (angle - prev_angle) / ((float)(cur_us - prev_us) * 1e-6);
    speed = speed * 0.85 + cur_speed * 0.15;
    cnt++;
    if (cur_us - prev_us > 1000000) {
        // printf("speed: %f, cnt: %d, angle: %f, target angle: %f\n", cur_speed, cnt, angle, target_angle);
        prev_us = cur_us;
        prev_angle = angle;
        cnt = 0;
    }

    int DIR = 1;
    float kp = 0.133;
    // float Uq = _constrain(kp * DIR * (target_angle - DIR * angle) * 180 / M_PI, -3, 3);
    float Uq = _constrain(kp * DIR * (target_angle - DIR * angle) * 180 / M_PI, -6, 6);
    // float Uq = _constrain(6, -6, 6);
    float eletricasl_angle = DIR * _electricalAngle(motor->shaft_angle - motor->zero_electric_angle, 7);
    // printf("Up: %f, electrical angle: %f\n", Uq, eletricasl_angle);
    setPhaseVoltage(motor, Uq, 0, eletricasl_angle);

    return Uq;
}

float velocityClosedloop(struct Motor* motor, float target_velocity) {
    float angle = 0;
    as5600_read_angle(motor, &angle);
    motor->shaft_angle = angle_without_track;

    // 获取速度
    static int64_t prev_us = 0;
    static float prev_angle = 0;
    int64_t cur_us = esp_timer_get_time();
    float dt = (float)(cur_us - prev_us) * 1e-6;
    float dangle = (angle - prev_angle);
    float cur_velocity = dangle / dt;
    prev_us = cur_us;
    prev_angle = angle;

    // 使用低通滤波算法获取速度
    float TF = 0.009;
    float alpha = TF / (TF + dt);
    static float velocity = 0;
    velocity = velocity * alpha + cur_velocity * (1 - alpha);
    static int cnt = 0;
    if (cnt >= 4000) {
        printf("velocity: %f, target velocity: %f\n", velocity, target_velocity);
        cnt = 0;
    }
    cnt++;

    // PI控制
    float dvelocity = target_velocity - velocity;
    float kp = 0.003, ki = 0.002;
    static float integral = 0;
    static float prev_dvelocity = 0;
    integral += (dvelocity + prev_dvelocity) * 0.5 * dt;
    prev_dvelocity = dvelocity;
    float pi = kp * dvelocity + ki * integral;
    float Up = _constrain(pi * 180 / M_PI, -6, 6);
    setTorque(motor, Up, _electricalAngle(motor->shaft_angle - motor->zero_electric_angle, 7));
    return 0;
}
