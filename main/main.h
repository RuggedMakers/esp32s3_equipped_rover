#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "driver/pulse_cnt.h"

#define mTIMER_HZ 10000000                   // 10MHz, 1 tick = 0.1us
#define mPWM_FREQ_HZ 25000                   // 25KHz PWM
#define mMAX_DUTY (mTIMER_HZ / mPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

/*  Motor 0  */
#define m0_PINA 7
#define m0_PINB 15
#define m0_ENCA 36
#define m0_ENCB 35

/*  Motor 1  */
#define m1_PINA 7
#define m1_PINB 15
#define m1_ENCA 36
#define m1_ENCB 35

/* Encoder events - Pulses per rev */
#define encPulsePerRev 610

/* PID configuration */
#define PID_UPDATE_ms 10       // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED 0 // expected motor speed, in the pulses counted by the rotary encoder


typedef struct
{
    int motor_number;
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int accumu_count;
    int report_pulses;
} motor_control_context_t;

bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    int *accumu_count = (int *)user_ctx;
    *accumu_count += edata->watch_point_value;
    return false;
}

void pid_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    cur_pulse_count += ctx->accumu_count;
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = m1_speed - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

esp_err_t initialize_motor((motor_control_context_t *) m){
    /* Create the motor contexts */
    static motor_control_context_t m0_ctx = {
        .motor_number = 0,
        .accumu_count = 0,
        .pcnt_encoder = NULL,
    };
    static motor_control_context_t m1_ctx = {
        .motor_number = 1,
        .accumu_count = 0,
        .pcnt_encoder = NULL,
    };

    /* Create the motors */
    bdc_motor_config_t m0_cfg = {
        .pwm_freq_hz = mPWM_FREQ_HZ,
        .pwma_gpio_num = m0_PINA,
        .pwmb_gpio_num = m0_PINB,
    };
    bdc_motor_config_t m1_cfg = {
        .pwm_freq_hz = mPWM_FREQ_HZ,
        .pwma_gpio_num = m1_PINA,
        .pwmb_gpio_num = m1_PINB,
    };

    bdc_motor_mcpwm_config_t m0_pwmcfg = {
        .group_id = 0,
        .resolution_hz = mPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t m1_pwmcfg = {
        .group_id = 0,
        .resolution_hz = mPWM_FREQ_HZ,
    };

    bdc_motor_handle_t m0 = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&m0_cfg, &m0_pwmcfg, &m0));
    m0_ctx.motor = m0;

    bdc_motor_handle_t m1 = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&m1_cfg, &m1_pwmcfg, &m1));
    m1_ctx.motor = m1;

    ESP_LOGI(mTAG, "Motors have been created");
}
