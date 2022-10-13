#include <stdio.h>
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

static const char *mTAG = "motor";
static const char *TAG = "example";

typedef struct
{
    int motor_number;
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int accumu_count;
    int report_pulses;
    int speed;
} motor_control_context_t;

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
static void pid_loop_cb(void *args);
static pcnt_unit_handle_t initialize_encoder(int *accum, int pcntEventVal, int encA, int encB);
void initialize_motor(motor_control_context_t *m, int pinA, int pinB, int grp);
void initialize_pid_counter(motor_control_context_t *m);

void initialize_pid(motor_control_context_t *m);

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
#define PID_UPDATE_ms 10

void app_main(void)
{
    /* Motor */
    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_handle_t m0 = NULL;
    bdc_motor_handle_t m1 = NULL;

    motor_control_context_t m0_ctx = {
        .motor = m0,
        .pcnt_encoder = NULL,
    };
    motor_control_context_t m1_ctx = {
        .motor = m1,
    };

    initialize_motor(&m0_ctx, m0_PINA, m0_PINB, 0);
    initialize_motor(&m1_ctx, m1_PINA, m1_PINB, 0);

    /* Encoder */
    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    initialize_encoder(&m0_ctx.accumu_count, encPulsePerRev, m0_ENCA, m0_ENCB);
    initialize_encoder(&m1_ctx.accumu_count, encPulsePerRev, m1_ENCA, m1_ENCB);

    pcnt_event_callbacks_t pcnt_cbs = {
        .on_reach = example_pcnt_on_reach, // accumulate the overflow in the callback
    };

    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &pcnt_cbs, ));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    return pcnt_unit;

    /* PID Controller */
    ESP_LOGI(TAG, "Create PID control block");
    initialize_pid(&m0_ctx);
    initialize_pid(&m1_ctx);

    /* PID Timer */
    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    initialize_pid_counter(&m0_ctx);
    initialize_pid_counter(&m1_ctx);

    /* Enable Motors */
    ESP_LOGI(TAG, "Enabling motors");
    ESP_ERROR_CHECK(bdc_motor_enable(m0_ctx.motor));
    ESP_ERROR_CHECK(bdc_motor_enable(m1_ctx.motor));

    /* Move Forward */
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(m0_ctx.motor));
    ESP_ERROR_CHECK(bdc_motor_forward(m1_ctx.motor));

    /* Begin PID Loop */
    ESP_LOGI(TAG, "Start motor speed loop");

    /* Enter Main Loop */
    int speeds[5] = {50, 100, 200, 300, 400};
    int index = 0;
    int i = 0;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (i == 30)
        {
            m0_ctx.speed = speeds[index++];
            m1_ctx.speed = speeds[index++];
            if (index >= 5)
            {
                index = 0;
            }
        }
    }

    // the following logging format is according to the requirement of serial-studio frame format
    // also see the dashboard config file `serial-studio-dashboard.json` for more information
}

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    int *accumu_count = (int *)user_ctx;
    *accumu_count += edata->watch_point_value;
    return false;
}

static void pid_loop_cb(void *args)
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
    float error = ctx->speed - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

static pcnt_unit_handle_t initialize_encoder(int *accum, int pcntEventVal, int encA, int encB)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = pcntEventVal,
        .low_limit = -pcntEventVal,
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = encA,
        .level_gpio_num = encB,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = encB,
        .level_gpio_num = encA,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, pcntEventVal));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, -pcntEventVal));
}

void initialize_pid(motor_control_context_t *m)
{
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = mMAX_DUTY - 1,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };

    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    m->pid_ctrl = pid_ctrl;
}

void initialize_motor(motor_control_context_t *m, int pinA, int pinB, int grp)
{

    /* Create the motor contexts */

    /* Create the motors */
    bdc_motor_config_t cfg = {
        .pwm_freq_hz = mPWM_FREQ_HZ,
        .pwma_gpio_num = pinA,
        .pwmb_gpio_num = pinB,
    };

    bdc_motor_mcpwm_config_t pwmcfg = {
        .group_id = grp,
        .resolution_hz = mTIMER_HZ,
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&cfg, &pwmcfg, &m->motor));
}

void initialize_pid_counter(motor_control_context_t *m)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = m,
        .name = "pid_loop"};

    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));
}