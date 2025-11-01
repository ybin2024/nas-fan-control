/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/temperature_sensor.h"
#include "driver/pulse_cnt.h"
/* power management */
#include "esp_pm.h"
//#include "esp_sleep.h"
//#include "console/console.h"

const static char *TAG = "Fan-CTL";
/*---------------------------------------------------------------
        GPIO Macros
---------------------------------------------------------------*/
#define GPIO_OUTPUT_LED_B    (13)
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_LED_B)
/*---------------------------------------------------------------
        LEDC Macros
---------------------------------------------------------------*/
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (12) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 8 bits
#define LEDC_DUTY               (255) // Set duty to 99%. (2 ** 8) * 99% = 255
#define LEDC_FREQUENCY          (25000) // Frequency in Hertz. Set frequency at 25 kHz
/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_4
//#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_5
#else
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_1
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define EXAMPLE_USE_ADC2            1
#endif

#if EXAMPLE_USE_ADC2
//ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#else
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#endif
#endif  //#if EXAMPLE_USE_ADC2

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12
/*---------------------------------------------------------------
        PCNT Macros
---------------------------------------------------------------*/
#define PCNT_HIGH_LIMIT 1024
#define PCNT_LOW_LIMIT  -1024

#define PCNT_GPIO_EDGE 11
#define PCNT_GPIO_LEVEL -1
/*---------------------------------------------------------------
        Fan control profile Macros
---------------------------------------------------------------*/
#define FAN_START_DUTY   0.23
//#define FAN_STOP_DUTY    0.17
#define FAN_SELFTEST_UPER    0.35

#define OFFSET0     0
#define OFFSET1     400
#define OFFSET2     1500
#define GAIN0       0.001000000
#define GAIN1       0.000363636
#define GAIN2       0.000100000

#define TERMAL_MAX 10.0
#define T_ZERO    25.0
#define T_MAX    50.0

#define DEVIDE      0.681
#define SCALE       (1.0/DEVIDE)
#define SELFHEAT    0.75

#define TSENS_FILTFACTOR    0.03125
//#define TCELL_FILTFACTOR    0.03125
#define TCELL_FILTFACTOR    0.0625
//#define CURRENT_FILTFACTOR  0.01953125
#define CURRENT_FILTFACTOR  0.09375

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
};

static float current_filted;
static float troom_filted;
static float tcell_filted;
static bool FAN_ON = true;
static bool ON_SYNC = false;
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
//static bool MODE = FAN_STOP;
static void example_adc_calibration_deinit(adc_cali_handle_t handle);


static void fan_pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 25 kHz
    //    .clk_cfg          = LEDC_AUTO_CLK
        .clk_cfg          = LEDC_USE_RC_FAST_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_DUTY, // Set duty to 99%
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE
//        .sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static float i2duty(float i)
{
    if(i < OFFSET1)
        return OFFSET0+GAIN0*i;
    else
        if(i < OFFSET2)
            return OFFSET0+GAIN0*OFFSET1-GAIN1*OFFSET1+GAIN1*i;
        else
            return OFFSET0+GAIN0*OFFSET1-GAIN1*OFFSET1+GAIN1*OFFSET2-GAIN2*OFFSET2+GAIN2*i;
}

static float tt2duty(float troom, float tcell)
{
    float termal;

    termal = tcell-troom;
    if(termal > TERMAL_MAX)
          return (tcell-T_ZERO)/(T_MAX-T_ZERO);
    else
        if(termal > 0)
        {
            termal /= TERMAL_MAX;
            return termal*(tcell-T_ZERO)/(T_MAX-T_ZERO);
        }
        else
            if(termal < -10.0)
            {
                ESP_LOGW(TAG, "Cell-sensor unhealth,room-temp used");
                return (troom-T_ZERO)/(T_MAX-T_ZERO);
            }
            else
                return 0.0;
}

static float fusion(float major,float minor)
{
    float merger;
    merger = major*DEVIDE+minor*(1-DEVIDE);
    merger *= SCALE;
    if(merger > 1.0)
        return 1.0;
    else
        if(merger < 0)
            return 0;
        else
            return merger;
}

static float ntc2temp(int voltage)
{
    float temp = 3.79e-5;

    temp -= 6.76e-9*voltage;
    temp *= voltage;
    temp *= voltage;
    temp -= 9.65e-2*voltage;
    return temp + 116.0;
}

void temp_read(void *arg)
{

    static float tsens_esp;
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    //-------------temp sensor Init---------------//
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: -10~80 ℃");
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
    //-------------temp sensor first read---------------//
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &troom_filted));
    ESP_LOGI(TAG, "Temperature first read: %.1f ℃", troom_filted);

    while(1)
    {
        if(FAN_ON == true)
            vTaskDelay(6000 / portTICK_PERIOD_MS);
        else
            vTaskDelay(4000 / portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_esp));
        troom_filted *= 1.0-TSENS_FILTFACTOR;
        troom_filted += (tsens_esp-SELFHEAT)*TSENS_FILTFACTOR;
    }
    
}

void adc_regulars(void *arg)
{
    const static char *TAG_adc = "ADC-Reg";
    static int adc_raw0;
    static int adc_raw1;
    static int vol_idc;
    static int vol_ntc;

    //-------------ADC1 Init---------------//
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));
    ESP_LOGI(TAG_adc, "ADC1 initialized and configured");

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
    bool do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);

    //-------------ADC1 ch0 first Read---------------//
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw0));
    ESP_LOGI(TAG_adc, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw0);
    if (do_calibration1_chan0) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw0, &vol_idc));
        ESP_LOGI(TAG_adc, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, vol_idc);
    }
    current_filted = (float)vol_idc;
    //-------------ADC1 ch1 first Read---------------//
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw1));
    ESP_LOGI(TAG_adc, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw1);
    if (do_calibration1_chan1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw1, &vol_ntc));
        ESP_LOGI(TAG_adc, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, vol_ntc);
    }
    tcell_filted = ntc2temp(vol_ntc);

    while(1)
    {
        //-------------ADC1 ch0 Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_chan0_handle, EXAMPLE_ADC1_CHAN0, &vol_idc));
        //-------------ADC1 ch1 Read---------------//
        ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(adc1_handle, adc1_cali_chan1_handle, EXAMPLE_ADC1_CHAN1, &vol_ntc));

        //-------------ADC1 Data Filter---------------//
        tcell_filted *= 1.0-TCELL_FILTFACTOR;
        tcell_filted += ntc2temp(vol_ntc)*TCELL_FILTFACTOR;

        current_filted *= 1.0-CURRENT_FILTFACTOR;
        current_filted += CURRENT_FILTFACTOR*vol_idc;

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if(ON_SYNC == false)
        {
            //-------------ADC1 De-Init---------------//
            ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
            vTaskDelay(6000 / portTICK_PERIOD_MS);
            //-------------ADC1 Re-Init---------------//
            ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
            //-------------ADC1 Config---------------//
            ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
            ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));
            ON_SYNC = FAN_ON;
        }

        ESP_LOGI(TAG_adc, "Room-T: %.1f℃, Cell-T: %.1f℃, Current: %.0fmA", troom_filted, tcell_filted, current_filted);
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
       example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    if (do_calibration1_chan1) {
       example_adc_calibration_deinit(adc1_cali_chan1_handle);
    }
}

void app_main(void *)
{
    //-------------Fan PWM Init---------------//
    // Set the LEDC peripheral configuration
    fan_pwm_init();
    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 1);
    ESP_LOGI(TAG, "FAN PWM initialized");
    //-------------GPIO Init---------------//
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_LED_B, 0);
    ESP_LOGI(TAG, "GPIO initialized");

#if EXAMPLE_USE_ADC2
    //-------------ADC2 Init---------------//
    adc_oneshot_unit_handle_t adc2_handle;
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Calibration Init---------------//
    adc_cali_handle_t adc2_cali_handle = NULL;
    bool do_calibration2 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC2_CHAN0, EXAMPLE_ADC_ATTEN, &adc2_cali_handle);

    //-------------ADC2 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN0, &config));
#endif  //#if EXAMPLE_USE_ADC2

    xTaskCreate(temp_read, "tempread_task", 4*1024, NULL, 2, NULL );
    xTaskCreate(adc_regulars, "adc_regular_task", 4*1024, NULL, 2, NULL );
    vTaskDelay(pdMS_TO_TICKS(2000));
    //-------------PCNT Init---------------//
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 6000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
//    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, NULL));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PCNT_GPIO_EDGE,
        .level_gpio_num = PCNT_GPIO_LEVEL,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // hold the counter on rising edge, increase the counter on falling edge
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    static int pulse_count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    ESP_LOGI(TAG, "waiting for Fan stop rotation...");
    while(pulse_count != 0)
    {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    //-------------Fan Self-Test program---------------//
    int DutyTestInv;
    ESP_LOGI(TAG, "Fan Self-testing processing..."); 
    for(DutyTestInv=255; DutyTestInv>(int)(255-FAN_SELFTEST_UPER*255); DutyTestInv--)
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, DutyTestInv));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(GPIO_OUTPUT_LED_B, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(GPIO_OUTPUT_LED_B, 0);

        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));

        if(pulse_count > 12)
        {
            ESP_LOGI(TAG, "Startup duty cycle detected at: %d%%", 100-(int)(0.392151*DutyTestInv)); //(1-DutyTestInv/255)*100
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }
    }
    if(DutyTestInv <= (int)(255-FAN_SELFTEST_UPER*255))
    {
        gpio_set_level(GPIO_OUTPUT_LED_B, 1);
        ESP_LOGE(TAG, "Fan Self-testing fail due to missing FG signal!"); 
    }

    static float idc;
    static float tdc;
    static float duty;
    static float duty_inv;

    #if CONFIG_PM_ENABLE
    // Configure dynamic frequency scaling:
    // maximum and minimum frequencies are set in sdkconfig,
    // automatic light sleep is enabled if tickless idle support is enabled.
    esp_pm_config_t pm_config = {
            .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz = CONFIG_XTAL_FREQ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
            .light_sleep_enable = true
#endif
    };
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );
#endif // CONFIG_PM_ENABLE

    while (1)
    {
        idc = i2duty(current_filted);
        tdc = tt2duty(troom_filted,tcell_filted);
        duty = fusion(tdc,idc);
        duty_inv = 1.0-duty;
        ESP_LOGI(TAG, "T->duty: %.02f,I->duty: %.02f =>Merger: %.02f", tdc, idc, duty);

        if(FAN_ON == true)
        {
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
            vTaskDelay(pdMS_TO_TICKS(2000));
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            if(pulse_count > 12 || duty > FAN_START_DUTY)
            {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (unsigned char)(duty_inv*255));
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                ESP_LOGI(TAG, "Duty: %d%%, RPM=%d", (int)(duty*100), pulse_count*15);   //pulse/2(pole)/2(sec)*60(sec)
            }
            else
            {
                ledc_stop(LEDC_MODE, LEDC_CHANNEL, 1);
                ESP_ERROR_CHECK(pcnt_unit_disable(pcnt_unit));
                ESP_LOGI(TAG, "Low RPM: %d%%, Fan => OFF", (int)(duty*100));
                FAN_ON = false;
                ON_SYNC = false;
            }
        }
        else
        {
            if(duty > FAN_START_DUTY)
            {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (unsigned char)(duty_inv*255));
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                ESP_LOGI(TAG, "Duty: %d%%, Fan => START", (int)(duty*100));
                //pcnt eable and start
                ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
                ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
                FAN_ON = true;
            }
            vTaskDelay(pdMS_TO_TICKS(4000));
        }
//        esp_pm_dump_locks(stdout);

#if EXAMPLE_USE_ADC2
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &adc_raw[1][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN0, adc_raw[1][0]);
        if (do_calibration2) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, adc_raw[1][0], &voltage[1][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_2 + 1, EXAMPLE_ADC2_CHAN0, voltage[1][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
#endif  //#if EXAMPLE_USE_ADC2
    }

#if EXAMPLE_USE_ADC2
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
    if (do_calibration2) {
        example_adc_calibration_deinit(adc2_cali_handle);
    }
#endif //#if EXAMPLE_USE_ADC2
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}