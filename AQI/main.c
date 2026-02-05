#include "qcc74x_adc.h"
#include "qcc74x_mtimer.h"
#include "board.h"
#include <stdio.h>

#define ADC_CH_NUM     ADC_CHANNEL_2   // GPIO2 -> MQ135 AO

struct qcc74x_device_s *adc;

int main(void)
{
    board_init();

    printf("\r\n=== MQ135 Air Quality Monitor ===\r\n");

    adc = qcc74x_device_get_by_name("adc");
    if (!adc)
    {
        printf("ADC device not found!\r\n");
        while (1);
    }

    /* -------- ADC configuration (from Qualcomm example) -------- */
    struct qcc74x_adc_config_s cfg;
    cfg.clk_div = ADC_CLK_DIV_32;
    cfg.scan_conv_mode = false;
    cfg.continuous_conv_mode = false;
    cfg.differential_mode = false;
    cfg.resolution = ADC_RESOLUTION_16B;
    cfg.vref = ADC_VREF_3P2V;

    struct qcc74x_adc_channel_s chan;
    chan.pos_chan = ADC_CH_NUM;
    chan.neg_chan = ADC_CHANNEL_GND;

    qcc74x_adc_init(adc, &cfg);
    qcc74x_adc_channel_config(adc, &chan, 1);

    printf("ADC initialized\r\n");
    printf("Warming up MQ135 (10 seconds)...\r\n");

    qcc74x_mtimer_delay_ms(10000);

    struct qcc74x_adc_result_s result;

    while (1)
    {
        qcc74x_adc_start_conversion(adc);

        while (qcc74x_adc_get_count(adc) == 0)
        {
            qcc74x_mtimer_delay_ms(1);
        }

        uint32_t raw_data = qcc74x_adc_read_raw(adc);
        qcc74x_adc_parse_result(adc, &raw_data, &result, 1);
        qcc74x_adc_stop_conversion(adc);

        float voltage = result.millivolt / 1000.0f;

        /* -------- Air Quality Decision -------- */
        if (voltage < 1.0f)
        {
            printf("Voltage: %.2f V  | Air Quality: CLEAN\r\n", voltage);
        }
        else if (voltage < 2.0f)
        {
            printf("Voltage: %.2f V  | Air Quality: MODERATE\r\n", voltage);
        }
        else if (voltage < 2.8f)
        {
            printf("Voltage: %.2f V  | Air Quality: POOR\r\n", voltage);
        }
        else
        {
            printf("Voltage: %.2f V  | Air Quality: VERY POOR\r\n", voltage);
        }

        qcc74x_mtimer_delay_ms(1000);
    }
}
