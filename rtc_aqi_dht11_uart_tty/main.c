#include "board.h"
#include "qcc74x_mtimer.h"
#include "qcc74x_i2c.h"
#include "qcc74x_uart.h"
#include "qcc74x_gpio.h"
#include "qcc74x_adc.h"
#include <stdio.h>
#include <string.h>

/* ================= RTC ================= */
#define DS1307_ADDR 0x68
#define DS1307_REG  0x00

/* ================= DHT11 =============== */
#define DHT11_PIN GPIO_PIN_3

/* ================= MQ135 =============== */
#define ADC_CH_NUM ADC_CHANNEL_2   // GPIO2 -> MQ135 AO

struct qcc74x_device_s *uart0;
static struct qcc74x_device_s *i2c0;
static struct qcc74x_device_s *gpio;
static struct qcc74x_device_s *adc;

/* ============== UTIL =================== */
static uint8_t dec_to_bcd(uint8_t v)
{
    return ((v / 10) << 4) | (v % 10);
}

static uint8_t bcd_to_dec(uint8_t v)
{
    return ((v >> 4) * 10) + (v & 0x0F);
}

static void uart_send_string(const char *s)
{
    while (*s) {
        qcc74x_uart_putchar(uart0, *s++);
    }
}

/* ============== UART =================== */
static void uart0_init(void)
{
    board_uartx_gpio_init();
    uart0 = qcc74x_device_get_by_name(DEFAULT_TEST_UART);

    struct qcc74x_uart_config_s cfg = {
        .baudrate = 115200,
        .data_bits = UART_DATA_BITS_8,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_NONE,
        .flow_ctrl = UART_FLOWCTRL_NONE,
        .tx_fifo_threshold = 0,
        .rx_fifo_threshold = 0,
        .bit_order = UART_LSB_FIRST,
    };

    qcc74x_uart_init(uart0, &cfg);
}

/* ============== I2C (RTC) ============== */
static void ds1307_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    struct qcc74x_i2c_msg_s msg[2];

    msg[0].addr = DS1307_ADDR;
    msg[0].flags = I2C_M_NOSTOP;
    msg[0].buffer = &reg;
    msg[0].length = 1;

    msg[1].addr = DS1307_ADDR;
    msg[1].flags = I2C_M_READ;
    msg[1].buffer = buf;
    msg[1].length = len;

    qcc74x_i2c_transfer(i2c0, msg, 2);
}

static void ds1307_set_time(
    uint8_t sec,
    uint8_t min,
    uint8_t hour,
    uint8_t day,
    uint8_t date,
    uint8_t month,
    uint8_t year)
{
    uint8_t data[7];

    data[0] = dec_to_bcd(sec) & 0x7F;   // CH = 0
    data[1] = dec_to_bcd(min);
    data[2] = dec_to_bcd(hour) & 0x3F;        // 24-hour mode
    data[3] = dec_to_bcd(day);         // 1–7
    data[4] = dec_to_bcd(date);
    data[5] = dec_to_bcd(month);
    data[6] = dec_to_bcd(year);        // 00–99

    struct qcc74x_i2c_msg_s msg[2];

    msg[0].addr = DS1307_ADDR;
    msg[0].flags = I2C_M_NOSTOP;
    msg[0].buffer = (uint8_t[]){DS1307_REG};
    msg[0].length = 1;

    msg[1].addr = DS1307_ADDR;
    msg[1].flags = 0;
    msg[1].buffer = data;
    msg[1].length = 7;

    qcc74x_i2c_transfer(i2c0, msg, 2);
}

/* ============== DHT11 ================== */
static void delay_us(uint32_t us)
{
    qcc74x_mtimer_delay_us(us);
}

int dht11_read(uint8_t *temp, uint8_t *hum)
{
    uint8_t data[5] = {0};
    uint32_t timeout;

    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_OUTPUT);
    qcc74x_gpio_reset(gpio, DHT11_PIN);
    qcc74x_mtimer_delay_ms(20);

    qcc74x_gpio_set(gpio, DHT11_PIN);
    delay_us(40);

    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_INPUT);

    timeout = 300;
    while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;

    timeout = 300;
    while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;

    timeout = 300;
    while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;

    for (int i = 0; i < 40; i++) {
        timeout = 300;
        while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);

        delay_us(50);
        if (qcc74x_gpio_read(gpio, DHT11_PIN))
            data[i / 8] |= (1 << (7 - (i % 8)));

        timeout = 300;
        while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    }

    if ((data[0] + data[1] + data[2] + data[3]) != data[4])
        return -1;

    *hum = data[0];
    *temp = data[2];
    return 0;
}

/* ============== ADC (MQ135) ============ */
static void adc_init_mq135(void)
{
    adc = qcc74x_device_get_by_name("adc");

    struct qcc74x_adc_config_s cfg = {
        .clk_div = ADC_CLK_DIV_32,
        .scan_conv_mode = false,
        .continuous_conv_mode = false,
        .differential_mode = false,
        .resolution = ADC_RESOLUTION_16B,
        .vref = ADC_VREF_3P2V,
    };

    struct qcc74x_adc_channel_s chan = {
        .pos_chan = ADC_CH_NUM,
        .neg_chan = ADC_CHANNEL_GND,
    };

    qcc74x_adc_init(adc, &cfg);
    qcc74x_adc_channel_config(adc, &chan, 1);
}

/* ============== MAIN =================== */
int main(void)
{
    uint8_t rtc[7], t, h;
    char msg[128];
    struct qcc74x_adc_result_s res;

    board_init();
    uart0_init();

    board_i2c0_gpio_init();
    i2c0 = qcc74x_device_get_by_name("i2c0");
    qcc74x_i2c_init(i2c0, 100000);

    gpio = qcc74x_device_get_by_name("gpio");
    adc_init_mq135();
#define DS1307_ADDR 0x68

    uart_send_string("CITY : HYDERABAD (INDIA)\r\n");
    uart_send_string("RTC + DHT11 + MQ135 STARTED\r\n");
    uart_send_string("--------------------------------\r\n");

    /* SET RTC TIME (RUN ONCE, THEN COMMENT THIS LINE) */
   ds1307_set_time(
        30,  // sec
        53,  // min
        17,  // hour
        1,   // day (Mon)
        3,   // date
        2,   // month
        26   // year (2026)
    );

    uart_send_string("Warming MQ135 (10s)...\r\n");
    qcc74x_mtimer_delay_ms(10000);

    while (1)
    {
        uart_send_string("CITY : HYDERABAD (INDIA)\r\n");

        /* RTC */
        ds1307_read(DS1307_REG, rtc, 7);
        sprintf(msg, "TIME %02d:%02d:%02d  DATE %02d/%02d/20%02d\r\n",
                bcd_to_dec(rtc[2] & 0x3F),
                bcd_to_dec(rtc[1]),
                bcd_to_dec(rtc[0] & 0x7F),
                bcd_to_dec(rtc[4]),
                bcd_to_dec(rtc[5]),
                bcd_to_dec(rtc[6]));
        uart_send_string(msg);

        /* DHT11 */
        if (dht11_read(&t, &h) == 0)
            sprintf(msg, "TEMP=%d C  HUM=%d %%\r\n", t, h);
        else
            sprintf(msg, "DHT11 ERROR\r\n");
        uart_send_string(msg);

        /* MQ135 */
        qcc74x_adc_start_conversion(adc);
        while (qcc74x_adc_get_count(adc) == 0);
        uint32_t raw = qcc74x_adc_read_raw(adc);
        qcc74x_adc_parse_result(adc, &raw, &res, 1);
        qcc74x_adc_stop_conversion(adc);

        float v = res.millivolt / 1000.0f;
        if (v < 1.0f)
            sprintf(msg, "AQI %.2f V | CLEAN\r\n", v);
        else if (v < 2.0f)
            sprintf(msg, "AQI %.2f V | MODERATE\r\n", v);
        else if (v < 2.8f)
            sprintf(msg, "AQI %.2f V | POOR\r\n", v);
        else
            sprintf(msg, "AQI %.2f V | VERY POOR\r\n", v);

        uart_send_string(msg);
        uart_send_string("--------------------------------\r\n");

        qcc74x_mtimer_delay_ms(1000);
    }
}
