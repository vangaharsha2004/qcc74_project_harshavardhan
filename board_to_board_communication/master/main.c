#include "board.h"
#include "qcc74x_mtimer.h"
#include "qcc74x_i2c.h"
#include "qcc74x_gpio.h"
#include "qcc74x_spi.h"
#include "qcc74x_adc.h"
#include <stdio.h>

// ================= PINS =================
#define SPI_CS_PIN      GPIO_PIN_12
#define SPI_SCK_PIN     GPIO_PIN_13
#define SPI_MISO_PIN    GPIO_PIN_14
#define SPI_MOSI_PIN    GPIO_PIN_15

#define RTC_SCL_PIN     GPIO_PIN_10
#define RTC_SDA_PIN     GPIO_PIN_11
#define DHT11_PIN       GPIO_PIN_3
#define MQ135_PIN       GPIO_PIN_2      

#define DS1307_ADDR     0x68
#define ADC_CH_NUM      ADC_CHANNEL_2

static struct qcc74x_device_s *spi0;
static struct qcc74x_device_s *gpio;
static struct qcc74x_device_s *i2c0;
static struct qcc74x_device_s *adc;

// ================= UTIL =================
static uint8_t bcd_to_dec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }
static uint8_t dec_to_bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }
static void delay_us(uint32_t us) { qcc74x_mtimer_delay_us(us); }

// ================= INIT =================
void system_init(void) {
    gpio = qcc74x_device_get_by_name("gpio");
    
    // SPI
    qcc74x_gpio_init(gpio, SPI_SCK_PIN,  GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_MISO_PIN, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_MOSI_PIN, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_CS_PIN,   GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_set(gpio, SPI_CS_PIN);

    // RTC
    qcc74x_gpio_init(gpio, RTC_SCL_PIN, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, RTC_SDA_PIN, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    // DHT11
    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_set(gpio, DHT11_PIN);

    // MQ135 (ADC Pin)
    qcc74x_gpio_init(gpio, MQ135_PIN, GPIO_ANALOG | GPIO_SMT_EN | GPIO_DRV_0);
}

void spi_init(void) {
    struct qcc74x_spi_config_s spi_cfg = {
        .freq = 500000, .role = SPI_ROLE_MASTER, .mode = SPI_MODE3,
        .data_width = SPI_DATA_WIDTH_8BIT, .bit_order = SPI_BIT_MSB, .byte_order = SPI_BYTE_LSB,
        .tx_fifo_threshold = 0, .rx_fifo_threshold = 0,
    };
    spi0 = qcc74x_device_get_by_name("spi0");
    qcc74x_spi_init(spi0, &spi_cfg);
}

void adc_init_mq135(void) {
    adc = qcc74x_device_get_by_name("adc");
    struct qcc74x_adc_config_s cfg = {
        .clk_div = ADC_CLK_DIV_32, .scan_conv_mode = false, .continuous_conv_mode = false,
        .differential_mode = false, .resolution = ADC_RESOLUTION_16B, .vref = ADC_VREF_3P2V
    };
    struct qcc74x_adc_channel_s chan = { .pos_chan = ADC_CH_NUM, .neg_chan = ADC_CHANNEL_GND };
    qcc74x_adc_init(adc, &cfg);
    qcc74x_adc_channel_config(adc, &chan, 1);
}

// ================= RTC FORCE START =================
void ds1307_force_start(void) {
    uint8_t data[4];
    data[0] = 0x00;              // Register 0
    data[1] = dec_to_bcd(0) & 0x7F; // Seconds=0, CLEAR CH BIT (Start Clock)
    data[2] = dec_to_bcd(30);    // Min=30
    data[3] = dec_to_bcd(15);    // Hour=15

    struct qcc74x_i2c_msg_s msg;
    msg.addr = DS1307_ADDR;
    msg.flags = 0;
    msg.buffer = data;
    msg.length = 4;

    qcc74x_i2c_transfer(i2c0, &msg, 1);
    printf(">> OSCILLATOR KICKSTARTED! <<\r\n");
}

// ================= DHT11 =================
int dht11_read(uint8_t *temp, uint8_t *hum) {
    uint8_t data[5] = {0};
    uint32_t timeout;
    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_OUTPUT);
    qcc74x_gpio_reset(gpio, DHT11_PIN);
    qcc74x_mtimer_delay_ms(20);
    qcc74x_gpio_set(gpio, DHT11_PIN);
    delay_us(40);
    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_INPUT);
    
    timeout = 500; while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;
    timeout = 500; while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;
    timeout = 500; while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    if (!timeout) return -1;
    
    for (int i = 0; i < 40; i++) {
        timeout = 500; while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
        delay_us(50);
        if (qcc74x_gpio_read(gpio, DHT11_PIN)) data[i / 8] |= (1 << (7 - (i % 8)));
        timeout = 500; while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--) delay_us(1);
    }
    if ((data[0] + data[1] + data[2] + data[3]) != data[4]) return -2;
    *hum = data[0]; *temp = data[2];
    return 0;
}

// ================= MAIN =================
int main(void) {
    board_init();
    system_init();
    i2c0 = qcc74x_device_get_by_name("i2c0");
    qcc74x_i2c_init(i2c0, 100000);
    adc_init_mq135();
    spi_init();

    // Force start the clock on boot
    ds1307_force_start();
    qcc74x_mtimer_delay_ms(100);

    uint8_t h=0, m=0, s=0, t=0, hum=0;
    struct qcc74x_adc_result_s res;
    uint8_t tx_buf[8], rx_dummy[8];
    struct qcc74x_i2c_msg_s msg[2];
    uint8_t reg = 0x00, i2c_buf[3];
    int stuck_counter = 0;

    while (1) {
        // --- 1. RTC READ ---
        msg[0].addr = DS1307_ADDR; msg[0].flags = I2C_M_NOSTOP; msg[0].buffer = &reg; msg[0].length = 1;
        msg[1].addr = DS1307_ADDR; msg[1].flags = I2C_M_READ;   msg[1].buffer = i2c_buf; msg[1].length = 3;
        
        if (qcc74x_i2c_transfer(i2c0, msg, 2) == 0) {
            s = bcd_to_dec(i2c_buf[0] & 0x7F);
            m = bcd_to_dec(i2c_buf[1]);
            h = bcd_to_dec(i2c_buf[2] & 0x3F);
        }

        // --- WATCHDOG FOR STUCK CLOCK ---
        // If time is 00:00:00 for more than 5 seconds, Force Reset again
        if (h==0 && m==0 && s==0) {
            stuck_counter++;
            if(stuck_counter > 5) {
                printf(">> DETECTED STUCK CLOCK! KICKING... <<\r\n");
                ds1307_force_start();
                stuck_counter = 0;
            }
        } else {
            stuck_counter = 0; // Clock is moving, reset counter
        }

        // --- 2. SENSORS ---
        if(dht11_read(&t, &hum) != 0) { /* Keep old values */ }
        
        // ADC READ (MQ135)
        qcc74x_adc_start_conversion(adc);
        while (qcc74x_adc_get_count(adc) == 0);
        uint32_t raw = qcc74x_adc_read_raw(adc);
        qcc74x_adc_parse_result(adc, &raw, &res, 1);
        qcc74x_adc_stop_conversion(adc);
        uint16_t voltage_mv = res.millivolt;

        // --- 3. SEND SPI ---
        tx_buf[0] = 0xAA;
        tx_buf[1] = t; tx_buf[2] = hum;
        tx_buf[3] = h; tx_buf[4] = m; tx_buf[5] = s;
        tx_buf[6] = (voltage_mv >> 8) & 0xFF;
        tx_buf[7] = voltage_mv & 0xFF;

        qcc74x_gpio_reset(gpio, SPI_CS_PIN);
        qcc74x_spi_poll_exchange(spi0, tx_buf, rx_dummy, 8);
        qcc74x_gpio_set(gpio, SPI_CS_PIN);

        printf("HYDERABAD MONITOR | Time: %02d:%02d:%02d | T:%d H:%d AQI:%d\r\n", h, m, s, t, hum, voltage_mv);
        qcc74x_mtimer_delay_ms(1000);
    }
}
