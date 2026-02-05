#include <stdio.h>
#include "qcc74x_mtimer.h"
#include "qcc74x_spi.h"
#include "qcc74x_gpio.h"
#include "qcc74x_uart.h"
#include "board.h"

#define SPI_CS_PIN      GPIO_PIN_12
#define SPI_SCK_PIN     GPIO_PIN_13
#define SPI_MISO_PIN    GPIO_PIN_14
#define SPI_MOSI_PIN    GPIO_PIN_15

static struct qcc74x_device_s *spi0;
static struct qcc74x_device_s *console_uart;
static struct qcc74x_device_s *gpio;

static void console_init(void) {
    board_uartx_gpio_init();
    console_uart = qcc74x_device_get_by_name(DEFAULT_TEST_UART);
    struct qcc74x_uart_config_s cfg = {
        .baudrate = 115200, .data_bits = UART_DATA_BITS_8, .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_NONE, .flow_ctrl = UART_FLOWCTRL_NONE,
        .tx_fifo_threshold = 0, .rx_fifo_threshold = 0, .bit_order = UART_LSB_FIRST
    };
    qcc74x_uart_init(console_uart, &cfg);
}

void spi_slave_init(void) {
    gpio = qcc74x_device_get_by_name("gpio");
    qcc74x_gpio_init(gpio, SPI_CS_PIN,   GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_SCK_PIN,  GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_MISO_PIN, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    qcc74x_gpio_init(gpio, SPI_MOSI_PIN, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    struct qcc74x_spi_config_s spi_cfg = {
        .freq = 1000000, .role = SPI_ROLE_SLAVE, .mode = SPI_MODE3,
        .data_width = SPI_DATA_WIDTH_8BIT, .bit_order = SPI_BIT_MSB, .byte_order = SPI_BYTE_LSB,
        .tx_fifo_threshold = 0, .rx_fifo_threshold = 0,
    };
    spi0 = qcc74x_device_get_by_name("spi0");
    qcc74x_spi_init(spi0, &spi_cfg);
}

int main(void) {
    board_init();
    console_init();
    spi_slave_init();

    printf("\r\n=== SLAVE READY: HYDERABAD STATION ===\r\n");

    uint8_t rx_buf[8]; // 8 Bytes
    uint8_t tx_dummy[8] = {0};

    while (1) {
        // Block until 8 bytes received
        qcc74x_spi_poll_exchange(spi0, tx_dummy, rx_buf, 8);

        if (rx_buf[0] == 0xAA) {
            uint8_t t   = rx_buf[1];
            uint8_t hum = rx_buf[2];
            uint8_t h   = rx_buf[3];
            uint8_t m   = rx_buf[4];
            uint8_t s   = rx_buf[5];
            
            // Reconstruct AQI Voltage
            uint16_t aqi_mv = (rx_buf[6] << 8) | rx_buf[7];
            float aqi_volts = aqi_mv / 1000.0f;

            // Determine Status
            char *status = "CLEAN";
            if (aqi_volts > 1.0f) status = "MODERATE";
            if (aqi_volts > 2.5f) status = "POOR (SMOG)";

            printf("CITY: HYDERABAD | %02d:%02d:%02d | T:%d C H:%d %% | AQI: %.2fV [%s]\r\n", 
                   h, m, s, t, hum, aqi_volts, status);
        } else {
            qcc74x_spi_feature_control(spi0, SPI_CMD_CLEAR_RX_FIFO, 0);
        }
    }
}
