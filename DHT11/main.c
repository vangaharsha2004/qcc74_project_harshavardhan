#include <stdio.h>
#include "board.h"
#include "qcc74x_gpio.h"
#include "qcc74x_mtimer.h"

#define DHT11_PIN GPIO_PIN_3

struct qcc74x_device_s *gpio;

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

    qcc74x_gpio_init(gpio, DHT11_PIN, GPIO_INPUT | GPIO_PULLUP);

    timeout = 1000;
    while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--)
        delay_us(1);
    if (!timeout) return -1;

    timeout = 1000;
    while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--)
        delay_us(1);
    if (!timeout) return -1;

    timeout = 1000;
    while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--)
        delay_us(1);
    if (!timeout) return -1;

    for (int i = 0; i < 40; i++)
    {
        timeout = 1000;
        while (!qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--)
            delay_us(1);

        delay_us(50);

        if (qcc74x_gpio_read(gpio, DHT11_PIN))
            data[i / 8] |= (1 << (7 - (i % 8)));

        timeout = 1000;
        while (qcc74x_gpio_read(gpio, DHT11_PIN) && timeout--)
            delay_us(1);
    }

    if ((data[0] + data[1] + data[2] + data[3]) != data[4])
        return -1;

    *hum  = data[0];
    *temp = data[2];
    return 0;
}

int main(void)
{
    uint8_t t, h;

    board_init();
    gpio = qcc74x_device_get_by_name("gpio");

    while (1)
    {
        if (dht11_read(&t, &h) == 0)
            printf("TEMP=%d C  HUM=%d %%\r\n", t, h);
        else
            printf("DHT11 ERROR\r\n");

        fflush(stdout);
        qcc74x_mtimer_delay_ms(3000);
    }
}:
