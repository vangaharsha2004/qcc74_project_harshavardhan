#include "qcc74x_mtimer.h"
#include "qcc74x_i2c.h"
#include "board.h"
#include <stdio.h>



#define DS1307_ADDR     0x68
#define DS1307_REG_TIME 0x00

static struct qcc74x_device_s *i2c0;

/* ---------- BCD helpers ---------- */
static uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

/* ---------- I2C Write ---------- */
static void ds1307_write(uint8_t reg, uint8_t *data, uint8_t len)
{
    struct qcc74x_i2c_msg_s msgs[2];

    msgs[0].addr   = DS1307_ADDR;
    msgs[0].flags  = I2C_M_NOSTOP;
    msgs[0].buffer = &reg;
    msgs[0].length = 1;

    msgs[1].addr   = DS1307_ADDR;
    msgs[1].flags  = 0;
    msgs[1].buffer = data;
    msgs[1].length = len;

    qcc74x_i2c_transfer(i2c0, msgs, 2);
}

/* ---------- I2C Read ---------- */
static void ds1307_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    struct qcc74x_i2c_msg_s msgs[2];

    msgs[0].addr   = DS1307_ADDR;
    msgs[0].flags  = I2C_M_NOSTOP;
    msgs[0].buffer = &reg;
    msgs[0].length = 1;

    msgs[1].addr   = DS1307_ADDR;
    msgs[1].flags  = I2C_M_READ;
    msgs[1].buffer = data;
    msgs[1].length = len;

    qcc74x_i2c_transfer(i2c0, msgs, 2);
}


/* Day: 1=Mon 2=Tue 3=Wed 4=Thu 5=Fri 6=Sat 7=Sun */
static void ds1307_set_time(void)
{
    uint8_t t[7];

    t[0] = dec_to_bcd(40) & 0x7F;   // Seconds
    t[1] = dec_to_bcd(25);          // Minutes
    t[2] = dec_to_bcd(16) & 0x3F;   // Hours (24h)
    t[3] = dec_to_bcd(3);           // Wednesday
    t[4] = dec_to_bcd(28);          // Date
    t[5] = dec_to_bcd(1);           // Month
    t[6] = dec_to_bcd(26);          // Year (2026)

    ds1307_write(DS1307_REG_TIME, t, 7);
}

/* ---------- READ & PRINT ---------- */
static void ds1307_get_time(void)
{
    uint8_t b[7];

    ds1307_read(DS1307_REG_TIME, b, 7);

    printf("CITY : HYDERABAD\r\n");
    printf("Time : %02d:%02d:%02d\r\n",
           bcd_to_dec(b[2] & 0x3F),
           bcd_to_dec(b[1]),
           bcd_to_dec(b[0] & 0x7F));

    printf("Date : %02d/%02d/20%02d\r\n",
           bcd_to_dec(b[4]),
           bcd_to_dec(b[5]),
           bcd_to_dec(b[6]));

    printf("----------------------------\r\n");
}

int main(void)
{
    board_init();
    board_i2c0_gpio_init();

    i2c0 = qcc74x_device_get_by_name("i2c0");
    qcc74x_i2c_init(i2c0, 100000);

    printf("RTC DISPLAY START\r\n");

    ds1307_set_time();   // once run this and put comment

    while (1)
    {
        ds1307_get_time();
        qcc74x_mtimer_delay_ms(1000);
    }
}
