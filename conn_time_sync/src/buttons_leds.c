#include <zephyr/drivers/gpio.h>
#include "conn_time_sync.h"

#define SW1_NODE DT_ALIAS(sw1)
#define SW2_NODE DT_ALIAS(sw2)
#define SW3_NODE DT_ALIAS(sw3)

#define LED0_NODE DT_ALIAS(led0)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

int choose_device_role(void)
{
    if (!device_is_ready(button1.port) || !device_is_ready(button2.port) || !device_is_ready(button3.port)) {
        printk("Button devices are not ready\n");
        return -1;
    }

    if (!device_is_ready(led0.port) || !device_is_ready(led2.port) || !device_is_ready(led3.port)) {
        printk("LED devices are not ready\n");
        return -1;
    }

	gpio_pin_configure_dt(&button1, GPIO_INPUT);
    gpio_pin_configure_dt(&button2, GPIO_INPUT);
    gpio_pin_configure_dt(&button3, GPIO_INPUT);

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);

    printk("Press Button 1 to dump flash data, Button 2 for CENTRAL role, or Button 3 for PERIPHERAL role.\n");

	while (1) {
        if (gpio_pin_get_dt(&button1) == 1) {
        	k_msleep(250);
            timed_i2s_flash_dump();
            led_on(3);
            break;
        }

        if (gpio_pin_get_dt(&button2) == 1) {
        	k_msleep(250);
            printk("Central. Starting scanning\n");
            central_start();
            break;
        }

        if (gpio_pin_get_dt(&button3) == 1) {
        	k_msleep(250);
            printk("Peripheral. Starting advertising\n");
            peripheral_start();
            break;
        }

        k_msleep(50);
    }

    return 0;
}

void led_on(int led)
{
    if (led == 0) {
        gpio_pin_set_dt(&led0, 1);
    } else if (led == 2) {
        gpio_pin_set_dt(&led2, 1);
    } else if (led == 3) {
        gpio_pin_set_dt(&led3, 1);
    }
}

void wait_for_button_press(void)
{
    while (1) {
        if (gpio_pin_get_dt(&button1) == 1) {
        	k_msleep(250);
            break;
        }
        k_msleep(50);
    }
}
