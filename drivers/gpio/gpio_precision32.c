/*
 * Copyright (c) 2017, Reto Schneider <code@reto-schneider.ch>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <gpio.h>
#include <soc.h>
#include <SI32_PBSTD_A_Type.h>

#include "gpio_utils.h"

enum { NUMBER_OF_PORTS = 4 };

struct gpio_precision32_common_config {
};

struct gpio_precision32_common_data {
	/* a list of all ports */
	struct device *ports[NUMBER_OF_PORTS];
	size_t count;
};

struct gpio_precision32_config {
};

struct gpio_precision32_data {
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* pin callback routine enable flags, by pin number */
	u32_t pin_callback_enables;
};

static inline void gpio_precision32_add_port(struct gpio_precision32_common_data *data,
				       struct device *dev)
{
	__ASSERT(dev, "No port device!");
	data->ports[data->count++] = dev;
}


static int gpio_precision32_configure(struct device *dev,
				int access_op, u32_t pin, int flags)
{
    (void)dev;
    (void)access_op;
    (void)pin;
    (void)flags;

	return 0;
}

static int gpio_precision32_write(struct device *dev,
			    int access_op, u32_t pin, u32_t value)
{
    (void)dev;
    (void)access_op;
    (void)pin;
    (void)value;

	return 0;
}

static int gpio_precision32_read(struct device *dev,
			   int access_op, u32_t pin, u32_t *value)
{
    (void)dev;
    (void)access_op;
    (void)pin;
    (void)value;

	return 0;
}

static int gpio_precision32_manage_callback(struct device *dev,
				      struct gpio_callback *callback, bool set)
{
	struct gpio_precision32_data *data = dev->driver_data;

	_gpio_manage_callback(&data->callbacks, callback, set);

	return 0;
}

static int gpio_precision32_enable_callback(struct device *dev,
				      int access_op, u32_t pin)
{
    (void)dev;
    (void)access_op;
    (void)pin;

	return 0;
}

static int gpio_precision32_disable_callback(struct device *dev,
				       int access_op, u32_t pin)
{
    (void)dev;
    (void)access_op;
    (void)pin;

	return 0;
}


static const struct gpio_driver_api gpio_precision32_driver_api = {
	.config = gpio_precision32_configure,
	.write = gpio_precision32_write,
	.read = gpio_precision32_read,
	.manage_callback = gpio_precision32_manage_callback,
	.enable_callback = gpio_precision32_enable_callback,
	.disable_callback = gpio_precision32_disable_callback,
};

static const struct gpio_driver_api gpio_precision32_common_driver_api = {
	.manage_callback = gpio_precision32_manage_callback,
	.enable_callback = gpio_precision32_enable_callback,
	.disable_callback = gpio_precision32_disable_callback,
};

#ifdef CONFIG_GPIO_PRECISION32
static int gpio_precision32_common_init(struct device *dev);

static const struct gpio_precision32_common_config gpio_precision32_common_config = {
};

static struct gpio_precision32_common_data gpio_precision32_common_data;

DEVICE_AND_API_INIT(gpio_precision32_common, CONFIG_GPIO_PRECISION32_COMMON_NAME,
		    gpio_precision32_common_init,
		    &gpio_precision32_common_data, &gpio_precision32_common_config,
		    POST_KERNEL, CONFIG_GPIO_PRECISION32_COMMON_INIT_PRIORITY,
		    &gpio_precision32_common_driver_api);

static int gpio_precision32_common_init(struct device *dev)
{
    (void)dev;

	return 0;
}
#endif /* CONFIG_GPIO_PRECISION32 */


#ifdef CONFIG_GPIO_PRECISION32_PORTBANK1ANK0
static int gpio_precision32_porta_init(struct device *dev);

static const struct gpio_precision32_config gpio_precision32_porta_config = {
	.gpio_base = &GPIO->P[gpioPortA],
	.gpio_index = gpioPortA,
};

static struct gpio_precision32_data gpio_precision32_porta_data;

DEVICE_AND_API_INIT(gpio_precision32_porta, CONFIG_GPIO_PRECISION32_PORTBANK1ANK0_NAME,
		    gpio_precision32_porta_init,
		    &gpio_precision32_porta_data, &gpio_precision32_porta_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_precision32_driver_api);

static int gpio_precision32_porta_init(struct device *dev)
{
	gpio_precision32_add_port(&gpio_precision32_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_PRECISION32_PORTBANK1ANK0 */

#ifdef CONFIG_GPIO_PRECISION32_PORTBANK1
static int gpio_precision32_portb_init(struct device *dev);

static const struct gpio_precision32_config gpio_precision32_portb_config = {
};

static struct gpio_precision32_data gpio_precision32_portb_data;

DEVICE_AND_API_INIT(gpio_precision32_portb, CONFIG_GPIO_PRECISION32_PORTBANK1_NAME,
		    gpio_precision32_portb_init,
		    &gpio_precision32_portb_data, &gpio_precision32_portb_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_precision32_driver_api);

static int gpio_precision32_portb_init(struct device *dev)
{
	gpio_precision32_add_port(&gpio_precision32_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_PRECISION32_PORTBANK1 */

#ifdef CONFIG_GPIO_PRECISION32_PORTBANK2
static int gpio_precision32_portc_init(struct device *dev);

static const struct gpio_precision32_config gpio_precision32_portc_config = {
};

static struct gpio_precision32_data gpio_precision32_portc_data;

DEVICE_AND_API_INIT(gpio_precision32_portc, CONFIG_GPIO_PRECISION32_PORTBANK2_NAME,
		    gpio_precision32_portc_init,
		    &gpio_precision32_portc_data, &gpio_precision32_portc_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_precision32_driver_api);

static int gpio_precision32_portc_init(struct device *dev)
{
	gpio_precision32_add_port(&gpio_precision32_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_PRECISION32_PORTBANK2 */

#ifdef CONFIG_GPIO_PRECISION32_PORTBANK3
static int gpio_precision32_portd_init(struct device *dev);

static const struct gpio_precision32_config gpio_precision32_portd_config = {
};

static struct gpio_precision32_data gpio_precision32_portd_data;

DEVICE_AND_API_INIT(gpio_precision32_portd, CONFIG_GPIO_PRECISION32_PORTBANK3_NAME,
		    gpio_precision32_portd_init,
		    &gpio_precision32_portd_data, &gpio_precision32_portd_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_precision32_driver_api);

static int gpio_precision32_portd_init(struct device *dev)
{
	gpio_precision32_add_port(&gpio_precision32_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_PRECISION32_PORTBANK3 */

#ifdef CONFIG_GPIO_PRECISION32_PORTBANK4
static int gpio_precision32_porte_init(struct device *dev);

static const struct gpio_precision32_config gpio_precision32_porte_config = {
};

static struct gpio_precision32_data gpio_precision32_porte_data;

DEVICE_AND_API_INIT(gpio_precision32_porte, CONFIG_GPIO_PRECISION32_PORTBANK4_NAME,
		    gpio_precision32_porte_init,
		    &gpio_precision32_porte_data, &gpio_precision32_porte_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &gpio_precision32_driver_api);

static int gpio_precision32_porte_init(struct device *dev)
{
	gpio_precision32_add_port(&gpio_precision32_common_data, dev);
	return 0;
}
#endif /* CONFIG_GPIO_PRECISION32_PORTBANK4 */
