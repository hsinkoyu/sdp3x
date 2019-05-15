/*
 * Sensirion SDP3x-Digital - Digital Differential Pressure Sensor
 *
 * Copyright (C) 2019 FIH Mobile Limited
 *
 * Author: Hsinko Yu <hsinkoyu@fih-foxconn.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>

enum sdp3x_mode {
	IDLE,
	MEASURING,
	SLEEP,
};

enum sdp3x_cmd {
	INVALID = -1,

	START_CONTINUOUS_MEASUREMENT,
	STOP_CONTINUOUS_MEASUREMENT,
	RESET_SENSOR,
	ENTER_SLEEP_MODE,
	EXIT_SLEEP_MODE,
	READ_PRODUCT_ID,
};

struct sdp3x_data {
	struct i2c_client *client;
	enum sdp3x_mode mode;
	enum sdp3x_cmd last_cmd;
	struct gpio_desc *irq_gpio;
	int irq;
	bool new_measurement; /* new measurement data is available */
	struct hrtimer sampling_timer;
	unsigned int sampling_period_us;
	/*
	 * sdp3x_recv_data() might sleep by i2c_transfer_lock, so we must not poll
	 * measurements in hrtimer_interrupt context. A workqueue takes the place
	 * of polling measurements.
	 */
	struct workqueue_struct *workqueue;
	struct work_struct polling_work;
	short dp; /* differential pressure raw data */
	short temp; /* temperature raw data */
	short scale_factor; /* scale factor differential pressure raw data */
};

#define sdp3x_i2c_write(client, buf, count) i2c_master_send(client, buf, count)
#define sdp3x_i2c_read(client, buf, count) i2c_master_recv(client, buf, count)

#define SDP3X_CRC8_POLYNOMIAL 0x31
#define SDP3X_CRC8_INIT       0xFF

DECLARE_CRC8_TABLE(sdp3x_crc8_table);

/* this lock is used to make this driver thread safe */
static DEFINE_MUTEX(i2c_transfer_lock);
/* this lock prevents data from reading and writing at the same time */
static DEFINE_MUTEX(sysfs_lock);

/**
 * sdp3x_recv_data -
 *
 * Returns negative errno, or else the number of bytes read.
 */
static int sdp3x_recv_data(struct i2c_client *client, char *data, int size)
{
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);
	int ret = -EPERM;
	int i;

	mutex_lock(&i2c_transfer_lock);

	if (drv_data->mode == MEASURING) {
		ret = sdp3x_i2c_read(client, data, size);
	} else if (drv_data->last_cmd == READ_PRODUCT_ID) {
		ret = sdp3x_i2c_read(client, data, size);
		drv_data->last_cmd = INVALID;
	}

	if (ret == size) {
		/* CRC-8 checksum */
		for (i = 2; i < size; i += 3) {
			if (data[i] != crc8(sdp3x_crc8_table, data + (i - 2), 2,
				SDP3X_CRC8_INIT)) {
				pr_err("E: checksum error\n");
				ret = -EINVAL;
				break;
			}
		}
	} else if (ret >= 0) {
		pr_err("E: expected to read %d bytes, but %d bytes returned\n", size,
			ret);
		ret = -EINVAL;
	}

	if (ret < 0)
		pr_err("failed receiving data from the sensor, err=%d\n", ret);

	mutex_unlock(&i2c_transfer_lock);

	return ret;
}

static int sdp3x_send_cmd(struct i2c_client *client, enum sdp3x_cmd cmd)
{
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);
	char buf[2];
	int ret;

	mutex_lock(&i2c_transfer_lock);

	switch (cmd) {
	/*
	 * Continuous measurements can be started up in different configurations by
	 * a set of commands.
	 *
	 * Command code (Hex) | Temperature compensation | Averaging
	 * ------------------------------------------------------------------------
	 * 0x3603               Mass flow                  Average till read
	 * 0x3608               Mass flow                  None - Update rate 0.5ms
	 * 0x3615               Differential pressure      Average till read
	 * 0x361E               Differential pressure      None - Update rate 0.5ms
	 *
	 * For simplicity, we force to use 0x3615 as vendor suggested.
	 */
	case START_CONTINUOUS_MEASUREMENT:
		if (drv_data->mode == IDLE) {
			buf[0] = 0x36;
			buf[1] = 0x15;
			ret = sdp3x_i2c_write(client, buf, 2);
			if (ret < 0) {
				pr_err("[START_CONTINUOUS_MEASUREMENT] failed sending command,"
					" err=%d\n", ret);
			} else {
				pr_info("[START_CONTINUOUS_MEASUREMENT] succeeded\n");
				drv_data->mode = MEASURING;
				hrtimer_start(&drv_data->sampling_timer,
					ns_to_ktime(drv_data->sampling_period_us * 1000L),
					HRTIMER_MODE_REL);
			}
		} else {
			pr_err("[START_CONTINUOUS_MEASUREMENT] E: the sensor is not in "
				"idle mode\n");
			ret = -EPERM;
		}
		break;
	case STOP_CONTINUOUS_MEASUREMENT:
		if (drv_data->mode == MEASURING) {
			buf[0] = 0x3f;
			buf[1] = 0xf9;
			ret = sdp3x_i2c_write(client, buf, 2);
			if (ret < 0) {
				pr_err("[STOP_CONTINUOUS_MEASUREMENT] failed sending command,"
					" err=%d\n", ret);
			} else {
				pr_info("[STOP_CONTINUOUS_MEASUREMENT] succeeded\n");
				drv_data->mode = IDLE;
				/*
				 * the sensor will be receptive for another command after
				 * 500us.
				 */
				udelay(500);
			}
		} else {
			pr_err("[STOP_CONTINUOUS_MEASUREMENT] E: the sensor is not in "
				"measuring mode\n");
			ret = -EPERM;
		}
		break;
	case RESET_SENSOR:
		/* the sensor cannot be soft reset in sleep mode */
		if (drv_data->mode != SLEEP) {
			buf[0] = 0x00;
			buf[1] = 0x06;
			ret = sdp3x_i2c_write(client, buf, 2);
			if (ret < 0) {
				pr_err("[RESET_SENSOR] failed sending command, err=%d\n", ret);
			} else {
				pr_info("[RESET_SENSOR] succeeded\n");
				drv_data->mode = IDLE;
				/* the sensor will take maximum 20ms to reset */
				mdelay(20);
			}
		} else {
			pr_err("[RESET_SENSOR] E: the sensor is in sleep mode\n");
			ret = -EPERM;
		}
		break;
	case ENTER_SLEEP_MODE:
		/* can only be entered from idle mode */
		if (drv_data->mode == IDLE) {
			buf[0] = 0x36;
			buf[1] = 0x77;
			ret = sdp3x_i2c_write(client, buf, 2);
			if (ret < 0) {
				pr_err("[ENTER_SLEEP_MODE] failed sending command, err=%d\n",
					ret);
			} else {
				pr_info("[ENTER_SLEEP_MODE] succeeded\n");
				drv_data->mode = SLEEP;
			}
		} else {
			pr_err("[ENTER_SLEEP_MODE] E: the sensor is not in idle mode\n");
			ret = -EPERM;
		}
		break;
	case EXIT_SLEEP_MODE:
		if (drv_data->mode == SLEEP) {
			sdp3x_i2c_write(client, NULL, 0); /* a write bit to wake it up */
			udelay(2000); /* take maximum 2ms to idle mode */
			/* to poll the sensor to see whether it has woken up */
			ret = sdp3x_i2c_write(client, NULL, 0);
			if (ret < 0) {
				pr_err("[EXIT_SLEEP_MODE] failed waking up the sensor, "
					"err=%d\n", ret);
			} else {
				pr_info("[EXIT_SLEEP_MODE] succeeded\n");
				drv_data->mode = IDLE;
			}
		} else {
			pr_err("[EXIT_SLEEP_MODE] E: the sensor is not in sleep mode\n");
			ret = -EPERM;
		}
		break;
	case READ_PRODUCT_ID:
		if (drv_data->mode == IDLE) {
			buf[0] = 0x36;
			buf[1] = 0x7c;
			ret = sdp3x_i2c_write(client, buf, 2);
			if (ret < 0) {
				pr_err("[READ_PRODUCT_ID] failed sending command(1/2), "
					"err=%d\n", ret);
				break;
			}
			buf[0] = 0xe1;
			buf[1] = 0x02;
			ret = sdp3x_i2c_write(client, buf, 2);
			if (ret < 0) {
				pr_err("[READ_PRODUCT_ID] failed sending command(2/2), "
					"err=%d\n", ret);
			}
		} else {
			pr_err("[READ_PRODUCT_ID] E: the sensor is not in idle mode\n");
			ret = -EPERM;
		}
		break;
	default:
		pr_err("E: unknown command: %d\n", cmd);
		ret = -EINVAL;
		break;
	}

	drv_data->last_cmd = ret >= 0 ? cmd : INVALID;

	mutex_unlock(&i2c_transfer_lock);

	return ret;
}

static ssize_t sdp3x_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	char c = 0;
	int cmd = -1;

	if (buf != NULL) {
		sscanf(buf, "%c", &c);
		if (c >= '0' && c <= '9')
			cmd = c - '0';
		else if (c >= 'a' && c <= 'z')
			cmd = c - 'a' + 10;
	}

	sdp3x_send_cmd(client, cmd);

	return count;
}

static ssize_t sdp3x_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);
	char *s;

	switch (drv_data->mode) {
	case IDLE:
		s = "Idle";
		break;
	case MEASURING:
		s = "Measuring";
		break;
	case SLEEP:
		s = "Sleep";
		break;
	default:
		s = "";
		break;
	}

	return sprintf(buf, "%s\n", s);
}

static ssize_t sdp3x_productid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);
	char data[18];
	int size = 0;
	int i;
	int buf_size = 0;

	if (drv_data->mode == IDLE && drv_data->last_cmd == READ_PRODUCT_ID) {
		size = sdp3x_recv_data(client, data, sizeof(data));
	}

	if (size > 0) {
		buf_size = sprintf(buf, "0x");
		for (i = 0; i < size; i++) {
			/* bypass CRC byte */
			if ((i + 1) % 3) {
				buf_size += sprintf(buf + buf_size, "%02x", data[i]);
			}
		}
		buf_size += sprintf(buf + buf_size, "\n");
	} else {
		*buf = '\0';
	}

	return buf_size;
}

static ssize_t sdp3x_sampling_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", drv_data->sampling_period_us);
}

static ssize_t sdp3x_sampling_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);
	unsigned int period;

	if (1 != sscanf(buf, "%d", &period))
		return 0;

	if (drv_data->mode == MEASURING) {
		hrtimer_cancel(&drv_data->sampling_timer);
		drv_data->sampling_period_us = period;
		hrtimer_start(&drv_data->sampling_timer,
			ns_to_ktime(drv_data->sampling_period_us * 1000L),
			HRTIMER_MODE_REL);
	} else {
		drv_data->sampling_period_us = period;
	}

	pr_info("sampling period is changed to %dus\n", period);

	return count;
}

static ssize_t sdp3x_measurement_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);
	int ret;

	if (drv_data->mode == MEASURING) {
		mutex_lock(&sysfs_lock);
		ret = sprintf(buf, "%d %d %d\n", drv_data->dp, drv_data->temp,
			drv_data->scale_factor);
		mutex_unlock(&sysfs_lock);
	} else {
		*buf = '\0';
		ret = 0;
	}

	return ret;
}

static struct device_attribute attrs[] =
{
	__ATTR(cmd, 0200, NULL, sdp3x_cmd_store),
	__ATTR(mode, 0444, sdp3x_mode_show, NULL),
	__ATTR(productid, 0444, sdp3x_productid_show, NULL),
	__ATTR(sampling_period_us, 0644, sdp3x_sampling_show,
		sdp3x_sampling_store),
	__ATTR(measurement, 0444, sdp3x_measurement_show, NULL),
};

static int sdp3x_create_sysfs_attr(struct i2c_client *client)
{
	int i;
	int error;

	for (i = 0; i < sizeof(attrs) / sizeof(attrs[0]); i++) {
		error = device_create_file(&client->dev, &attrs[i]);
		if (error) {
			pr_err("failed creating device attribute files, err=%d\n", error);
			break;
		}
	}

	if (i < sizeof(attrs) / sizeof(attrs[0])) {
		for (i--; i >= 0; i--)
			device_remove_file(&client->dev, &attrs[i]);
	}

	return error;
}

static void sdp3x_remove_sysfs_attr(struct i2c_client *client)
{
	int i;

	for (i = 0; i < sizeof(attrs) / sizeof(attrs[0]); i++)
		device_remove_file(&client->dev, &attrs[i]);
}

static irqreturn_t new_measurement_handler(int irq, void *p)
{
	struct sdp3x_data *drv_data = i2c_get_clientdata((struct i2c_client *)p);

	if (gpiod_get_value(drv_data->irq_gpio))
		drv_data->new_measurement = false;
	else
		drv_data->new_measurement = true;

	pr_info("is new measurement data avaliable? %s\n",
		drv_data->new_measurement ? "yes" : "no");

	return IRQ_HANDLED;
}

static void sdp3x_polling_worker(struct work_struct *work) {
	struct sdp3x_data *drv_data = container_of(work, struct sdp3x_data,
		polling_work);
	char data[9];

	if (sdp3x_recv_data(drv_data->client, data, sizeof(data)) > 0) {
		mutex_lock(&sysfs_lock);
		drv_data->dp = (data[0] << 8) | data[1];
		drv_data->temp = (data[3] << 8) | data[4];
		drv_data->scale_factor = (data[6] << 8) | data[7];
		mutex_unlock(&sysfs_lock);
	}
}

static enum hrtimer_restart update_measurement(struct hrtimer *timer)
{
	struct sdp3x_data *drv_data = container_of(timer, struct sdp3x_data,
		sampling_timer);

	if (drv_data->mode == MEASURING) {
		queue_work(drv_data->workqueue, &drv_data->polling_work);
		hrtimer_forward_now(timer,
			ns_to_ktime(drv_data->sampling_period_us * 1000L));
		return HRTIMER_RESTART;
	} else {
		/* measuring has been stopped by stop/reset command */
		pr_info("measuring has been stopped\n");
		return HRTIMER_NORESTART;
	}
}

static int sdp3x_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device_node *dtn = client->dev.of_node;
	struct sdp3x_data *drv_data;
	int error;

	pr_info("probed i2c client '%s', functionality=0x%x\n", client->name,
		i2c_get_functionality(client->adapter));

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	drv_data = devm_kzalloc(&client->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data) {
		pr_err("failed memory allocation\n");
		return -ENOMEM;
	}

	drv_data->mode = IDLE;
	drv_data->last_cmd = INVALID;
	drv_data->sampling_period_us = 10000;
	of_property_read_u32(dtn, "sampling-period-us",
		&drv_data->sampling_period_us);
	hrtimer_init(&drv_data->sampling_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv_data->sampling_timer.function = update_measurement;

	crc8_populate_msb(sdp3x_crc8_table, SDP3X_CRC8_POLYNOMIAL);

	drv_data->client = client;
	i2c_set_clientdata(client, drv_data);

	drv_data->irq_gpio = devm_gpiod_get_optional(&client->dev, "irq",
		GPIOD_IN);
	if (IS_ERR(drv_data->irq_gpio)) {
		pr_info("keep IRQn pin floating when not used\n");
	} else {
		int irq = gpiod_to_irq(drv_data->irq_gpio);

		if (irq < 0) {
			pr_err("failed retrieving irq corresponding to gpio-%d, err=%d\n",
				desc_to_gpio(drv_data->irq_gpio), irq);
			return irq;
		}

		error = devm_request_any_context_irq(&client->dev, irq, new_measurement_handler,
			IRQ_TYPE_EDGE_BOTH, dev_name(&client->dev), client);
		if (error < 0) {
			pr_err("failed claiming irq for gpio-%d, err=%d\n",
				desc_to_gpio(drv_data->irq_gpio), error);
			return error;
		}

		drv_data->irq = irq;

		/* unused for now, so disable it */
		disable_irq(drv_data->irq);
	}

	drv_data->workqueue = create_singlethread_workqueue("workqueue");
	if (drv_data->workqueue == NULL) {
		pr_err("failed creating the workqueue\n");
		error = -ENOMEM;
		goto err_workqueue;
	}

	INIT_WORK(&drv_data->polling_work, sdp3x_polling_worker);

	error = sdp3x_create_sysfs_attr(client);
	if (error) {
		pr_err("failed creating sysfs device attributes, err=%d\n", error);
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	destroy_workqueue(drv_data->workqueue);
err_workqueue:

	return error;
}

static int sdp3x_remove(struct i2c_client *client)
{
	struct sdp3x_data *drv_data = i2c_get_clientdata(client);

	/* stay in idle mode */
	if (drv_data->mode == MEASURING)
		sdp3x_send_cmd(client, STOP_CONTINUOUS_MEASUREMENT);
	else if (drv_data->mode == SLEEP)
		sdp3x_send_cmd(client, EXIT_SLEEP_MODE);

	sdp3x_remove_sysfs_attr(client);
	destroy_workqueue(drv_data->workqueue);

	return 0;
}

static int sdp3x_suspend(struct device *dev)
{
	return 0;
}

static int sdp3x_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sdp3x_pm_ops, sdp3x_suspend, sdp3x_resume);

static const struct i2c_device_id sdp3x_id[] = {
	{ "sdp3x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sdp3x_id);

#ifdef CONFIG_OF
static const struct of_device_id sdp3x_of_match[] = {
	{ .compatible = "sensirion,sdp3x", },
	{ }
};
MODULE_DEVICE_TABLE(of, sdp3x_of_match);
#endif

static struct i2c_driver sdp3x_driver = {
	.probe		= sdp3x_probe,
	.remove		= sdp3x_remove,
	.driver		= {
		.name	= "sdp3x-digital",
		.of_match_table = of_match_ptr(sdp3x_of_match),
		.pm	= &sdp3x_pm_ops,
	},
	.id_table = sdp3x_id,
};

module_i2c_driver(sdp3x_driver);

MODULE_AUTHOR("Hsinko Yu <hsinkoyu@fih-foxconn.com>");
MODULE_DESCRIPTION("Driver for Sensirion SDP3x-Digital");
MODULE_LICENSE("GPL v2");
