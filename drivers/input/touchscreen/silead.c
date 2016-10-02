/* -------------------------------------------------------------------------
 * Copyright (C) 2014-2015, Intel Corporation
 *
 * Derived from:
 *  gslX68X.c
 *  Copyright (C) 2010-2015, Shanghai Sileadinc Co.Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * ------------------------------------------------------------------------- */

//#define DEBUG

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/pm.h>
#include <linux/irq.h>

#define SILEAD_TS_NAME "gslX680"

#define SILEAD_REG_RESET	0xE0
#define SILEAD_REG_DATA		0x80
#define SILEAD_REG_TOUCH_NR	0x80
#define SILEAD_REG_POWER	0xBC
#define SILEAD_REG_CLOCK	0xE4
#define SILEAD_REG_STATUS	0xB0
#define SILEAD_REG_ID		0xFC
#define SILEAD_REG_MEM_CHECK	0xB0

#define SILEAD_STATUS_OK	0x5A5A5A5A
#define SILEAD_TS_DATA_LEN	44
#define SILEAD_CLOCK		0x04
#define SILEAD_CLOCK_OFF	0xB5

#define SILEAD_CMD_RESET	0x88
#define SILEAD_CMD_START	0x00

#define SILEAD_POINT_DATA_LEN	0x04
#define SILEAD_POINT_Y_OFF      0x00
#define SILEAD_POINT_Y_MSB_OFF	0x01
#define SILEAD_POINT_X_OFF	0x02
#define SILEAD_POINT_X_MSB_OFF	0x03
#define SILEAD_POINT_HSB_MASK	0x0F
#define SILEAD_TOUCH_ID_MASK	0xF0

#define SILEAD_DP_X_INVERT	"touchscreen-inverted-x"
#define SILEAD_DP_Y_INVERT	"touchscreen-inverted-y"
#define SILEAD_DP_XY_SWAP	"touchscreen-swapped-x-y"
#define SILEAD_DP_X_MAX		"touchscreen-size-x"
#define SILEAD_DP_Y_MAX		"touchscreen-size-y"
#define SILEAD_DP_MAX_FINGERS	"touchscreen-max-fingers"
#define SILEAD_DP_FW_NAME	"touchscreen-fw-name"
#define SILEAD_PWR_GPIO_NAME	"power-gpio"
#define SILEAD_IRQ_GPIO_NAME	"irq-gpio"

#define SILEAD_CMD_SLEEP_MIN	10000
#define SILEAD_CMD_SLEEP_MAX	20000

#define SILEAD_POWER_SLEEP	20
#define SILEAD_STARTUP_SLEEP	30

#define SILEAD_MAX_FINGERS	5
#define SILEAD_MAX_X		1820
#define SILEAD_MAX_Y		1280
#define SILEAD_OFFSET_X		40
#define SILEAD_OFFSET_Y		0
#define SILEAD_JITTER      0


#define GSL_PAGE_REG 0xf0
#define DMA_TRANS_LEN 0x20

enum silead_ts_power {
	SILEAD_POWER_ON  = 1,
	SILEAD_POWER_OFF = 0
};

struct silead_ts_data {
	struct i2c_client *client;
	struct gpio_desc *gpio_irq;
	struct gpio_desc *gpio_power;
	struct input_dev *input;
	const char *custom_fw_name;
	char fw_name[I2C_NAME_SIZE];
	u32 x_max;
	u32 y_max;
	u32 max_fingers;
	bool x_invert;
	bool y_invert;
	bool xy_swap;
	u32 x_offset;
	u32 y_offset;
	u32 jitter;
	u32 chip_id;
	struct input_mt_pos pos[SILEAD_MAX_FINGERS];
	int slots[SILEAD_MAX_FINGERS];
	int id[SILEAD_MAX_FINGERS];
	bool is_suspended;
	struct mutex sus_lock;

};

struct silead_fw_data {
	u32 offset;
	u32 val;
};

static u32 id_sign[SILEAD_MAX_FINGERS+1] = {0};
static u8 id_state_flag[SILEAD_MAX_FINGERS+1] = {0};
static u8 id_state_old_flag[SILEAD_MAX_FINGERS+1] = {0};
static u16 x_old[SILEAD_MAX_FINGERS+1] = {0};
static u16 y_old[SILEAD_MAX_FINGERS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;


static int silead_ts_request_input_dev(struct silead_ts_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	data->input = devm_input_allocate_device(dev);
	if (!data->input) {
		dev_err(dev,
			"Failed to allocate input device\n");
		return -ENOMEM;
	}

	input_set_capability(data->input, EV_ABS, ABS_X);
	input_set_capability(data->input, EV_ABS, ABS_Y);
	
	input_set_abs_params(data->input, ABS_MT_TRACKING_ID, 0, data->max_fingers, 0, 0);
	
	if (!data->xy_swap) {
		input_set_abs_params(data->input, ABS_MT_POSITION_X, 0,
				     data->x_max, data->jitter, 0);
		input_set_abs_params(data->input, ABS_MT_POSITION_Y, 0,
				     data->y_max, data->jitter, 0);
	} else {
		input_set_abs_params(data->input, ABS_MT_POSITION_X, 0,
				     data->y_max, data->jitter, 0);
		input_set_abs_params(data->input, ABS_MT_POSITION_Y, 0,
				     data->x_max, data->jitter, 0);
	}

	input_mt_init_slots(data->input, data->max_fingers,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED |
			    INPUT_MT_TRACK);

	input_set_drvdata(data->input, data);

	data->input->name = SILEAD_TS_NAME;
	data->input->phys = "input/ts";
	data->input->id.bustype = BUS_I2C;

	error = input_register_device(data->input);
	if (error) {
		dev_err(dev, "Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}


static void record_point(u16 x, u16 y , u8 id)
{
    u16 x_err =0;
    u16 y_err =0;

    id_sign[id]=id_sign[id]+1;

    if (id_sign[id]==1) {
        x_old[id]=x;
        y_old[id]=y;
    }

    x = (x_old[id] + x)/2;
    y = (y_old[id] + y)/2;
   
    if (x>x_old[id]) {
        x_err=x -x_old[id];
    } else {
        x_err=x_old[id]-x;
    }

    if (y>y_old[id]) {
        y_err=y -y_old[id];
    } else {
        y_err=y_old[id]-y;
    }

    if ( (x_err > 6 && y_err > 2) || (x_err > 2 && y_err > 6) ) {
        x_new = x;     x_old[id] = x;
        y_new = y;     y_old[id] = y;
    } else {
        if (x_err > 6) {
            x_new = x;     x_old[id] = x;
        } else
            x_new = x_old[id];
        if (y_err> 6) {
            y_new = y;     y_old[id] = y;
        } else
            y_new = y_old[id];
    }

    if (id_sign[id]==1) {
        x_new= x_old[id];
        y_new= y_old[id];
    }

}


static void silead_ts_report_touch(struct silead_ts_data *data, u16 x, u16 y,
				   u8 id)
{

	input_mt_slot(data->input, id);
	input_mt_report_slot_state(data->input, MT_TOOL_FINGER, true);
	input_report_abs(data->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(data->input, ABS_MT_POSITION_X, x);
	input_report_abs(data->input, ABS_MT_POSITION_Y, y);
}

static void silead_ts_set_power(struct i2c_client *client,
				enum silead_ts_power state)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
#ifdef CONFIG_ACPI
	int error;
#endif

	if (data) {
			if (data->gpio_power) {
				gpiod_set_value_cansleep(data->gpio_power, state);
	#ifdef CONFIG_ACPI
			} else {
				dev_dbg(&client->dev,"ACPI set_power.\n");
				error = acpi_bus_set_power(ACPI_HANDLE(&client->dev), state==SILEAD_POWER_OFF ? ACPI_STATE_D3 : ACPI_STATE_D0);
				if (error) {
					dev_warn(&client->dev, "%s: error changing power state: %d\n", __func__, error);
				}
	#endif
			}
			usleep_range(20000, 50000);
	}

}

static void silead_ts_read_data(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	u8 buf[SILEAD_TS_DATA_LEN];
	int x, y, id, touch_nr, error, i, offset, index;

	dev_dbg(dev,"silead_ts_read_data\n");


	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_DATA,
					    SILEAD_TS_DATA_LEN, buf);
	if (error < 0) {
		dev_err(dev, "Data read error %d\n", error);
		return;
	}

	touch_nr = buf[0];

	if (touch_nr < 0)
		return;

  for(i=0;i<SILEAD_MAX_FINGERS;i++) {
      if (touch_nr == 0)
          id_sign[i] = 0;
      id_state_flag[i] = 0;
	}

	dev_dbg(dev, "Touch number: %d\n", touch_nr);


	for (i = 1; i <= touch_nr; i++) {
		offset = i * SILEAD_POINT_DATA_LEN;

		// Bits 4-7 are the touch id 
		id = (buf[offset + SILEAD_POINT_X_MSB_OFF] &
		      SILEAD_TOUCH_ID_MASK) >> 4;

		// Bits 0-3 are MSB of X 
		buf[offset + SILEAD_POINT_X_MSB_OFF] =
					buf[offset + SILEAD_POINT_X_MSB_OFF] &
					SILEAD_POINT_HSB_MASK;

		// Bits 0-3 are MSB of Y 
		buf[offset + SILEAD_POINT_Y_MSB_OFF] =
					buf[offset + SILEAD_POINT_Y_MSB_OFF] &
					SILEAD_POINT_HSB_MASK;

		y = le16_to_cpup((__le16 *)(buf + offset + SILEAD_POINT_Y_OFF));
		x = le16_to_cpup((__le16 *)(buf + offset + SILEAD_POINT_X_OFF));

		x = x - data->x_offset;
		y = y - data->y_offset;
		if (data->x_invert)
			x = data->x_max - x;

		if (data->y_invert)
			y = data->y_max - y;

		if (data->xy_swap)
			swap(x, y);

		index = i - 1;
		data->pos[index].x = x;
		data->pos[index].y = y;
		data->id[index] = id;
	}

	input_mt_assign_slots(data->input, data->slots, data->pos,
				      touch_nr);

	for (i = 0; i < touch_nr; i++) {
		if (data->slots[i]<0 || data->slots[i]>=SILEAD_MAX_FINGERS) {
			dev_warn(dev, "Invalid slot id returned by input_mt_assign_slots: %d\n", data->slots[i]);
			continue;
		}
		id_state_flag[data->slots[i]] = 1;
		record_point(data->pos[i].x, data->pos[i].y, data->slots[i]);
		silead_ts_report_touch(data, x_new, y_new, data->slots[i]);

		dev_dbg(dev, "x=%d y=%d hw_id=%d sw_id=%d\n", x_new, y_new, data->id[i],
			data->slots[i]);
	}


  for(i=0;i<SILEAD_MAX_FINGERS;i++) {
      if ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) {
          input_mt_slot(data->input, i);
          input_mt_report_slot_state(data->input, MT_TOOL_FINGER, false);
          id_sign[i]=0;
      }
      id_state_old_flag[i] = id_state_flag[i];
	}

	input_mt_sync_frame(data->input);
	input_sync(data->input);
}

static int silead_ts_init(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
					SILEAD_CMD_RESET);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_TOUCH_NR,
					data->max_fingers);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_CLOCK,
					  SILEAD_CLOCK);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
					SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	return 0;

i2c_write_err:
	dev_err(&client->dev, "Registers clear error %d\n", error);
	return error;
}

static int silead_ts_reset(struct i2c_client *client)
{
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
					SILEAD_CMD_RESET);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_CLOCK,
					  SILEAD_CLOCK);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_POWER,
					SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	return 0;

i2c_write_err:
	dev_err(&client->dev, "Chip reset error %d\n", error);
	return error;
}

static int silead_ts_startup(struct i2c_client *client)
{
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET, 0x00);
	if (error) {
		dev_err(&client->dev, "Startup error %d\n", error);
		return error;
	}
	msleep(SILEAD_STARTUP_SLEEP);

	return 0;
}

static int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
    struct i2c_msg xfer_msg[1];

    buf[0] = reg;

    xfer_msg[0].addr = client->addr;
    xfer_msg[0].len = num + 1;
    xfer_msg[0].flags = client->flags & I2C_M_TEN;
    xfer_msg[0].buf = buf;

    return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
    u32 *u32_buf = (int *)buf;
    *u32_buf = *fw;
}

static int gsl_load_fw(struct i2c_client *client)
{
    u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
    u8 send_flag = 1;
    u8 *cur = buf + 1;
    const struct firmware *fw = NULL;
    size_t i;
    int rc;
		struct silead_ts_data *data = i2c_get_clientdata(client);

    printk("=============gsl_load_fw start==============\n");

		if (data->custom_fw_name)
			rc = request_firmware(&fw, data->custom_fw_name, &client->dev);
		else
			rc = request_firmware(&fw, data->fw_name, &client->dev);
    if (rc) {
        dev_err(&client->dev, "Unable to open firmware.\n");
        return rc;
    }

    for(i=0; i < fw->size; i+=8) {
        u32 *reg = (u32*)&(fw->data[i]);
        u32 *value = (u32*)&(fw->data[i + 4]);

        fw2buf(cur, value);

        if (*reg == GSL_PAGE_REG) {
            rc = gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
            if (rc < 0) {
                pr_info("%s: gsl_write_interface failed. \n", __func__);
                goto error;
            }
            send_flag = 1;
        } else {
            if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
                    buf[0] = (u8)*reg;

            cur += 4;

            if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) {
                    rc = gsl_write_interface(client, buf[0], buf, cur - buf - 1);
                    if (rc < 0) {
                        pr_info("%s: gsl_write_interface failed. \n", __func__);
                        goto error;
                    }
                    cur = buf + 1;
            }

            send_flag++;
        }
    }

    printk("=============gsl_load_fw end==============\n");

error:
    release_firmware(fw);

    return rc;
}


static u32 silead_ts_get_status(struct i2c_client *client)
{
	int error;
	u32 status;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_STATUS, 4,
					    (u8 *)&status);
	if (error < 0) {
		dev_err(&client->dev, "Status read error %d\n", error);
		return error;
	}

	return le32_to_cpu(status);
}

static int silead_ts_get_id(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	int error;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_ID, 4,
					    (u8 *)&data->chip_id);

	data->chip_id = le32_to_cpu(data->chip_id);

	if (error < 0) {
		dev_err(&client->dev, "Chip ID read error %d\n", error);
		return error;
	}

	return 0;
}

static int silead_ts_setup(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int error;
	u32 status;

	error = silead_ts_get_id(client);
	if (error)
		return error;
	dev_dbg(dev, "Chip ID: 0x%8X", data->chip_id);

	error = silead_ts_init(client);
	if (error)
		return error;

	error = silead_ts_reset(client);
	if (error)
		return error;

	error = gsl_load_fw(client);
	if (error)
		return error;

	error = silead_ts_startup(client);
	if (error)
		return error;

	status = silead_ts_get_status(client);
	if (status != SILEAD_STATUS_OK) {
		dev_err(dev, "Initialization error, status: 0x%X\n", status);
		return -ENODEV;
	}

	return 0;
}

static irqreturn_t silead_ts_threaded_irq_handler(int irq, void *id)
{
	struct silead_ts_data *data = id;
	struct i2c_client *client = data->client;

	mutex_lock(&data->sus_lock);

  if (data->is_suspended == false)
		silead_ts_read_data(client);

	mutex_unlock(&data->sus_lock);

	return IRQ_HANDLED;
}

#ifdef KERNEL4

static int silead_ts_read_props(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int error;

	error = device_property_read_u32(dev, SILEAD_DP_X_MAX, &data->x_max);
	if (error) {
		dev_dbg(dev, "Resolution X read error %d\n", error);
		data->x_max = SILEAD_MAX_X;
	}
	data->x_max--; /* Property contains size not max */

	error = device_property_read_u32(dev, SILEAD_DP_Y_MAX, &data->y_max);
	if (error) {
		dev_dbg(dev, "Resolution Y read error %d\n", error);
		data->y_max = SILEAD_MAX_Y;
	}
	data->y_max--; /* Property contains size not max */

	error = device_property_read_u32(dev, SILEAD_DP_MAX_FINGERS,
					 &data->max_fingers);
	if (error) {
		dev_dbg(dev, "Max fingers read error %d\n", error);
		data->max_fingers = SILEAD_MAX_FINGERS;
	}

	error = device_property_read_string(dev, SILEAD_DP_FW_NAME,
					  &data->custom_fw_name);
	if (error)
		dev_dbg(dev, "Firmware file name read error. Using default.");

	data->x_invert = device_property_read_bool(dev, SILEAD_DP_X_INVERT);
	data->y_invert = device_property_read_bool(dev, SILEAD_DP_Y_INVERT);
	data->xy_swap = device_property_read_bool(dev, SILEAD_DP_XY_SWAP);

	dev_dbg(dev, "x_max = %d, y_max = %d, max_fingers = %d, x_invert = %d, y_invert = %d, xy_swap = %d",
		data->x_max, data->y_max, data->max_fingers, data->x_invert,
		data->y_invert, data->xy_swap);

	return 0;
}

#endif

#ifdef CONFIG_ACPI
static int silead_ts_set_default_fw_name(struct silead_ts_data *data,
					 const struct i2c_device_id *id)
{
	const struct acpi_device_id *acpi_id;
	struct device *dev = &data->client->dev;
	int i;

	if (ACPI_HANDLE(dev)) {
		acpi_id = acpi_match_device(dev->driver->acpi_match_table, dev);
		if (!acpi_id)
			return -ENODEV;

		snprintf(data->fw_name, sizeof(data->fw_name), "%s.fw", acpi_id->id);

		for (i = 0; i < strlen(data->fw_name); i++)
			data->fw_name[i] = tolower(data->fw_name[i]);
	} else {
		snprintf(data->fw_name, sizeof(data->fw_name), "%s.fw", id->name);
	}

	return 0;
}
#else
static int silead_ts_set_default_fw_name(struct silead_ts_data *data,
					 const struct i2c_device_id *id)
{
	snprintf(data->fw_name, sizeof(data->fw_name), "%s.fw", id->name);
	return 0;
}
#endif

static int silead_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct silead_ts_data *data;
	struct device *dev = &client->dev;
	int error;
	int ret;
	bool acpipower;

	dev_dbg(dev,"probing silead\n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK |
				     I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		dev_err(dev, "I2C functionality check failed\n");
		return -ENXIO;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;

	error = silead_ts_set_default_fw_name(data, id);
	if (error) {
		dev_err(dev, "silead_ts_set_default_fw_name failed\n");
		return error;
	}

	data->x_max = SILEAD_MAX_X;
	data->y_max = SILEAD_MAX_Y;
	data->max_fingers = SILEAD_MAX_FINGERS;
	data->x_invert = false;
	data->y_invert = true;
	data->xy_swap = true;
	data->x_offset = SILEAD_OFFSET_X;
	data->y_offset = SILEAD_OFFSET_Y;
        data->jitter = SILEAD_JITTER;

	/* If the IRQ is not filled by DT or ACPI subsytem
	 * try using the named GPIO */
	if (client->irq <= 0) {
		dev_dbg(dev, "requesting IRQ from GPIO\n");

		data->gpio_irq = devm_gpiod_get_index(dev, SILEAD_IRQ_GPIO_NAME,1);
		if (IS_ERR(data->gpio_irq)) {
			dev_err(dev, "IRQ GPIO request failed\n");
			return -ENODEV;
		}

		ret = gpiod_direction_input(data->gpio_irq);
		if (ret) {
			dev_err(dev, "IRQ GPIO direction set failed\n");
			return ret;
		}

		client->irq = gpiod_to_irq(data->gpio_irq);
		if (client->irq <= 0) {
			dev_err(dev, "GPIO to IRQ translation failed %d\n",
				client->irq);
			return client->irq;
		}
	}

/* Try to use ACPI power methods first */
	acpipower = false;
#ifdef CONFIG_ACPI
	if (ACPI_COMPANION(&client->dev)) {
		dev_dbg(dev,"Trying acpi wake up.\n");
		/* Wake the device up with a power on reset */
		if (acpi_bus_set_power(ACPI_HANDLE(&client->dev), ACPI_STATE_D3)) {
			dev_warn(&client->dev, "%s: failed to wake up device through ACPI: %d, using GPIO controls instead\n", __func__, error);
		} else {
			acpipower = true;
		}
	}
#endif

	if (!acpipower) {
	/* Power GPIO pin */
		data->gpio_power = devm_gpiod_get_index(dev, SILEAD_PWR_GPIO_NAME,
							1);
		if (IS_ERR(data->gpio_power)) {
			dev_dbg(dev, "Shutdown GPIO request failed\n");
			data->gpio_power = NULL;
		}
	} else {
			data->gpio_power = NULL;
	}


	silead_ts_set_power(client, SILEAD_POWER_ON);

#ifdef KERNEL4
	error = silead_ts_read_props(client);
	if (error)
		return error;
#endif
	error = silead_ts_setup(client);
	if (error)
		return error;

	error = silead_ts_request_input_dev(data);
	if (error)
		return error;

	mutex_init(&data->sus_lock);

	error = devm_request_threaded_irq(dev, client->irq, NULL,
					silead_ts_threaded_irq_handler,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					client->name, data);
	if (error) {
		dev_err(dev, "IRQ request failed %d\n", error);
		return error;
	}

	dev_dbg(dev, "Probing succeded\n");
	return 0;
}

static int __maybe_unused silead_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct silead_ts_data *ts = i2c_get_clientdata(client);

	mutex_lock(&ts->sus_lock);

  ts->is_suspended = true;

	dev_dbg(dev, "silead_ts_suspend\n");
	silead_ts_set_power(client, SILEAD_POWER_OFF);
	mutex_unlock(&ts->sus_lock);
	return 0;
}

static int __maybe_unused silead_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct silead_ts_data *ts = i2c_get_clientdata(client);
	int error, status;

	dev_dbg(dev, "silead_ts_resume\n");
	silead_ts_set_power(client, SILEAD_POWER_ON);

	error = silead_ts_reset(client);
	if (error)
		return error;

	error = silead_ts_startup(client);
	if (error)
		return error;

	status = silead_ts_get_status(client);
	if (status != SILEAD_STATUS_OK) {
		dev_err(dev, "Resume error, status: 0x%X\n", status);
		return -ENODEV;
	}
  ts->is_suspended = false;

	return 0;
}

static SIMPLE_DEV_PM_OPS(silead_ts_pm, silead_ts_suspend, silead_ts_resume);


static const struct i2c_device_id silead_ts_id[] = {
	{ "gsl1680", 0 },
	{ "gsl1688", 0 },
	{ "gsl3670", 0 },
	{ "gsl3675", 0 },
	{ "gsl3692", 0 },
	{ "mssl1680", 0 },
	{ "gslX680", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, silead_ts_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id silead_ts_acpi_match[] = {
	{ "GSL1680", 0 },
	{ "GSL1688", 0 },
	{ "GSL3670", 0 },
	{ "GSL3675", 0 },
	{ "GSL3692", 0 },
	{ "MSSL1680", 0 },
	{ "MSSL0017", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, silead_ts_acpi_match);
#endif



static struct i2c_driver silead_ts_driver = {
	.probe = silead_ts_probe,
	.id_table = silead_ts_id,
	.driver = {
		.name = SILEAD_TS_NAME,
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(silead_ts_acpi_match),
		.pm = &silead_ts_pm,
	},
};
module_i2c_driver(silead_ts_driver);

MODULE_AUTHOR("Robert Dolca <robert.dolca@intel.com>");
MODULE_DESCRIPTION("Silead I2C touchscreen driver");
MODULE_LICENSE("GPL");
