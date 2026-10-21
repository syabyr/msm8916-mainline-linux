// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT3267 capacitive touchscreen driver (minimal mainline port).
 *
 * The CMCC W1A watch has an FT3267 on BLSP I2C5 (address 0x38), with the
 * reset line on TLMM gpio12 (active low) and the interrupt line on
 * TLMM gpio13 (falling edge).  The register protocol below is taken from
 * the vendor driver (downstream focaltech_touch, FT3267 configuration):
 *
 *   - touch report: read len 3 + 6*max_points starting at register 0x00.
 *     buf[2] & 0x0f = number of valid points; each point is 6 bytes:
 *       byte0: event type (bits 7:6: 0=down, 1=up, 2=contact) | x[11:8]
 *       byte1: x[7:0]
 *       byte2: finger id (bits 7:4) | y[11:8]
 *       byte3: y[7:0]
 *       byte4: pressure
 *       byte5: touch area (bits 7:4)
 *   - chip id: registers 0xA3 (high) / 0x9F (low).
 *   - reset: pulse active-low reset for ~6 ms, then wait ~200 ms for the
 *     firmware to come up.
 *
 * This port keeps only the pieces needed to bring up a working
 * touchscreen: power rails, reset, probe/chip-id check, interrupt and
 * multi-touch reporting.  Firmware flashing, gestures and ESD polling
 * from the vendor driver are intentionally omitted.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define FT3267_REG_WORKMODE		0x00
#define FT3267_REG_CHIP_ID_H		0xa3
#define FT3267_REG_CHIP_ID_L		0x9f

#define FT3267_MAX_POINTS		3
#define FT3267_ONE_POINT_LEN		6
#define FT3267_READ_LEN			(3 + FT3267_ONE_POINT_LEN * FT3267_MAX_POINTS)

#define FT3267_TOUCH_DOWN		0
#define FT3267_TOUCH_UP			1
#define FT3267_TOUCH_CONTACT		2

#define FT3267_CHIP_ID_H_3267		0x64
#define FT3267_CHIP_ID_L_3267		0x26

struct ft3267 {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;	/* analog / touch supply */
	struct regulator *vio;	/* 1.8 V I/O supply */
	struct work_struct work;
	unsigned int touchs;
};

static int ft3267_i2c_read(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;
	if (ret < 0)
		return ret;
	return -EIO;
}

static int ft3267_i2c_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
	return ret == sizeof(buf) ? 0 : (ret < 0 ? ret : -EIO);
}

static void ft3267_reset(struct ft3267 *ts, unsigned int delay_ms)
{
	/* reset-gpios is GPIO_ACTIVE_LOW: logical 1 asserts reset */
	if (ts->reset_gpio) {
		gpiod_set_value_cansleep(ts->reset_gpio, 1);
		msleep(6);
		gpiod_set_value_cansleep(ts->reset_gpio, 0);
	}
	msleep(delay_ms);
}

static int ft3267_wait_ready(struct ft3267 *ts)
{
	u8 id_h = 0, id_l = 0;
	int i;

	for (i = 0; i < 15; i++) {
		if (!ft3267_i2c_read(ts->client, FT3267_REG_CHIP_ID_H,
				     &id_h, 1) &&
		    !ft3267_i2c_read(ts->client, FT3267_REG_CHIP_ID_L,
				     &id_l, 1)) {
			if (id_h != 0x00 && id_h != 0xff &&
			    id_l != 0x00 && id_l != 0xff) {
				dev_info(&ts->client->dev,
					 "FT3267 ready, chip id %02x:%02x\n",
					 id_h, id_l);
				if (id_h != FT3267_CHIP_ID_H_3267 ||
				    id_l != FT3267_CHIP_ID_L_3267)
					dev_warn(&ts->client->dev,
						 "unexpected chip id %02x:%02x (expected %02x:%02x)\n",
						 id_h, id_l,
						 FT3267_CHIP_ID_H_3267,
						 FT3267_CHIP_ID_L_3267);
				return 0;
			}
		}
		msleep(20);
	}

	dev_err(&ts->client->dev, "FT3267 not responding (id %02x:%02x)\n",
		id_h, id_l);
	return -ETIMEDOUT;
}

static void ft3267_report_work(struct work_struct *work)
{
	struct ft3267 *ts = container_of(work, struct ft3267, work);
	struct input_dev *input = ts->input;
	u8 buf[FT3267_READ_LEN];
	unsigned int touchs = 0;
	int i, points;

	if (ft3267_i2c_read(ts->client, FT3267_REG_WORKMODE,
			    buf, sizeof(buf)))
		return;

	points = buf[2] & 0x0f;
	if (points > FT3267_MAX_POINTS)
		points = FT3267_MAX_POINTS;

	for (i = 0; i < points; i++) {
		const u8 *p = &buf[3 + i * FT3267_ONE_POINT_LEN];
		unsigned int event = p[0] >> 6;
		unsigned int id = p[2] >> 4;
		unsigned int x = ((p[0] & 0x0f) << 8) | p[1];
		unsigned int y = ((p[2] & 0x0f) << 8) | p[3];
		unsigned int area = p[5] >> 4;

		if (id >= FT3267_MAX_POINTS)
			continue;

		input_mt_slot(input, id);
		if (event == FT3267_TOUCH_DOWN ||
		    event == FT3267_TOUCH_CONTACT) {
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   true);
			input_report_abs(input, ABS_MT_POSITION_X, x);
			input_report_abs(input, ABS_MT_POSITION_Y, y);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR,
					 area ? area : 1);
			input_report_abs(input, ABS_MT_PRESSURE, p[4]);
			touchs |= BIT(id);
		} else {
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   false);
		}
	}

	/* release fingers that vanished without an explicit up event */
	for (i = 0; i < FT3267_MAX_POINTS; i++) {
		if ((ts->touchs & BIT(i)) && !(touchs & BIT(i))) {
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER,
						   false);
		}
	}
	ts->touchs = touchs;

	input_report_key(input, BTN_TOUCH, touchs != 0);
	input_sync(input);
}

static irqreturn_t ft3267_irq(int irq, void *data)
{
	struct ft3267 *ts = data;

	schedule_work(&ts->work);
	return IRQ_HANDLED;
}

static int ft3267_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ft3267 *ts;
	struct input_dev *input;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(ts->vdd))
		ts->vdd = NULL;
	ts->vio = devm_regulator_get_optional(dev, "vio");
	if (IS_ERR(ts->vio))
		ts->vio = NULL;

	if (ts->vdd) {
		ret = regulator_enable(ts->vdd);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable vdd\n");
	}
	if (ts->vio) {
		ret = regulator_enable(ts->vio);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable vio\n");
	}

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						 GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	ft3267_reset(ts, 200);

	ret = ft3267_wait_ready(ts);
	if (ret)
		return ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	ts->input = input;

	input->name = "ft3267-touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 400, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 400, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_X, 0, 400, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 400, 0, 0);

	ret = input_mt_init_slots(input, FT3267_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init MT slots\n");

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input\n");

	INIT_WORK(&ts->work, ft3267_report_work);

	ret = devm_request_threaded_irq(dev, client->irq, NULL, ft3267_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					"ft3267", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	/* make sure the controller is in normal (non-factory) mode */
	ft3267_i2c_write_reg(client, FT3267_REG_WORKMODE, 0x00);

	dev_info(dev, "FT3267 touchscreen registered (irq %d)\n",
		 client->irq);
	return 0;
}

static const struct of_device_id ft3267_of_match[] = {
	{ .compatible = "focaltech,ft3267" },
	{ }
};
MODULE_DEVICE_TABLE(of, ft3267_of_match);

static const struct i2c_device_id ft3267_i2c_id[] = {
	{ "ft3267" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft3267_i2c_id);

static struct i2c_driver ft3267_driver = {
	.driver = {
		.name = "ft3267",
		.of_match_table = ft3267_of_match,
	},
	.probe = ft3267_probe,
	.id_table = ft3267_i2c_id,
};
module_i2c_driver(ft3267_driver);

MODULE_DESCRIPTION("FocalTech FT3267 touchscreen");
MODULE_LICENSE("GPL");
