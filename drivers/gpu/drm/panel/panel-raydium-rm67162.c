// SPDX-License-Identifier: GPL-2.0
/*
 * DRM panel driver for the Raydium RM67162 WQVGA (400x400 round) AMOLED
 * panel in DSI command mode, as found on the CMCC W1A kids locator watch.
 *
 * The vendor command set is page-switched through the WRMAUCCTR (0xFE)
 * register. Init/off sequences are taken verbatim from the lk2nd panel
 * table that powers this panel on every boot
 * (lk2nd/display/panel/generated/lk_panel_rm67162_wqvga_cmd.h), which
 * matches the downstream 3.18 DTB (rm67162_wqvga_cmd node).
 *
 * Single DSI data lane, RGB888, DCS-command backlight (0x51), TE from the
 * panel TE pin (DCS 0x35).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/* Write Manufacture Command Set Control (page switch) */
#define WRMAUCCTR 0xFE

struct rm67162_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct gpio_desc *backlight_enable;

	struct regulator_bulk_data supplies[2];

	bool prepared;
};

static const struct drm_display_mode rm67162_mode = {
	/*
	 * 400x400@60. vtotal 436 / htotal 480 from the stock DTB timings
	 * (hfp 20 hsync 20 hbp 40, vfp 20 vsync 4 vbp 12). The pixel clock
	 * drives the DSI byte clock: pclk * 24bpp / 8 / 1 lane = 37.7 MHz,
	 * the rate the link runs at when lk2nd brings the panel up.
	 */
	.clock = 12557,
	.hdisplay = 400,
	.hsync_start = 400 + 20,
	.hsync_end = 400 + 20 + 20,
	.htotal = 400 + 20 + 20 + 40,
	.vdisplay = 400,
	.vsync_start = 400 + 20,
	.vsync_end = 400 + 20 + 4,
	.vtotal = 400 + 20 + 4 + 12,
	.width_mm = 35,
	.height_mm = 35,
};

static inline struct rm67162_panel *to_rm67162_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rm67162_panel, panel);
}

static int rm67162_write(struct mipi_dsi_device *dsi, u8 cmd, u8 value)
{
	u8 buf[2] = { cmd, value };

	return mipi_dsi_generic_write(dsi, buf, sizeof(buf));
}

static int rm67162_push_cmd_list(struct mipi_dsi_device *dsi)
{
	int ret;

	/* select page 0x0a */
	ret = rm67162_write(dsi, WRMAUCCTR, 0x0a);
	if (ret < 0)
		return ret;
	ret = rm67162_write(dsi, 0x29, 0x10);
	if (ret < 0)
		return ret;

	/* select page 0x05 */
	ret = rm67162_write(dsi, WRMAUCCTR, 0x05);
	if (ret < 0)
		return ret;
	ret = rm67162_write(dsi, 0x05, 0x00);
	if (ret < 0)
		return ret;

	/* back to page 0x00 */
	ret = rm67162_write(dsi, WRMAUCCTR, 0x00);
	if (ret < 0)
		return ret;
	ret = rm67162_write(dsi, 0x51, 0xff);
	if (ret < 0)
		return ret;
	ret = rm67162_write(dsi, 0x53, 0x20);
	if (ret < 0)
		return ret;

	return 0;
}

static int rm67162_panel_prepare(struct drm_panel *panel)
{
	struct rm67162_panel *ctx = to_rm67162_panel(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(ctx->backlight_enable, 1);

	/* reset: high 20ms, low 20ms, high 20ms (stock reset-sequence) */
	gpiod_set_value_cansleep(ctx->reset, 1);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset, 1);
	msleep(20);

	ctx->prepared = true;

	return 0;
}

static int rm67162_panel_unprepare(struct drm_panel *panel)
{
	struct rm67162_panel *ctx = to_rm67162_panel(panel);

	/* keep reset asserted-high (stock mdss suspend pinctrl holds it high) */
	gpiod_set_value_cansleep(ctx->reset, 1);
	gpiod_set_value_cansleep(ctx->backlight_enable, 0);

	regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);

	ctx->prepared = false;

	return 0;
}

static int rm67162_panel_enable(struct drm_panel *panel)
{
	struct rm67162_panel *ctx = to_rm67162_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = rm67162_push_cmd_list(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to send vendor commands (%d)\n", ret);
		return ret;
	}

	/* tear on, sourced from the TE pin (DCS 0x35 0x00) */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear ON (%d)\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode (%d)\n", ret);
		return ret;
	}

	msleep(150);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display ON (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int rm67162_panel_disable(struct drm_panel *panel)
{
	struct rm67162_panel *ctx = to_rm67162_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	msleep(120);

	return rm67162_write(dsi, 0x4f, 0x01);
}

static int rm67162_panel_get_modes(struct drm_panel *panel,
				   struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &rm67162_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			rm67162_mode.hdisplay, rm67162_mode.vdisplay,
			drm_mode_vrefresh(&rm67162_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = rm67162_mode.width_mm;
	connector->display_info.height_mm = rm67162_mode.height_mm;

	return 1;
}

static int rm67162_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rm67162_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!ctx->prepared)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = rm67162_write(dsi, 0x51, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static int rm67162_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops rm67162_bl_ops = {
	.update_status = rm67162_bl_update_status,
	.get_brightness = rm67162_bl_get_brightness,
};

static const struct drm_panel_funcs rm67162_panel_funcs = {
	.prepare = rm67162_panel_prepare,
	.unprepare = rm67162_panel_unprepare,
	.enable = rm67162_panel_enable,
	.disable = rm67162_panel_disable,
	.get_modes = rm67162_panel_get_modes,
};

static int rm67162_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rm67162_panel *ctx;
	struct backlight_properties bl_props;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct rm67162_panel, panel,
				   &rm67162_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dsi = dsi;

	dsi->lanes = 1;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* command mode panel: no MIPI_DSI_MODE_VIDEO */
	dsi->mode_flags = 0;

	ctx->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return PTR_ERR(ctx->reset);

	/*
	 * Requested HIGH to match the lk2nd/boot state (stock mdss pinctrl
	 * holds gpio37 high while the display runs); unprepare() drops it.
	 */
	ctx->backlight_enable = devm_gpiod_get_optional(dev, "backlight-enable",
							GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->backlight_enable))
		return PTR_ERR(ctx->backlight_enable);

	ctx->supplies[0].supply = "vdd";	/* 2.85 V */
	ctx->supplies[1].supply = "vddio";	/* 1.8 V */
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret)
		return ret;

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	ctx->panel.backlight = devm_backlight_device_register(
					dev, dev_name(dev), dev, dsi,
					&rm67162_bl_ops, &bl_props);
	if (IS_ERR(ctx->panel.backlight))
		return PTR_ERR(ctx->panel.backlight);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		dev_err(dev, "failed to attach to DSI host (%d)\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void rm67162_panel_remove(struct mipi_dsi_device *dsi)
{
	struct rm67162_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(&dsi->dev, "failed to detach from DSI host (%d)\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id rm67162_of_match[] = {
	{ .compatible = "raydium,rm67162" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rm67162_of_match);

static struct mipi_dsi_driver rm67162_panel_driver = {
	.driver = {
		.name = "panel-raydium-rm67162",
		.of_match_table = rm67162_of_match,
	},
	.probe = rm67162_panel_probe,
	.remove = rm67162_panel_remove,
};
module_mipi_dsi_driver(rm67162_panel_driver);

MODULE_DESCRIPTION("DRM panel driver for Raydium RM67162 command mode DSI panel");
MODULE_LICENSE("GPL");
