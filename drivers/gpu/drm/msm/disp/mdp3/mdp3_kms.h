/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MDP3 KMS driver (MSM8909): private structures.
 *
 * MDP3 is the minimal display engine of MSM8909: one RGB fetch pipe
 * (DMA_P) feeding DSI command mode.  No layer mixer, no overlay pipes -
 * a single plane scans out directly, so the DRM object model is one
 * plane, one CRTC, one (DSI) encoder.
 */
#ifndef __MDP3_KMS_H__
#define __MDP3_KMS_H__

#include "msm_drv.h"
#include "msm_kms.h"
#include "disp/mdp_kms.h"
#include "mdp3_hwio.h"

struct mdp3_kms {
	struct mdp_kms base;

	struct drm_device *dev;

	void __iomem *mmio;   /* core reg space @ 0x1a00000 */
	void __iomem *vbif;   /* vbif reg space @ 0x1ab0000 */
	/* DSI ctrl reg space for the command-mode MDP trigger; mapped
	 * without requesting the region, the msm dsi host owns it */
	void __iomem *dsi_ctrl;

	struct clk *core_clk;   /* gcc_mdss_mdp_clk */
	struct clk *iface_clk;  /* gcc_mdss_ahb_clk */
	struct clk *bus_clk;    /* gcc_mdss_axi_clk */
	struct clk *vsync_clk;  /* gcc_mdss_vsync_clk */

	/* set on the first enable; lk2nd leaves the clocks running but
	 * the clk framework does not know, so a prepare_enable pair on a
	 * bootloader-on clock would just balance back to off on disable.
	 * Keep runtime PM referencing this counter. */
	int rpm_enabled;

	/* cached irq mask from the mdp_kms irq list */
	uint32_t cur_intr_enable;
};
#define to_mdp3_kms(x) container_of(x, struct mdp3_kms, base)

static inline void mdp3_write(struct mdp3_kms *mdp3_kms, u32 reg, u32 data)
{
	writel(data, mdp3_kms->mmio + reg);
}

static inline u32 mdp3_read(struct mdp3_kms *mdp3_kms, u32 reg)
{
	return readl(mdp3_kms->mmio + reg);
}

static inline void vbif_write(struct mdp3_kms *mdp3_kms, u32 reg, u32 data)
{
	writel(data, mdp3_kms->vbif + reg);
}

static inline u32 vbif_read(struct mdp3_kms *mdp3_kms, u32 reg)
{
	return readl(mdp3_kms->vbif + reg);
}

static inline void mdp3_dsi_trigger(struct mdp3_kms *mdp3_kms)
{
	writel(1, mdp3_kms->dsi_ctrl + REG_DSI_CMD_MODE_MDP_SW_TRIGGER);
}

struct drm_plane *mdp3_plane_init(struct drm_device *dev);
struct drm_crtc *mdp3_crtc_init(struct drm_device *dev, struct drm_plane *plane);
struct drm_encoder *mdp3_dsi_encoder_init(struct drm_device *dev);

void mdp3_crtc_kick(struct drm_crtc *crtc);
void mdp3_crtc_wait_for_flush_done(struct drm_crtc *crtc);

void mdp3_kms_dump(struct mdp3_kms *mdp3_kms, const char *tag);
void w1a_mb(const char *fmt, ...);    /* W1A ramoops mailbox trace */

#endif /* __MDP3_KMS_H__ */
