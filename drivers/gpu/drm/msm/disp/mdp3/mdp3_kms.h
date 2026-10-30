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
	/* DSI PHY (0x1ac8500) + PLL (0x1ac8300) + GCC (0x1800000) windows
	 * for the bring-up register dumps - read-only, owned by their
	 * drivers */
	void __iomem *dsi_phy;
	void __iomem *dsi_pll;
	void __iomem *gcc_base;

	/* m4a: commit counter + takeover flag - autorefresh is disarmed at
	 * the SECOND flush_commit (first frame with our config completed,
	 * see mdp3_flush_commit) */
	unsigned int commit_count;
	bool autorefresh_taken;

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
	/*
	 * W1A (v39): force the DSI clock lane into HS before triggering the
	 * MDP stream, exactly as the 3.18 stock driver does for kicks
	 * (mdss_dsi_start_hs_clk_lane in the from_mdp path).  After a
	 * display power cycle the lane sits in LP stop; without the force
	 * the MDP stream never transmits and CMD_MODE_MDP_BUSY stays set
	 * forever (v38 wake: dsi_status0 bit2 stuck, no TE, no
	 * DMA_P_DONE -> black screen).  Boot worked only because lk2nd
	 * handed over a running HS clock lane.  The bit stays set while
	 * the panel is enabled (autorefresh streams continuously); the
	 * command path releases/re-forces it per transfer.
	 */
	writel(readl(mdp3_kms->dsi_ctrl + REG_DSI_LANE_CTRL) | BIT(28),
	       mdp3_kms->dsi_ctrl + REG_DSI_LANE_CTRL);
	writel(1, mdp3_kms->dsi_ctrl + REG_DSI_CMD_MODE_MDP_SW_TRIGGER);
}

struct drm_plane *mdp3_plane_init(struct drm_device *dev);
struct drm_crtc *mdp3_crtc_init(struct drm_device *dev, struct drm_plane *plane);
struct drm_encoder *mdp3_dsi_encoder_init(struct drm_device *dev);

void mdp3_crtc_kick(struct drm_crtc *crtc);
/* W1A (v49): re-run the MDP3 display-on programming after a DPMS wake */
void mdp3_w1a_reinit_stream(struct mdp3_kms *mdp3_kms);
extern bool w1a_dsi_pa_active;
extern bool w1a_reinit_done;
void mdp3_crtc_wait_for_flush_done(struct drm_crtc *crtc);

void mdp3_kms_dump(struct mdp3_kms *mdp3_kms, const char *tag);
void mdp3_dsi_raw_dump(struct mdp3_kms *mdp3_kms, const char *tag);
void w1a_mb(const char *fmt, ...);    /* W1A ramoops mailbox trace */

#endif /* __MDP3_KMS_H__ */
