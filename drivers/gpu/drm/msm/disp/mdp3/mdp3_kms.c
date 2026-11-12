// SPDX-License-Identifier: GPL-2.0-only
/*
 * MDP3 KMS driver (MSM8909).
 *
 * MDP3 scans a single RGB framebuffer line by line (DMA_P pipe) into
 * DSI command mode.  Panel/DSI/clocks are the ones lk2nd set up: the
 * msm dsi host + panel drivers own the DSI side, this driver owns the
 * fetch engine, the MDP3 interrupt (vblank = DMA_P_DONE) and the
 * command-mode kick (DMA_P_START + DSI MDP_SW_TRIGGER).
 *
 * Register knowledge comes from the downstream 8909 drop
 * (drivers/video/msm/mdss/mdp3*) and from lk2nd - whose DMA_P
 * configuration (CONFIG 0x000821bf, RGB888 command mode) is proven on
 * this panel.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <drm/drm_dumb_buffers.h>
#include <drm/drm_vblank.h>

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_mmu.h"
#include "mdp3_kms.h"

/* m4b: runtime autorefresh control, applied at the 2nd-commit takeover
 * (see mdp3_flush_commit) and live afterwards via
 * /sys/module/msm/parameters/w1a_autorefresh:
 *   0 = disarmed - per-commit kicks are the only frame source
 *   N = armed at 60/N fps (lk2nd's splash config: BIT(28)|1 = 60Hz)
 * Default 1 because m4a unmasked the truth: with autorefresh disarmed
 * the per-kick path delivered fps 3-13 (and m3h, autorefresh accidentally
 * off, showed 1-9 + frozen screen) while every "working" boot since m3g
 * had lk2nd's 60Hz autorefresh silently re-fetching our buffer.  The
 * screen works out of the box with this default; the kick path is the
 * thing under investigation. */
static struct mdp3_kms *w1a_kms;
static unsigned int w1a_autorefresh = 1;

static int w1a_autorefresh_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret;

	ret = kstrtouint(val, 0, &v);
	if (ret)
		return ret;
	*(unsigned int *)kp->arg = v;

	if (w1a_kms && w1a_kms->autorefresh_taken) {
		u32 reg = v ? (BIT(28) | v) : 0;

		mdp3_write(w1a_kms, REG_MDP3_AUTOREFRESH_CONFIG_P, reg);
	}
	return 0;
}

static int w1a_autorefresh_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%u",
			 *(unsigned int *)kp->arg);
}

static const struct kernel_param_ops w1a_autorefresh_ops = {
	.set = w1a_autorefresh_set,
	.get = w1a_autorefresh_get,
};
module_param_cb(w1a_autorefresh, &w1a_autorefresh_ops, &w1a_autorefresh, 0644);

static int mdp3_hw_init(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));
	struct drm_device *dev = mdp3_kms->dev;
	u32 version;

	pm_runtime_get_sync(dev->dev);

	/* ROOT CAUSE FIX (m2v, found via downstream mdp3_ctrl_on): the port
	 * originally SET CGC bits 10/18 and wrote VBIF_FORCE_EN=0 here, which
	 * is downstream's IDLE state (mdp3_dynamic_clock_gating_ctrl(1)),
	 * not its display-ON state.  With the DMA (bit 10) and DSI (bit 18)
	 * paths dynamically clock-gated and the VBIF unforced, every MDSS
	 * AXI fetch hangs silently - no SMMU fault, FIFOs empty, status0=3,
	 * both engines, deterministically (m2q-m2u).  Downstream's
	 * display-ON sequence (mdp3_ctrl.c: mdp3_ctrl_on ->
	 * mdp3_dynamic_clock_gating_ctrl(0)) clears the CGC bits and forces
	 * the VBIF on: do exactly that. */
	mdp3_write(mdp3_kms, REG_MDP3_CGC_EN,
		   mdp3_read(mdp3_kms, REG_MDP3_CGC_EN) & ~(BIT(10) | BIT(18)));
	vbif_write(mdp3_kms, REG_MDP3_VBIF_FORCE_EN, 0x3);

	/* clear + mask all interrupts; vblank unmask happens via
	 * ->enable_vblank once userspace asks for them */
	mdp3_write(mdp3_kms, REG_MDP3_INTR_CLEAR, 0x7ffffff);
	mdp3_write(mdp3_kms, REG_MDP3_INTR_ENABLE, 0);

	/* m3g ROOT CAUSE FIX (WARN storm / missing continuous vblank):
	 * program the tear-check ("sync") block and switch vblank to the
	 * panel's TE signal, exactly as downstream does for DSI_CMD
	 * (mdp3_dma.c mdp3_dma_vsync_cfg + dsi_v2.c dsi_update_pconfig:
	 * DSI_CMD => vsync_enable=1, hw_vsync_mode=1).  The m3f2 self-test
	 * proved per-kick DMA_P_DONE works (kstat delta 2, handler cleared
	 * the latch between the 20ms polls) - but a "vblank" only exists
	 * per frame transfer, so every trailing drm_client_modeset_
	 * wait_for_vblank times out: the WARN storm.  The panel already
	 * sends TE (rm67162 enable: SET_TEAR_ON vblank mode); the DT
	 * pinctrl muxes gpio24 to mdp_vsync, and SYNC_PRIMARY_LINE (BIT8)
	 * fires when the tear-check line counter passes rd_ptr_irq.
	 * Values = downstream defaults (mdss_dsi_panel.c
	 * mdss_panel_parse_te_params) for this 400x400@60 panel:
	 *   sync_cfg_height 0xfff0 -> clamp 0x7ff (mdp3 limit),
	 *   init_val = start_pos = yres = 400, rd_ptr_irq = yres+1 = 401,
	 *   thresholds 4/4, refx100 6000,
	 *   vclks_line = 19200000*100/(436*6000) = 733.
	 * Order matters: TEAR_CHECK_EN goes last (it enables the block). */
	{
		u32 cfg = (0x7ff << 21) | BIT(19) | BIT(20) | 733;

		mdp3_write(mdp3_kms, REG_MDP3_SYNC_CONFIG_0, cfg);
		mdp3_write(mdp3_kms, REG_MDP3_VSYNC_SEL, 0x024);
		mdp3_write(mdp3_kms, REG_MDP3_PRIMARY_VSYNC_INIT_VAL, 400);
		mdp3_write(mdp3_kms, REG_MDP3_PRIMARY_RD_PTR_IRQ, 401);
		mdp3_write(mdp3_kms, REG_MDP3_SYNC_THRESH_0, (4 << 16) | 4);
		mdp3_write(mdp3_kms, REG_MDP3_PRIMARY_START_POS, 400);
		mdp3_write(mdp3_kms, REG_MDP3_TEAR_CHECK_EN, 1);
	}

	/* lk2nd arms the DMA_P autorefresh fetcher before jumping to the
	 * kernel (AUTOREFRESH_CONFIG_P=BIT(28)|interval, IBUF=lk2nd's
	 * buffer at 0x83200000).  m2k/m2l DISARMED it here and those are
	 * exactly the boots that die inside the first commit - writing 0
	 * to the config while a refresh cycle is mid-flight is not a
	 * documented-safe operation.  So: hands off at init.  Once the
	 * commit rewrites DMA_P config + IBUF, autorefresh refreshes OUR
	 * buffer; m4a disarms it deterministically after the first frame
	 * with our config has landed (see mdp3_flush_commit). */
	mdp3_write(mdp3_kms, REG_MDP3_DSI_CMD_MODE_ID_MAP, 0x10);
	mdp3_write(mdp3_kms, REG_MDP3_DSI_CMD_MODE_TRIGGER_EN, 0x11);
	w1a_kms = mdp3_kms;	/* m4b: for the w1a_autorefresh param callback */
	version = mdp3_read(mdp3_kms, REG_MDP3_HW_VERSION);
	DRM_DEV_INFO(dev->dev, "MDP3 HW_VERSION 0x%08x", version);

	pm_runtime_put_sync(dev->dev);

	return 0;
}

static irqreturn_t mdp3_irq(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));
	struct drm_device *dev = mdp3_kms->dev;
	struct drm_crtc *crtc;
	uint32_t status, enable;

	enable = mdp3_read(mdp3_kms, REG_MDP3_INTR_ENABLE);
	status = mdp3_read(mdp3_kms, REG_MDP3_INTR_STATUS) & enable;

	mdp3_write(mdp3_kms, REG_MDP3_INTR_CLEAR, status);

	VERB("status=%08x", status);

	mdp_dispatch_irqs(&mdp3_kms->base, status);

	/* DMA_P_DONE is the frame completion event for command mode.
	 * m3g: SYNC_PRIMARY_LINE (panel TE through the tear-check block)
	 * is the continuous vsync between transfers - either counts. */
	drm_for_each_crtc(crtc, dev)
		if (status & (MDP3_INTR_DMA_P_DONE | MDP3_INTR_SYNC_PRIMARY_LINE))
			drm_crtc_handle_vblank(crtc);

	return IRQ_HANDLED;
}

static void mdp3_irq_preinstall(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));

	mdp3_write(mdp3_kms, REG_MDP3_INTR_CLEAR, 0x7ffffff);
	mdp3_write(mdp3_kms, REG_MDP3_INTR_ENABLE, 0);
}

static int mdp3_irq_postinstall(struct msm_kms *kms)
{
	return 0;
}

static void mdp3_irq_uninstall(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));

	mdp3_write(mdp3_kms, REG_MDP3_INTR_ENABLE, 0);
	mdp3_write(mdp3_kms, REG_MDP3_INTR_CLEAR, 0x7ffffff);
}

static int mdp3_enable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));

	/* m3g: TE (continuous) + DMA_P_DONE (per-transfer) both count */
	mdp_update_vblank_mask(&mdp3_kms->base,
			       MDP3_INTR_DMA_P_DONE | MDP3_INTR_SYNC_PRIMARY_LINE,
			       true);

	return 0;
}

static void mdp3_disable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));

	mdp_update_vblank_mask(&mdp3_kms->base,
			       MDP3_INTR_DMA_P_DONE | MDP3_INTR_SYNC_PRIMARY_LINE,
			       false);
}

static void mdp3_set_irqmask(struct mdp_kms *mdp_kms, uint32_t irqmask,
			     uint32_t old_irqmask)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(mdp_kms);

	/* clear any newly-enabled latched bits so they do not fire
	 * immediately on unmask (mdp4 pattern) */
	mdp3_write(mdp3_kms, REG_MDP3_INTR_CLEAR,
		   irqmask ^ (irqmask & old_irqmask));
	mdp3_write(mdp3_kms, REG_MDP3_INTR_ENABLE, irqmask);
}

static void mdp3_enable_commit(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));

	pm_runtime_get_sync(mdp3_kms->dev->dev);
}

static void mdp3_disable_commit(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));

	/* only drop the reference taken by enable_commit - one reference
	 * is held forever from mdp3_kms_init, so the MDSS power domain
	 * (which also feeds the DSI host + PHY the msm dsi driver just
	 * configured) never powers off behind our back mid-bring-up */
	pm_runtime_put_sync(mdp3_kms->dev->dev);
}

static void mdp3_wait_flush(struct msm_kms *kms, unsigned crtc_mask)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));
	struct drm_crtc *crtc;

	for_each_crtc_mask(mdp3_kms->dev, crtc, crtc_mask)
		mdp3_crtc_wait_for_flush_done(crtc);
}

static void mdp3_flush_commit(struct msm_kms *kms, unsigned crtc_mask)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));
	struct drm_crtc *crtc;

	/* the frame kick belongs here: at this point planes have been
	 * programmed AND bridges enabled, so the DMA fetch has somewhere
	 * to push to.  (atomic_flush runs before bridges enable on the
	 * first modeset, too early to kick.) */
	for_each_crtc_mask(mdp3_kms->dev, crtc, crtc_mask)
		mdp3_crtc_kick(crtc);

	/* m4a/m4b: lk2nd leaves the autorefresh fetcher armed (splash
	 * screen self-refresh).  The first commit above rewrote DMA_P
	 * config and IBUF to OUR scanout buffer and issued the first
	 * fetch - but that fetch is still in flight when this function
	 * returns (~16 ms transfer), and writing AUTOREFRESH_CONFIG_P
	 * mid-cycle is the exact hazard that killed the m2k/m2l boots.
	 * So the takeover happens one commit later: on the SECOND flush
	 * the first frame has certainly completed, the pipe state is
	 * fully ours, and we apply the w1a_autorefresh policy (default:
	 * keep lk2nd's 60Hz armed - it is what has been delivering
	 * frames all along; see the m4b comment at the param). */
	if (!mdp3_kms->autorefresh_taken && ++mdp3_kms->commit_count == 2) {
		u32 reg = w1a_autorefresh ? (BIT(28) | w1a_autorefresh) : 0;
		u32 was = mdp3_read(mdp3_kms, REG_MDP3_AUTOREFRESH_CONFIG_P);

		mdp3_kms->autorefresh_taken = true;
		if (was != reg)
			mdp3_write(mdp3_kms, REG_MDP3_AUTOREFRESH_CONFIG_P, reg);
	}
}

static void mdp3_complete_commit(struct msm_kms *kms, unsigned crtc_mask)
{
}

static void mdp3_destroy(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));
	struct device *dev = mdp3_kms->dev->dev;

	if (kms->vm) {
		struct msm_mmu *mmu = to_msm_vm(kms->vm)->mmu;

		mmu->funcs->detach(mmu);
		drm_gpuvm_put(kms->vm);
	}

	if (mdp3_kms->rpm_enabled)
		pm_runtime_disable(dev);

	mdp_kms_destroy(&mdp3_kms->base);
}

static const struct mdp_kms_funcs kms_funcs = {
	.base = {
		.hw_init         = mdp3_hw_init,
		.irq_preinstall  = mdp3_irq_preinstall,
		.irq_postinstall = mdp3_irq_postinstall,
		.irq_uninstall   = mdp3_irq_uninstall,
		.irq             = mdp3_irq,
		.enable_vblank   = mdp3_enable_vblank,
		.disable_vblank  = mdp3_disable_vblank,
		.enable_commit   = mdp3_enable_commit,
		.disable_commit  = mdp3_disable_commit,
		.wait_flush      = mdp3_wait_flush,
		.flush_commit    = mdp3_flush_commit,
		.complete_commit = mdp3_complete_commit,
		.destroy         = mdp3_destroy,
	},
	.set_irqmask         = mdp3_set_irqmask,
};

#ifdef CONFIG_DRM_MSM_DSI
static int mdp3_modeset_init_dsi(struct mdp3_kms *mdp3_kms)
{
	struct drm_device *dev = mdp3_kms->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_encoder *encoder;
	int ret;

	if (!priv->kms->dsi[0])
		return 0;

	encoder = mdp3_dsi_encoder_init(dev);
	if (IS_ERR(encoder)) {
		DRM_DEV_ERROR(dev->dev, "failed to construct DSI encoder\n");
		return PTR_ERR(encoder);
	}

	encoder->possible_crtcs = BIT(0);

	ret = msm_dsi_modeset_init(priv->kms->dsi[0], dev, encoder);
	if (ret) {
		DRM_DEV_ERROR(dev->dev, "failed to initialize DSI: %d\n", ret);
		return ret;
	}

	return 0;
}
#else
static int mdp3_modeset_init_dsi(struct mdp3_kms *mdp3_kms)
{
	return 0;
}
#endif

static int modeset_init(struct mdp3_kms *mdp3_kms)
{
	struct drm_device *dev = mdp3_kms->dev;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	int ret;

	plane = mdp3_plane_init(dev);
	if (IS_ERR(plane)) {
		DRM_DEV_ERROR(dev->dev, "failed to construct plane\n");
		return PTR_ERR(plane);
	}

	crtc = mdp3_crtc_init(dev, plane);
	if (IS_ERR(crtc)) {
		DRM_DEV_ERROR(dev->dev, "failed to construct crtc\n");
		return PTR_ERR(crtc);
	}

	ret = mdp3_modeset_init_dsi(mdp3_kms);
	if (ret)
		return ret;

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	return 0;
}

static int mdp3_kms_init(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(priv->kms));
	struct msm_kms *kms = NULL;
	struct drm_gpuvm *vm;
	int ret;

	ret = mdp_kms_init(&mdp3_kms->base, &kms_funcs);
	if (ret) {
		DRM_DEV_ERROR(dev->dev, "failed to init kms\n");
		goto fail;
	}

	kms = priv->kms;
	mdp3_kms->dev = dev;

	/* MDP3 reads through the apps SMMU (mdp_0 context bank) so scanout
	 * buffers do not need to be physically contiguous */
	vm = msm_kms_init_vm(mdp3_kms->dev, NULL);
	if (IS_ERR(vm)) {
		ret = PTR_ERR(vm);
		DRM_DEV_ERROR(dev->dev, "kms vm init failed (no display iommu?): %d\n", ret);
		goto fail;
	}
	kms->vm = vm;

	pm_runtime_enable(dev->dev);
	mdp3_kms->rpm_enabled = true;

	/* hold the display power domain (MDSS_GDSC, also feeds DSI+PHY)
	 * for the lifetime of the driver: between commits nothing else
	 * would keep it on, and losing it would wipe the state the msm
	 * dsi driver programmed into the DSI ctrl/PHY */
	pm_runtime_get_sync(dev->dev);

	ret = modeset_init(mdp3_kms);
	if (ret) {
		DRM_DEV_ERROR(dev->dev, "modeset_init failed: %d\n", ret);
		goto fail;
	}

	return 0;

fail:
	if (kms)
		mdp3_destroy(kms);

	return ret;
}

static const struct dev_pm_ops mdp3_pm_ops = {
	.prepare = msm_kms_pm_prepare,
	.complete = msm_kms_pm_complete,
};

static int mdp3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mdp3_kms *mdp3_kms;
	int irq;

	mdp3_kms = devm_kzalloc(dev, sizeof(*mdp3_kms), GFP_KERNEL);
	if (!mdp3_kms)
		return -ENOMEM;

	mdp3_kms->mmio = msm_ioremap(pdev, "core");
	if (IS_ERR(mdp3_kms->mmio))
		return PTR_ERR(mdp3_kms->mmio);

	mdp3_kms->vbif = msm_ioremap(pdev, "vbif");
	if (IS_ERR(mdp3_kms->vbif))
		return PTR_ERR(mdp3_kms->vbif);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get irq\n");

	mdp3_kms->base.base.irq = irq;

	mdp3_kms->core_clk = devm_clk_get(dev, "core_clk");
	if (IS_ERR(mdp3_kms->core_clk))
		return dev_err_probe(dev, PTR_ERR(mdp3_kms->core_clk), "failed to get core_clk\n");

	mdp3_kms->iface_clk = devm_clk_get(dev, "iface_clk");
	if (IS_ERR(mdp3_kms->iface_clk))
		return dev_err_probe(dev, PTR_ERR(mdp3_kms->iface_clk), "failed to get iface_clk\n");

	mdp3_kms->bus_clk = devm_clk_get(dev, "bus_clk");
	if (IS_ERR(mdp3_kms->bus_clk))
		return dev_err_probe(dev, PTR_ERR(mdp3_kms->bus_clk), "failed to get bus_clk\n");

	mdp3_kms->vsync_clk = devm_clk_get_optional(dev, "vsync_clk");
	if (IS_ERR(mdp3_kms->vsync_clk))
		return dev_err_probe(dev, PTR_ERR(mdp3_kms->vsync_clk), "failed to get vsync_clk\n");

	/* m3d ROOT CAUSE FIX: the late-init clk_disable_unused sweep gates
	 * every clock with enable_count==0.  The mdp3 driver never enabled
	 * any of its clocks (it surfed on lk2nd's leftover state); MDP/AXI/
	 * AHB only survived because the msm dsi driver holds them as its
	 * bus clocks - GCC_MDSS_VSYNC_CLK was held by NOBODY.  The sweep
	 * gated it at ~6.4s ("clk: Disabling unused clocks") and the very
	 * next read of the autorefresh/sync block (AUTOREFRESH_CONFIG_P
	 * @0x34c) hung the bus and hard-reset the SoC (m3a/m3b/m3c: death
	 * pinpointed to that exact readl, only ever AFTER the sweep line).
	 * Own the clocks like mdp4/mdp5 do: take a runtime refcount on all
	 * four.  clk_prepare_enable on hw-already-on branches is harmless
	 * (re-writes a set enable bit); the dangerous direction is the
	 * disable of an on-hw/zero-count clock, which is what the sweep
	 * did.  These refs are never dropped - bring-up simplicity, and
	 * the display pipe has no reason to ever be clock-gated mid-run. */
	{
		struct clk *clks[] = { mdp3_kms->core_clk, mdp3_kms->iface_clk,
				       mdp3_kms->bus_clk, mdp3_kms->vsync_clk };
		int i, ret;

		for (i = 0; i < ARRAY_SIZE(clks); i++) {
			if (!clks[i])
				continue;
			ret = clk_prepare_enable(clks[i]);
			if (ret) {
				dev_err(dev, "failed to enable clock %d: %d\n", i, ret);
				return ret;
			}
		}
	}

	/* the command-mode kick writes the DSI MDP_SW_TRIGGER register the
	 * msm dsi host already owns; plain ioremap, no request_mem_region */
	mdp3_kms->dsi_ctrl = devm_ioremap(dev, 0x01ac8000, SZ_4K);
	if (!mdp3_kms->dsi_ctrl)
		return -ENOMEM;

	return msm_drv_probe(&pdev->dev, mdp3_kms_init, &mdp3_kms->base.base);
}

static void mdp3_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &msm_drm_ops);
}

static const struct of_device_id mdp3_dt_match[] = {
	{ .compatible = "qcom,mdp3" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mdp3_dt_match);

static struct platform_driver mdp3_platform_driver = {
	.probe      = mdp3_probe,
	.remove     = mdp3_remove,
	.shutdown   = msm_kms_shutdown,
	.driver     = {
		.name   = "mdp3",
		.of_match_table = mdp3_dt_match,
		.pm     = &mdp3_pm_ops,
	},
};

void __init msm_mdp3_register(void)
{
	platform_driver_register(&mdp3_platform_driver);
}

void __exit msm_mdp3_unregister(void)
{
	platform_driver_unregister(&mdp3_platform_driver);
}
