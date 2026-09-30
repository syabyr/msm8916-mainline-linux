// SPDX-License-Identifier: GPL-2.0-only
/*
 * MDP3 CRTC (MSM8909).
 *
 * The CRTC is the DMA_P pipe's frame pacing: the KMS flush_commit kicks
 * the fetch (DMA_P_START; DSI MDP_SW_TRIGGER is poked on top as a
 * belt-and-braces kick - downstream 8909 relies on the implicit trigger
 * alone), the MDP3 interrupt reports DMA_P_DONE, and that doubles as
 * vblank.  There is no mixer or interface timing to program - command
 * mode means the panel scans its own GRAM and the DSI host (already
 * driven by the msm dsi driver) owns the link side.
 */

#include <linux/printk.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mode.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "mdp3_kms.h"

/* Bring-up bisect switch: "w1a.nokick" on the cmdline skips the DMA fetch
 * kick entirely (everything else - DSI host power on, PHY, panel init -
 * still runs).  If the system survives to userspace with this set, the
 * silent-freeze death is the first scanout fetch through the SMMU, not
 * the DSI link. */
static bool mdp3_nokick;
static int __init mdp3_nokick_setup(char *s)
{
	mdp3_nokick = true;
	return 1;
}
__setup("w1a.nokick", mdp3_nokick_setup);

/* m3j: first-kick-only marker (see mdp3_crtc_kick) */
static unsigned int kick_count;

struct mdp3_crtc {
	struct drm_crtc base;

	bool enabled;

	/* if there is a pending flip, these will be non-null: */
	struct drm_pending_vblank_event *event;

	struct mdp_irq vblank;
};
#define to_mdp3_crtc(x) container_of(x, struct mdp3_crtc, base)

static struct mdp3_kms *get_kms(struct drm_crtc *crtc)
{
	struct msm_drm_private *priv = crtc->dev->dev_private;
	return to_mdp3_kms(to_mdp_kms(priv->kms));
}

/* the command-mode frame kick: start the DMA fetch and poke the DSI
 * command-mode MDP trigger (absolute 0x1ac8094 - see mdp3_hwio.h; the
 * old raw+0x90 poke hit the DSI command DMA trigger instead, which is
 * the register the msm dsi host uses for panel-init transfers). */
void mdp3_crtc_kick(struct drm_crtc *crtc)
{
	struct mdp3_kms *mdp3_kms = get_kms(crtc);

	/* Markers are BARE pr_info on purpose: the first atomic commit runs
	 * under console_lock (fbcon takeover), and pr_flush() takes
	 * console_lock internally - a pr_flush() here would deadlock the
	 * machine (this is exactly what killed M2 boots v3..v7).  The
	 * W1A watchdog panic replay is what makes these visible if the
	 * commit hangs. */
	if (mdp3_nokick) {
		DRM_DEV_INFO(crtc->dev->dev, "MDP3DBG kick: SKIPPED (w1a.nokick)");
		w1a_mb("kick SKIPPED (nokick)");
		return;
	}

	/* m3j: per-kick markers removed (13 lines/frame at 30fps drowned
	 * dmesg).  The pre-start/trig reads only ever served the bring-up
	 * bisects; keep one first-kick-only line as pipe-alive evidence. */
	if (!kick_count++) {
		DRM_DEV_INFO(crtc->dev->dev,
			     "MDP3DBG kick: first (start=%08x trig=%08x trigctrl=%08x)",
			     mdp3_read(mdp3_kms, REG_MDP3_DMA_P_START),
			     readl(mdp3_kms->dsi_ctrl + REG_DSI_CMD_MODE_MDP_SW_TRIGGER),
			     readl(mdp3_kms->dsi_ctrl + REG_DSI_TRIG_CTRL));
		w1a_mb("kick: first");
	}

	mdp3_write(mdp3_kms, REG_MDP3_DMA_P_START, 1);
	mdp3_dsi_trigger(mdp3_kms);
}

/* if file!=NULL, this is preclose potential cancel-flip path */
static void complete_flip(struct drm_crtc *crtc, struct drm_file *file)
{
	struct mdp3_crtc *mdp3_crtc = to_mdp3_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = mdp3_crtc->event;
	if (event) {
		mdp3_crtc->event = NULL;
		DBG("send event: %p", event);
		drm_crtc_send_vblank_event(crtc, event);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static void mdp3_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct mdp3_crtc *mdp3_crtc = to_mdp3_crtc(crtc);
	unsigned long flags;

	DBG("");

	if (WARN_ON(!mdp3_crtc->enabled))
		return;

	/* Disable/save vblank irq handling before power is disabled */
	drm_crtc_vblank_off(crtc);

	if (crtc->state->event && !crtc->state->active) {
		WARN_ON(mdp3_crtc->event);
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}

	mdp3_crtc->enabled = false;
}

static void mdp3_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct mdp3_crtc *mdp3_crtc = to_mdp3_crtc(crtc);

	DRM_DEV_INFO(crtc->dev->dev, "MDP3DBG crtc atomic_enable");

	if (WARN_ON(mdp3_crtc->enabled))
		return;

	/* Restore vblank irq handling after power is enabled */
	drm_crtc_vblank_on(crtc);

	mdp3_crtc->enabled = true;

	/* The first frame kick happens from the DSI encoder's enable,
	 * which runs after the DSI host bridge is enabled - see
	 * mdp3_dsi_encoder.c */
}

static int mdp3_crtc_atomic_check(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	return 0;
}

static void mdp3_crtc_atomic_begin(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
}

static void mdp3_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct mdp3_crtc *mdp3_crtc = to_mdp3_crtc(crtc);
	struct mdp3_kms *mdp3_kms = get_kms(crtc);
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	DBG("event: %p", crtc->state->event);

	WARN_ON(mdp3_crtc->event);

	spin_lock_irqsave(&dev->event_lock, flags);
	mdp3_crtc->event = crtc->state->event;
	crtc->state->event = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	/* request a vblank irq to complete the flip.  The actual frame
	 * kick happens later from mdp3_flush_commit() - after the bridge
	 * chain has been enabled, see the commit tail ordering */
	mdp_irq_register(&mdp3_kms->base, &mdp3_crtc->vblank);
}

static const struct drm_crtc_funcs mdp3_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank  = msm_crtc_enable_vblank,
	.disable_vblank = msm_crtc_disable_vblank,
};

static const struct drm_crtc_helper_funcs mdp3_crtc_helper_funcs = {
	.atomic_check = mdp3_crtc_atomic_check,
	.atomic_begin = mdp3_crtc_atomic_begin,
	.atomic_flush = mdp3_crtc_atomic_flush,
	.atomic_enable = mdp3_crtc_atomic_enable,
	.atomic_disable = mdp3_crtc_atomic_disable,
};

static void mdp3_crtc_vblank_irq(struct mdp_irq *irq, uint32_t irqstatus)
{
	struct mdp3_crtc *mdp3_crtc = container_of(irq, struct mdp3_crtc, vblank);
	struct drm_crtc *crtc = &mdp3_crtc->base;

	DBG("irqstatus=%08x", irqstatus);

	mdp_irq_unregister(&get_kms(crtc)->base, &mdp3_crtc->vblank);

	complete_flip(crtc, NULL);
}

void mdp3_crtc_wait_for_flush_done(struct drm_crtc *crtc)
{
	struct mdp3_crtc *mdp3_crtc = to_mdp3_crtc(crtc);
	wait_queue_head_t *queue = drm_crtc_vblank_waitqueue(crtc);
	int ret;

	if (!mdp3_crtc->enabled)
		return;

	ret = drm_crtc_vblank_get(crtc);
	if (ret)
		return;

	ret = wait_event_timeout(*queue, !mdp3_crtc->event,
				 msecs_to_jiffies(50));
	if (ret <= 0) {
		struct mdp3_kms *kms = get_kms(crtc);

		dev_warn(crtc->dev->dev, "MDP3DBG vblank timeout\n");
		w1a_mb("wait_for_flush_done: TIMEOUT (no DMA_P_DONE)");
		/* m2y (M3): discriminate WHY DMA_P_DONE never dispatches:
		 *  (a) fetch wedged: DMA_P_START still set (never consumed)
		 *  (b) fetch done + IRQ masked: INTR_STATUS has BIT14 latched
		 *      while INTR_ENABLE lacks it
		 *  (c) GIC 72 dead: status clean, enable armed, start clear
		 * Also AUTOREFRESH (lk2nd left it armed; it re-fetches on its
		 * own 12fps schedule and its done is BIT25, not BIT14) and the
		 * DSI-side state (cmd-mode MDP engine busy?).
		 */
		dev_warn(crtc->dev->dev,
			 "MDP3DBG vblank TO dump: start=%08x intr_en=%08x intr_st=%08x autorefresh=%08x\n",
			 mdp3_read(kms, REG_MDP3_DMA_P_START),
			 mdp3_read(kms, REG_MDP3_INTR_ENABLE),
			 mdp3_read(kms, REG_MDP3_INTR_STATUS),
			 mdp3_read(kms, REG_MDP3_AUTOREFRESH_CONFIG_P));
		dev_warn(crtc->dev->dev,
			 "MDP3DBG vblank TO dump2: dsi_ctrl=%08x dsi_status0=%08x dsi_fifo=%08x\n",
			 readl(kms->dsi_ctrl),
			 readl(kms->dsi_ctrl + 0x04),
			 readl(kms->dsi_ctrl + 0x08));
	}

	drm_crtc_vblank_put(crtc);
}

/* initialize crtc */
struct drm_crtc *mdp3_crtc_init(struct drm_device *dev,
				struct drm_plane *plane)
{
	struct mdp3_crtc *mdp3_crtc;
	struct drm_crtc *crtc;

	mdp3_crtc = drmm_crtc_alloc_with_planes(dev, struct mdp3_crtc, base,
						plane, NULL,
						&mdp3_crtc_funcs, NULL);
	if (IS_ERR(mdp3_crtc))
		return ERR_CAST(mdp3_crtc);

	crtc = &mdp3_crtc->base;

	mdp3_crtc->vblank.irqmask = MDP3_INTR_DMA_P_DONE;
	mdp3_crtc->vblank.irq = mdp3_crtc_vblank_irq;

	drm_crtc_helper_add(crtc, &mdp3_crtc_helper_funcs);

	return crtc;
}
