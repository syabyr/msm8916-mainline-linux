// SPDX-License-Identifier: GPL-2.0-only
/*
 * MDP3 DSI encoder (MSM8909).
 *
 * Deliberately nearly empty: MDP3 drives the DSI in command mode, so
 * the panel scans its own GRAM and there are no video timings for the
 * encoder to program.  The kick path (DMA_P_START + DSI
 * MDP_SW_TRIGGER) runs from the KMS flush_commit; all this file has
 * to provide is the drm_encoder object that ties the CRTC to the
 * msm dsi host's bridge chain.
 */

#include <linux/printk.h>

#include <drm/drm_crtc.h>
#include <drm/drm_probe_helper.h>

#include "mdp3_kms.h"

#ifdef CONFIG_DRM_MSM_DSI

struct mdp3_dsi_encoder {
	struct drm_encoder base;
	bool enabled;
};
#define to_mdp3_dsi_encoder(x) container_of(x, struct mdp3_dsi_encoder, base)

static void mdp3_dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct mdp3_dsi_encoder *mdp3_dsi_encoder = to_mdp3_dsi_encoder(encoder);

	mdp3_dsi_encoder->enabled = true;

	DRM_DEV_INFO(encoder->dev->dev, "MDP3DBG encoder enable");
	w1a_mb("encoder enable");
}

static void mdp3_dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct mdp3_dsi_encoder *mdp3_dsi_encoder = to_mdp3_dsi_encoder(encoder);

	mdp3_dsi_encoder->enabled = false;
}

static const struct drm_encoder_helper_funcs mdp3_dsi_encoder_helper_funcs = {
	.enable = mdp3_dsi_encoder_enable,
	.disable = mdp3_dsi_encoder_disable,
};

/* initialize encoder */
struct drm_encoder *mdp3_dsi_encoder_init(struct drm_device *dev)
{
	struct mdp3_dsi_encoder *mdp3_dsi_encoder;
	struct drm_encoder *encoder;

	mdp3_dsi_encoder = drmm_encoder_alloc(dev, struct mdp3_dsi_encoder, base,
					      NULL, DRM_MODE_ENCODER_DSI, NULL);
	if (IS_ERR(mdp3_dsi_encoder))
		return ERR_CAST(mdp3_dsi_encoder);

	encoder = &mdp3_dsi_encoder->base;

	drm_encoder_helper_add(encoder, &mdp3_dsi_encoder_helper_funcs);

	return encoder;
}
#endif /* CONFIG_DRM_MSM_DSI */
