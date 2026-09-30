// SPDX-License-Identifier: GPL-2.0-only
/*
 * MDP3 plane (MSM8909).
 *
 * One plane = the DMA_P fetch pipe.  It scans a linear RGB buffer
 * straight to the DSI (command mode), no scaling, no mixing: whatever
 * the framebuffer holds is the frame.  atomic_update programs the
 * 0x90000 register group; the CRTC's flush then kicks the DMA.
 *
 * The format table mirrors what lk2nd/the downstream driver actually
 * program: fmt enum in bits 25-27 of DMA_P_CONFIG, pack pattern in
 * bits 8-13 (lk2nd uses 0x21 = RGB for its RGB888 fb, colors verified
 * correct on this panel).
 */

#include <linux/printk.h>

#include <drm/drm_atomic.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>

#include "mdp3_kms.h"

struct mdp3_plane {
	struct drm_plane base;
};
#define to_mdp3_plane(x) container_of(x, struct mdp3_plane, base)

static struct mdp3_kms *get_kms(struct drm_plane *plane)
{
	struct msm_drm_private *priv = plane->dev->dev_private;
	return to_mdp3_kms(to_mdp_kms(priv->kms));
}

struct mdp3_format {
	u32 drm_fourcc;
	u32 fmt;		/* DMA_P_CONFIG fmt field */
	u32 pack_pattern;
};

static const struct mdp3_format mdp3_formats[] = {
	{ DRM_FORMAT_RGB888,    MDP3_DMA_P_FMT_RGB888,   MDP3_DMA_P_PACK_PATTERN_RGB },
	{ DRM_FORMAT_BGR888,    MDP3_DMA_P_FMT_RGB888,   MDP3_DMA_P_PACK_PATTERN_BGR },
	{ DRM_FORMAT_RGB565,    MDP3_DMA_P_FMT_RGB565,   MDP3_DMA_P_PACK_PATTERN_RGB },
	{ DRM_FORMAT_XRGB8888,  MDP3_DMA_P_FMT_XRGB8888, MDP3_DMA_P_PACK_PATTERN_RGB },
};

static const struct mdp3_format *mdp3_get_format(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mdp3_formats); i++)
		if (mdp3_formats[i].drm_fourcc == fourcc)
			return &mdp3_formats[i];

	return NULL;
}

static int mdp3_plane_prepare_fb(struct drm_plane *plane,
				 struct drm_plane_state *new_state)
{
	if (!new_state->fb)
		return 0;

	drm_gem_plane_helper_prepare_fb(plane, new_state);

	/* m3i: pass needs_dirtyfb=true.  mdp3 drives a command-mode panel:
	 * the display re-reads the framebuffer only on an explicit kick, so
	 * a damage-only update (same fb, only fb_damage_clips changed - the
	 * fbcon/fbdev path) must go through msm_framebuffer_dirtyfb ->
	 * drm_atomic_helper_dirtyfb -> atomic commit -> flush_commit kick.
	 * With false, msm_fb->dirtyfb stays at its init value of 1 and
	 * msm_framebuffer_dirtyfb() silently returns 0 - every fbcon damage
	 * commit is dropped and the console freezes on its first frame
	 * (dpu passes the encoder's needs_dirtyfb, true for cmd mode). */
	return msm_framebuffer_prepare(new_state->fb, true);
}

static void mdp3_plane_cleanup_fb(struct drm_plane *plane,
				  struct drm_plane_state *old_state)
{
	if (!old_state->fb)
		return;

	msm_framebuffer_cleanup(old_state->fb, true);
}

static int mdp3_plane_atomic_check(struct drm_plane *plane,
				   struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);

	if (!new_state->crtc || !new_state->fb)
		return 0;

	if (!mdp3_get_format(new_state->fb->format->format))
		return -EINVAL;

	/* DMA_P cannot scale: src must equal dst (in 16.16) */
	if (new_state->src_w != new_state->crtc_w << 16 ||
	    new_state->src_h != new_state->crtc_h << 16)
		return -EINVAL;

	return 0;
}

static void mdp3_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct mdp3_kms *mdp3_kms = get_kms(plane);
	const struct mdp3_format *format;
	struct drm_framebuffer *fb = new_state->fb;
	u32 cfg;

	if (!new_state->crtc || !fb)
		return;

	format = mdp3_get_format(fb->format->format);
	/* atomic_check has ensured this cannot fail */
	if (WARN_ON(!format))
		return;

	DBG("plane: FB[%u] %ux%u -> CRTC[%u]",
	    fb->base.id, fb->width, fb->height, new_state->crtc->base.id);

	cfg = format->fmt << MDP3_DMA_P_CONFIG_FMT_SHIFT;
	cfg |= MDP3_DMA_P_OUT_SEL_DSI_CMD << MDP3_DMA_P_CONFIG_OUT_SEL_SHIFT;
	cfg |= format->pack_pattern << MDP3_DMA_P_CONFIG_PACK_SHIFT;
	cfg |= MDP3_DMA_P_CONFIG_PACK_ALIGN_MSB;
	cfg |= MDP3_DMA_P_CONFIG_COMP_OUT_8BPC;

	mdp3_write(mdp3_kms, REG_MDP3_DMA_P_CONFIG, cfg);
	mdp3_write(mdp3_kms, REG_MDP3_DMA_P_SIZE,
		   fb->width | (fb->height << 16));
	mdp3_write(mdp3_kms, REG_MDP3_DMA_P_IBUF_ADDR,
		   msm_framebuffer_iova(fb, 0));
	mdp3_write(mdp3_kms, REG_MDP3_DMA_P_IBUF_Y_STRIDE, fb->pitches[0]);
	/* lk2nd writes OUT_XY 0; crtc_x/y are always 0 on a full-screen
	 * pipe but program them anyway like downstream does */
	mdp3_write(mdp3_kms, REG_MDP3_DMA_P_OUT_XY,
		   new_state->crtc_x | (new_state->crtc_y << 16));
}

static const struct drm_plane_funcs mdp3_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const struct drm_plane_helper_funcs mdp3_plane_helper_funcs = {
	.prepare_fb = mdp3_plane_prepare_fb,
	.cleanup_fb = mdp3_plane_cleanup_fb,
	.atomic_check = mdp3_plane_atomic_check,
	.atomic_update = mdp3_plane_atomic_update,
};

/* initialize plane */
struct drm_plane *mdp3_plane_init(struct drm_device *dev)
{
	struct mdp3_plane *mdp3_plane;
	struct drm_plane *plane;
	u32 formats[ARRAY_SIZE(mdp3_formats)];
	int i;

	for (i = 0; i < ARRAY_SIZE(mdp3_formats); i++)
		formats[i] = mdp3_formats[i].drm_fourcc;

	mdp3_plane = drmm_universal_plane_alloc(dev, struct mdp3_plane, base,
						0, &mdp3_plane_funcs,
						formats, ARRAY_SIZE(formats),
						NULL, DRM_PLANE_TYPE_PRIMARY,
						NULL);
	if (IS_ERR(mdp3_plane))
		return ERR_CAST(mdp3_plane);

	plane = &mdp3_plane->base;

	drm_plane_helper_add(plane, &mdp3_plane_helper_funcs);

	drm_plane_enable_fb_damage_clips(plane);

	return plane;
}
