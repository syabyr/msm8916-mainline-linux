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
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/kernel_stat.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/panic.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/stdarg.h>
#include <linux/workqueue.h>

#include <drm/drm_dumb_buffers.h>
#include <drm/drm_vblank.h>

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_mmu.h"
#include "mdp3_kms.h"

/*
 * W1A bring-up watchdog, v2.  The M2 black-screen boots all ended in a
 * silent freeze; this tells a hard bus hang apart from a wedged CPU:
 *
 *  - a delayed work heartbeat on the boot CPU advances a counter every
 *    500ms (bare pr_info: must never block, never take console_lock);
 *  - an hrtimer stall-checker armed SYNCHRONOUSLY on the LAST cpu via
 *    smp_call_function_single() - so a hard hang of the cpu running the
 *    deferred-probe kworker (typically cpu0) cannot take the watchdog
 *    down with it.  Four stalled ticks => panic().  panic() with
 *    panic_console_replay=1 force-flushes the whole printk ringbuffer
 *    (including bare records emitted under console_lock) and
 *    panic_sys_info= dumps blocked tasks and all CPU backtraces to the
 *    ramoops console.
 *
 *  CRITICAL LESSON (M2 boots v3..v7): the first atomic commit runs under
 *  console_lock (fbcon takeover: register_framebuffer -> fbcon_fb_registered
 *  holds the lock across do_fbcon_takeover -> fbcon_init -> set_par ->
 *  drm_client_modeset_commit).  pr_flush() takes console_lock internally
 *  (kernel/printk/printk.c) => any pr_flush() reachable from an atomic
 *  commit DEADLOCKS THE MACHINE.  Never put pr_flush() in kms/encoder/
 *  bridge/panel/plane code; bare pr_info only - the panic replay above
 *  is what makes them visible.
 */
static atomic_t w1a_hb_count = ATOMIC_INIT(0);
static unsigned int w1a_hb_last;
static unsigned int w1a_hb_stalls;
static struct delayed_work w1a_hb_work;
static struct hrtimer w1a_hb_timer;

/*
 * W1A bring-up mailbox, v3.  printk records only reach the ramoops
 * console via the printk kthread, which needs console_lock - and the
 * first atomic commit runs under console_lock (fbcon takeover), so a
 * hang anywhere in there freezes the console log at the takeover entry
 * (m2k/m2l).  This mailbox writes ASCII trace lines DIRECTLY into the
 * lk2nd ramoops region (dump-area offset +200K, well clear of the
 * pstore console record and of panic dump record 0), bypassing printk
 * entirely.  Post-mortem readout:
 *
 *    fastboot oem ramoops raw && fastboot get_staged ram.bin
 *    grep -a W1AMB ram.bin
 *
 * Heartbeat beats + commit-chain markers both land here, so one raw
 * dump after a freeze shows both WHERE the chain stopped and WHETHER
 * the boot cpu kept living (heartbeats continuing => sleeping
 * deadlock; heartbeats stopped => hard cpu/bus hang).
 */
static void __iomem *w1a_mb_mem;
static u32 w1a_mb_off;
static u32 w1a_mb_seq;
static DEFINE_RAW_SPINLOCK(w1a_mb_lock);

static void w1a_mb_init(void)
{
	struct device_node *np;
	struct resource regs;
	resource_size_t base;

	np = of_find_compatible_node(NULL, NULL, "ramoops");
	if (!np)
		goto none;
	if (of_address_to_resource(np, 0, &regs)) {
		of_node_put(np);
		goto none;
	}
	of_node_put(np);
	base = regs.start;

	/* +200K sits in the dump-record area (records only written by
	 * pstore on panic, record 0 first) - never touched while alive.
	 * m2t: memremap, not ioremap - the ramoops region is RAM and
	 * arm32 ioremap refuses RAM pages (mailbox was dead in m2s). */
	w1a_mb_mem = memremap(base + 200 * 1024, 4096, MEMREMAP_WC);
	pr_info("MDP3DBG mb: ramoops %pa, mailbox at +200K\n", &base);
	pr_flush(1000, true);
	return;
none:
	pr_info("MDP3DBG mb: no ramoops node, mailbox disabled\n");
	pr_flush(1000, true);
}

/* safe from any context, incl. atomic and while console_lock is held */
__printf(1, 2)
void w1a_mb(const char *fmt, ...)
{
	char buf[160];
	va_list ap;
	unsigned long flags;
	int n;

	if (!w1a_mb_mem)
		return;

	n = scnprintf(buf, sizeof(buf), "W1AMB%04u ", ++w1a_mb_seq);
	va_start(ap, fmt);
	n += vscnprintf(buf + n, sizeof(buf) - n, fmt, ap);
	va_end(ap);
	if (n < sizeof(buf) - 1)
		buf[n++] = '\n';

	raw_spin_lock_irqsave(&w1a_mb_lock, flags);
	if (w1a_mb_off + n <= 4096) {
		memcpy_toio(w1a_mb_mem + w1a_mb_off, buf, n);
		w1a_mb_off += n;
	} else {
		/* wrap to the start of the 4K window */
		int tail = 4096 - w1a_mb_off;

		memcpy_toio(w1a_mb_mem + w1a_mb_off, buf, tail);
		memcpy_toio(w1a_mb_mem, buf + tail, n - tail);
		w1a_mb_off = n - tail;
	}
	raw_spin_unlock_irqrestore(&w1a_mb_lock, flags);
}
EXPORT_SYMBOL_GPL(w1a_mb);

static void w1a_hb_fn(struct work_struct *work)
{
	int n = atomic_inc_return(&w1a_hb_count);

	/* m3j: silent beat.  The counter, not the print, is what the
	 * last-cpu watchdog checks; the bring-up heartbeat prints (every
	 * 0.5s, 10-line reg dump + TE report every 5s) drowned dmesg now
	 * that the display works.  If a hang shows up again, re-add prints
	 * here - the m3-series bisects relied on them. */
	(void)n;
	schedule_delayed_work(&w1a_hb_work, msecs_to_jiffies(500));
}

static enum hrtimer_restart w1a_hb_timer_fn(struct hrtimer *timer)
{
	unsigned int now = atomic_read(&w1a_hb_count);

	if (now != w1a_hb_last) {
		w1a_hb_last = now;
		w1a_hb_stalls = 0;
	} else if (++w1a_hb_stalls >= 4) {
		w1a_mb("WATCHDOG PANIC: hb stalled at %u", now);
		panic("MDP3DBG heartbeat stalled (hard cpu hang?) hb=%u",
		      now);
	}

	hrtimer_forward_now(timer, ms_to_ktime(1000));
	return HRTIMER_RESTART;
}

/* runs on the LAST cpu: the watchdog must outlive a hard hang of the
 * deferred-probe kworker's cpu */
static void w1a_arm_watchdog(void *unused)
{
	hrtimer_setup(&w1a_hb_timer, w1a_hb_timer_fn, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);
	hrtimer_start(&w1a_hb_timer, ms_to_ktime(1000), HRTIMER_MODE_REL);
}

static void w1a_watchdog_start(void)
{
	unsigned int cpu = num_possible_cpus() - 1;

	INIT_DELAYED_WORK(&w1a_hb_work, w1a_hb_fn);
	schedule_delayed_work(&w1a_hb_work, msecs_to_jiffies(500));
	/* synchronous: the last-cpu hrtimer is live before this returns,
	 * so even an immediate atomic hang on this cpu still gets caught */
	smp_call_function_single(cpu, w1a_arm_watchdog, NULL, true);
	pr_info("MDP3DBG watchdog started (hb 500ms on boot cpu, stall panic after 4s on cpu%u)\n", cpu);
	pr_flush(1000, true);
}

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
	pr_info("MDP3DBG m2v pre-fix: CGC_EN=%08x VBIF_FORCE_EN=%08x\n",
		mdp3_read(mdp3_kms, REG_MDP3_CGC_EN),
		vbif_read(mdp3_kms, REG_MDP3_VBIF_FORCE_EN));
	mdp3_write(mdp3_kms, REG_MDP3_CGC_EN,
		   mdp3_read(mdp3_kms, REG_MDP3_CGC_EN) & ~(BIT(10) | BIT(18)));
	vbif_write(mdp3_kms, REG_MDP3_VBIF_FORCE_EN, 0x3);
	pr_info("MDP3DBG m2v post-fix: CGC_EN=%08x VBIF_FORCE_EN=%08x\n",
		mdp3_read(mdp3_kms, REG_MDP3_CGC_EN),
		vbif_read(mdp3_kms, REG_MDP3_VBIF_FORCE_EN));

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
		pr_info("MDP3DBG m3g te: sync_cfg=%08x vsync_sel=%08x init=%08x rdptr=%08x thresh=%08x startpos=%08x tc_en=%08x\n",
			mdp3_read(mdp3_kms, REG_MDP3_SYNC_CONFIG_0),
			mdp3_read(mdp3_kms, REG_MDP3_VSYNC_SEL),
			mdp3_read(mdp3_kms, REG_MDP3_PRIMARY_VSYNC_INIT_VAL),
			mdp3_read(mdp3_kms, REG_MDP3_PRIMARY_RD_PTR_IRQ),
			mdp3_read(mdp3_kms, REG_MDP3_SYNC_THRESH_0),
			mdp3_read(mdp3_kms, REG_MDP3_PRIMARY_START_POS),
			mdp3_read(mdp3_kms, REG_MDP3_TEAR_CHECK_EN));
		w1a_mb("te block programmed");
	}

	/* lk2nd arms the DMA_P autorefresh fetcher before jumping to the
	 * kernel (AUTOREFRESH_CONFIG_P=0x10000001, IBUF=lk2nd's buffer at
	 * 0x83200000).  m2k/m2l DISARMED it here and those are exactly the
	 * boots that die inside the first commit - writing 0 to the config
	 * while a refresh cycle is mid-flight is not a documented-safe
	 * operation, and the v2 boot survived with it left armed (SMMU
	 * attached, 160ms of runtime).  So: hands off.  Once the commit
	 * rewrites DMA_P config + IBUF, autorefresh refreshes OUR buffer -
	 * which is what command-mode panels want anyway. */
	version = mdp3_read(mdp3_kms, REG_MDP3_HW_VERSION);
	DRM_DEV_INFO(dev->dev, "MDP3 HW_VERSION 0x%08x", version);
	pr_info("MDP3DBG hw_init done (autorefresh LEFT ARMED)\n");
	w1a_mb("hw_init done");
	pr_flush(1000, true);

	pm_runtime_put_sync(dev->dev);

	return 0;
}

static void mdp3_dump_show(struct mdp3_kms *mdp3_kms, const char *tag)
{
	struct drm_device *dev = mdp3_kms->dev;

	DRM_DEV_INFO(dev->dev,
		     "%s: INTR en=%08x st=%08x | DMA_P cfg=%08x size=%08x ibuf=%08x stride=%08x xy=%08x | AUTOREFRESH=%08x",
		     tag,
		     mdp3_read(mdp3_kms, REG_MDP3_INTR_ENABLE),
		     mdp3_read(mdp3_kms, REG_MDP3_INTR_STATUS),
		     mdp3_read(mdp3_kms, REG_MDP3_DMA_P_CONFIG),
		     mdp3_read(mdp3_kms, REG_MDP3_DMA_P_SIZE),
		     mdp3_read(mdp3_kms, REG_MDP3_DMA_P_IBUF_ADDR),
		     mdp3_read(mdp3_kms, REG_MDP3_DMA_P_IBUF_Y_STRIDE),
		     mdp3_read(mdp3_kms, REG_MDP3_DMA_P_OUT_XY),
		     mdp3_read(mdp3_kms, REG_MDP3_AUTOREFRESH_CONFIG_P));
}

void mdp3_kms_dump(struct mdp3_kms *mdp3_kms, const char *tag)
{
	pm_runtime_get_sync(mdp3_kms->dev->dev);
	mdp3_dump_show(mdp3_kms, tag);
	pm_runtime_put_sync(mdp3_kms->dev->dev);
}

static irqreturn_t mdp3_irq(struct msm_kms *kms)
{
	struct mdp3_kms *mdp3_kms = to_mdp3_kms(to_mdp_kms(kms));
	struct drm_device *dev = mdp3_kms->dev;
	struct drm_crtc *crtc;
	uint32_t status, enable;

	enable = mdp3_read(mdp3_kms, REG_MDP3_INTR_ENABLE);
	status = mdp3_read(mdp3_kms, REG_MDP3_INTR_STATUS) & enable;

	/* m3e print, m3g-rate-limited: with TE flowing at 60Hz an
	 * unconditional print recreates the ring-scroll problem that
	 * cost us the m3e/m3f evidence.  TE/DONE are the expected
	 * steady-state bits - only anything ELSE is news. */
	if (status & ~(MDP3_INTR_SYNC_PRIMARY_LINE | MDP3_INTR_DMA_P_DONE))
		pr_info("MDP3DBG mdp3_irq: rawst=%08x en=%08x disp=%08x\n",
			status | (mdp3_read(mdp3_kms, REG_MDP3_INTR_STATUS) & ~enable),
			enable, status);

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

	/* dump what lk2nd left behind before the first commit rewrites it -
	 * the reference for the DMA_P field encodings on this panel */
	mdp3_kms_dump(mdp3_kms, "lk2nd state");

	/* W1A bring-up m2q: dump the FULL DSI controller state exactly as
	 * lk2nd left it (before the msm dsi driver touches anything).
	 * lk2nd drives this RM67162 panel successfully, so whatever is in
	 * these registers IS a known-good command-mode config; the -110
	 * fail dump in dsi_host.c prints the same registers after OUR
	 * setup, and the diff is the missing programming.  Raw bank
	 * offsets (base 0x1ac8000) = DSI xml offset + 4. */
	{
		void __iomem *d = mdp3_kms->dsi_ctrl;

		pr_info("MDP3DBG lk2nd dsi A: ctrl=%08x st0=%08x fifo=%08x dma_ctrl=%08x cfg0=%08x cfg1=%08x\n",
			readl(d + 0x004), readl(d + 0x008), readl(d + 0x00c),
			readl(d + 0x03c), readl(d + 0x040), readl(d + 0x044));
		pr_info("MDP3DBG lk2nd dsi B: base=%08x len=%08x wm50=%08x str0ctl=%08x str0tot=%08x str1ctl=%08x str1tot=%08x\n",
			readl(d + 0x048), readl(d + 0x04c), readl(d + 0x050),
			readl(d + 0x058), readl(d + 0x05c),
			readl(d + 0x060), readl(d + 0x064));
		pr_info("MDP3DBG lk2nd dsi C: ackerr=%08x trig=%08x lanest=%08x lanectl=%08x swap=%08x phyerr=%08x\n",
			readl(d + 0x068), readl(d + 0x084), readl(d + 0x0a8),
			readl(d + 0x0ac), readl(d + 0x0b0), readl(d + 0x0b4));
		pr_info("MDP3DBG lk2nd dsi D: lptim=%08x hstim=%08x tmo=%08x clkout=%08x eot=%08x\n",
			readl(d + 0x0b8), readl(d + 0x0bc), readl(d + 0x0c0),
			readl(d + 0x0c4), readl(d + 0x0cc));
		pr_info("MDP3DBG lk2nd dsi E: errmask=%08x intctl=%08x reset=%08x clkctl=%08x clkst=%08x phyrst=%08x\n",
			readl(d + 0x10c), readl(d + 0x110), readl(d + 0x118),
			readl(d + 0x11c), readl(d + 0x120), readl(d + 0x12c));
		pr_info("MDP3DBG lk2nd dsi F: preext=%08x mdpctl2=%08x str2ctl=%08x ver=%08x\n",
			readl(d + 0x180), readl(d + 0x1b8), readl(d + 0x1bc),
			readl(d + 0x1f4));
		/* MDP3 QoS remapper + panic regs (bank 0x1a90000): lk2nd's
		 * msm8909 target programs 0x1A9 into 0x1a90090 unconditionally
		 * (platform/msm8909/include/platform/iomap.h); mainline never
		 * touches it.  Power-on default vs 0x1A9 is a data point for
		 * whether the fetch QoS matters. */
		pr_info("MDP3DBG lk2nd mdp3 qos: remapper=%08x wm0=%08x wm1=%08x wm2=%08x panic=%08x lut0=%08x\n",
			mdp3_read(mdp3_kms, 0x90090),
			mdp3_read(mdp3_kms, 0x90094),
			mdp3_read(mdp3_kms, 0x90098),
			mdp3_read(mdp3_kms, 0x9009c),
			mdp3_read(mdp3_kms, 0x900a0),
			mdp3_read(mdp3_kms, 0x900a4));
	}

	ret = modeset_init(mdp3_kms);
	if (ret) {
		DRM_DEV_ERROR(dev->dev, "modeset_init failed: %d\n", ret);
		goto fail;
	}

	/* W1A bring-up: arm the heartbeat/panic watchdog and the ramoops
	 * mailbox (see the comments at the top of this file) so a silent
	 * freeze gets dissected */
	w1a_mb_init();
	w1a_watchdog_start();

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
		pr_info("MDP3DBG m3d: core/iface/bus/vsync clocks enabled (survive clk_disable_unused)\n");
	}

	/* the command-mode kick writes the DSI MDP_SW_TRIGGER register the
	 * msm dsi host already owns; plain ioremap, no request_mem_region */
	mdp3_kms->dsi_ctrl = devm_ioremap(dev, 0x01ac8000, SZ_4K);
	if (!mdp3_kms->dsi_ctrl)
		return -ENOMEM;

	/*
	 * m2u: capture the SMMU-path clock votes BEFORE msm_drv_probe runs
	 * mdp3_kms_init and the qcom_iommu attach (which now also enables
	 * the "tbu" clock from DT).  APCS SMMU CLOCK_BRANCH_ENA_VOTE
	 * @ 0x0184500c: BIT(1)=apss_tcu, BIT(4)=mdp_tbu.  Pure MMIO inside
	 * the gcc block; the clk framework writes this exact register for
	 * every voted branch enable (apss_tcu/gfx_tcu at qcom_iommu resume
	 * - proven live by the working GPU), so one readl from probe
	 * context is the same access class.
	 */
	{
		void __iomem *vote = ioremap(0x01845000, 0x10);

		if (vote) {
			u32 v = readl(vote + 0x0c);

			pr_info("MDP3DBG m2u APCS_SMMU_VOTE %08x: mdp_tbu(BIT4)=%d apss_tcu(BIT1)=%d\n",
				v, !!(v & BIT(4)), !!(v & BIT(1)));
			iounmap(vote);
		} else {
			pr_err("MDP3DBG m2u APCS_SMMU_VOTE: ioremap failed\n");
		}
	}

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
