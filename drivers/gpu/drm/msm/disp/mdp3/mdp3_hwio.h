/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MDP3 register map (MSM8909).
 *
 * Sources: downstream drivers/video/msm/mdss/mdp3_hwio.h (8909 drop),
 * lk2nd platform/msm_shared/mdp3.c (the configuration actually running
 * on the W1A panel), and live /dev/mem probing during B-full phase 0/1.
 *
 * Two reg spaces: "core" @ 0x1a00000 (includes the DMA_P block at
 * offset 0x90000) and "vbif" @ 0x1ab0000.
 */
#ifndef __MDP3_HWIO_H__
#define __MDP3_HWIO_H__

#include <linux/bits.h>

/* identity + reset */
#define REG_MDP3_HW_VERSION			0x0070
#define REG_MDP3_SW_RESET			0x0074

/* interrupts */
#define REG_MDP3_INTR_ENABLE			0x0020
#define REG_MDP3_INTR_STATUS			0x0024
#define REG_MDP3_INTR_CLEAR			0x0028

#define MDP3_INTR_DMA_P_DONE			BIT(14)
/* latched in INTR_STATUS when an autorefresh frame completes; observed
 * live, downstream INTR_AUTOREFRESH_DONE */
#define MDP3_INTR_AUTOREFRESH_DONE		BIT(25)
/* m3g: downstream mdp3_dma.c uses SYNC_PRIMARY_LINE (+ dma_sel, 0 for
 * DMA_P) as THE vsync irq for DSI_CMD output: the tear-check block
 * counts the panel's TE pulses (gpio24 muxed to mdp_vsync) against the
 * vsync clock and raises this when the line counter passes rd_ptr_irq.
 * This is the continuous vblank source command mode needs (a "vblank"
 * only exists per frame transfer otherwise, so every trailing
 * wait-for-vblank times out - the m2y/m3 WARN storm). */
#define MDP3_INTR_SYNC_PRIMARY_LINE		BIT(8)

/* operation control */
#define REG_MDP3_DMA_P_START			0x0044
#define REG_MDP3_DISPLAY_STATUS			0x0038

/* clock gating */
#define REG_MDP3_CGC_EN			0x0100

/* tear check (MDP3 "sync" block; programmed by downstream
 * mdp3_dma_vsync_cfg - our values in mdp3_hw_init, m3g) */
#define REG_MDP3_SYNC_THRESH_0			0x0200
#define REG_MDP3_TEAR_CHECK_EN			0x020c
#define REG_MDP3_PRIMARY_START_POS		0x0210
#define REG_MDP3_PRIMARY_RD_PTR_IRQ		0x021c
#define REG_MDP3_SYNC_CONFIG_0			0x0300
#define REG_MDP3_SYNC_STATUS_2			0x0314
#define REG_MDP3_VSYNC_SEL			0x0324
#define REG_MDP3_PRIMARY_VSYNC_INIT_VAL		0x0328
#define REG_MDP3_AUTOREFRESH_CONFIG_P		0x034c

/* DMA_P fetch engine (offset 0x90000 inside the core reg space) */
#define REG_MDP3_DMA_P_CONFIG			0x90000
#define REG_MDP3_DMA_P_SIZE			0x90004
#define REG_MDP3_DMA_P_IBUF_ADDR		0x90008
#define REG_MDP3_DMA_P_IBUF_Y_STRIDE		0x9000c
#define REG_MDP3_DMA_P_OUT_XY			0x90010

/* DMA_P_CONFIG fields (downstream mdp3_dmap_config + lk2nd cmd-mode
 * value 0x000821bf, proven on this panel):
 *   fmt<<25 | dither<<24 | out_sel<<19 | bit_mask_polarity<<18 |
 *   components_flip<<14 | pack_pattern<<8 | pack_align<<7 |
 *   color_comp_out_bits (0x3f = 8bpc GBR)
 * out_sel is 2 bits: DSI_CMD=1 (BIT(19) only; lk2nd's video-path
 * 0x1800bf has BIT(19)|BIT(20) = DSI_VIDEO).  pack_align: 0=LSB,1=MSB. */
#define MDP3_DMA_P_CONFIG_FMT_SHIFT		25
#define MDP3_DMA_P_FMT_RGB888			0
#define MDP3_DMA_P_FMT_RGB565			1
#define MDP3_DMA_P_FMT_XRGB8888			2
#define MDP3_DMA_P_CONFIG_OUT_SEL_SHIFT		19
#define MDP3_DMA_P_OUT_SEL_DSI_CMD		1
#define MDP3_DMA_P_CONFIG_PACK_SHIFT		8
#define MDP3_DMA_P_PACK_PATTERN_RGB		0x21
#define MDP3_DMA_P_PACK_PATTERN_BGR		0x12
#define MDP3_DMA_P_CONFIG_PACK_ALIGN_MSB	BIT(7)
#define MDP3_DMA_P_CONFIG_COMP_OUT_8BPC		0x3f
/* lk2nd mdp_dsi_cmd_config(): RGB888 + DSI_CMD, the live-proven value */
#define MDP3_DMA_P_CONFIG_RGB888_DSI_CMD	0x000821bf

/* VBIF (reg space "vbif" @ 0x1ab0000) */
#define REG_MDP3_VBIF_FORCE_EN			0x0004

/* DSI command-mode MDP trigger, in the DSI ctrl reg space.  The DSI 6G
 * v1.x register bank on MSM8909 starts at 0x1ac8004 (raw DT base + 4,
 * see DSI_6G_REG_SHIFT); within that bank this register is at +0x90.
 * The ioremap base below is the RAW base 0x1ac8000 (mdp3_probe), so the
 * raw offset is 0x94 - absolute 0x1ac8094.  (0x1ac8090, what the old
 * 0x0090 value hit, is the command DMA SW trigger TRIG_DMA, the register
 * the msm dsi host fires its panel-init command transfers through.)
 * Downstream 8909 never pokes this register at all (its frame kick is
 * DMA_P_START only, with DSI TRIG_CTRL MDP_TRIGGER=NONE = auto trigger);
 * this poke is a belt-and-braces kick on top of the implicit one. */
#define REG_DSI_CMD_MODE_MDP_SW_TRIGGER		0x0094

/* DSI TRIG_CTRL (raw-base offset; absolute 0x1ac8084) - printed by the
 * kick so the dsi_host.c TE-bit fix can be verified from the boot log
 * alone: the value must read 0x00000004 on W1A, not 0x80001004 */
#define REG_DSI_TRIG_CTRL			0x0084

#endif /* __MDP3_HWIO_H__ */
