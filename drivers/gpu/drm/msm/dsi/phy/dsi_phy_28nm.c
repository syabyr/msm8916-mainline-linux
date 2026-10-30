// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 */

#include <dt-bindings/clock/qcom,dsi-phy-28nm.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/printk.h>

#include "dsi_phy.h"
#include "dsi.xml.h"
#include "dsi_phy_28nm.xml.h"

/*
 * DSI PLL 28nm - clock diagram (eg: DSI0):
 *
 *         dsi0analog_postdiv_clk
 *                             |         dsi0indirect_path_div2_clk
 *                             |          |
 *                   +------+  |  +----+  |  |\   dsi0byte_mux
 *  dsi0vco_clk --o--| DIV1 |--o--| /2 |--o--| \   |
 *                |  +------+     +----+     | m|  |  +----+
 *                |                          | u|--o--| /4 |-- dsi0pllbyte
 *                |                          | x|     +----+
 *                o--------------------------| /
 *                |                          |/
 *                |          +------+
 *                o----------| DIV3 |------------------------- dsi0pll
 *                           +------+
 */

#define POLL_MAX_READS			10
#define POLL_TIMEOUT_US		50

#define VCO_REF_CLK_RATE		19200000
#define VCO_MIN_RATE			350000000
#define VCO_MAX_RATE			750000000

/* v2.0.0 28nm LP implementation */
#define DSI_PHY_28NM_QUIRK_PHY_LP	BIT(0)
#define DSI_PHY_28NM_QUIRK_PHY_8226	BIT(1)

#define LPFR_LUT_SIZE			10
struct lpfr_cfg {
	unsigned long vco_rate;
	u32 resistance;
};

/* Loop filter resistance: */
static const struct lpfr_cfg lpfr_lut[LPFR_LUT_SIZE] = {
	{ 479500000,  8 },
	{ 480000000, 11 },
	{ 575500000,  8 },
	{ 576000000, 12 },
	{ 610500000,  8 },
	{ 659500000,  9 },
	{ 671500000, 10 },
	{ 672000000, 14 },
	{ 708500000, 10 },
	{ 750000000, 11 },
};

struct pll_28nm_cached_state {
	unsigned long vco_rate;
	u8 postdiv3;
	u8 postdiv1;
	u8 byte_mux;
};

/*
 * W1A (v5): raw PLL configuration captured from the lk2nd handoff.
 * The mainline set_rate() seed math produces a DIFFERENT (locking, but
 * functionally wrong for this panel/PHY) configuration than lk2nd's
 * proven one - sdm1 0x0e vs 0x4e, postdiv3 0x2f vs 0x11 - leaving LP
 * commands working but the HS data path dead (black screen).  With
 * link-clk set_rate() skipped entirely (w1a_dsi_rateskip), the
 * restore path below rewrites these registers verbatim instead of
 * recomputing them.
 */
struct pll_28nm_raw_state {
	bool captured;
	u32 refclk;	/* 0x00 */
	u32 postdiv1;	/* 0x04 */
	u32 vreg;	/* 0x10 */
	u32 postdiv3;	/* 0x28 */
	u32 sdm0, sdm1, sdm2, sdm3, sdm4;	/* 0x38..0x48 */
	u32 cal_cfg0, cal_cfg1;		/* 0x6c, 0x70 */
};

struct dsi_pll_28nm {
	struct clk_hw clk_hw;

	struct msm_dsi_phy *phy;

	struct pll_28nm_cached_state cached_state;

	struct pll_28nm_raw_state raw;
};

#define to_pll_28nm(x)	container_of(x, struct dsi_pll_28nm, clk_hw)

static bool pll_28nm_poll_for_ready(struct dsi_pll_28nm *pll_28nm,
				u32 nb_tries, u32 timeout_us)
{
	bool pll_locked = false;
	u32 val;

	while (nb_tries--) {
		val = readl(pll_28nm->phy->pll_base + REG_DSI_28nm_PHY_PLL_STATUS);
		pll_locked = !!(val & DSI_28nm_PHY_PLL_STATUS_PLL_RDY);

		if (pll_locked)
			break;

		udelay(timeout_us);
	}
	DBG("DSI PLL is %slocked", pll_locked ? "" : "*not* ");

	return pll_locked;
}

static void pll_28nm_software_reset(struct dsi_pll_28nm *pll_28nm)
{
	void __iomem *base = pll_28nm->phy->pll_base;

	/*
	 * Add HW recommended delays after toggling the software
	 * reset bit off and back on.
	 */
	writel(DSI_28nm_PHY_PLL_TEST_CFG_PLL_SW_RESET, base + REG_DSI_28nm_PHY_PLL_TEST_CFG);
	udelay(1);
	writel(0, base + REG_DSI_28nm_PHY_PLL_TEST_CFG);
	udelay(1);
}

/*
 * Clock Callbacks
 */
static int dsi_pll_28nm_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);
	struct device *dev = &pll_28nm->phy->pdev->dev;
	void __iomem *base = pll_28nm->phy->pll_base;
	unsigned long div_fbx1000, gen_vco_clk;
	u32 refclk_cfg, frac_n_mode, frac_n_value;
	u32 sdm_cfg0, sdm_cfg1, sdm_cfg2, sdm_cfg3;
	u32 cal_cfg10, cal_cfg11;
	u32 rem;
	int i;

	VERB("rate=%lu, parent's=%lu", rate, parent_rate);

	/*
	 * W1A (v8): with lk2nd's raw handoff config captured at probe,
	 * program THOSE seeds instead of the computed ones.  Two effects:
	 * (a) on the boot handoff this writes the exact values already in
	 * the registers (zero disturbance to the running, locked PLL) while
	 * keeping the clk framework bookkeeping happy - unlike the v6/v7
	 * no-op variant, which let the byte-clock enable reach
	 * vco_prepare_lp with the "already locked" shortcut somehow
	 * missing and the subsequent software_reset wiped the seeds ->
	 * "DSI PLL lock failed" -> power_on -22;
	 * (b) after a real power cycle (DPMS off) the seeds are rewritten
	 * for the relock.  The computed math locks at a functionally
	 * different configuration (sdm1 0x4e vs 0x0e) that leaves the HS
	 * data path dead - black screen (v2-v4).
	 */
	if (pll_28nm->raw.captured) {
		struct pll_28nm_raw_state *r = &pll_28nm->raw;

		writel(r->refclk, base + 0x00);
		writel(r->postdiv1, base + 0x04);
		writel(r->postdiv3, base + 0x28);
		writel(r->sdm0, base + 0x38);
		writel(r->sdm1, base + 0x3c);
		writel(r->sdm2, base + 0x40);
		writel(r->sdm3, base + 0x44);
		writel(r->sdm4, base + 0x48);
		writel(r->cal_cfg0, base + 0x6c);
		writel(r->cal_cfg1, base + 0x70);
		pll_28nm->cached_state.vco_rate = rate;
		DRM_DEV_INFO(dev, "MDP3DBG pll set_rate(%lu) -> raw lk2nd seeds rewritten\n",
			     rate);
		return 0;
	}

	/* Force postdiv2 to be div-4 */
	writel(3, base + REG_DSI_28nm_PHY_PLL_POSTDIV2_CFG);

	/* Configure the Loop filter resistance */
	for (i = 0; i < LPFR_LUT_SIZE; i++)
		if (rate <= lpfr_lut[i].vco_rate)
			break;
	if (i == LPFR_LUT_SIZE) {
		DRM_DEV_ERROR(dev, "unable to get loop filter resistance. vco=%lu\n",
				rate);
		return -EINVAL;
	}
	writel(lpfr_lut[i].resistance, base + REG_DSI_28nm_PHY_PLL_LPFR_CFG);

	/* Loop filter capacitance values : c1 and c2 */
	writel(0x70, base + REG_DSI_28nm_PHY_PLL_LPFC1_CFG);
	writel(0x15, base + REG_DSI_28nm_PHY_PLL_LPFC2_CFG);

	rem = rate % VCO_REF_CLK_RATE;
	if (rem) {
		refclk_cfg = DSI_28nm_PHY_PLL_REFCLK_CFG_DBLR;
		frac_n_mode = 1;
		div_fbx1000 = rate / (VCO_REF_CLK_RATE / 500);
		gen_vco_clk = div_fbx1000 * (VCO_REF_CLK_RATE / 500);
	} else {
		refclk_cfg = 0x0;
		frac_n_mode = 0;
		div_fbx1000 = rate / (VCO_REF_CLK_RATE / 1000);
		gen_vco_clk = div_fbx1000 * (VCO_REF_CLK_RATE / 1000);
	}

	DBG("refclk_cfg = %d", refclk_cfg);

	rem = div_fbx1000 % 1000;
	frac_n_value = (rem << 16) / 1000;

	DBG("div_fb = %lu", div_fbx1000);
	DBG("frac_n_value = %d", frac_n_value);

	DBG("Generated VCO Clock: %lu", gen_vco_clk);
	rem = 0;
	sdm_cfg1 = readl(base + REG_DSI_28nm_PHY_PLL_SDM_CFG1);
	sdm_cfg1 &= ~DSI_28nm_PHY_PLL_SDM_CFG1_DC_OFFSET__MASK;
	if (frac_n_mode) {
		sdm_cfg0 = 0x0;
		sdm_cfg0 |= DSI_28nm_PHY_PLL_SDM_CFG0_BYP_DIV(0);
		sdm_cfg1 |= DSI_28nm_PHY_PLL_SDM_CFG1_DC_OFFSET(
				(u32)(((div_fbx1000 / 1000) & 0x3f) - 1));
		sdm_cfg3 = frac_n_value >> 8;
		sdm_cfg2 = frac_n_value & 0xff;
	} else {
		sdm_cfg0 = DSI_28nm_PHY_PLL_SDM_CFG0_BYP;
		sdm_cfg0 |= DSI_28nm_PHY_PLL_SDM_CFG0_BYP_DIV(
				(u32)(((div_fbx1000 / 1000) & 0x3f) - 1));
		sdm_cfg1 |= DSI_28nm_PHY_PLL_SDM_CFG1_DC_OFFSET(0);
		sdm_cfg2 = 0;
		sdm_cfg3 = 0;
	}

	DBG("sdm_cfg0=%d", sdm_cfg0);
	DBG("sdm_cfg1=%d", sdm_cfg1);
	DBG("sdm_cfg2=%d", sdm_cfg2);
	DBG("sdm_cfg3=%d", sdm_cfg3);

	cal_cfg11 = (u32)(gen_vco_clk / (256 * 1000000));
	cal_cfg10 = (u32)((gen_vco_clk % (256 * 1000000)) / 1000000);
	DBG("cal_cfg10=%d, cal_cfg11=%d", cal_cfg10, cal_cfg11);

	writel(0x02, base + REG_DSI_28nm_PHY_PLL_CHGPUMP_CFG);
	writel(0x2b, base + REG_DSI_28nm_PHY_PLL_CAL_CFG3);
	writel(0x06, base + REG_DSI_28nm_PHY_PLL_CAL_CFG4);
	writel(0x0d, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);

	writel(sdm_cfg1, base + REG_DSI_28nm_PHY_PLL_SDM_CFG1);
	writel(DSI_28nm_PHY_PLL_SDM_CFG2_FREQ_SEED_7_0(sdm_cfg2),
	       base + REG_DSI_28nm_PHY_PLL_SDM_CFG2);
	writel(DSI_28nm_PHY_PLL_SDM_CFG3_FREQ_SEED_15_8(sdm_cfg3),
	       base + REG_DSI_28nm_PHY_PLL_SDM_CFG3);
	writel(0, base + REG_DSI_28nm_PHY_PLL_SDM_CFG4);

	/* Add hardware recommended delay for correct PLL configuration */
	if (pll_28nm->phy->cfg->quirks & DSI_PHY_28NM_QUIRK_PHY_LP)
		udelay(1000);
	else
		udelay(1);

	writel(refclk_cfg, base + REG_DSI_28nm_PHY_PLL_REFCLK_CFG);
	writel(0x00, base + REG_DSI_28nm_PHY_PLL_PWRGEN_CFG);
	writel(0x31, base + REG_DSI_28nm_PHY_PLL_VCOLPF_CFG);
	writel(sdm_cfg0, base + REG_DSI_28nm_PHY_PLL_SDM_CFG0);
	writel(0x12, base + REG_DSI_28nm_PHY_PLL_CAL_CFG0);
	writel(0x30, base + REG_DSI_28nm_PHY_PLL_CAL_CFG6);
	writel(0x00, base + REG_DSI_28nm_PHY_PLL_CAL_CFG7);
	writel(0x60, base + REG_DSI_28nm_PHY_PLL_CAL_CFG8);
	writel(0x00, base + REG_DSI_28nm_PHY_PLL_CAL_CFG9);
	writel(cal_cfg10 & 0xff, base + REG_DSI_28nm_PHY_PLL_CAL_CFG10);
	writel(cal_cfg11 & 0xff, base + REG_DSI_28nm_PHY_PLL_CAL_CFG11);
	writel(0x20, base + REG_DSI_28nm_PHY_PLL_EFUSE_CFG);

	return 0;
}

static int dsi_pll_28nm_clk_is_enabled(struct clk_hw *hw)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);

	return pll_28nm_poll_for_ready(pll_28nm, POLL_MAX_READS,
					POLL_TIMEOUT_US);
}

static unsigned long dsi_pll_28nm_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);
	void __iomem *base = pll_28nm->phy->pll_base;
	u32 sdm0, doubler, sdm_byp_div;
	u32 sdm_dc_off, sdm_freq_seed, sdm2, sdm3;
	u32 ref_clk = VCO_REF_CLK_RATE;
	unsigned long vco_rate;

	VERB("parent_rate=%lu", parent_rate);

	/* Check to see if the ref clk doubler is enabled */
	doubler = readl(base + REG_DSI_28nm_PHY_PLL_REFCLK_CFG) &
			DSI_28nm_PHY_PLL_REFCLK_CFG_DBLR;
	ref_clk += (doubler * VCO_REF_CLK_RATE);

	/* see if it is integer mode or sdm mode */
	sdm0 = readl(base + REG_DSI_28nm_PHY_PLL_SDM_CFG0);
	if (sdm0 & DSI_28nm_PHY_PLL_SDM_CFG0_BYP) {
		/* integer mode */
		sdm_byp_div = FIELD(
				readl(base + REG_DSI_28nm_PHY_PLL_SDM_CFG0),
				DSI_28nm_PHY_PLL_SDM_CFG0_BYP_DIV) + 1;
		vco_rate = ref_clk * sdm_byp_div;
	} else {
		/* sdm mode */
		sdm_dc_off = FIELD(
				readl(base + REG_DSI_28nm_PHY_PLL_SDM_CFG1),
				DSI_28nm_PHY_PLL_SDM_CFG1_DC_OFFSET);
		DBG("sdm_dc_off = %d", sdm_dc_off);
		sdm2 = FIELD(readl(base + REG_DSI_28nm_PHY_PLL_SDM_CFG2),
				DSI_28nm_PHY_PLL_SDM_CFG2_FREQ_SEED_7_0);
		sdm3 = FIELD(readl(base + REG_DSI_28nm_PHY_PLL_SDM_CFG3),
				DSI_28nm_PHY_PLL_SDM_CFG3_FREQ_SEED_15_8);
		sdm_freq_seed = (sdm3 << 8) | sdm2;
		DBG("sdm_freq_seed = %d", sdm_freq_seed);

		vco_rate = (ref_clk * (sdm_dc_off + 1)) +
			mult_frac(ref_clk, sdm_freq_seed, BIT(16));
		DBG("vco rate = %lu", vco_rate);
	}

	DBG("returning vco rate = %lu", vco_rate);

	return vco_rate;
}

static int _dsi_pll_28nm_vco_prepare_hpm(struct dsi_pll_28nm *pll_28nm)
{
	struct device *dev = &pll_28nm->phy->pdev->dev;
	void __iomem *base = pll_28nm->phy->pll_base;
	u32 max_reads = 5, timeout_us = 100;
	bool locked;
	u32 val;
	int i;

	DBG("id=%d", pll_28nm->phy->id);

	pll_28nm_software_reset(pll_28nm);

	/*
	 * PLL power up sequence.
	 * Add necessary delays recommended by hardware.
	 */
	val = DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRDN_B;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	udelay(1);

	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRGEN_PWRDN_B;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	udelay(200);

	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_LDO_PWRDN_B;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	udelay(500);

	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_ENABLE;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	udelay(600);

	for (i = 0; i < 2; i++) {
		/* DSI Uniphy lock detect setting */
		writel(0x0c, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);
		udelay(100);
		writel(0x0d, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);

		/* poll for PLL ready status */
		locked = pll_28nm_poll_for_ready(pll_28nm, max_reads,
						 timeout_us);
		if (locked)
			break;

		pll_28nm_software_reset(pll_28nm);

		/*
		 * PLL power up sequence.
		 * Add necessary delays recommended by hardware.
		 */
		val = DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRDN_B;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(1);

		val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRGEN_PWRDN_B;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(200);

		val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_LDO_PWRDN_B;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(250);

		val &= ~DSI_28nm_PHY_PLL_GLB_CFG_PLL_LDO_PWRDN_B;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(200);

		val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_LDO_PWRDN_B;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(500);

		val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_ENABLE;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(600);
	}

	if (unlikely(!locked))
		DRM_DEV_ERROR(dev, "DSI PLL lock failed\n");
	else
		DBG("DSI PLL Lock success");

	return locked ? 0 : -EINVAL;
}

static int dsi_pll_28nm_vco_prepare_hpm(struct clk_hw *hw)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);
	int i, ret;

	if (unlikely(pll_28nm->phy->pll_on))
		return 0;

	for (i = 0; i < 3; i++) {
		ret = _dsi_pll_28nm_vco_prepare_hpm(pll_28nm);
		if (!ret) {
			pll_28nm->phy->pll_on = true;
			return 0;
		}
	}

	return ret;
}

static int dsi_pll_28nm_vco_prepare_8226(struct clk_hw *hw)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);
	struct device *dev = &pll_28nm->phy->pdev->dev;
	void __iomem *base = pll_28nm->phy->pll_base;
	u32 max_reads = 5, timeout_us = 100;
	bool locked;
	u32 val;
	int i;

	DBG("id=%d", pll_28nm->phy->id);

	pll_28nm_software_reset(pll_28nm);

	/*
	 * PLL power up sequence.
	 * Add necessary delays recommended by hardware.
	 */
	writel(0x34, base + REG_DSI_28nm_PHY_PLL_CAL_CFG1);

	val = DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRDN_B;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	udelay(200);

	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRGEN_PWRDN_B;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	udelay(200);

	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_LDO_PWRDN_B;
	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_ENABLE;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	udelay(600);

	for (i = 0; i < 7; i++) {
		/* DSI Uniphy lock detect setting */
		writel(0x0d, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);
		writel(0x0c, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);
		udelay(100);
		writel(0x0d, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);

		/* poll for PLL ready status */
		locked = pll_28nm_poll_for_ready(pll_28nm,
						max_reads, timeout_us);
		if (locked)
			break;

		pll_28nm_software_reset(pll_28nm);

		/*
		 * PLL power up sequence.
		 * Add necessary delays recommended by hardware.
		 */
		writel(0x00, base + REG_DSI_28nm_PHY_PLL_PWRGEN_CFG);
		udelay(50);

		val = DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRDN_B;
		val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRGEN_PWRDN_B;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(100);

		val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_LDO_PWRDN_B;
		val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_ENABLE;
		writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
		udelay(600);
	}

	if (unlikely(!locked))
		DRM_DEV_ERROR(dev, "DSI PLL lock failed\n");
	else
		DBG("DSI PLL Lock success");

	return locked ? 0 : -EINVAL;
}

static int dsi_pll_28nm_vco_prepare_lp(struct clk_hw *hw)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);
	struct device *dev = &pll_28nm->phy->pdev->dev;
	void __iomem *base = pll_28nm->phy->pll_base;
	bool locked;
	u32 max_reads = 10, timeout_us = 50;
	u32 val;

	DBG("id=%d", pll_28nm->phy->id);

	if (unlikely(pll_28nm->phy->pll_on))
		return 0;

	/* W1A lk2nd handoff (m2w): the bootloader leaves the PLL locked and
	 * its SDM/rate registers programmed.  The reset + power-cycle below
	 * discards that state and the relock then fails because no
	 * clk_set_rate() ever ran to reprogram the rate registers (the dsi
	 * host inherit path skips set_rate) - seen as "DSI PLL lock failed"
	 * -> -EINVAL from clk_prepare_enable(byte_clk) -> host power_on fail
	 * -> every panel command -EINVAL (m2v black screen).  An already
	 * locked VCO needs nothing: mark it on and let the clk framework
	 * take its refcount.
	 *
	 * v5: this is also the one-time opportunity to capture the raw PLL
	 * configuration - the seeds lk2nd actually drives this panel with -
	 * for the raw restore path in dsi_28nm_pll_restore_state(). */
	if (dsi_pll_28nm_clk_is_enabled(hw)) {
		if (!pll_28nm->raw.captured) {
			struct pll_28nm_raw_state *r = &pll_28nm->raw;
			void __iomem *b = pll_28nm->phy->pll_base;

			r->refclk = readl(b + 0x00);
			r->postdiv1 = readl(b + 0x04);
			r->vreg = readl(b + 0x10);
			r->postdiv3 = readl(b + 0x28);
			r->sdm0 = readl(b + 0x38);
			r->sdm1 = readl(b + 0x3c);
			r->sdm2 = readl(b + 0x40);
			r->sdm3 = readl(b + 0x44);
			r->sdm4 = readl(b + 0x48);
			r->cal_cfg0 = readl(b + 0x6c);
			r->cal_cfg1 = readl(b + 0x70);
			r->captured = true;
			dev_info(dev,
				 "MDP3DBG pll handoff captured: refclk=%02x postdiv1=%02x postdiv3=%02x sdm=%02x %02x %02x %02x %02x cal=%02x %02x\n",
				 r->refclk, r->postdiv1, r->postdiv3,
				 r->sdm0, r->sdm1, r->sdm2, r->sdm3, r->sdm4,
				 r->cal_cfg0, r->cal_cfg1);
		}
		dev_info(dev, "MDP3DBG pll inherit: VCO already locked, skipping reprogram\n");
		pll_28nm->phy->pll_on = true;
		return 0;
	}

	pll_28nm_software_reset(pll_28nm);

	/*
	 * W1A (v8): the software reset above wipes the SDM/CAL seed
	 * registers to their reset defaults - relocking on those fails
	 * ("DSI PLL lock failed", the m2w lesson).  Rewrite the captured
	 * lk2nd seeds between the reset and the power-up so the relock
	 * lands on the proven configuration.  (On the boot handoff this
	 * path is not reached at all - the locked-VCO shortcut above
	 * returns early.)
	 */
	if (pll_28nm->raw.captured) {
		struct pll_28nm_raw_state *r = &pll_28nm->raw;

		writel(r->refclk, base + 0x00);
		writel(r->postdiv1, base + 0x04);
		writel(r->postdiv3, base + 0x28);
		writel(r->sdm0, base + 0x38);
		writel(r->sdm1, base + 0x3c);
		writel(r->sdm2, base + 0x40);
		writel(r->sdm3, base + 0x44);
		writel(r->sdm4, base + 0x48);
		writel(r->cal_cfg0, base + 0x6c);
		writel(r->cal_cfg1, base + 0x70);
		dev_info(dev, "MDP3DBG pll prepare: raw seeds rewritten after sw reset\n");
	}

	/*
	 * PLL power up sequence.
	 * Add necessary delays recommended by hardware.
	 */
	writel(0x34, base + REG_DSI_28nm_PHY_PLL_CAL_CFG1);
	ndelay(500);

	val = DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRDN_B;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	ndelay(500);

	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_PWRGEN_PWRDN_B;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	ndelay(500);

	val |= DSI_28nm_PHY_PLL_GLB_CFG_PLL_LDO_PWRDN_B |
		DSI_28nm_PHY_PLL_GLB_CFG_PLL_ENABLE;
	writel(val, base + REG_DSI_28nm_PHY_PLL_GLB_CFG);
	ndelay(500);

	/* DSI PLL toggle lock detect setting */
	writel(0x04, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);
	ndelay(500);
	writel(0x05, base + REG_DSI_28nm_PHY_PLL_LKDET_CFG2);
	udelay(512);

	locked = pll_28nm_poll_for_ready(pll_28nm, max_reads, timeout_us);

	if (unlikely(!locked)) {
		DRM_DEV_ERROR(dev, "DSI PLL lock failed\n");
		return -EINVAL;
	}

	DBG("DSI PLL lock success");
	pll_28nm->phy->pll_on = true;

	return 0;
}

static void dsi_pll_28nm_vco_unprepare(struct clk_hw *hw)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);

	DBG("id=%d", pll_28nm->phy->id);

	if (unlikely(!pll_28nm->phy->pll_on))
		return;

	writel(0, pll_28nm->phy->pll_base + REG_DSI_28nm_PHY_PLL_GLB_CFG);

	pll_28nm->phy->pll_on = false;
}

static int dsi_pll_28nm_clk_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(hw);

	req->rate = clamp_t(unsigned long, req->rate,
			    pll_28nm->phy->cfg->min_pll_rate,
			    pll_28nm->phy->cfg->max_pll_rate);

	return 0;
}

static const struct clk_ops clk_ops_dsi_pll_28nm_vco_hpm = {
	.determine_rate = dsi_pll_28nm_clk_determine_rate,
	.set_rate = dsi_pll_28nm_clk_set_rate,
	.recalc_rate = dsi_pll_28nm_clk_recalc_rate,
	.prepare = dsi_pll_28nm_vco_prepare_hpm,
	.unprepare = dsi_pll_28nm_vco_unprepare,
	.is_enabled = dsi_pll_28nm_clk_is_enabled,
};

static const struct clk_ops clk_ops_dsi_pll_28nm_vco_lp = {
	.determine_rate = dsi_pll_28nm_clk_determine_rate,
	.set_rate = dsi_pll_28nm_clk_set_rate,
	.recalc_rate = dsi_pll_28nm_clk_recalc_rate,
	.prepare = dsi_pll_28nm_vco_prepare_lp,
	.unprepare = dsi_pll_28nm_vco_unprepare,
	.is_enabled = dsi_pll_28nm_clk_is_enabled,
};

static const struct clk_ops clk_ops_dsi_pll_28nm_vco_8226 = {
	.determine_rate = dsi_pll_28nm_clk_determine_rate,
	.set_rate = dsi_pll_28nm_clk_set_rate,
	.recalc_rate = dsi_pll_28nm_clk_recalc_rate,
	.prepare = dsi_pll_28nm_vco_prepare_8226,
	.unprepare = dsi_pll_28nm_vco_unprepare,
	.is_enabled = dsi_pll_28nm_clk_is_enabled,
};

/*
 * PLL Callbacks
 */

static void dsi_28nm_pll_save_state(struct msm_dsi_phy *phy)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(phy->vco_hw);
	struct pll_28nm_cached_state *cached_state = &pll_28nm->cached_state;
	void __iomem *base = pll_28nm->phy->pll_base;

	cached_state->postdiv3 =
			readl(base + REG_DSI_28nm_PHY_PLL_POSTDIV3_CFG);
	cached_state->postdiv1 =
			readl(base + REG_DSI_28nm_PHY_PLL_POSTDIV1_CFG);
	cached_state->byte_mux = readl(base + REG_DSI_28nm_PHY_PLL_VREG_CFG);
	if (dsi_pll_28nm_clk_is_enabled(phy->vco_hw))
		cached_state->vco_rate = clk_hw_get_rate(phy->vco_hw);
	else
		cached_state->vco_rate = 0;
}

/*
 * W1A (v9): rewrite the captured lk2nd PLL configuration - ALL eleven
 * registers including the POSTDIV3_CFG (0x28, the dsi0pll divider) and
 * VREG_CFG (0x10, the byte mux) that the clk framework's divider ops
 * reprogram AFTER dsi_pll_28nm_clk_set_rate() returns.  The computed
 * divider chain (postdiv3 0x11 vs lk2nd's 0x2f) leaves the byte clock
 * functionally wrong: HS FIFO underflow, no picture (v8).  Call this
 * from the dsi host power_on once the link clocks are enabled - at
 * that point nothing in the clk core touches these registers again.
 */
void msm_dsi_phy_pll_write_raw(struct msm_dsi_phy *phy)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(phy->vco_hw);
	struct pll_28nm_raw_state *r = &pll_28nm->raw;
	void __iomem *base = pll_28nm->phy->pll_base;

	if (!r->captured || !base)
		return;

	writel(r->refclk, base + 0x00);
	writel(r->postdiv1, base + 0x04);
	writel(r->vreg, base + 0x10);
	writel(r->postdiv3, base + 0x28);
	writel(r->sdm0, base + 0x38);
	writel(r->sdm1, base + 0x3c);
	writel(r->sdm2, base + 0x40);
	writel(r->sdm3, base + 0x44);
	writel(r->sdm4, base + 0x48);
	writel(r->cal_cfg0, base + 0x6c);
	writel(r->cal_cfg1, base + 0x70);
	DRM_DEV_INFO(&phy->pdev->dev,
		     "MDP3DBG pll raw config rewritten post-enable (postdiv3=%02x vreg=%02x)\n",
		     r->postdiv3, r->vreg);
}

static int dsi_28nm_pll_restore_state(struct msm_dsi_phy *phy)
{
	struct dsi_pll_28nm *pll_28nm = to_pll_28nm(phy->vco_hw);
	struct pll_28nm_cached_state *cached_state = &pll_28nm->cached_state;
	struct pll_28nm_raw_state *r = &pll_28nm->raw;
	void __iomem *base = pll_28nm->phy->pll_base;
	int ret;

	/*
	 * W1A (v5): with the raw handoff config captured, reprogram the
	 * PLL verbatim - lk2nd's proven seeds - instead of recomputing
	 * them through clk_set_rate() (whose math produces a different,
	 * HS-breaking configuration on this panel/PHY).
	 */
	if (r->captured) {
		writel(r->refclk, base + 0x00);
		writel(r->postdiv1, base + 0x04);
		writel(r->vreg, base + 0x10);
		writel(r->postdiv3, base + 0x28);
		writel(r->sdm0, base + 0x38);
		writel(r->sdm1, base + 0x3c);
		writel(r->sdm2, base + 0x40);
		writel(r->sdm3, base + 0x44);
		writel(r->sdm4, base + 0x48);
		writel(r->cal_cfg0, base + 0x6c);
		writel(r->cal_cfg1, base + 0x70);
		DRM_DEV_INFO(&pll_28nm->phy->pdev->dev,
			     "MDP3DBG pll restore: raw lk2nd config rewritten\n");
		return 0;
	}

	/*
	 * W1A (noinherit): a never-saved cached state is all zeroes.
	 * Restoring it calls dsi_pll_28nm_clk_set_rate(0) - which writes
	 * divider/SDM garbage over the RUNNING bootloader-locked PLL - and
	 * programs zero post-dividers.  The subsequent link-clk set_rate +
	 * VCO prepare then has to relock through the wreckage while the
	 * byte0/pclk0 RCG parent switch races the relock ("rcg didn't
	 * update its configuration").  A real saved state always has a
	 * nonzero vco_rate and postdividers; skip the all-zero one.
	 */
	if (!cached_state->vco_rate && !cached_state->postdiv3 &&
	    !cached_state->postdiv1 && !cached_state->byte_mux) {
		DRM_DEV_INFO(&pll_28nm->phy->pdev->dev,
			     "MDP3DBG pll restore: skipped (no saved state - PLL left as lk2nd handed it over)\n");
		return 0;
	}

	ret = dsi_pll_28nm_clk_set_rate(phy->vco_hw,
					cached_state->vco_rate, 0);
	if (ret) {
		DRM_DEV_ERROR(&pll_28nm->phy->pdev->dev,
			"restore vco rate failed. ret=%d\n", ret);
		return ret;
	}

	writel(cached_state->postdiv3, base + REG_DSI_28nm_PHY_PLL_POSTDIV3_CFG);
	writel(cached_state->postdiv1, base + REG_DSI_28nm_PHY_PLL_POSTDIV1_CFG);
	writel(cached_state->byte_mux, base + REG_DSI_28nm_PHY_PLL_VREG_CFG);

	return 0;
}

static int pll_28nm_register(struct dsi_pll_28nm *pll_28nm, struct clk_hw **provided_clocks)
{
	char clk_name[32];
	struct clk_init_data vco_init = {
		.parent_data = &(const struct clk_parent_data) {
			.fw_name = "ref", .name = "xo",
		},
		.num_parents = 1,
		.name = clk_name,
		.flags = CLK_IGNORE_UNUSED,
	};
	struct device *dev = &pll_28nm->phy->pdev->dev;
	struct clk_hw *hw, *analog_postdiv, *indirect_path_div2, *byte_mux;
	int ret;

	DBG("%d", pll_28nm->phy->id);

	if (pll_28nm->phy->cfg->quirks & DSI_PHY_28NM_QUIRK_PHY_LP)
		vco_init.ops = &clk_ops_dsi_pll_28nm_vco_lp;
	else if (pll_28nm->phy->cfg->quirks & DSI_PHY_28NM_QUIRK_PHY_8226)
		vco_init.ops = &clk_ops_dsi_pll_28nm_vco_8226;
	else
		vco_init.ops = &clk_ops_dsi_pll_28nm_vco_hpm;

	snprintf(clk_name, sizeof(clk_name), "dsi%dvco_clk", pll_28nm->phy->id);
	pll_28nm->clk_hw.init = &vco_init;
	ret = devm_clk_hw_register(dev, &pll_28nm->clk_hw);
	if (ret)
		return ret;

	snprintf(clk_name, sizeof(clk_name), "dsi%danalog_postdiv_clk", pll_28nm->phy->id);
	analog_postdiv = devm_clk_hw_register_divider_parent_hw(dev, clk_name,
			&pll_28nm->clk_hw, CLK_SET_RATE_PARENT,
			pll_28nm->phy->pll_base +
				REG_DSI_28nm_PHY_PLL_POSTDIV1_CFG,
			0, 4, 0, NULL);
	if (IS_ERR(analog_postdiv))
		return PTR_ERR(analog_postdiv);

	snprintf(clk_name, sizeof(clk_name), "dsi%dindirect_path_div2_clk", pll_28nm->phy->id);
	indirect_path_div2 = devm_clk_hw_register_fixed_factor_parent_hw(dev,
			clk_name, analog_postdiv, CLK_SET_RATE_PARENT, 1, 2);
	if (IS_ERR(indirect_path_div2))
		return PTR_ERR(indirect_path_div2);

	snprintf(clk_name, sizeof(clk_name), "dsi%dpll", pll_28nm->phy->id);
	hw = devm_clk_hw_register_divider_parent_hw(dev, clk_name,
			&pll_28nm->clk_hw, 0, pll_28nm->phy->pll_base +
				REG_DSI_28nm_PHY_PLL_POSTDIV3_CFG,
			0, 8, 0, NULL);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	provided_clocks[DSI_PIXEL_PLL_CLK] = hw;

	snprintf(clk_name, sizeof(clk_name), "dsi%dbyte_mux", pll_28nm->phy->id);
	byte_mux = devm_clk_hw_register_mux_parent_hws(dev, clk_name,
			((const struct clk_hw *[]){
				&pll_28nm->clk_hw,
				indirect_path_div2,
			}), 2, CLK_SET_RATE_PARENT, pll_28nm->phy->pll_base +
				REG_DSI_28nm_PHY_PLL_VREG_CFG, 1, 1, 0, NULL);
	if (IS_ERR(byte_mux))
		return PTR_ERR(byte_mux);

	snprintf(clk_name, sizeof(clk_name), "dsi%dpllbyte", pll_28nm->phy->id);
	hw = devm_clk_hw_register_fixed_factor_parent_hw(dev, clk_name,
			byte_mux, CLK_SET_RATE_PARENT, 1, 4);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	provided_clocks[DSI_BYTE_PLL_CLK] = hw;

	return 0;
}

static int dsi_pll_28nm_init(struct msm_dsi_phy *phy)
{
	struct platform_device *pdev = phy->pdev;
	struct dsi_pll_28nm *pll_28nm;
	int ret;

	if (!pdev)
		return -ENODEV;

	pll_28nm = devm_kzalloc(&pdev->dev, sizeof(*pll_28nm), GFP_KERNEL);
	if (!pll_28nm)
		return -ENOMEM;

	pll_28nm->phy = phy;

	/*
	 * W1A (v6): capture lk2nd's raw PLL configuration HERE, at probe,
	 * before any clk_set_rate()/phy-enable path can rewrite it.  The
	 * mainline set_rate() seed math produces a functionally different
	 * configuration than lk2nd's proven one (sdm1 0x4e vs 0x0e,
	 * postdiv3 0x11 vs 0x2f) - LP commands keep working but the HS
	 * data path dies (black screen, v2-v4).  With the raw config
	 * stashed, dsi_pll_28nm_clk_set_rate() below becomes a no-op and
	 * the restore path rewrites these registers verbatim after a real
	 * power cycle.
	 */
	if (pll_28nm_poll_for_ready(pll_28nm, POLL_MAX_READS, POLL_TIMEOUT_US)) {
		struct pll_28nm_raw_state *r = &pll_28nm->raw;
		void __iomem *b = pll_28nm->phy->pll_base;

		r->refclk = readl(b + 0x00);
		r->postdiv1 = readl(b + 0x04);
		r->vreg = readl(b + 0x10);
		r->postdiv3 = readl(b + 0x28);
		r->sdm0 = readl(b + 0x38);
		r->sdm1 = readl(b + 0x3c);
		r->sdm2 = readl(b + 0x40);
		r->sdm3 = readl(b + 0x44);
		r->sdm4 = readl(b + 0x48);
		r->cal_cfg0 = readl(b + 0x6c);
		r->cal_cfg1 = readl(b + 0x70);
		r->captured = true;
		dev_info(&pdev->dev,
			 "MDP3DBG pll handoff captured at probe: refclk=%02x postdiv1=%02x postdiv3=%02x sdm=%02x %02x %02x %02x %02x cal=%02x %02x\n",
			 r->refclk, r->postdiv1, r->postdiv3,
			 r->sdm0, r->sdm1, r->sdm2, r->sdm3, r->sdm4,
			 r->cal_cfg0, r->cal_cfg1);
	} else {
		dev_info(&pdev->dev, "MDP3DBG pll not locked at probe, no handoff capture\n");
	}

	ret = pll_28nm_register(pll_28nm, phy->provided_clocks->hws);
	if (ret) {
		DRM_DEV_ERROR(&pdev->dev, "failed to register PLL: %d\n", ret);
		return ret;
	}

	phy->vco_hw = &pll_28nm->clk_hw;

	return 0;
}

static void dsi_28nm_dphy_set_timing(struct msm_dsi_phy *phy,
		struct msm_dsi_dphy_timing *timing)
{
	void __iomem *base = phy->base;

	writel(DSI_28nm_PHY_TIMING_CTRL_0_CLK_ZERO(timing->clk_zero),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_0);
	writel(DSI_28nm_PHY_TIMING_CTRL_1_CLK_TRAIL(timing->clk_trail),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_1);
	writel(DSI_28nm_PHY_TIMING_CTRL_2_CLK_PREPARE(timing->clk_prepare),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_2);
	if (timing->clk_zero & BIT(8))
		writel(DSI_28nm_PHY_TIMING_CTRL_3_CLK_ZERO_8,
		       base + REG_DSI_28nm_PHY_TIMING_CTRL_3);
	writel(DSI_28nm_PHY_TIMING_CTRL_4_HS_EXIT(timing->hs_exit),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_4);
	writel(DSI_28nm_PHY_TIMING_CTRL_5_HS_ZERO(timing->hs_zero),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_5);
	writel(DSI_28nm_PHY_TIMING_CTRL_6_HS_PREPARE(timing->hs_prepare),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_6);
	writel(DSI_28nm_PHY_TIMING_CTRL_7_HS_TRAIL(timing->hs_trail),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_7);
	writel(DSI_28nm_PHY_TIMING_CTRL_8_HS_RQST(timing->hs_rqst),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_8);
	writel(DSI_28nm_PHY_TIMING_CTRL_9_TA_GO(timing->ta_go) |
	       DSI_28nm_PHY_TIMING_CTRL_9_TA_SURE(timing->ta_sure),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_9);
	writel(DSI_28nm_PHY_TIMING_CTRL_10_TA_GET(timing->ta_get),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_10);
	writel(DSI_28nm_PHY_TIMING_CTRL_11_TRIG3_CMD(0),
	       base + REG_DSI_28nm_PHY_TIMING_CTRL_11);
}

static void dsi_28nm_phy_regulator_enable_dcdc(struct msm_dsi_phy *phy)
{
	void __iomem *base = phy->reg_base;

	writel(0x0, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_0);
	writel(1, base + REG_DSI_28nm_PHY_REGULATOR_CAL_PWR_CFG);
	writel(0, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_5);
	writel(0, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_3);
	writel(0x3, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_2);
	writel(0x9, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_1);
	writel(0x7, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_0);
	writel(0x20, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_4);
	writel(0x00, phy->base + REG_DSI_28nm_PHY_LDO_CNTRL);
}

static void dsi_28nm_phy_regulator_enable_ldo(struct msm_dsi_phy *phy)
{
	void __iomem *base = phy->reg_base;

	writel(0x0, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_0);
	writel(0, base + REG_DSI_28nm_PHY_REGULATOR_CAL_PWR_CFG);
	/* h/w recommended delay (lk2nd: mdss_dsi_phy_regulator_init) */
	udelay(1000);
	writel(0x7, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_5);
	writel(0, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_3);
	writel(0x1, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_2);
	writel(0x1, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_1);
	writel(0x20, base + REG_DSI_28nm_PHY_REGULATOR_CTRL_4);

	if (phy->cfg->quirks & DSI_PHY_28NM_QUIRK_PHY_LP)
		writel(0x05, phy->base + REG_DSI_28nm_PHY_LDO_CNTRL);
	else
		writel(0x0d, phy->base + REG_DSI_28nm_PHY_LDO_CNTRL);
}

static void dsi_28nm_phy_regulator_ctrl(struct msm_dsi_phy *phy, bool enable)
{
	if (!enable) {
		writel(0, phy->reg_base + REG_DSI_28nm_PHY_REGULATOR_CAL_PWR_CFG);
		return;
	}

	if (phy->regulator_ldo_mode)
		dsi_28nm_phy_regulator_enable_ldo(phy);
	else
		dsi_28nm_phy_regulator_enable_dcdc(phy);
}

static int dsi_28nm_phy_enable(struct msm_dsi_phy *phy,
				struct msm_dsi_phy_clk_request *clk_req)
{
	struct msm_dsi_dphy_timing *timing = &phy->timing;
	int i;
	void __iomem *base = phy->base;
	u32 val;

	DBG("");

	if (phy->timing_override) {
		/*
		 * Vendor-provided timing (raw GCDB register values): follow
		 * lk2nd's proven mdss_dsi_phy_28nm_init sequence instead of
		 * the calculated one.  The calculated v1 timings are 2-6x
		 * larger than the vendor table for the same link rate and
		 * were shown to keep the DSI link dead on MSM8909-class
		 * PHYs (every panel init command times out).
		 */
		static const u32 bist_ctrl[6] = {
			0x00, 0x00, 0xb1, 0xff, 0x00, 0x00
		};

		dev_info(&phy->pdev->dev,
			 "MDP3DBG phy vendor enable: start (timing[0]=%08x)\n",
			 phy->timing_regs[0]);

		writel(0x5b, base + REG_DSI_28nm_PHY_CTRL_0);
		writel(0xff, base + REG_DSI_28nm_PHY_STRENGTH_0);

		dsi_28nm_phy_regulator_ctrl(phy, true);

		for (i = 0; i < 12; i++)
			writel(phy->timing_regs[i],
			       base + REG_DSI_28nm_PHY_TIMING_CTRL_0 + 4 * i);

		for (i = 0; i < 4; i++) {
			writel(0, base + REG_DSI_28nm_PHY_LN_CFG_0(i));
			writel(0, base + REG_DSI_28nm_PHY_LN_CFG_1(i));
			writel(0, base + REG_DSI_28nm_PHY_LN_CFG_2(i));
			writel(0, base + REG_DSI_28nm_PHY_LN_CFG_3(i));
			writel(0, base + REG_DSI_28nm_PHY_LN_CFG_4(i));
			writel(0, base + REG_DSI_28nm_PHY_LN_TEST_DATAPATH(i));
			writel(0, base + REG_DSI_28nm_PHY_LN_DEBUG_SEL(i));
			writel(0x1, base + REG_DSI_28nm_PHY_LN_TEST_STR_0(i));
			writel(0x97, base + REG_DSI_28nm_PHY_LN_TEST_STR_1(i));
		}

		writel(0, base + REG_DSI_28nm_PHY_LNCK_CFG_4);
		writel(0xc0, base + REG_DSI_28nm_PHY_LNCK_CFG_1);
		writel(0x1, base + REG_DSI_28nm_PHY_LNCK_TEST_STR0);
		writel(0xbb, base + REG_DSI_28nm_PHY_LNCK_TEST_STR1);

		writel(0x0a, base + REG_DSI_28nm_PHY_CTRL_4);

		writel(0x01, base + REG_DSI_28nm_PHY_GLBL_TEST_CTRL);

		writel(0x5f, base + REG_DSI_28nm_PHY_CTRL_0);

		writel(0x6, base + REG_DSI_28nm_PHY_STRENGTH_1);

		for (i = 0; i < 6; i++)
			writel(bist_ctrl[i],
			       base + REG_DSI_28nm_PHY_BIST_CTRL_0 + 4 * i);

		dev_info(&phy->pdev->dev, "MDP3DBG phy vendor enable: done\n");

		return 0;
	}

	if (msm_dsi_dphy_timing_calc(timing, clk_req)) {
		DRM_DEV_ERROR(&phy->pdev->dev,
			      "%s: D-PHY timing calculation failed\n",
			      __func__);
		return -EINVAL;
	}

	writel(0xff, base + REG_DSI_28nm_PHY_STRENGTH_0);

	dsi_28nm_phy_regulator_ctrl(phy, true);

	dsi_28nm_dphy_set_timing(phy, timing);

	writel(0x00, base + REG_DSI_28nm_PHY_CTRL_1);
	writel(0x5f, base + REG_DSI_28nm_PHY_CTRL_0);

	writel(0x6, base + REG_DSI_28nm_PHY_STRENGTH_1);

	for (i = 0; i < 4; i++) {
		writel(0, base + REG_DSI_28nm_PHY_LN_CFG_0(i));
		writel(0, base + REG_DSI_28nm_PHY_LN_CFG_1(i));
		writel(0, base + REG_DSI_28nm_PHY_LN_CFG_2(i));
		writel(0, base + REG_DSI_28nm_PHY_LN_CFG_3(i));
		writel(0, base + REG_DSI_28nm_PHY_LN_CFG_4(i));
		writel(0, base + REG_DSI_28nm_PHY_LN_TEST_DATAPATH(i));
		writel(0, base + REG_DSI_28nm_PHY_LN_DEBUG_SEL(i));
		writel(0x1, base + REG_DSI_28nm_PHY_LN_TEST_STR_0(i));
		writel(0x97, base + REG_DSI_28nm_PHY_LN_TEST_STR_1(i));
	}

	writel(0, base + REG_DSI_28nm_PHY_LNCK_CFG_4);
	writel(0xc0, base + REG_DSI_28nm_PHY_LNCK_CFG_1);
	writel(0x1, base + REG_DSI_28nm_PHY_LNCK_TEST_STR0);
	writel(0xbb, base + REG_DSI_28nm_PHY_LNCK_TEST_STR1);

	writel(0x5f, base + REG_DSI_28nm_PHY_CTRL_0);

	val = readl(base + REG_DSI_28nm_PHY_GLBL_TEST_CTRL);
	if (phy->id == DSI_1 && phy->usecase == MSM_DSI_PHY_SLAVE)
		val &= ~DSI_28nm_PHY_GLBL_TEST_CTRL_BITCLK_HS_SEL;
	else
		val |= DSI_28nm_PHY_GLBL_TEST_CTRL_BITCLK_HS_SEL;
	writel(val, base + REG_DSI_28nm_PHY_GLBL_TEST_CTRL);

	return 0;
}

static void dsi_28nm_phy_disable(struct msm_dsi_phy *phy)
{
	writel(0, phy->base + REG_DSI_28nm_PHY_CTRL_0);
	dsi_28nm_phy_regulator_ctrl(phy, false);

	/*
	 * Wait for the registers writes to complete in order to
	 * ensure that the phy is completely disabled
	 */
	wmb();
}

static const struct regulator_bulk_data dsi_phy_28nm_regulators[] = {
	{ .supply = "vddio", .init_load_uA = 100000 },
};

const struct msm_dsi_phy_cfg dsi_phy_28nm_hpm_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_28nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_28nm_regulators),
	.ops = {
		.enable = dsi_28nm_phy_enable,
		.disable = dsi_28nm_phy_disable,
		.pll_init = dsi_pll_28nm_init,
		.save_pll_state = dsi_28nm_pll_save_state,
		.restore_pll_state = dsi_28nm_pll_restore_state,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0xfd922b00, 0xfd923100 },
	.num_dsi_phy = 2,
};

const struct msm_dsi_phy_cfg dsi_phy_28nm_hpm_famb_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_28nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_28nm_regulators),
	.ops = {
		.enable = dsi_28nm_phy_enable,
		.disable = dsi_28nm_phy_disable,
		.pll_init = dsi_pll_28nm_init,
		.save_pll_state = dsi_28nm_pll_save_state,
		.restore_pll_state = dsi_28nm_pll_restore_state,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0x1a94400, 0x1a96400 },
	.num_dsi_phy = 2,
};

const struct msm_dsi_phy_cfg dsi_phy_28nm_lp_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_28nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_28nm_regulators),
	.ops = {
		.enable = dsi_28nm_phy_enable,
		.disable = dsi_28nm_phy_disable,
		.pll_init = dsi_pll_28nm_init,
		.save_pll_state = dsi_28nm_pll_save_state,
		.restore_pll_state = dsi_28nm_pll_restore_state,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0x1a98500 },
	.num_dsi_phy = 1,
	.quirks = DSI_PHY_28NM_QUIRK_PHY_LP,
};

const struct msm_dsi_phy_cfg dsi_phy_28nm_8909_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_28nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_28nm_regulators),
	.ops = {
		.enable = dsi_28nm_phy_enable,
		.disable = dsi_28nm_phy_disable,
		.pll_init = dsi_pll_28nm_init,
		.save_pll_state = dsi_28nm_pll_save_state,
		.restore_pll_state = dsi_28nm_pll_restore_state,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0x1ac8500 },
	.num_dsi_phy = 1,
	.quirks = DSI_PHY_28NM_QUIRK_PHY_LP,
};

const struct msm_dsi_phy_cfg dsi_phy_28nm_8226_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_28nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_28nm_regulators),
	.ops = {
		.enable = dsi_28nm_phy_enable,
		.disable = dsi_28nm_phy_disable,
		.pll_init = dsi_pll_28nm_init,
		.save_pll_state = dsi_28nm_pll_save_state,
		.restore_pll_state = dsi_28nm_pll_restore_state,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0xfd922b00 },
	.num_dsi_phy = 1,
	.quirks = DSI_PHY_28NM_QUIRK_PHY_8226,
};

const struct msm_dsi_phy_cfg dsi_phy_28nm_8937_cfgs = {
	.has_phy_regulator = true,
	.regulator_data = dsi_phy_28nm_regulators,
	.num_regulators = ARRAY_SIZE(dsi_phy_28nm_regulators),
	.ops = {
		.enable = dsi_28nm_phy_enable,
		.disable = dsi_28nm_phy_disable,
		.pll_init = dsi_pll_28nm_init,
		.save_pll_state = dsi_28nm_pll_save_state,
		.restore_pll_state = dsi_28nm_pll_restore_state,
	},
	.min_pll_rate = VCO_MIN_RATE,
	.max_pll_rate = VCO_MAX_RATE,
	.io_start = { 0x1a94400, 0x1a96400 },
	.num_dsi_phy = 2,
	.quirks = DSI_PHY_28NM_QUIRK_PHY_LP,
};
