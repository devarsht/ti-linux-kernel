// SPDX-License-Identifier: GPL-2.0
/*
 * DRM driver for e-ink display panels using Solomon SSD16xx family controllers
 *
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com/
 *
 * References:
 * https://github.com/Lesords/epaper
 * https://github.com/waveshareteam/e-Paper/blob/master/STM32/STM32-F103ZET6/User/e-Paper/EPD_4in2_V2.c
 *
 * Author: Devarsh Thakkar <devarsht@ti.com>
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#define SSD16XX_CMD_DRIVER_OUTPUT_CONTROL		0x01
#define SSD16XX_CMD_DEEP_SLEEP_MODE			0x10
#define SSD16XX_CMD_DATA_ENTRY_MODE			0x11
#define SSD16XX_CMD_SW_RESET				0x12
#define SSD16XX_CMD_TEMPERATURE_SENSOR_CONTROL		0x18
#define SSD16XX_CMD_MASTER_ACTIVATION			0x20
#define SSD16XX_CMD_DISPLAY_UPDATE_CONTROL1		0x21
#define SSD16XX_CMD_DISPLAY_UPDATE_CONTROL2		0x22
#define SSD16XX_CMD_WRITE_RAM_BW			0x24
#define SSD16XX_CMD_WRITE_RAM_RED			0x26
#define SSD16XX_CMD_BORDER_WAVEFORM_CONTROL		0x3C
#define SSD16XX_CMD_SET_RAM_X_ADDRESS_START_END		0x44
#define SSD16XX_CMD_SET_RAM_Y_ADDRESS_START_END		0x45
#define SSD16XX_CMD_SET_RAM_X_ADDRESS_COUNTER		0x4E
#define SSD16XX_CMD_SET_RAM_Y_ADDRESS_COUNTER		0x4F

/*
 * Deep Sleep Mode (command 0x10) - from SSD1683 datasheet
 * After entering deep sleep, BUSY pin stays HIGH.
 * Exit requires hardware reset (HWRESET).
 */
#define SSD16XX_DEEP_SLEEP_MODE_1			0x01  /* Deep Sleep Mode 1 */
#define SSD16XX_DEEP_SLEEP_MODE_2			0x03  /* Deep Sleep Mode 2 */

/*
 * Data Entry Mode (command 0x11) - from SSD1683 datasheet
 * Bits [2:0] = AM, ID1, ID0
 * - AM: Address direction (0=X direction, 1=Y direction)
 * - ID[1:0]: Address increment/decrement
 *   00 = Y decrement, X decrement
 *   01 = Y decrement, X increment
 *   10 = Y increment, X decrement
 *   11 = Y increment, X increment (POR default)
 */
#define SSD16XX_DATA_ENTRY_XDEC_YDEC		0x00  /* X--, Y-- */
#define SSD16XX_DATA_ENTRY_XINC_YDEC		0x01  /* X++, Y-- */
#define SSD16XX_DATA_ENTRY_XDEC_YINC		0x02  /* X--, Y++ */
#define SSD16XX_DATA_ENTRY_XINC_YINC		0x03  /* X++, Y++ (default) */
#define SSD16XX_DATA_ENTRY_YDEC_XDEC		0x04  /* Y--, X-- (Y-direction) */
#define SSD16XX_DATA_ENTRY_YINC_XDEC		0x05  /* Y++, X-- (Y-direction) */
#define SSD16XX_DATA_ENTRY_YDEC_XINC		0x06  /* Y--, X++ (Y-direction) */
#define SSD16XX_DATA_ENTRY_YINC_XINC		0x07  /* Y++, X++ (Y-direction) */

/*
 * Driver Output Control (command 0x01) byte 3 - from SSD1683 datasheet
 * Bit 2 (TB): Source shift direction and display direction control
 * Bit 1 (SM): Gate scan sequence control
 * Bit 0 (GD): Gate driver output select (first output gate)
 */
#define SSD16XX_DRIVER_OUTPUT_TB		BIT(2)  /* Source output mode */
#define SSD16XX_DRIVER_OUTPUT_SM		BIT(1)  /* Gate scan sequence */
#define SSD16XX_DRIVER_OUTPUT_GD		BIT(0)  /* 1st output gate */

/*
 * Border Waveform Control (command 0x3C) - from SSD1683 datasheet
 * Bits [7:6]: VBD option select
 *   00 = GS Transition (defined in bits [1:0])
 *   01 = Fix Level (defined in bits [5:4])
 *   10 = VCOM
 *   11 = HiZ (default)
 * Bits [5:4]: Fix Level for VBD (when bits[7:6]=01)
 *   00 = VSS, 01 = VSH1, 10 = VSL, 11 = VSH2
 * Bits [1:0]: GS Transition for VBD (when bits[7:6]=00)
 *   00 = LUT0, 01 = LUT1, 10 = LUT2, 11 = LUT3
 */
#define SSD16XX_BORDER_WAVEFORM_HIZ		0xC0  /* HiZ (default) */
#define SSD16XX_BORDER_WAVEFORM_LUT0		0x00  /* GS Transition LUT0 */
#define SSD16XX_BORDER_WAVEFORM_LUT1		0x01  /* GS Transition LUT1 */
#define SSD16XX_BORDER_WAVEFORM_LUT2		0x02  /* GS Transition LUT2 */
#define SSD16XX_BORDER_WAVEFORM_LUT3		0x03  /* GS Transition LUT3 */
#define SSD16XX_BORDER_WAVEFORM_FIXLVL_VSS	0x40  /* Fix Level VSS */
#define SSD16XX_BORDER_WAVEFORM_FIXLVL_VSH1	0x50  /* Fix Level VSH1 */
#define SSD16XX_BORDER_WAVEFORM_FIXLVL_VSL	0x60  /* Fix Level VSL */
#define SSD16XX_BORDER_WAVEFORM_FIXLVL_VSH2	0x70  /* Fix Level VSH2 */
#define SSD16XX_BORDER_WAVEFORM_VCOM		0x80  /* Follow VCOM */

/*
 * Temperature Sensor Control (command 0x18) value to select internal sensor.
 * Bit 7 set = use internal temperature sensor.
 * When internal sensor is selected, controller automatically senses temperature
 * and selects appropriate LUT - no need to manually write temperature value.
 */
#define SSD16XX_TEMP_SENSOR_INTERNAL		0x80

/*
 * Display Update Control 2 (command 0x22) sequence to load temperature and LUT.
 * Used during fast refresh initialization to preload temperature/LUT.
 * Bits: ENABLE_CLK | LOAD_LUT | DISABLE_CLK (0x91)
 * Does not include DISPLAY bit, so no screen update occurs.
 */
#define SSD16XX_CTRL2_LOAD_TEMP_LUT		0x91

/*
 * Display Update Control 1 (0x21) byte 1 definitions - from SSD1683 datasheet
 *
 * Byte 1 controls RAM configuration:
 * - Bit 7: Not used
 * - Bit 6: RED RAM option (0 = normal, 1 = bypass RED RAM)
 * - Bit 5: Not used
 * - Bit 4: Not used
 * - Bits 3-0: Not used
 *
 * Byte 2 is usually 0x00 (default settings)
 */
#define SSD16XX_CTRL1_NORMAL			0x00  /* Both RAMs enabled (default) */
#define SSD16XX_CTRL1_BYPASS_RED_RAM		0x40  /* Bypass RED RAM (force RED=0) */
#define SSD16XX_CTRL1_BYTE2_DEFAULT		0x00  /* Byte 2 default value */

/* Display Update Control 2 (0x22) bit definitions - from SSD1683 datasheet */
#define SSD16XX_CTRL2_ENABLE_CLK			BIT(7)  /* Enable clock signal */
#define SSD16XX_CTRL2_ENABLE_ANALOG			BIT(6)  /* Enable analog */
#define SSD16XX_CTRL2_LOAD_TEMPERATURE			BIT(5)  /* Load temperature value */
#define SSD16XX_CTRL2_LOAD_LUT				BIT(4)  /* Load LUT with DISPLAY Mode 1 */
#define SSD16XX_CTRL2_MODE2				BIT(3)  /* Display Mode 2 */
#define SSD16XX_CTRL2_DISPLAY  				BIT(2)  /* Display Mode 1 */
#define SSD16XX_CTRL2_DISABLE_ANALOG			BIT(1)  /* Disable analog */
#define SSD16XX_CTRL2_DISABLE_CLK			BIT(0)  /* Disable clock signal */

/*
 * Display Update Control 2 (0x22) refresh mode definitions
 *
 * Three refresh modes based on Seeed reference implementation:
 *
 * 1. Full Refresh Mode (0xF7):
 *    - 3-color mode (bit 3 = 0)
 *    - Loads temperature and LUT on every update
 *    - Uses LUTB/LUTW (8 groups × 4 phases = 32 phases)
 *    - Best quality, ~1.5-2s update time
 *    - Use for: Initial clear, baseline establishment, 3-color panels
 *
 * 2. Fast Refresh Mode (0xC7):
 *    - 3-color mode (bit 3 = 0)
 *    - Does NOT load temperature or LUT (uses existing from init)
 *    - Uses LUTB/LUTW (8 groups × 4 phases = 32 phases)
 *    - Good quality, faster than full refresh
 *    - Use for: Frequent updates when temperature stable
 *
 * 3. Partial Refresh Mode (0xFF):
 *    - BW mode (bit 3 = 1)
 *    - Loads temperature and LUT on every update
 *    - Uses LUTBB/LUTWB/LUTBW/LUTWW (6 groups × 4 phases = 24 phases)
 *    - Fastest, ~300-500ms update time
 *    - Use for: Frequent updates on BW panels
 */

/* Full refresh: 3-color mode with temperature and LUT load */
#define SSD1683_CTRL2_FULL_REFRESH (SSD16XX_CTRL2_ENABLE_CLK | \
				    SSD16XX_CTRL2_ENABLE_ANALOG | \
				    SSD16XX_CTRL2_LOAD_TEMPERATURE | \
				    SSD16XX_CTRL2_LOAD_LUT | \
				    SSD16XX_CTRL2_DISPLAY | \
				    SSD16XX_CTRL2_DISABLE_ANALOG | \
				    SSD16XX_CTRL2_DISABLE_CLK)  /* 0xF7 */

/* Fast refresh: 3-color mode without temperature/LUT load */
#define SSD1683_CTRL2_FAST_REFRESH (SSD16XX_CTRL2_ENABLE_CLK | \
				    SSD16XX_CTRL2_ENABLE_ANALOG | \
				    SSD16XX_CTRL2_DISPLAY | \
				    SSD16XX_CTRL2_DISABLE_ANALOG | \
				    SSD16XX_CTRL2_DISABLE_CLK)  /* 0xC7 */

/* Partial refresh: BW mode with temperature and LUT load */
#define SSD1683_CTRL2_PARTIAL_REFRESH (SSD16XX_CTRL2_ENABLE_CLK | \
				       SSD16XX_CTRL2_ENABLE_ANALOG | \
				       SSD16XX_CTRL2_LOAD_TEMPERATURE | \
				       SSD16XX_CTRL2_LOAD_LUT | \
				       SSD16XX_CTRL2_MODE2 | \
				       SSD16XX_CTRL2_DISPLAY | \
				       SSD16XX_CTRL2_DISABLE_ANALOG | \
				       SSD16XX_CTRL2_DISABLE_CLK)  /* 0xFF */

#define SSD16XX_SPI_BITS_PER_WORD			8
#define SSD16XX_SPI_SPEED_DEFAULT			1000000

MODULE_IMPORT_NS("DMA_BUF");

/*
 * Module parameter to override default refresh mode
 * Values: 0 = partial, 1 = full, 2 = fast, -1 = use panel default
 */
static int refresh_mode_override = -1;
module_param(refresh_mode_override, int, 0644);
MODULE_PARM_DESC(refresh_mode_override,
		 "Override refresh mode (0=partial, 1=full, 2=fast, -1=default)");

enum ssd16xx_controller {
	SSD1683 = 1,
};

enum ssd16xx_model {
	GDEY042T81 = 1,
};

/*
 * Refresh mode selection for atomic updates
 * - Default mode set per panel type in panel config
 * - Can be overridden at runtime via module parameter
 */
enum ssd16xx_refresh_mode {
	SSD16XX_REFRESH_PARTIAL = 0,  /* Partial refresh (typical for BW panels) */
	SSD16XX_REFRESH_FULL,         /* Full refresh (typical for 3-color panels) */
	SSD16XX_REFRESH_FAST,         /* Fast refresh (skip temperature load) */
};

/*
 * Display modes and color support:
 *
 * The driver can operate in two display modes controlled by Display Update Control 2:
 *
 * Black/White mode (0xFF):
 *   - BW RAM contains the display data
 *   - RED RAM contains baseline for partial refresh transitions
 *   - Display Update Control 1 can bypass RED RAM (0x40) for full refresh
 *   - Uses 5 LUTs: LUTC, LUTBB, LUTWB, LUTBW, LUTWW (transition tables)
 *   - Suitable for: BW panels, or 3-color panels displaying only BW
 *
 * 3-Color mode (0xF7):
 *   - Both RED RAM and BW RAM are actively read for EVERY update
 *   - RAM bit combination determines color:
 *     RED=0, BW=0 → Black (LUTB)
 *     RED=0, BW=1 → White (LUTW)
 *     RED=1, BW=0 → Red (LUTR)
 *   - Display Update Control 1 MUST enable both RAMs (0x00)
 *   - Uses 4 LUTs: LUTC, LUTR, LUTW, LUTB (color selection tables)
 *   - Suitable for: 3-color panels displaying red, or BW panels for full refresh
 *
 * Panel capability vs display mode:
 * - BW panels: Can use BW mode (0xFF) or 3-color mode (0xF7) for better quality
 * - 3-color panels: Can use 3-color mode (0xF7) or BW mode (0xFF) to suppress red
 *
 * See SSD1683_LUT_EXPLANATION.md for complete details.
 */

struct ssd16xx_controller_config {
	u16 max_width;
	u16 max_height;
	u8 ram_x_address_bits;  /* Width of X RAM address parameter: 8 or 16 bits */
	u8 ram_y_address_bits;  /* Width of Y RAM address parameter: 8 or 16 bits */
};

struct ssd16xx_panel_config {
	/*
	 * Panel hardware capability: Does this panel support red color?
	 * - true: 3-color panel (red/yellow/black/white)
	 * - false: 2-color panel (black/white only)
	 *
	 * Note: This indicates hardware capability, not current display mode.
	 * A 3-color panel can display in BW mode (red suppressed).
	 * A BW panel can use 3-color mode (0xF7) for better full refresh quality.
	 */
	bool red_supported;

	/* Data Entry Mode - controls X/Y increment direction */
	u8 data_entry_mode;

	/* Driver Output Control - third byte (scan direction) */
	u8 driver_output_ctrl_byte3;

	/* Border Waveform Control - set once during initialization */
	u8 border_waveform_init;

	/*
	 * Display Update Control 1 (command 0x21)
	 *
	 * Byte 0 controls RED RAM and BW RAM usage:
	 *   Bits[7:4] - RED RAM option: 0x0=Normal, 0x4=Bypass, 0x8=Inverse
	 *   Bits[3:0] - BW RAM option:  0x0=Normal, 0x4=Bypass, 0x8=Inverse
	 *
	 * Note: Control 1 values are determined dynamically at runtime
	 * based on refresh mode and panel type. See ssd16xx_fb_dirty().
	 */

	/* Deep Sleep Mode */
	u8 deep_sleep_mode;

	/* Default refresh mode for this panel type */
	enum ssd16xx_refresh_mode default_refresh_mode;
};

struct ssd16xx_error_ctx {
	int errno_code;
};

struct ssd16xx_panel {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;

	struct spi_device *spi;
	struct gpio_desc *reset;
	struct gpio_desc *busy;
	struct gpio_desc *dc;

	enum ssd16xx_model model;
	enum ssd16xx_controller controller;
	const struct ssd16xx_controller_config *controller_cfg;
	const struct ssd16xx_panel_config *panel_cfg;
	const struct drm_display_mode *mode;
	u32 width;
	u32 height;

	enum ssd16xx_refresh_mode refresh_mode;  /* Configured via DT or defaults */
	bool partial_mode_ready;
	bool temperature_loaded;  /* Temperature LUT loaded for fast refresh */
	bool initialized;
	bool skip_clear_on_resume;  /* Skip display clear on resume if RAM preserved */
};

static inline struct ssd16xx_panel *to_ssd16xx_panel(struct drm_device *drm)
{
	return container_of(drm, struct ssd16xx_panel, drm);
}

static const struct ssd16xx_controller_config ssd16xx_controller_configs[] = {
	[SSD1683] = {
		.max_width = 400,
		.max_height = 300,
		.ram_x_address_bits = 8,
		.ram_y_address_bits = 16,
	},
};

/* Experimental values based on panel & controller datasheet and references as shared in header */
static const struct ssd16xx_panel_config ssd16xx_panel_configs[] = {
	[GDEY042T81] = {
		.red_supported = false,  /* 2-color panel: black/white only */
		.data_entry_mode = SSD16XX_DATA_ENTRY_XINC_YINC,
		.driver_output_ctrl_byte3 = 0x00,  /* No special flags */
		.border_waveform_init = SSD16XX_BORDER_WAVEFORM_VCOM,
		.deep_sleep_mode = SSD16XX_DEEP_SLEEP_MODE_1,
		.default_refresh_mode = SSD16XX_REFRESH_PARTIAL,  /* Fast updates for BW panel */
	},
};

/*
 * Refresh Mode Strategy (based on Seeed reference implementation):
 *
 * Default mode set per panel type in panel config.
 * Can be overridden via module parameter: refresh_mode_override
 *   echo 0 > /sys/module/ssd16xx/parameters/refresh_mode_override  # partial
 *   echo 1 > /sys/module/ssd16xx/parameters/refresh_mode_override  # full
 *   echo 2 > /sys/module/ssd16xx/parameters/refresh_mode_override  # fast
 *
 * 1. FULL REFRESH MODE:
 *    - Control 1: 0x40 for BW panels (bypass RED RAM), 0x00 for 3-color (enable RED RAM)
 *    - Control 2: 0xF7 (full refresh, loads temperature + LUT)
 *    - LUTs: LUTB/LUTW (8 groups × 4 phases = 32 phases)
 *    - Quality: Best, no ghosting
 *    - Time: ~1.5-2s
 *    - Use: When highest quality needed, can show red on 3-color panels
 *
 * 2. FAST REFRESH MODE:
 *    - Control 1: 0x40 (bypass RED RAM)
 *    - Control 2: 0xC7 (fast refresh, uses existing temperature/LUT from init)
 *    - LUTs: LUTB/LUTW (8 groups × 4 phases = 32 phases)
 *    - Quality: Good
 *    - Time: ~1.0-1.5s (skips temperature load)
 *    - Use: When faster updates needed without temperature reload
 *
 * 3. PARTIAL REFRESH MODE:
 *    - Control 1: 0x00 (both RAMs enabled for transitions)
 *    - Control 2: 0xFF (partial refresh, loads temperature + LUT)
 *    - LUTs: LUTBB/LUTWB/LUTBW/LUTWW (6 groups × 4 phases = 24 phases)
 *    - Quality: Good, minor ghosting
 *    - Time: ~300-500ms
 *    - Use: When fastest updates needed, BW panels default
 *
 * Special case: pipe_enable (clear display) always uses FULL REFRESH with RED RAM bypass
 */

/* BUSY pin is ACTIVE HIGH: 1=busy, 0=ready */
static void ssd16xx_wait_for_panel(struct ssd16xx_panel *panel)
{
	unsigned int timeout_ms = 10000;
	unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(timeout_ms);
	unsigned long start_ms = jiffies_to_msecs(jiffies);
	int busy_val;

	busy_val = gpiod_get_value_cansleep(panel->busy);
	drm_dbg(&panel->drm, "BUSY initial value: %d\n", busy_val);

	while (gpiod_get_value_cansleep(panel->busy) == 1) {
		if (time_after(jiffies, timeout_jiffies)) {
			unsigned long elapsed_ms;

			elapsed_ms = jiffies_to_msecs(jiffies) - start_ms;
			drm_err(&panel->drm,
				"Busy wait timed out after %lums\n",
				elapsed_ms);
			return;
		}
		usleep_range(100, 200);
	}

	drm_dbg(&panel->drm, "BUSY became ready after %lums\n",
		jiffies_to_msecs(jiffies) - start_ms);
}

static void ssd16xx_spi_sync(struct spi_device *spi, struct spi_message *msg,
			     struct ssd16xx_error_ctx *err)
{
	int ret;

	if (err->errno_code)
		return;

	ret = spi_sync(spi, msg);
	if (ret < 0)
		err->errno_code = ret;
}

static void ssd16xx_send_cmd(struct ssd16xx_panel *panel, u8 cmd,
			     struct ssd16xx_error_ctx *err)
{
	struct spi_transfer xfer = {
		.tx_buf = &cmd,
		.len = 1,
	};
	struct spi_message msg;

	if (err->errno_code)
		return;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	gpiod_set_value_cansleep(panel->dc, 0);
	ssd16xx_spi_sync(panel->spi, &msg, err);
}

static void ssd16xx_send_data(struct ssd16xx_panel *panel, u8 data,
			      struct ssd16xx_error_ctx *err)
{
	struct spi_transfer xfer = {
		.tx_buf = &data,
		.len = 1,
	};
	struct spi_message msg;

	if (err->errno_code)
		return;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	gpiod_set_value_cansleep(panel->dc, 1);
	ssd16xx_spi_sync(panel->spi, &msg, err);
}


static void ssd16xx_send_x_param(struct ssd16xx_panel *panel, u16 x,
				 struct ssd16xx_error_ctx *err)
{
	if (panel->controller_cfg->ram_x_address_bits == 8) {
		ssd16xx_send_data(panel, (u8)x, err);
	} else {
		ssd16xx_send_data(panel, x & 0xFF, err);
		ssd16xx_send_data(panel, (x >> 8) & 0xFF, err);
	}
}

static void ssd16xx_send_y_param(struct ssd16xx_panel *panel, u16 y,
				 struct ssd16xx_error_ctx *err)
{
	if (panel->controller_cfg->ram_y_address_bits == 8) {
		ssd16xx_send_data(panel, (u8)y, err);
	} else {
		ssd16xx_send_data(panel, y & 0xFF, err);
		ssd16xx_send_data(panel, (y >> 8) & 0xFF, err);
	}
}

static void ssd16xx_send_data_bulk(struct ssd16xx_panel *panel,
				   const u8 *data, size_t len,
				   struct ssd16xx_error_ctx *err)
{
	struct spi_transfer xfer = {
		.tx_buf = data,
		.len = len,
	};
	struct spi_message msg;

	if (err->errno_code)
		return;

	if (!data) {
		drm_err(&panel->drm, "Bulk transfer called with NULL buffer\n");
		err->errno_code = -EINVAL;
		return;
	}

	if (len == 0)
		return;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	gpiod_set_value_cansleep(panel->dc, 1);
	ssd16xx_spi_sync(panel->spi, &msg, err);
}

/*
 * Trigger display update with Display Update Control configuration
 *
 * @ctrl1_byte1: Display Update Control 1 byte 1
 *               SSD16XX_CTRL1_NORMAL = Both RAMs enabled
 *               SSD16XX_CTRL1_BYPASS_RED_RAM = Bypass RED RAM
 * @ctrl1_byte2: Display Update Control 1 byte 2
 *               SSD16XX_CTRL1_BYTE2_DEFAULT = Default settings
 * @ctrl2_mode: Display Update Control 2 mode
 *              SSD1683_CTRL2_FULL_REFRESH = Full refresh (~1.5-2s)
 *              SSD1683_CTRL2_FAST_REFRESH = Fast refresh (~1.0-1.5s)
 *              SSD1683_CTRL2_PARTIAL_REFRESH = Partial refresh (~300-500ms)
 */
static void ssd16xx_display_update(struct ssd16xx_panel *panel,
				   u8 ctrl1_byte1, u8 ctrl1_byte2, u8 ctrl2_mode,
				   struct ssd16xx_error_ctx *err)
{
	int busy_before;

	/* Wait for panel to be ready BEFORE sending update command */
	drm_dbg(&panel->drm, "display_update: Waiting for panel ready before update...\n");
	ssd16xx_wait_for_panel(panel);

	busy_before = gpiod_get_value_cansleep(panel->busy);
	drm_dbg(&panel->drm, "display_update: Setting ctrl1=0x%02x,0x%02x mode=0x%02x (BUSY=%d)\n",
		ctrl1_byte1, ctrl1_byte2, ctrl2_mode, busy_before);

	/* Set Display Update Control 1 */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_DISPLAY_UPDATE_CONTROL1, err);
	ssd16xx_send_data(panel, ctrl1_byte1, err);
	ssd16xx_send_data(panel, ctrl1_byte2, err);

	/* Set Display Update Control 2 and activate */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_DISPLAY_UPDATE_CONTROL2, err);
	ssd16xx_send_data(panel, ctrl2_mode, err);
	ssd16xx_send_cmd(panel, SSD16XX_CMD_MASTER_ACTIVATION, err);

	busy_before = gpiod_get_value_cansleep(panel->busy);
	drm_dbg(&panel->drm,
		"display_update: Master activation sent (BUSY now=%d), waiting for completion...\n",
		busy_before);

	if (!err->errno_code)
		ssd16xx_wait_for_panel(panel);
}

static int ssd16xx_hw_init(struct ssd16xx_panel *panel)
{
	struct ssd16xx_error_ctx err = { .errno_code = 0 };

	/* Hardware reset */
	gpiod_set_value_cansleep(panel->reset, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(panel->reset, 1);
	usleep_range(10000, 11000);

	/* Software reset */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_SW_RESET, &err);
	ssd16xx_wait_for_panel(panel);

	/* Driver output control - set height and scan direction */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_DRIVER_OUTPUT_CONTROL, &err);
	ssd16xx_send_y_param(panel, panel->height - 1, &err);
	ssd16xx_send_data(panel, panel->panel_cfg->driver_output_ctrl_byte3, &err);

	/* Border waveform control */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_BORDER_WAVEFORM_CONTROL, &err);
	ssd16xx_send_data(panel, panel->panel_cfg->border_waveform_init, &err);

	/*
	 * Temperature Sensor Selection: Configure controller to use
	 * internal temperature sensor and automatically select optimal
	 * waveform from OTP based on measured temperature.
	 * Using internal sensor (0x80).
	 */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_TEMPERATURE_SENSOR_CONTROL, &err);
	ssd16xx_send_data(panel, SSD16XX_TEMP_SENSOR_INTERNAL, &err);

	/*
	 * For FAST refresh mode, load temperature and LUT once during initialization.
	 * Fast mode (0xC7) skips temperature load on each update for speed.
	 * FULL (0xF7) and PARTIAL (0xFF) modes load temperature on every update,
	 * so pre-loading here is unnecessary for those modes.
	 *
	 * CRITICAL: Must set Display Update Control 1 BEFORE loading temperature.
	 * This sequence matches Seeed GDEY042T81 reference implementation
	 * (EPD_HW_Init_Fast() and EPD_HW_Init_Fast2()).
	 */
	if (panel->refresh_mode == SSD16XX_REFRESH_FAST) {
		/*
		 * Display Update Control 1: Configure RAM usage.
		 * Value 0x40 = BYPASS_RED_RAM (force RED RAM to 0).
		 * Must be set before temperature load to define controller behavior.
		 */
		ssd16xx_send_cmd(panel, SSD16XX_CMD_DISPLAY_UPDATE_CONTROL1, &err);
		ssd16xx_send_data(panel, SSD16XX_CTRL1_BYPASS_RED_RAM, &err);
		ssd16xx_send_data(panel, SSD16XX_CTRL1_BYTE2_DEFAULT, &err);

		/*
		 * Display Update Control 2: Load temperature and LUT (0x91).
		 * Since we're using internal sensor, controller automatically
		 * senses temperature - no need to manually write value.
		 * This loads temperature without triggering display update.
		 */
		ssd16xx_send_cmd(panel, SSD16XX_CMD_DISPLAY_UPDATE_CONTROL2, &err);
		ssd16xx_send_data(panel, SSD16XX_CTRL2_LOAD_TEMP_LUT, &err);

		/*
		 * Master Activation: Execute the temperature/LUT load operation.
		 * Wait for BUSY to go low (operation complete).
		 */
		ssd16xx_send_cmd(panel, SSD16XX_CMD_MASTER_ACTIVATION, &err);
		ssd16xx_wait_for_panel(panel);

		panel->temperature_loaded = true;
	}

	/* Data entry mode */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_DATA_ENTRY_MODE, &err);
	ssd16xx_send_data(panel, panel->panel_cfg->data_entry_mode, &err);
	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_X_ADDRESS_START_END, &err);
	ssd16xx_send_x_param(panel, 0x00, &err);
	ssd16xx_send_x_param(panel, (panel->width / 8) - 1, &err);

	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_Y_ADDRESS_START_END, &err);
	ssd16xx_send_y_param(panel, 0x00, &err);
	ssd16xx_send_y_param(panel, panel->height - 1, &err);

	/* Set initial cursor position */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_X_ADDRESS_COUNTER, &err);
	ssd16xx_send_x_param(panel, 0x00, &err);

	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_Y_ADDRESS_COUNTER, &err);
	ssd16xx_send_y_param(panel, 0x00, &err);

	ssd16xx_wait_for_panel(panel);

	if (err.errno_code) {
		drm_err(&panel->drm, "Hardware initialization failed: %d\n",
			err.errno_code);
		return err.errno_code;
	}

	/*
	 * Note: partial_mode_ready is NOT set here. It will be set after
	 * the first full refresh establishes a baseline in RED RAM.
	 */
	return 0;
}

/* Clear display: Write 0xFF (white) to both BW and RED RAM */
static void ssd16xx_clear_display(struct ssd16xx_panel *panel)
{
	struct ssd16xx_error_ctx err = { .errno_code = 0 };
	unsigned int data_size = (panel->width * panel->height) / 8;
	u8 *white_buffer;

	white_buffer = kmalloc(data_size, GFP_KERNEL);
	if (!white_buffer)
		return;
	memset(white_buffer, 0xFF, data_size);

	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_X_ADDRESS_COUNTER, &err);
	ssd16xx_send_x_param(panel, 0x00, &err);

	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_Y_ADDRESS_COUNTER, &err);
	ssd16xx_send_y_param(panel, 0x00, &err);

	/* Write white to BW RAM */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_WRITE_RAM_BW, &err);
	ssd16xx_send_data_bulk(panel, white_buffer, data_size, &err);

	ssd16xx_send_cmd(panel, SSD16XX_CMD_WRITE_RAM_RED, &err);
	ssd16xx_send_data_bulk(panel, white_buffer, data_size, &err);
	/*
	 * Clear display with FULL REFRESH mode:
	 * - Display Update Control 1 = BYPASS_RED_RAM
	 * - Display Update Control 2 = FULL_REFRESH (loads temperature + LUT)
	 * - Uses LUTB/LUTW with 8 groups (32 phases) for clean baseline
	 * - RED RAM not written since it's bypassed
	 * - Update time: ~1.5-2s
	 */
	ssd16xx_display_update(panel, SSD16XX_CTRL1_NORMAL,
			       SSD16XX_CTRL1_BYTE2_DEFAULT,
			       SSD1683_CTRL2_FULL_REFRESH, &err);

	if (err.errno_code)
		drm_err(&panel->drm, "Clear display failed: %d\n", err.errno_code);

	kfree(white_buffer);
}


/*
 * Convert XRGB8888 framebuffer to 1-bit monochrome
 * Uses luminance threshold: (0.299*R + 0.587*G + 0.114*B) > 127
 * Optimized: Compare (299*R + 587*G + 114*B) > 127000 to avoid division
 */
static void ssd16xx_convert_fb_to_1bpp(u8 *dst, struct iosys_map *src,
				       struct drm_framebuffer *fb,
				       struct drm_rect *rect)
{
	unsigned int x, y;
	u32 *src_line;
	u8 byte = 0;
	unsigned int bit_pos = 0;
	unsigned int dst_idx = 0;

	for (y = rect->y1; y < rect->y2; y++) {
		src_line = (u32 *)(src->vaddr + y * fb->pitches[0]);

		for (x = rect->x1; x < rect->x2; x++) {
			u32 pixel = src_line[x];
			u8 r = (pixel >> 16) & 0xFF;
			u8 g = (pixel >> 8) & 0xFF;
			u8 b = pixel & 0xFF;

			/*
			 * Calculate luminance using ITU-R BT.601 coefficients:
			 * Y = 0.299*R + 0.587*G + 0.114*B
			 * Threshold at 127, scaled by 1000 to avoid division:
			 * (299*R + 587*G + 114*B) > 127000
			 */
			unsigned int luma_scaled = 299 * r + 587 * g + 114 * b;

			/* Threshold: >127000 = white (1), <=127000 = black (0) */
			if (luma_scaled > 127000)
				byte |= (1 << (7 - bit_pos));

			bit_pos++;
			if (bit_pos == 8) {
				dst[dst_idx++] = byte;
				byte = 0;
				bit_pos = 0;
			}
		}

		/* Handle partial byte at end of line */
		if (bit_pos > 0) {
			dst[dst_idx++] = byte;
			byte = 0;
			bit_pos = 0;
		}
	}
}

/*
 * Convert XRGB8888 framebuffer to 3-color format
 * Separates red component and grayscale for 3-color e-paper panels
 *
 * For each pixel:
 * - If red component dominant (R > threshold AND R > G AND R > B): set red bit
 * - Otherwise: calculate grayscale and set BW bit
 *
 * This allows displaying red/black/white content on 3-color panels
 */
static void ssd16xx_convert_fb_to_3color(u8 *bw_dst, u8 *red_dst,
					 struct iosys_map *src,
					 struct drm_framebuffer *fb,
					 struct drm_rect *rect)
{
	unsigned int x, y;
	u32 *src_line;
	u8 bw_byte = 0, red_byte = 0;
	unsigned int bit_pos = 0;
	unsigned int dst_idx = 0;

	for (y = rect->y1; y < rect->y2; y++) {
		src_line = (u32 *)(src->vaddr + y * fb->pitches[0]);

		for (x = rect->x1; x < rect->x2; x++) {
			u32 pixel = src_line[x];
			u8 r = (pixel >> 16) & 0xFF;
			u8 g = (pixel >> 8) & 0xFF;
			u8 b = pixel & 0xFF;

			/*
			 * Detect red pixels: R component must be dominant
			 * Threshold: R > 127 AND R > G AND R > B
			 */
			if (r > 127 && r > g && r > b) {
				/* Red pixel: set red bit, clear BW bit (will show red) */
				red_byte |= (1 << (7 - bit_pos));
			} else {
				/* Not red: calculate grayscale for BW RAM */
				unsigned int luma_scaled = 299 * r + 587 * g + 114 * b;

				/* White pixels: set BW bit */
				if (luma_scaled > 127000)
					bw_byte |= (1 << (7 - bit_pos));
				/* Black pixels: both bits clear (will show black) */
			}

			bit_pos++;
			if (bit_pos == 8) {
				bw_dst[dst_idx] = bw_byte;
				red_dst[dst_idx] = red_byte;
				dst_idx++;
				bw_byte = 0;
				red_byte = 0;
				bit_pos = 0;
			}
		}

		/* Handle partial byte at end of line */
		if (bit_pos > 0) {
			bw_dst[dst_idx] = bw_byte;
			red_dst[dst_idx] = red_byte;
			dst_idx++;
			bw_byte = 0;
			red_byte = 0;
			bit_pos = 0;
		}
	}
}

static void ssd16xx_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect,
			     struct ssd16xx_panel *panel)
{
	struct drm_gem_dma_object *dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	struct iosys_map map;
	struct ssd16xx_error_ctx err = { .errno_code = 0 };
	unsigned int data_size = (panel->width * panel->height) / 8;
	u8 *mono_buffer;
	u8 *red_buffer = NULL;
	bool use_3color_conversion = false;

	mono_buffer = kzalloc(data_size, GFP_KERNEL);
	if (!mono_buffer)
		return;

	/*
	 * For 3-color panels in FULL refresh mode, we need separate
	 * red and grayscale buffers to properly display red content
	 */
	if (panel->partial_mode_ready &&
	    panel->panel_cfg->red_supported &&
	    panel->refresh_mode == SSD16XX_REFRESH_FULL) {
		red_buffer = kzalloc(data_size, GFP_KERNEL);
		if (!red_buffer) {
			kfree(mono_buffer);
			return;
		}
		use_3color_conversion = true;
	}

	iosys_map_set_vaddr(&map, dma_obj->vaddr);

	/*
	 * Use damage tracking for partial updates when available.
	 * Align rect to byte boundaries (8 pixels) for 1bpp conversion.
	 */
	if (panel->partial_mode_ready) {
		rect->x1 = ALIGN_DOWN(rect->x1, 8);
		rect->x2 = ALIGN(rect->x2, 8);
		drm_dbg(&panel->drm, "Partial update: (%d,%d) to (%d,%d)\n",
			rect->x1, rect->y1, rect->x2, rect->y2);
	} else {
		/* Full display update during initialization */
		rect->x1 = 0;
		rect->y1 = 0;
		rect->x2 = panel->width;
		rect->y2 = panel->height;
		drm_dbg(&panel->drm, "Full display update: %dx%d\n",
			panel->width, panel->height);
	}

	if (use_3color_conversion) {
		/* Separate red and grayscale for 3-color panels */
		ssd16xx_convert_fb_to_3color(mono_buffer, red_buffer, &map, fb, rect);
	} else {
		/* Standard grayscale conversion */
		ssd16xx_convert_fb_to_1bpp(mono_buffer, &map, fb, rect);
	}

	/* Set RAM address counters to start position */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_X_ADDRESS_COUNTER, &err);
	ssd16xx_send_x_param(panel, 0x00, &err);

	ssd16xx_send_cmd(panel, SSD16XX_CMD_SET_RAM_Y_ADDRESS_COUNTER, &err);
	ssd16xx_send_y_param(panel, 0x00, &err);

	/* Write to BW RAM (current frame) */
	ssd16xx_send_cmd(panel, SSD16XX_CMD_WRITE_RAM_BW, &err);
	ssd16xx_send_data_bulk(panel, mono_buffer, data_size, &err);

	if (panel->partial_mode_ready) {
		/*
		 * Atomic updates: Use configured refresh mode from device tree
		 */
		switch (panel->refresh_mode) {
		case SSD16XX_REFRESH_FULL:
			/*
			 * FULL REFRESH MODE:
			 * - For BW panels: Control 1 = BYPASS_RED_RAM
			 * - For 3-color panels: Control 1 = NORMAL (enable RED RAM)
			 * - Control 2 = FULL_REFRESH (loads temperature + LUT)
			 * - Uses LUTB/LUTW with 8 groups (32 phases)
			 * - Update time: ~1.5-2s
			 */
			if (panel->panel_cfg->red_supported) {
				ssd16xx_display_update(panel, SSD16XX_CTRL1_NORMAL,
						       SSD16XX_CTRL1_BYTE2_DEFAULT,
						       SSD1683_CTRL2_FULL_REFRESH, &err);
				/* Write red component to RED RAM for 3-color panels */
				ssd16xx_send_cmd(panel, SSD16XX_CMD_WRITE_RAM_RED, &err);
				ssd16xx_send_data_bulk(panel, red_buffer, data_size, &err);
			} else {
				ssd16xx_display_update(panel, SSD16XX_CTRL1_BYPASS_RED_RAM,
						       SSD16XX_CTRL1_BYTE2_DEFAULT,
						       SSD1683_CTRL2_FULL_REFRESH, &err);
			}
			break;

		case SSD16XX_REFRESH_FAST:
			/*
			 * FAST REFRESH MODE:
			 * - Control 1 = BYPASS_RED_RAM
			 * - Control 2 = FAST_REFRESH (skip temperature load)
			 * - Uses LUTB/LUTW with 8 groups (32 phases)
			 * - Temperature loaded once during hw_init
			 * - Update time: ~1.0-1.5s
			 */
			ssd16xx_display_update(panel, SSD16XX_CTRL1_BYPASS_RED_RAM,
					       SSD16XX_CTRL1_BYTE2_DEFAULT,
					       SSD1683_CTRL2_FAST_REFRESH, &err);
			break;

		case SSD16XX_REFRESH_PARTIAL:
		default:
			/*
			 * PARTIAL REFRESH MODE:
			 * - Control 1 = NORMAL (both RAMs enabled for transitions)
			 * - Control 2 = PARTIAL_REFRESH (loads temperature + LUT)
			 * - Uses LUTBB/LUTWB/LUTBW/LUTWW with 6 groups (24 phases)
			 * - Update time: ~300-500ms
			 * - Sync RED RAM after update for proper transitions
			 */
			ssd16xx_display_update(panel, SSD16XX_CTRL1_NORMAL,
					       SSD16XX_CTRL1_BYTE2_DEFAULT,
					       SSD1683_CTRL2_PARTIAL_REFRESH, &err);
			ssd16xx_send_cmd(panel, SSD16XX_CMD_WRITE_RAM_RED, &err);
			ssd16xx_send_data_bulk(panel, mono_buffer, data_size, &err);
			break;
		}
	} else {
		/*
		 * Baseline establishment: First FULL REFRESH after init
		 * - Write to both BW RAM and RED RAM to establish baseline
		 * - Display Update Control 1 = BYPASS_RED_RAM
		 * - Display Update Control 2 = FULL_REFRESH (loads temperature + LUT)
		 * - Uses LUTB/LUTW with 8 groups (32 phases)
		 * - Update time: ~1.5-2s
		 */
		ssd16xx_send_cmd(panel, SSD16XX_CMD_WRITE_RAM_RED, &err);
		ssd16xx_send_data_bulk(panel, mono_buffer, data_size, &err);
		ssd16xx_display_update(panel, SSD16XX_CTRL1_BYPASS_RED_RAM,
				       SSD16XX_CTRL1_BYTE2_DEFAULT,
				       SSD1683_CTRL2_FULL_REFRESH, &err);
	}

	if (err.errno_code)
		drm_err(&panel->drm, "Display update failed: %d\n", err.errno_code);

	kfree(mono_buffer);
	kfree(red_buffer);
}

static void ssd16xx_pipe_update(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *old_state)
{
	struct drm_plane_state *plane_state = pipe->plane.state;
	struct ssd16xx_panel *panel = to_ssd16xx_panel(pipe->crtc.dev);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_rect rect;

	if (!panel->initialized)
		return;

	if (!pipe->crtc.state->active)
		return;

	if (!fb)
		return;

	if (!drm_atomic_helper_damage_merged(old_state, plane_state, &rect))
		return;

	ssd16xx_fb_dirty(fb, &rect, panel);
}

static void ssd16xx_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	struct ssd16xx_panel *panel = to_ssd16xx_panel(pipe->crtc.dev);
	int idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	if (ssd16xx_hw_init(panel)) {
		drm_err(&panel->drm, "Hardware initialization failed\n");
		goto out_exit;
	}

	/*
	 * Clear display to establish baseline, unless BOTH conditions met:
	 * 1. Resuming from suspend (skip_clear_on_resume = true)
	 * 2. Baseline is valid (partial_mode_ready = true, i.e., mode 1)
	 *
	 * Scenarios:
	 * - Fresh enable: partial_mode_ready = false → CLEAR
	 * - Resume mode 1: both true → SKIP CLEAR (RAM preserved)
	 * - Resume mode 2: partial_mode_ready = false → CLEAR (RAM lost)
	 */
	if (!panel->skip_clear_on_resume || !panel->partial_mode_ready) {
		ssd16xx_clear_display(panel);
	}
	panel->skip_clear_on_resume = false;  /* Reset flag after use */

	/*
	 * Mark partial mode as ready. From this point, fb_dirty() will
	 * use partial refresh (~300ms) instead of full refresh (~2s).
	 * Display Update Control 1 will be set appropriately in each
	 * display_update call (0x00 for partial, 0x40 for full refresh).
	 */
	panel->partial_mode_ready = true;
	panel->initialized = true;

out_exit:
	drm_dev_exit(idx);
}

static void ssd16xx_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct ssd16xx_panel *panel = to_ssd16xx_panel(pipe->crtc.dev);
	struct ssd16xx_error_ctx err = { .errno_code = 0 };

	panel->initialized = false;

	/*
	 * Deep sleep mode 2 does NOT preserve RAM content. Reset partial mode
	 * flag so next enable will clear display to establish new baseline.
	 * Mode 1 preserves RAM, so baseline remains valid.
	 */
	if (panel->panel_cfg->deep_sleep_mode == SSD16XX_DEEP_SLEEP_MODE_2)
		panel->partial_mode_ready = false;

	ssd16xx_send_cmd(panel, SSD16XX_CMD_DEEP_SLEEP_MODE, &err);
	ssd16xx_send_data(panel, panel->panel_cfg->deep_sleep_mode, &err);
}

static enum drm_mode_status
ssd16xx_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			const struct drm_display_mode *mode)
{
	struct ssd16xx_panel *panel = to_ssd16xx_panel(pipe->crtc.dev);

	return drm_crtc_helper_mode_valid_fixed(&pipe->crtc, mode, panel->mode);
}

static const struct drm_simple_display_pipe_funcs ssd16xx_pipe_funcs = {
	.mode_valid = ssd16xx_pipe_mode_valid,
	.enable = ssd16xx_pipe_enable,
	.disable = ssd16xx_pipe_disable,
	.update = ssd16xx_pipe_update,
};

static int ssd16xx_connector_get_modes(struct drm_connector *connector)
{
	struct ssd16xx_panel *panel = to_ssd16xx_panel(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, panel->mode);
}

static const struct drm_connector_helper_funcs ssd16xx_connector_helper_funcs = {
	.get_modes = ssd16xx_connector_get_modes,
};

static const struct drm_connector_funcs ssd16xx_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const u32 ssd16xx_formats[] = {
	DRM_FORMAT_XRGB8888,
};

/* Panel-specific display modes */
static const struct drm_display_mode gdey042t81_mode = {
	DRM_SIMPLE_MODE(400, 300, 85, 64),  /* 4.2" diagonal, 84.8Ã63.6mm active area (rounded) */
};

DEFINE_DRM_GEM_FOPS(ssd16xx_fops);

static struct drm_driver ssd16xx_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &ssd16xx_fops,
	.name = "ssd16xx",
	.desc = "DRM driver for SSD16xx e-ink controller family",
	.major = 1,
	.minor = 0,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static const struct drm_mode_config_funcs ssd16xx_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int ssd16xx_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ssd16xx_panel *panel;
	struct drm_device *drm;
	const struct spi_device_id *spi_id;
	const struct drm_display_mode *mode;
	const void *match;
	enum ssd16xx_model model;
	int ret;

	match = device_get_match_data(dev);
	if (match) {
		model = (enum ssd16xx_model)(uintptr_t)match;
	} else {
		spi_id = spi_get_device_id(spi);
		model = (enum ssd16xx_model)spi_id->driver_data;
	}

	/*
	 * The SPI device is used to allocate DMA memory for fbdev.
	 * Use DMA_BIT_MASK(64) instead of restrictive 32-bit mask.
	 * The DMA subsystem will handle address limitations automatically.
	 */
	if (!dev->coherent_dma_mask) {
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(dev, "Failed to set DMA mask: %d\n", ret);
			return ret;
		}
	}

	panel = devm_drm_dev_alloc(dev, &ssd16xx_drm_driver,
				   struct ssd16xx_panel, drm);
	if (IS_ERR(panel))
		return PTR_ERR(panel);

	drm = &panel->drm;
	panel->spi = spi;
	panel->model = model;
	spi_set_drvdata(spi, panel);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = SSD16XX_SPI_BITS_PER_WORD;

	if (!spi->max_speed_hz) {
		drm_warn(drm, "spi-max-frequency not specified, using %u Hz\n",
			 SSD16XX_SPI_SPEED_DEFAULT);
		spi->max_speed_hz = SSD16XX_SPI_SPEED_DEFAULT;
	}

	ret = spi_setup(spi);
	if (ret < 0) {
		drm_err(drm, "SPI setup failed: %d\n", ret);
		return ret;
	}

	switch (model) {
	case GDEY042T81:
		mode = &gdey042t81_mode;
		panel->controller = SSD1683;
		break;
	default:
		drm_err(drm, "Unknown panel model: %d\n", model);
		return -EINVAL;
	}

	/* Validate and set controller configuration */
	if (panel->controller >= ARRAY_SIZE(ssd16xx_controller_configs) ||
	    !ssd16xx_controller_configs[panel->controller].max_width) {
		drm_err(drm, "Invalid controller: %d\n", panel->controller);
		return -EINVAL;
	}
	panel->controller_cfg = &ssd16xx_controller_configs[panel->controller];

	/* Validate and set panel configuration */
	if (model >= ARRAY_SIZE(ssd16xx_panel_configs)) {
		drm_err(drm, "Invalid panel model: %d\n", model);
		return -EINVAL;
	}
	panel->panel_cfg = &ssd16xx_panel_configs[model];

	panel->mode = mode;
	panel->width = mode->hdisplay;
	panel->height = mode->vdisplay;

	/*
	 * Set refresh mode: panel default or module parameter override
	 * This is a software policy choice, not hardware description,
	 * so it doesn't belong in device tree.
	 */
	if (refresh_mode_override >= 0 && refresh_mode_override <= 2) {
		panel->refresh_mode = refresh_mode_override;
		drm_info(drm, "Using refresh mode override: %s\n",
			 panel->refresh_mode == SSD16XX_REFRESH_FULL ? "full" :
			 panel->refresh_mode == SSD16XX_REFRESH_FAST ? "fast" : "partial");
	} else {
		panel->refresh_mode = panel->panel_cfg->default_refresh_mode;
		drm_info(drm, "Using panel default refresh mode: %s\n",
			 panel->refresh_mode == SSD16XX_REFRESH_FULL ? "full" :
			 panel->refresh_mode == SSD16XX_REFRESH_FAST ? "fast" : "partial");
	}

	/* Get GPIOs - reset starts HIGH (inactive), DC starts LOW */
	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->reset)) {
		drm_err(drm, "Failed to get RESET GPIO: %ld\n", PTR_ERR(panel->reset));
		return PTR_ERR(panel->reset);
	}

	panel->busy = devm_gpiod_get(dev, "busy", GPIOD_IN);
	if (IS_ERR(panel->busy)) {
		drm_err(drm, "Failed to get BUSY GPIO: %ld\n", PTR_ERR(panel->busy));
		return PTR_ERR(panel->busy);
	}

	panel->dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(panel->dc)) {
		drm_err(drm, "Failed to get DC GPIO: %ld\n", PTR_ERR(panel->dc));
		return PTR_ERR(panel->dc);
	}

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.funcs = &ssd16xx_mode_config_funcs;
	drm->mode_config.min_width = panel->width;
	drm->mode_config.max_width = panel->width;
	drm->mode_config.min_height = panel->height;
	drm->mode_config.max_height = panel->height;

	drm_connector_helper_add(&panel->connector, &ssd16xx_connector_helper_funcs);
	ret = drm_connector_init(drm, &panel->connector,
				 &ssd16xx_connector_funcs,
				 DRM_MODE_CONNECTOR_SPI);
	if (ret)
		return ret;

	ret = drm_simple_display_pipe_init(drm, &panel->pipe, &ssd16xx_pipe_funcs,
					   ssd16xx_formats, ARRAY_SIZE(ssd16xx_formats),
					   NULL, &panel->connector);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_fbdev_dma_setup(drm, 0);
	drm_dbg(drm, "SSD16xx e-ink display initialized (%dx%d)\n",
		panel->width, panel->height);

	return 0;
}

static void ssd16xx_remove(struct spi_device *spi)
{
	struct ssd16xx_panel *panel = spi_get_drvdata(spi);
	struct drm_device *drm = &panel->drm;

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static int __maybe_unused ssd16xx_pm_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct ssd16xx_panel *panel = to_ssd16xx_panel(drm);

	/*
	 * Mark resume path to potentially skip display clear.
	 * Mode 1: RAM preserved, partial_mode_ready stays true → skip clear
	 * Mode 2: RAM lost, partial_mode_ready reset to false → force clear
	 */
	panel->skip_clear_on_resume = true;

	return drm_mode_config_helper_suspend(drm);
}

static int __maybe_unused ssd16xx_pm_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	drm_mode_config_helper_resume(drm);

	return 0;
}

static int __maybe_unused ssd16xx_pm_runtime_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);
	struct ssd16xx_panel *panel = to_ssd16xx_panel(drm);

	/*
	 * Runtime suspend: Similar to system suspend.
	 * partial_mode_ready flag handles mode differences.
	 */
	panel->skip_clear_on_resume = true;

	return drm_mode_config_helper_suspend(drm);
}

static int __maybe_unused ssd16xx_pm_runtime_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	drm_mode_config_helper_resume(drm);

	return 0;
}

static const struct dev_pm_ops ssd16xx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ssd16xx_pm_suspend, ssd16xx_pm_resume)
	SET_RUNTIME_PM_OPS(ssd16xx_pm_runtime_suspend, ssd16xx_pm_runtime_resume, NULL)
};

static void ssd16xx_shutdown(struct spi_device *spi)
{
	struct ssd16xx_panel *panel = spi_get_drvdata(spi);

	drm_atomic_helper_shutdown(&panel->drm);
}

static const struct of_device_id ssd16xx_of_match[] = {
	{ .compatible = "gooddisplay,gdey042t81", .data = (void *)GDEY042T81 },
	{ }
};
MODULE_DEVICE_TABLE(of, ssd16xx_of_match);

static const struct spi_device_id ssd16xx_id[] = {
	{ "gdey042t81", GDEY042T81 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ssd16xx_id);

static struct spi_driver ssd16xx_spi_driver = {
	.driver = {
		.name = "ssd16xx",
		.of_match_table = ssd16xx_of_match,
		.pm = &ssd16xx_pm_ops,
	},
	.probe = ssd16xx_probe,
	.remove = ssd16xx_remove,
	.shutdown = ssd16xx_shutdown,
	.id_table = ssd16xx_id,
};
module_spi_driver(ssd16xx_spi_driver);

MODULE_AUTHOR("Devarsh Thakkar <devarsht@ti.com>");
MODULE_DESCRIPTION("DRM driver for Solomon SSD16xx e-ink display controller family");
MODULE_LICENSE("GPL");
