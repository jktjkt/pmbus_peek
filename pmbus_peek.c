/*
 * pmbus_peek.c - initial userspace interrogation of PMBus devices
 *
 * Copyright (C) 2008 David Brownell
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>

//#include <stdbool.h>
#define bool u8
#define true 1
#define false 0

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>


#define HACK		/* can issue no-arguments (W0) mfr-specific calls */

/*
 * This is a simple "tell me about that PMBus device" tool.  It can probe
 * PMBus devices and (for PMBus 1.1 devices) show their self-advertised
 * capabilities.  It knows about all the operations defined in the spec,
 * and can invoke a number of them.
 *
 * Note that those "self-advertised" capabilities are optional, and may
 * be better suited to firmware-based devices rather than pure hardware.
 * A microcontroller with just 2 KB of program memory can support these
 * capabilities, but a bit more memory would help it do Real Work too.
 *
 * Tested on Linux 2.6.24+ using the i2c-gpio bitbanger.
 *
 * See www.pmbus.org to get PMBus 1.1 (or newer) specifications.
 */

typedef __u8 u8;
typedef __u16 u16;
typedef __s8 s8;
typedef __s16 s16;

enum pmbus_cmd_type {
	/* _undef_ = 0, */

	RW1 = 1,	/* read/write one byte */
	RW2,		/* read/write two byte "word" */
	RWB,		/* read/write block (up to 255 bytes) */
	RWB14,		/* read/write block (of 14 bytes) */

	RWP_QUERY,	/* block write/read process call for QUERY */
	RWP_COEFF,	/* block write/read process call for COEFFICIENTS */
	RWB_APP_PROFILE, /* block read with embedded byte count */

	W0,		/* write zero bytes ("send byte", command only) */
	W1,		/* write one byte */

	R1,		/* read one byte */
	R2,		/* read two byte "word" */
};

enum pmbus_unit_type {
	/* _undef_ = 0, */

	/* for RW2 and R2 types */
	VOLTS = 1,
	AMPERES,
	MILLISECONDS,
	DEGREES_C,
	WATTS,
	BITS,

	/* REVISIT:  there are quite a few more units:
	 *  - mV/uSec for V transition rates
	 *  - mV/A (mOhm) for V droop
	 *  - Ohms for calibration
	 *  - percent for duty cycle and fan speed
	 *  - RPM for fan speed
	 *  - kHz for frequency
	 *  - ...
	 */

	/* for various RWB bits of inventory data */
	STRING,
};

struct pmbus_coefficients {
	u8	valid;

	/* Coefficients of the device, valid only after initialization.
	 *
	 * Yeech ... the PMBus spec doesn't talk about signs, except to
	 * say that output voltages are always positive.  Here we will
	 * assume that all values include sign bits.
	 */
	s8	R;
	s16	m;
	s16	b;
};

/*
 * This struct captures the PMBus 1.1 command summary data, in Part II
 * Appendix I of the spec and updated to include units.  It's set up so
 * it can be easily augmented with device-specfic query data and other
 * data as needed to work with these commands.
 */

struct pmbus_cmd_desc {
	/* data from Part II of PMBus spec (constant) */
	const char	*tag;
	u16		cmd;
	u8		type;
	u8		units;

	/* data just for this utility */
	u8		flags;
#if 0
	void		(*decode)(struct pmbus_cmd_desc *op, int value);
	// REVISIT encode too
#endif

	/* from device (variable) */
	u8		query;
	struct pmbus_coefficients c[2];	/* 0 = w, 1 = r */
};

/* some of the command codes found in pmbus_cmd_desc.cmd;
 * these are specifically recognized in this code.
 */
#define PMB_CLEAR_FAULT		0x03
#define PMB_CAPABILITY		0x19
#define PMB_QUERY		0x1a
#define PMB_VOUT_MODE		0x20
#define PMB_COEFFICIENTS	0x30

#define PMB_STATUS_BYTE		0x78
#define PMB_STATUS_WORD		0x79
#define PMB_STATUS_VOUT		0x7a
#define PMB_STATUS_IOUT		0x7b
#define PMB_STATUS_INPUT	0x7c
#define PMB_STATUS_TEMPERATURE	0x7d
#define PMB_STATUS_CML		0x7e
#define PMB_STATUS_OTHER	0x7f

#define PMB_STATUS_MFR_SPECIFIC	0x80
#define PMB_STATUS_FANS_1_2	0x81
#define PMB_STATUS_FANS_3_4	0x82

#define PMB_PMBUS_REVISION	0x98
#define PMB_MFR_ID		0x99
#define PMB_MFR_MODEL		0x9a
#define PMB_MFR_REVISION	0x9b
#define PMB_MFR_LOCATION	0x9c
#define PMB_MFR_DATE		0x9d
#define PMB_MFR_SERIAL		0x9e
#define PMB_APP_PROFILES	0x9f
#define PMB_IC_DEVICE_ID	0xad
#define PMB_IC_DEVICE_REV	0xae
#define PMB_USER_DATA(x)	(0xb0 + (x))		/* 0 <= x <= 15 */
#define PMB_MFR_SPECIFIC(x)	(0xd0 + (x))		/* 0 <= x <= 45 */

#define PMB_MFR_EXT(x)		(0xfe00 + (x))		/* 0 <= x <= 255 */
#define PMB_EXT(x)		(0xff00 + (x))		/* 0 <= x <= 255 */

/* flags for pmbus_cmd-desc.flags */
#define FLG_SHOW_P1		(1 << 0)
#define FLG_STATUS		(1 << 1)
#define FLG_FORMAT_VOUT		(1 << 2)

static inline int is_pmb_8bit(u16 cmd)
{
	/* pure 8 bit command */
	return (cmd & 0xff00) == 0 && (cmd & 0xfe) != 0xfe;
}

static inline int is_pmb_extended(u16 cmd)
{
	/* 9 bit command, with 2nd (upper) byte */
	return (cmd & 0xfe00) == 0xfe00;
}

/*----------------------------------------------------------------------*/

/*
 * NOTE:  first version expects these static values to be morphed
 * in place ... maximum of one device per process.  To support more
 * than one PMBus device per process, make this table "const" then
 * use these as examplars.
 *
 * REVISIT more of these should probably have units...
 */
static struct pmbus_cmd_desc pmbus_ops[] = {

/* These are in numeric order, modulo sequence gaps in the PMBus spec. */

{ .cmd = 0x00, .tag = "page", .type = RW1, },
{ .cmd = 0x01, .tag = "operation", .type = RW1, },
{ .cmd = 0x02, .tag = "on_off_config", .type = RW1, },
{ .cmd = PMB_CLEAR_FAULT, .tag = "clear_fault", .type = W0, },
{ .cmd = 0x04, .tag = "phase", .type = RW1, },
{ .cmd = 0x05, .tag = "page_plus_write", .type = RWB, },
{ .cmd = 0x05, .tag = "page_plus_read", .type = RWB, },

{ .cmd = 0x10, .tag = "write_protect", .type = RW1, },
{ .cmd = 0x11, .tag = "store_default_all", .type = W0, },
{ .cmd = 0x12, .tag = "restore_default_all", .type = W0, },
{ .cmd = 0x13, .tag = "store_default_code", .type = W1, },
{ .cmd = 0x14, .tag = "restore_default_code", .type = W1, },
{ .cmd = 0x15, .tag = "store_user_all", .type = W0, },
{ .cmd = 0x16, .tag = "restore_user_all", .type = W0, },
{ .cmd = 0x17, .tag = "store_user_code", .type = W1, },
{ .cmd = 0x18, .tag = "restore_user_code", .type = W1, },
{ .cmd = PMB_CAPABILITY, .tag = "capability", .type = R1,
		.flags = FLG_SHOW_P1, },
{ .cmd = PMB_QUERY, .tag = "query", .type = RWP_QUERY, },
{ .cmd = 0x1b, .tag = "smbalert_mask", .type = RWB, },

{ .cmd = PMB_VOUT_MODE, .tag = "vout_mode", .type = RW1, },
{ .cmd = 0x21, .tag = "vout_command", .type = RW2, },
{ .cmd = 0x22, .tag = "vout_trim", .type = RW2, .units = VOLTS, },
{ .cmd = 0x23, .tag = "vout_cal_offset", .type = RW2, .units = VOLTS, },
{ .cmd = 0x24, .tag = "vout_max", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT },
{ .cmd = 0x25, .tag = "vout_margin_high", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x26, .tag = "vout_margin_low", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x27, .tag = "vout_transition_rate", .type = RW2, },
{ .cmd = 0x28, .tag = "vout_droop", .type = RW2, },
{ .cmd = 0x29, .tag = "vout_scale_loop", .type = RW2, },
{ .cmd = 0x2a, .tag = "vout_scale_monitor", .type = RW2, },

{ .cmd = PMB_COEFFICIENTS, .tag = "coefficients", .type = RWP_COEFF, },
{ .cmd = 0x31, .tag = "pout_max", .type = RW2, .units = WATTS, },
{ .cmd = 0x32, .tag = "max_duty", .type = RW2, },
{ .cmd = 0x33, .tag = "frequency_switch", .type = RW2, },
{ .cmd = 0x35, .tag = "vin_on", .type = RW2, .units = VOLTS, },
{ .cmd = 0x36, .tag = "vin_off", .type = RW2, .units = VOLTS, },
{ .cmd = 0x37, .tag = "interleave", .type = RW2, },
{ .cmd = 0x38, .tag = "iout_cal_gain", .type = RW2, },
{ .cmd = 0x39, .tag = "iout_cal_offset", .type = RW2, .units = AMPERES, },
{ .cmd = 0x3a, .tag = "fan_config_1_2", .type = RW1, },
{ .cmd = 0x3b, .tag = "fan_command_1", .type = RW2, },
{ .cmd = 0x3c, .tag = "fan_command_2", .type = RW2, },
{ .cmd = 0x3d, .tag = "fan_config_3_4", .type = RW1, },
{ .cmd = 0x3e, .tag = "fan_command_3", .type = RW2, },
{ .cmd = 0x3f, .tag = "fan_command_4", .type = RW2, },

{ .cmd = 0x40, .tag = "vout_ov_fault_limit", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x41, .tag = "vout_ov_fault_response", .type = RW1, },
{ .cmd = 0x42, .tag = "vout_ov_warn_limit", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x43, .tag = "vout_uv_warn_limit", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x44, .tag = "vout_uv_fault_limit", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x45, .tag = "vout_uv_fault_response", .type = RW1, },
{ .cmd = 0x46, .tag = "iout_oc_fault_limit", .type = RW2, .units = AMPERES, },
{ .cmd = 0x47, .tag = "iout_oc_fault_response", .type = RW1, },
{ .cmd = 0x48, .tag = "iout_oc_lv_fault_limit",
		.type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x49, .tag = "iout_oc_lv_fault_response", .type = RW1, },
{ .cmd = 0x4a, .tag = "iout_oc_warn_limit", .type = RW2, .units = AMPERES, },
{ .cmd = 0x4b, .tag = "iout_uc_fault_limit", .type = RW2, .units = AMPERES, },
{ .cmd = 0x4c, .tag = "iout_uc_fault_response", .type = RW1, },

{ .cmd = 0x4f, .tag = "ot_fault_limit", .type = RW2, .units = DEGREES_C,},

{ .cmd = 0x50, .tag = "ot_fault_response", .type = RW1, },
{ .cmd = 0x51, .tag = "ot_warn_limit", .type = RW2, .units = DEGREES_C,},
{ .cmd = 0x52, .tag = "ut_warn_limit", .type = RW2, .units = DEGREES_C,},
{ .cmd = 0x53, .tag = "ut_fault_limit", .type = RW2, .units = DEGREES_C,},
{ .cmd = 0x54, .tag = "ut_fault_response", .type = RW1, },
{ .cmd = 0x55, .tag = "vin_ov_fault_limit", .type = RW2, .units = VOLTS, },
{ .cmd = 0x56, .tag = "vin_ov_fault_response", .type = RW1, },
{ .cmd = 0x57, .tag = "vin_ov_warn_limit", .type = RW2, .units = VOLTS, },
{ .cmd = 0x58, .tag = "vin_uv_warn_limit", .type = RW2, .units = VOLTS, },
{ .cmd = 0x59, .tag = "vin_uv_fault_limit", .type = RW2, .units = VOLTS, },
{ .cmd = 0x5a, .tag = "vin_uv_fault_response", .type = RW1, },
{ .cmd = 0x5b, .tag = "iin_oc_fault_limit", .type = RW2, .units = AMPERES, },
{ .cmd = 0x5c, .tag = "iin_oc_fault_response", .type = RW1, },
{ .cmd = 0x5d, .tag = "iin_oc_warn_limit", .type = RW2, .units = AMPERES, },
{ .cmd = 0x5e, .tag = "power_good_on", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x5f, .tag = "power_good_off", .type = RW2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },

{ .cmd = 0x60, .tag = "ton_delay", .type = RW2, .units = MILLISECONDS, },
{ .cmd = 0x61, .tag = "ton_rise", .type = RW2, .units = MILLISECONDS, },
{ .cmd = 0x62, .tag = "ton_max_fault_limit",
		.type = RW2, .units = MILLISECONDS, },
{ .cmd = 0x63, .tag = "ton_max_fault_response", .type = RW1, },
{ .cmd = 0x64, .tag = "toff_delay", .type = RW2, .units = MILLISECONDS, },
{ .cmd = 0x65, .tag = "toff_fall", .type = RW2, .units = MILLISECONDS, },
{ .cmd = 0x66, .tag = "toff_max_warn_limit",
		.type = RW2, .units = MILLISECONDS, },

{ .cmd = 0x68, .tag = "pout_op_fault_limit", .type = RW2, .units = WATTS, },
{ .cmd = 0x69, .tag = "pout_op_fault_response", .type = RW1, },
{ .cmd = 0x6a, .tag = "pout_op_warn_limit", .type = RW2, .units = WATTS, },
{ .cmd = 0x6b, .tag = "pin_op_warn_limit", .type = RW2, .units = WATTS, },

{ .cmd = PMB_STATUS_BYTE, .tag = "status_byte", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_WORD, .tag = "status_word", .type = R2, .units = BITS,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_VOUT, .tag = "status_vout", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_IOUT, .tag = "status_iout", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_INPUT, .tag = "status_input", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_TEMPERATURE, .tag = "status_temperature", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_CML, .tag = "status_cml", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_OTHER, .tag = "status_other", .type = R1,
		.flags = FLG_STATUS, },

{ .cmd = PMB_STATUS_MFR_SPECIFIC, .tag = "status_mfr_specific", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_FANS_1_2, .tag = "status_fans_1_2", .type = R1,
		.flags = FLG_STATUS, },
{ .cmd = PMB_STATUS_FANS_3_4, .tag = "status_fans_3_4", .type = R1,
		.flags = FLG_STATUS, },

{ .cmd = 0x88, .tag = "read_vin", .type = R2, .units = VOLTS, },
{ .cmd = 0x89, .tag = "read_iin", .type = R2, .units = AMPERES, },
{ .cmd = 0x8a, .tag = "read_vcap", .type = R2, .units = VOLTS, },
{ .cmd = 0x8b, .tag = "read_vout", .type = R2, .units = VOLTS, .flags = FLG_FORMAT_VOUT, },
{ .cmd = 0x8c, .tag = "read_iout", .type = R2, .units = AMPERES, },
{ .cmd = 0x8d, .tag = "read_temperature_1", .type = R2, .units = DEGREES_C, },
{ .cmd = 0x8e, .tag = "read_temperature_2", .type = R2, .units = DEGREES_C, },
{ .cmd = 0x8f, .tag = "read_temperature_3", .type = R2, .units = DEGREES_C, },

{ .cmd = 0x90, .tag = "read_fan_speed_1", .type = R2, },
{ .cmd = 0x91, .tag = "read_fan_speed_2", .type = R2, },
{ .cmd = 0x92, .tag = "read_fan_speed_3", .type = R2, },
{ .cmd = 0x93, .tag = "read_fan_speed_4", .type = R2, },
{ .cmd = 0x94, .tag = "read_duty_cycle", .type = R2, },
{ .cmd = 0x95, .tag = "read_frequency", .type = R2, },
{ .cmd = 0x96, .tag = "read_pout", .type = R2, .units = WATTS, },
{ .cmd = 0x97, .tag = "read_pin", .type = R2, .units = WATTS, },
{ .cmd = PMB_PMBUS_REVISION, .tag = "pmbus_revision", .type = R1,
		.flags = FLG_SHOW_P1, },
{ .cmd = PMB_MFR_ID, .tag = "mfr_id", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },
{ .cmd = PMB_MFR_MODEL, .tag = "mfr_model", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },
{ .cmd = PMB_MFR_REVISION, .tag = "mfr_revision", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },
{ .cmd = PMB_MFR_LOCATION, .tag = "mfr_location", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },
{ .cmd = PMB_MFR_DATE, .tag = "mfr_date", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },
{ .cmd = PMB_MFR_SERIAL, .tag = "mfr_serial", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },
{ .cmd = PMB_APP_PROFILES, .tag = "app_profile_support",
		.type = RWB_APP_PROFILE, .flags = FLG_SHOW_P1, },

{ .cmd = 0xa0, .tag = "mfr_vin_min", .type = R2, .units = VOLTS, },
{ .cmd = 0xa1, .tag = "mfr_vin_max", .type = R2, .units = VOLTS, },
{ .cmd = 0xa2, .tag = "mfr_iin_max", .type = R2, .units = AMPERES, },
{ .cmd = 0xa3, .tag = "mfr_pin_max", .type = R2, .units = WATTS, },
{ .cmd = 0xa4, .tag = "mfr_vout_min", .type = R2, .units = VOLTS, },
{ .cmd = 0xa5, .tag = "mfr_vout_max", .type = R2, .units = VOLTS, },
{ .cmd = 0xa5, .tag = "mfr_iout_max", .type = R2, .units = AMPERES, },
{ .cmd = 0xa7, .tag = "mfr_pout_max", .type = R2, .units = WATTS, },
{ .cmd = 0xa8, .tag = "mfr_tambient_max", .type = R2, .units = DEGREES_C, },
{ .cmd = 0xa9, .tag = "mfr_tambient_min", .type = R2, .units = DEGREES_C, },
{ .cmd = 0xaa, .tag = "mfr_efficiency_ll", .type = RWB14, },
{ .cmd = 0xab, .tag = "mfr_efficiency_hl", .type = RWB14, },
{ .cmd = 0xac, .tag = "mfr_pin_accuracy", .type = R1, },
{ .cmd = PMB_IC_DEVICE_ID, .tag = "ic_device_id", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },
{ .cmd = PMB_IC_DEVICE_REV, .tag = "ic_device_rev", .type = RWB,
		.units = STRING, .flags = FLG_SHOW_P1, },

{ .cmd = PMB_USER_DATA(0),  .tag = "user_data_00", .type = RWB, },
{ .cmd = PMB_USER_DATA(1),  .tag = "user_data_01", .type = RWB, },
{ .cmd = PMB_USER_DATA(2),  .tag = "user_data_02", .type = RWB, },
{ .cmd = PMB_USER_DATA(3),  .tag = "user_data_03", .type = RWB, },
{ .cmd = PMB_USER_DATA(4),  .tag = "user_data_04", .type = RWB, },
{ .cmd = PMB_USER_DATA(5),  .tag = "user_data_05", .type = RWB, },
{ .cmd = PMB_USER_DATA(6),  .tag = "user_data_06", .type = RWB, },
{ .cmd = PMB_USER_DATA(7),  .tag = "user_data_07", .type = RWB, },
{ .cmd = PMB_USER_DATA(8),  .tag = "user_data_08", .type = RWB, },
{ .cmd = PMB_USER_DATA(9),  .tag = "user_data_09", .type = RWB, },
{ .cmd = PMB_USER_DATA(10), .tag = "user_data_10", .type = RWB, },
{ .cmd = PMB_USER_DATA(11), .tag = "user_data_11", .type = RWB, },
{ .cmd = PMB_USER_DATA(12), .tag = "user_data_12", .type = RWB, },
{ .cmd = PMB_USER_DATA(13), .tag = "user_data_13", .type = RWB, },
{ .cmd = PMB_USER_DATA(14), .tag = "user_data_14", .type = RWB, },
{ .cmd = PMB_USER_DATA(15), .tag = "user_data_15", .type = RWB, },

{ .cmd = 0xc0, .tag = "mfr_max_temp_1", .type = RW2, .units = DEGREES_C, },
{ .cmd = 0xc1, .tag = "mfr_max_temp_1", .type = RW2, .units = DEGREES_C, },
{ .cmd = 0xc2, .tag = "mfr_max_temp_1", .type = RW2, .units = DEGREES_C, },

{ .cmd = PMB_MFR_SPECIFIC(0),  .tag = "mfr_specific_00", },
{ .cmd = PMB_MFR_SPECIFIC(1),  .tag = "mfr_specific_01", },
{ .cmd = PMB_MFR_SPECIFIC(2),  .tag = "mfr_specific_02", },
{ .cmd = PMB_MFR_SPECIFIC(3),  .tag = "mfr_specific_03", },
{ .cmd = PMB_MFR_SPECIFIC(4),  .tag = "mfr_specific_04", },
{ .cmd = PMB_MFR_SPECIFIC(5),  .tag = "mfr_specific_05", },
{ .cmd = PMB_MFR_SPECIFIC(6),  .tag = "mfr_specific_06", },
{ .cmd = PMB_MFR_SPECIFIC(7),  .tag = "mfr_specific_07", },
{ .cmd = PMB_MFR_SPECIFIC(8),  .tag = "mfr_specific_08", },
{ .cmd = PMB_MFR_SPECIFIC(9),  .tag = "mfr_specific_09", },
{ .cmd = PMB_MFR_SPECIFIC(10), .tag = "mfr_specific_10", },
{ .cmd = PMB_MFR_SPECIFIC(11), .tag = "mfr_specific_11", },
{ .cmd = PMB_MFR_SPECIFIC(12), .tag = "mfr_specific_12", },
{ .cmd = PMB_MFR_SPECIFIC(13), .tag = "mfr_specific_13", },
{ .cmd = PMB_MFR_SPECIFIC(14), .tag = "mfr_specific_14", },
{ .cmd = PMB_MFR_SPECIFIC(15), .tag = "mfr_specific_15", },

{ .cmd = PMB_MFR_SPECIFIC(16), .tag = "mfr_specific_16", },
{ .cmd = PMB_MFR_SPECIFIC(17), .tag = "mfr_specific_17", },
{ .cmd = PMB_MFR_SPECIFIC(18), .tag = "mfr_specific_18", },
{ .cmd = PMB_MFR_SPECIFIC(19), .tag = "mfr_specific_19", },
{ .cmd = PMB_MFR_SPECIFIC(20), .tag = "mfr_specific_20", },
{ .cmd = PMB_MFR_SPECIFIC(21), .tag = "mfr_specific_21", },
{ .cmd = PMB_MFR_SPECIFIC(22), .tag = "mfr_specific_22", },
{ .cmd = PMB_MFR_SPECIFIC(23), .tag = "mfr_specific_23", },
{ .cmd = PMB_MFR_SPECIFIC(24), .tag = "mfr_specific_24", },
{ .cmd = PMB_MFR_SPECIFIC(25), .tag = "mfr_specific_25", },
{ .cmd = PMB_MFR_SPECIFIC(26), .tag = "mfr_specific_26", },
{ .cmd = PMB_MFR_SPECIFIC(27), .tag = "mfr_specific_27", },
{ .cmd = PMB_MFR_SPECIFIC(29), .tag = "mfr_specific_28", },
{ .cmd = PMB_MFR_SPECIFIC(29), .tag = "mfr_specific_29", },
{ .cmd = PMB_MFR_SPECIFIC(30), .tag = "mfr_specific_30", },
{ .cmd = PMB_MFR_SPECIFIC(31), .tag = "mfr_specific_31", },

{ .cmd = PMB_MFR_SPECIFIC(32), .tag = "mfr_specific_32", },
{ .cmd = PMB_MFR_SPECIFIC(33), .tag = "mfr_specific_33", },
{ .cmd = PMB_MFR_SPECIFIC(34), .tag = "mfr_specific_34", },
{ .cmd = PMB_MFR_SPECIFIC(35), .tag = "mfr_specific_35", },
{ .cmd = PMB_MFR_SPECIFIC(36), .tag = "mfr_specific_36", },
{ .cmd = PMB_MFR_SPECIFIC(37), .tag = "mfr_specific_37", },
{ .cmd = PMB_MFR_SPECIFIC(38), .tag = "mfr_specific_38", },
{ .cmd = PMB_MFR_SPECIFIC(39), .tag = "mfr_specific_39", },
{ .cmd = PMB_MFR_SPECIFIC(40), .tag = "mfr_specific_40", },
{ .cmd = PMB_MFR_SPECIFIC(41), .tag = "mfr_specific_41", },
{ .cmd = PMB_MFR_SPECIFIC(42), .tag = "mfr_specific_42", },
{ .cmd = PMB_MFR_SPECIFIC(43), .tag = "mfr_specific_43", },
{ .cmd = PMB_MFR_SPECIFIC(44), .tag = "mfr_specific_44", },
{ .cmd = PMB_MFR_SPECIFIC(45), .tag = "mfr_specific_45", },
{ .cmd = 0xfe, .tag = "mfr_specific_command_ext", },
{ .cmd = 0xff, .tag = "pmbus_command_ext", },

{ /* ZEROES TERMINATE THIS LIST */ },
};

static struct pmbus_cmd_desc unsupported = { .tag = "UNSUPPORTED", };

/*----------------------------------------------------------------------*/

struct pmbus_dev {
	int			fd;
	unsigned long		funcs;
	char			*bus;
	u8			addr;		/* on bus */
	u8			revision;
	u8			capability;
	u8			no_query;
	u8			use_pec;
	struct pmbus_cmd_desc	*op[256];
};

static int verbose;
static int enable_pec;

/*----------------------------------------------------------------------*/

/*
 * The only userspace code for SMBus ops I found comes with a libsensors
 * package that clobbers <linux/i2c-dev.h> on install.  And what we need
 * is PMBus ops anyway... hence this code, which understands some of the
 * ways that PMBus differs from SMBus.
 *
 * Use of 2-byte commands is rejected, but comments show how it could
 * be implemented with reasonable portability.
 *
 * REVISIT return values handling ... "errno" seems to be meaningless
 * since it's almost always EPERM (1), sigh.
 */

/* Send a bit to the device, if it's present. */
static inline int smbus_quick(int fd, int flag)
{
	struct i2c_smbus_ioctl_data	arg;

	memset(&arg, 0, sizeof arg);
	arg.read_write = flag ? I2C_SMBUS_READ : I2C_SMBUS_WRITE;
	/* no command */
	arg.size = I2C_SMBUS_QUICK;
	/* no data */

	if (ioctl(fd, I2C_SMBUS, &arg) < 0)
		return -errno;
	return 0;
}

/*----------------------------------------------------------------------*/

/*
 * READ operations
 */

/* Returns a byte, or negative errno. */
static int pmbus_read_byte_data(int fd, u16 cmd)
{
	struct i2c_smbus_ioctl_data	arg;
	u8				byte;

	/* use i2c; or READ_I2C_BLOCK_2: 2 byte cmd, 1 byte block */
	if (is_pmb_extended(cmd))
		return -ENOSYS;

	if (!is_pmb_8bit(cmd))
		return -EINVAL;

	memset(&arg, 0, sizeof arg);
	arg.read_write = I2C_SMBUS_READ;
	arg.command = cmd;
	arg.size = I2C_SMBUS_BYTE_DATA;
	arg.data = (union i2c_smbus_data *) &byte;

	if (ioctl(fd, I2C_SMBUS, &arg) < 0)
		return -errno;

	return byte;
}

/* Returns a word, or negative errno. */
static int pmbus_read_word_data(int fd, u16 cmd)
{
	struct i2c_smbus_ioctl_data	arg;
	u16				word;

	/* use i2c; or READ_I2C_BLOCK_2: 2 byte cmd, 2 byte block */
	if (is_pmb_extended(cmd))
		return -ENOSYS;

	if (!is_pmb_8bit(cmd))
		return -EINVAL;

	memset(&arg, 0, sizeof arg);
	arg.read_write = I2C_SMBUS_READ;
	arg.command = cmd;
	arg.size = I2C_SMBUS_WORD_DATA;
	arg.data = (union i2c_smbus_data *) &word;

	if (ioctl(fd, I2C_SMBUS, &arg) < 0)
		return -errno;

	/* adapter code handled byteswapping if needed */
	return word;
}

/* Returns the number of bytes copied into read_buf, or negative errno.
 * If the block is bigger than read_len, read_len is copied and -E2BIG
 * is returned (so the caller can recover, somewhat).
 */
static int pmbus_read_block(struct pmbus_dev *pmdev, u16 cmd,
		unsigned read_len, u8 *read_buf)
{
	struct i2c_smbus_ioctl_data	arg;
	union i2c_smbus_data		data;
	int				retval;
	int				len;

	if (!read_buf || read_len == 0)
		return -EINVAL;

	/* use i2c; or READ_I2C_BLOCK_2: 2 byte cmd, N byte block */
	if (is_pmb_extended(cmd))
		return -ENOSYS;

	if (!is_pmb_8bit(cmd))
		return -EINVAL;

	/* Handle "large" blocks sanely by issuing an extra read to
	 * prevent one fault-path traversal (e.g. SMBALERT#) when the
	 * block is bigger than the morsel allowed by SMBus.
	 * As we are only reading a part of the packet from the slave,
	 * we have to temporarily disable PEC.
	 */
	if (pmdev->use_pec) {
		if (ioctl(pmdev->fd, I2C_PEC, 0)) {
			fprintf(stderr, "Cannot temporarily disable PEC");
		}
	}
	len = pmbus_read_byte_data(pmdev->fd, cmd);
	if (len < 0)
		return len;
	if (pmdev->use_pec) {
		if (ioctl(pmdev->fd, I2C_PEC, 1)) {
			fprintf(stderr, "Cannot re-enable PEC");
		}
	}

	if (len > I2C_SMBUS_BLOCK_MAX) {
		retval = -EFBIG;
		goto try_i2c;
	}

	/* If there's no SMBus support, I2C will work (we checked).
	 * Or, we might use "I2C block read" for up to 31 data bytes.
	 */
	if (!(pmdev->funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA)) {
		retval = -EOPNOTSUPP;
		goto try_i2c;
	}

	memset(&arg, 0, sizeof arg);
	arg.read_write = I2C_SMBUS_READ;
	arg.command = cmd;
	arg.size = I2C_SMBUS_BLOCK_DATA;
	arg.data = &data;

	/* When this fails, we can't really know why.  In case it's the
	 * SMBus code saying "block too big", try again (if possible).
	 */
	if (ioctl(pmdev->fd, I2C_SMBUS, &arg) < 0) {
		retval = -errno;
		goto try_i2c;
	}

	if (data.block[0] <= read_len) {
		if (data.block[0] > 32) {
			/* NOTE:  this probably won't be visible */
			retval = -EFBIG;
			goto try_i2c;
		}
		retval = read_len = data.block[0];
	} else
		retval = -E2BIG;
	memcpy(read_buf, &data.block[1], read_len);

	return retval;

try_i2c:
	/* NOTE: no PEC here, but it *could* be done here in userspace */
	if (pmdev->funcs & I2C_FUNC_I2C) {
		struct i2c_msg			msg[2];
		struct i2c_rdwr_ioctl_data	msgdat;
		u8				buf[256];

		msgdat.msgs = msg;
		msgdat.nmsgs = 2;

		msg[0].addr = msg[1].addr = pmdev->addr;

		buf[0] = cmd & 0x0ff;

		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = buf;

		msg[1].flags = I2C_M_RD;
		msg[1].len = len + 1;
		msg[1].buf = buf;

		if (ioctl(pmdev->fd, I2C_RDWR, &msgdat) < 0)
			return -errno;

		if (buf[0] <= read_len)
			retval = read_len = buf[0];
		else
			retval = -E2BIG;
		memcpy(read_buf, &buf[1], read_len);
	}

	return retval;
}

/*----------------------------------------------------------------------*/

/*
 * WRITE operations
 */

/* This is ONLY to temporarily shut up some gcc warnings; most
 * of the write-side infrastructure is not yet used here.
 */
#define SHADDAP	inline

/* With PMBus, transactions may never start with the "read" bit set.
 * That includes "Quick" messages.  Break that rule and get a CML
 * alert (e.g. SMBALERT#).
 */
static int pmbus_quick(int fd)
{
	return smbus_quick(fd, 0);
}

/* Returns zero, or negative errno. */
static int smbus_write_byte(int fd, u8 byte)
{
	struct i2c_smbus_ioctl_data	arg;

	memset(&arg, 0, sizeof arg);
	arg.read_write = I2C_SMBUS_WRITE;
	arg.command = byte;
	arg.size = I2C_SMBUS_BYTE;

	if (ioctl(fd, I2C_SMBUS, &arg) < 0)
		return -errno;
	return 0;
}

/* Returns zero, or negative errno. */
static SHADDAP int pmbus_write_byte_data(int fd, u16 cmd, u8 byte)
{
	struct i2c_smbus_ioctl_data	arg;

	/* use i2c; or WRITE_I2C_BLOCK_2: 2 byte cmd, 1 byte block;
	 * or tweak params to SMBus "write word"
	 */
	if (is_pmb_extended(cmd))
		return -ENOSYS;

	if (!is_pmb_8bit(cmd))
		return -EINVAL;

	memset(&arg, 0, sizeof arg);
	arg.read_write = I2C_SMBUS_WRITE;
	arg.command = cmd;
	arg.size = I2C_SMBUS_BYTE_DATA;
	arg.data = (union i2c_smbus_data *) &byte;

	if (ioctl(fd, I2C_SMBUS, &arg) < 0)
		return -errno;
	return 0;
}

/* Returns zero, or negative errno. */
static SHADDAP int pmbus_write_word_data(int fd, u16 cmd, u16 word)
{
	struct i2c_smbus_ioctl_data	arg;

	/* use i2c; or WRITE_I2C_BLOCK_2: 2 byte cmd, 2 byte block */
	if (is_pmb_extended(cmd))
		return -ENOSYS;

	if (!is_pmb_8bit(cmd))
		return -EINVAL;

	/* adapter code handles byteswapping if needed */
	memset(&arg, 0, sizeof arg);
	arg.read_write = I2C_SMBUS_WRITE;
	arg.command = cmd;
	arg.size = I2C_SMBUS_WORD_DATA;
	arg.data = (union i2c_smbus_data *) &word;

	if (ioctl(fd, I2C_SMBUS, &arg) < 0)
		return -errno;
	return 0;
}

/* Returns zero, or negative errno. */
static SHADDAP int pmbus_write_block(struct pmbus_dev *pmdev, u16 cmd,
		unsigned write_len, u8 *write_buf)
{
	struct i2c_smbus_ioctl_data	arg;
	union i2c_smbus_data		data;
	int				retval;

	if (!write_buf || write_len == 0 || write_len > 255)
		return -EINVAL;

	/* use i2c; or WRITE_I2C_BLOCK_2: 2 byte cmd, N byte block */
	if (is_pmb_extended(cmd))
		return -ENOSYS;

	if (!is_pmb_8bit(cmd))
		return -EINVAL;

	/* If there's no SMBus support, I2C might work (we did NOT ensure
	 * there will be one or the other)
	 */
	if (!(pmdev->funcs & I2C_FUNC_SMBUS_WRITE_BLOCK_DATA)) {
		retval = -EOPNOTSUPP;
		goto try_i2c;
	}
	if (write_len > 32) {
		retval = -EFBIG;
		goto try_i2c;
	}

	data.block[0] = write_len;
	memcpy(&data.block[1], write_buf, write_len);

	memset(&arg, 0, sizeof arg);
	arg.read_write = I2C_SMBUS_WRITE;
	arg.command = cmd;
	arg.size = I2C_SMBUS_BLOCK_DATA;
	arg.data = &data;

	if (ioctl(pmdev->fd, I2C_SMBUS, &arg) < 0)
		return -errno;
	return 0;

try_i2c:
	/* NOTE: no PEC here, but it *could* be done here in userspace */
	if (pmdev->funcs & I2C_FUNC_I2C) {
		struct i2c_msg			msg;
		struct i2c_rdwr_ioctl_data	msgdat;
		u8				buf[257];

		msgdat.msgs = &msg;
		msgdat.nmsgs = 1;

		msg.addr = pmdev->addr;
		msg.flags = 0;
		msg.len = 2 + write_len;
		msg.buf = buf;

		buf[0] = cmd;
		buf[1] = write_len;
		memcpy(&buf[2], write_buf, write_len);

		retval = (ioctl(pmdev->fd, I2C_RDWR, &msgdat) < 0)
				? -errno : 0;
	}

	return retval;
}

/*----------------------------------------------------------------------*/

static void
coefficients(struct pmbus_dev *pmdev, struct pmbus_cmd_desc *op, int read)
{
	union i2c_smbus_data		data;
	int				status;
	struct pmbus_coefficients	*c;

	read = !!read;
	c = op->c + read;

	/* This is specified as a block proc call, which is currently not
	 * widely supported.  The I2C-level backup makes sure that many
	 * more systems can use this call.
	 *
	 * NOTE as with QUERY, handling for "extended" commands is not
	 * specified by PMBus 1.1 ...
	 */
	if (pmdev->funcs & I2C_FUNC_SMBUS_BLOCK_PROC_CALL) {
		struct i2c_smbus_ioctl_data	arg;

		data.block[0] = 2;
		data.block[1] = op->cmd;
		data.block[2] = read;

		memset(&arg, 0, sizeof arg);
		arg.read_write = I2C_SMBUS_WRITE;
		arg.command = PMB_COEFFICIENTS;
		arg.size = I2C_SMBUS_BLOCK_PROC_CALL;
		arg.data = &data;

		status = ioctl(pmdev->fd, I2C_SMBUS, &arg);

	/* NOTE: no PEC here, but it *could* be done here in userspace */
	} else if (pmdev->funcs & I2C_FUNC_I2C) {
		struct i2c_msg			msg[2];
		struct i2c_rdwr_ioctl_data	msgdat;

		msgdat.msgs = msg;
		msgdat.nmsgs = 2;

		msg[0].addr = msg[1].addr = pmdev->addr;

		msg[0].flags = 0;
		msg[0].len = 4;
		msg[0].buf = data.block;

		data.block[0] = PMB_COEFFICIENTS;
		data.block[1] = 2;
		data.block[2] = op->cmd;
		data.block[3] = read;

		msg[1].flags = I2C_M_RD;
		msg[1].len = 6;
		msg[1].buf = data.block;

		status = ioctl(pmdev->fd, I2C_RDWR, &msgdat);

	} else
		status = -EOPNOTSUPP;

	if (status < 0)
		return;

	if (data.block[0] != 5)
		return;

	c->m = (data.block[2] << 8) | data.block[1];
	c->b = (data.block[4] << 8) | data.block[3];
	c->R = data.block[5];
	c->valid = 1;
}

static void query(struct pmbus_dev *pmdev, struct pmbus_cmd_desc *op)
{
	struct i2c_smbus_ioctl_data	arg;
	u16				word;

	/* NOTE query for "extended" commands is not specified by PMBus 1.1;
	 * presumably that will just send a two byte block (which can't use
	 * the portability tweak below).  Fortunately there's no evident
	 * need to support such two-byte commands yet.
	 */
	if (op->cmd > 0xff)
		return;

	/* This is specified as a block proc call, which is currently not
	 * widely supported.  Instead, implement this using normal proc
	 * calls, which are more widely available.
	 */
	word = (op->cmd << 8) | 1;

	memset(&arg, 0, sizeof arg);
	arg.command = PMB_QUERY;
	arg.size = I2C_SMBUS_PROC_CALL;
	arg.data = (union i2c_smbus_data *) &word;

	if (ioctl(pmdev->fd, I2C_SMBUS, &arg) < 0 || (word & 0x00ff) != 1) {
		/* REVISIT we _really_ want QUERY to work, so it'd be nice
		 * to recover from transient faults here.  If we could tell
		 * such faults from real ones, that is... instead of seeing
		 * false EPERM reports for everything in the I2C stack.
		 */
		pmdev->no_query = 1;
		return;
	}

	word >>= 8;

	/* query supported, but not this operation? */
	if (!(word & (1 << 7))) {
		pmdev->op[op->cmd] = &unsupported;
		return;
	}

	/* NOTE eventually use copy/clone here instead of in-place mutate,
	 * when managing multiple devices concurrently
	 */
	op->query = word;
	pmdev->op[op->cmd] = op;

	/* Try to get the coefficients for DIRECT format numbers */
	if (((word >> 2) & 7) == 3 && pmdev->op[PMB_COEFFICIENTS]) {
		if (word & (1 << 5))	/* read (always) */
			coefficients(pmdev, op, 1);
		if (word & (1 << 6))	/* write */
			coefficients(pmdev, op, 0);
	}

	if (op->cmd == PMB_VOUT_MODE) {
		/* VOUT_MODE is a special snowflake, its coefficients are
		 * at least per-page, not per-command.
		 */
		int value = pmbus_read_byte_data(pmdev->fd, op->cmd);
		op->c[0].R = op->c[1].R = value;
		return;
	}

}

/* Return:  negative = can't tell, 0 = no, 1 = yes */
static int checksupport(struct pmbus_dev *pmdev, u16 cmd)
{
	/* NOTE:  no revision check.  QUERY is a PMBUS 1.1 addition,
	 * but the device's PMBus revision may not be exposed.
	 *
	 * NOTE:  this assumes we can check prefixes for PMB_MFR_EXT(x)
	 * and PMB_EXT(x) operations, even though we can't query those
	 * operations directly...
	 */
	if (is_pmb_extended(cmd))
		return -1;

	if (pmdev->op[PMB_QUERY] == &unsupported || pmdev->no_query)
		return -1;

	if (!pmdev->op[cmd]) {
		struct pmbus_cmd_desc *op = pmbus_ops;

		for (op = pmbus_ops; !pmdev->no_query && op->tag; op++) {
			if (op->cmd == cmd) {
				query(pmdev, op);
				break;
			}
		}
	}

	return pmdev->op[cmd] != &unsupported;
}

static char *pmbus_read_string(struct pmbus_dev *pmdev, u16 cmd)
{
	u8 buf[256];
	int status;

	/* For non-queryable devices we'll still try to read inventory
	 * data strings.  That should be harmless but informative.
	 */
	if (checksupport(pmdev, cmd) == 0)
		return NULL;

	memset(buf, 0, sizeof buf);
	status = pmbus_read_block(pmdev, cmd, sizeof buf - 1, buf);
	return (status > 0) ? strdup((void *)buf) : NULL;
}

/*----------------------------------------------------------------------*/

static char *units(struct pmbus_cmd_desc *op)
{
	switch (op->units) {
	case VOLTS:
		return "Volts";
	case AMPERES:
		return "Amperes";
	case MILLISECONDS:
		return "milliseconds";
	case DEGREES_C:
		return "degrees Celsius";
	case WATTS:
		return "Watts";
	default:
		return NULL;
	}
}


/*----------------------------------------------------------------------*/

static void pmbus_list_inventory(struct pmbus_dev *pmdev)
{
	char *mfr = pmbus_read_string(pmdev, PMB_MFR_ID);
	char *model = pmbus_read_string(pmdev, PMB_MFR_MODEL);
	char *revision = pmbus_read_string(pmdev, PMB_MFR_REVISION);
	char *location = pmbus_read_string(pmdev, PMB_MFR_LOCATION);
	char *date = pmbus_read_string(pmdev, PMB_MFR_DATE);
	char *serial = pmbus_read_string(pmdev, PMB_MFR_SERIAL);
	char *ic_device_id = pmbus_read_string(pmdev, PMB_IC_DEVICE_ID);
	char *ic_device_rev = pmbus_read_string(pmdev, PMB_IC_DEVICE_REV);

	if (!mfr && !model && !revision && !location && !date && !serial
			&& !ic_device_id &&!ic_device_rev)
		return;

	/* REVISIT might be useful to fetch all these strings and save
	 * them away.  They're the first handle we have on just what
	 * kind of device we're working with...
	 */

	printf("Inventory Data:\n");
	if (mfr) {
		printf("  Manufacturer:\t\t%s\n", mfr);
		free(mfr);
	}
	if (model) {
		printf("  Model:\t\t%s\n", model);
		free(model);
	}
	if (mfr) {
		printf("  Revision:\t\t%s\n", revision);
		free(revision);
	}
	if (location) {
		printf("  Built at:\t\t%s\n", location);
		free(location);
	}
	if (date) {
		printf("  Built on:\t\t%s\n", date);
		free(date);
	}
	if (serial) {
		printf("  Serial:\t\t%s\n", serial);
		free(serial);
	}
	if (ic_device_id) {
		printf("  IC Device:\t\t%s\n", ic_device_id);
		free(ic_device_id);
	}
	if (ic_device_rev) {
		printf("  IC Device Revision:\t\t%s\n", ic_device_rev);
		free(ic_device_rev);
	}
	printf("\n");
}

static void pmbus_dev_show_p1(struct pmbus_dev *pmdev)
{
	const char		*s0, *s1;
	struct pmbus_cmd_desc	*op;

	printf("PMBus slave on %s, address %#02x\n\n", pmdev->bus, pmdev->addr);

	pmbus_list_inventory(pmdev);

	/*
	 * NOTE spec bug:  table shows two four bit masks, which are
	 * described as two fields, 3 bits 7-5 and 5 bits 4-0  ... in
	 * fact part II probably deserves an extra bit.  Rev 1.0 of the
	 * spec said the fields are 3 and 5 bits; let's be compatible.
	 */
	switch ((pmdev->revision >> 5) & 0x07) {
	case 0:		s1 = "1.0"; break;
	case 1:		s1 = "1.1"; break;
	case 2:		s1 = "1.2"; break;
	default:	s1 = "?"; break;
	}
	switch (pmdev->revision & 0x1f) {
	case 0:		s0 = "1.0"; break;
	case 1:		s0 = "1.1"; break;
	case 2:		s0 = "1.2"; break;
	default:	s0 = "?"; break;
	}
	printf("PMBus revisions (%#02x):\tpart I, ver %s; part II, ver %s\n",
		pmdev->revision, s1, s0);

	if (pmdev->capability & 0xf0) {
		switch ((pmdev->capability >> 5) & 3) {
		case 0:		s0 = "100 KHz"; break;
		case 1:		s0 = "400 KHz"; break;
		default:	s0 = "?speed?"; break;
		}
		printf("Capabilities (%#02x):\t%s%s%s\n",
			pmdev->capability,
			(pmdev->capability & (1 << 7)) ? "PEC, " : "",
			(pmdev->capability & (1 << 4)) ? "SMBALERT#, ": "",
			s0);
	}
	printf("\n");

	if (checksupport(pmdev, PMB_APP_PROFILES) == 1) {
		printf("Application Profiles:\n");

		u8 buf[513];
		int size = pmbus_read_block(pmdev, PMB_APP_PROFILES, sizeof(buf), buf);
		for (int i = 0; size > 1 && i < size / 2; ++i) {
			u8 profile_id = buf[2 * i];
			u8 revision = buf[2 * i + 1];
			if (profile_id == 0) {
				printf(" No Application Profiles\n");
			} else {
				switch (profile_id) {
				case 0:
					/* handled above */
					break;
				case 1:
					printf(" Server AC-DC Power Supply");
					break;
				case 2:
					printf(" DC-DC Converters for Microprocessor Power and other Copmuter Applications");
					break;
				case 3:
					printf(" DC-DC Converters for General-Purpose Use");
					break;
				default:
					printf(" (reserved)");
				}
				printf(": rev %d.%d\n", (revision >> 4), (revision & 0xf));
			}
		}
		printf("\n");
	}

	if (pmdev->no_query) {
		printf("Device can't QUERY for supported commands\n");
		return;
	}

	for (op = pmbus_ops; !pmdev->no_query && op->tag; op++)
		query(pmdev, op);
}

/*----------------------------------------------------------------------*/

static void showbits(u16 mask, int i, char *bits[])
{
	int comma = 0;

	while (i-- > 0) {
		if (!(mask & (1 << i)))
			continue;
		printf("%s%s", comma++ ? ", " : "", bits[i] ? : "?");
	}
}

void status_byte(struct pmbus_dev *pmdev, u16 cmd, char *label, char *bits[])
{
	int value;
	int mode;

	mode = checksupport(pmdev, cmd);
	if (mode == 0)
		return;
	value = pmbus_read_byte_data(pmdev->fd, cmd);
	if (value < 0) {
		if (mode == 1)
			printf("  ** Device failed read of STATUS_%s?\n",
					label);
		return;
	}
	printf("  %-21s %02x: ", label, value);
	showbits(value, 8, bits);
	printf("\n");
}

static void show_status_vout(struct pmbus_dev *pmdev)
{
	static char *bits[8] = {
		"VOUT Tracking Error",
		"TOFF_MAX_WARNING",
		"TON_MAX_FAULT",
		"Attempted to exceed VOUT_MAX",
		"Output Undervoltage Fault",
		"Output Undervoltage Warning",
		"Output Overvoltage Warning",
		"Output Overvoltage Fault",
	};

	status_byte(pmdev, PMB_STATUS_VOUT, "VOUT", bits);
}

static void show_status_iout(struct pmbus_dev *pmdev)
{
	static char *bits[8] = {
		"Output Overpower Warning",
		"Output Overpower Fault",
		"In Power Limiting Mode",
		"Current Share Fault",
		"Output Undercurrent Fault",
		"Output Overcurrent Warning",
		"Output Overcurrent and Low Voltage Fault",
		"Output Overcurrent Fault",
	};

	status_byte(pmdev, PMB_STATUS_IOUT, "IOUT", bits);
}

static void show_status_input(struct pmbus_dev *pmdev)
{
	static char *bits[8] = {
		"Input Overpower Warning",
		"Input Overcurrent Warning",
		"Input Overcurrent Fault",
		"Unit Off for Insufficient Input Voltage",
		"Input Undervoltage Fault",
		"Input Undervoltage Warning",
		"Input Overvoltage Warning",
		"Input Overvoltage Fault",
	};

	status_byte(pmdev, PMB_STATUS_INPUT, "INPUT", bits);
}

static void show_status_mfr_specific(struct pmbus_dev *pmdev)
{
	static char *bits[8] = {
		"mfr_status_0",
		"mfr_status_1",
		"mfr_status_2",
		"mfr_status_3",
		"mfr_status_4",
		"mfr_status_5",
		"mfr_status_6",
		"mfr_status_7",
	};

	status_byte(pmdev, PMB_STATUS_MFR_SPECIFIC, "MFR_SPECIFIC", bits);
}

static void show_status_fans(struct pmbus_dev *pmdev)
{
	static char *bits1_2[8] = {
		"airflow warning",
		"airflow fault",
		"fan 2 speed override",
		"fan 1 speed override",
		"fan 2 warning",
		"fan 1 warning",
		"fan 2 fault",
		"fan 1 fault",
	};
	static char *bits3_4[8] = {
		"(reserved)",
		"(reserved)",
		"fan 4 speed override",
		"fan 3 speed override",
		"fan 4 warning",
		"fan 4 warning",
		"fan 3 fault",
		"fan 3 fault",
	};

	status_byte(pmdev, PMB_STATUS_FANS_1_2, "FANS_1_2", bits1_2);
	status_byte(pmdev, PMB_STATUS_FANS_3_4, "FANS_3_4", bits3_4);
}

static void show_status_other(struct pmbus_dev *pmdev)
{
	static char *bits[8] = {
		"(reserved)",
		"Output OR-ing Device Fault",
		"Input B OR-ing Device Fault",
		"Input A OR-ing Device Fault",
		"Input B Fuse or Circuit Breaker Fault",
		"Input A Fuse or Circuit Breaker Fault",
		"(reserved)",
		"(reserved)",
	};

	status_byte(pmdev, PMB_STATUS_OTHER, "OTHER", bits);
}

static void show_status_temperature(struct pmbus_dev *pmdev)
{
	static char *bits[8] = {
		"(reserved)",
		"(reserved)",
		"(reserved)",
		"(reserved)",
		"overtemp warning",
		"overtemp fault",
		"undertemp warning",
		"undertemp fault",
	};

	status_byte(pmdev, PMB_STATUS_TEMPERATURE, "TEMPERATURE", bits);
}

static void show_status_cml(struct pmbus_dev *pmdev)
{
	static char *bits[8] = {
		"other memory/logic fault",
		"other comm fault",
		"(reserved)",
		"processor fault",
		"memory fault",
		"PEC",
		"invalid data",
		"invalid command",
	};

	status_byte(pmdev, PMB_STATUS_CML, "CML", bits);
}

static void pmbus_dev_show_status(struct pmbus_dev *pmdev)
{
	static char *bits[16] = {
		"unspecified",
		"comm/memory/logic",
		"temperature",
		"vin_underflow",
		"iout_overflow",
		"vout_overflow",
		"off",
		"busy",
		"unknown",
		"other",
		"fan",
		"power_good#",
		"mfr",
		"vin",
		"iout",
		"vout",
	};

	int value = -EINVAL;
	int mode;

	/* prefer full status word if it's available */
	mode = checksupport(pmdev, PMB_STATUS_WORD);
	if (mode != 0) {
		value = pmbus_read_word_data(pmdev->fd, PMB_STATUS_WORD);
		if (mode == 1 && value < 0) {
			printf("  ** Device failed read of STATUS_%s?\n",
					"WORD");
			return;
		}
		if (value >= 0)
			printf("Status %04x: ", value);
	}

	/* else use status byte */
	if (value < 0) {
		mode = checksupport(pmdev, PMB_STATUS_BYTE);
		if (mode != 0) {
			value = pmbus_read_byte_data(pmdev->fd,
					PMB_STATUS_BYTE);
			if (value < 0) {
				if (mode == 1)
					printf("  ** Device failed read"
							"of STATUS_%s?\n",
							"BYTE");
				return;
			}
			printf("Status %02x: ", value);
		}
	}

	showbits(value, 16, bits);
	printf("\n");

	if (value & ((1 << 15) | (1 << 5)))
		show_status_vout(pmdev);
	if (value & ((1 << 14) | (1 << 4)))
		show_status_iout(pmdev);
	if (value & ((1 << 13) | (1 << 3)))
		show_status_input(pmdev);
	if (value & (1 << 12))
		show_status_mfr_specific(pmdev);

	if (value & (1 << 10))
		show_status_fans(pmdev);
	if (value & (1 << 9))
		show_status_other(pmdev);

	if (value & (1 << 2))
		show_status_temperature(pmdev);
	if (value & (1 << 1))
		show_status_cml(pmdev);

	printf("\n");
}

static bool vout_mode_is_linear(struct pmbus_dev *pmdev)
{
	if (pmdev->op[PMB_VOUT_MODE] == &unsupported)
		return false;

	if (pmdev->op[PMB_VOUT_MODE]->c[0].R & 0xe0)
		return false;

	return true;
}

static double pmbus_to_vout_format(struct pmbus_dev *pmdev, const int value)
{
	int exponent = pmdev->op[PMB_VOUT_MODE]->c[0].R & 0x1f;
	const int mask = 0xf;
	double result = value;

	if (exponent & 0x10) {
		/* convert from two's complement */
		exponent = -(-(exponent & mask) + (exponent & ~mask));
	}

	if (exponent > 0) {
		for (int i = 0; i < exponent; ++i) {
			result *= 2.0;
		}
	} else {
		for (int i = 0; i < -exponent; ++i) {
			result /= 2.0;
		}
	}
	return result;
}

/*----------------------------------------------------------------------*/

static void pmbus_dev_show_commands(struct pmbus_dev *pmdev)
{
	unsigned		i;
	struct pmbus_cmd_desc	*op;

	printf("Supported Commands:\n");
	for (i = 0; i < 255; i++) {
		char			*format;
		int			direct = 0;

		op = pmdev->op[i];
		if (op == &unsupported || !op)
			continue;

		/* command inputs and outputs */
		switch (op->type) {
		case W0:
			format = "nodata";
			break;
		case RW1:
		case W1:
		case R1:
			format = "u8 (bitmask)";
			break;
		case RW2:
		case R2:
			if (op->flags == FLG_FORMAT_VOUT && vout_mode_is_linear(pmdev)) {
				format = "x16 (VOUT_MODE)";
				break;
			}
			switch ((op->query >> 2) & 7) {
			case 0:
				if (op->units == BITS)
					format = "u16 (bitmask)";
				else
					format = "s16 (LINEAR)";
				break;
			case 3:
				format = "s16 (DIRECT)";
				direct = 1;
				break;
			case 5:
				format = "u16 (VID)";
				break;
			case 6:
				format = "x16 (MFR)";
				break;
			default:
				format = "x16 (UNKNOWN)";
				break;
			}
			break;
		case RWB:
		case RWB14:
			format = "block";
			break;
		case RWP_QUERY:
		case RWP_COEFF:
			format = "process_call";
			break;
		case RWB_APP_PROFILE:
			format = "(Application Profile)";
			break;
		default:
			format = "(UNKNOWN call syntax)";
			break;
		}

		/* Now display it all */
		printf("  %02x %-25s %c%c %s",
			op->cmd, op->tag,
			(op->query & (1 << 5)) ? 'r' : ' ',
			(op->query & (1 << 6)) ? 'w' : ' ',
			format);

		format = units(op);
		if (format == NULL && op->units == STRING)
			format = "ISO 8859/1 string";
		if (format)
			printf(", %s", format);

		printf("\n");

		/* dump coefficients; "always" R, maybe W too */
		if (direct && (op->c[1].valid || op->c[0].valid)) {
			printf("     Coefficients: ");
			if (op->c[1].valid)	/* Read */
				printf("READ b=%d m=%d R=%d",
					op->c[1].b, op->c[1].m, op->c[1].R);
			else
				printf("no READ coefficients?");
			if (op->c[0].valid)	/* Write */
				printf("; WRITE b=%d m=%d R=%d",
					op->c[0].b, op->c[0].m, op->c[0].R);
			printf("\n");
		}
	}
}

static void pmbus_dev_show_values(struct pmbus_dev *pmdev)
{
	unsigned		i;
	struct pmbus_cmd_desc	*op;

	printf("Attribute Values:\n");
	for (i = 0; i < 255; i++) {
		int		value;
		const char	*name;

		op = pmdev->op[i];
		if (op == &unsupported || !op)
			continue;
		if (op->flags & (FLG_SHOW_P1|FLG_STATUS))
			continue;

		name = op->tag;
		if (strncmp(name, "read_", 5) == 0)
			name += 5;

		/* read and display values, where that's meaningful */
		switch (op->type) {
		case W0:
		case W1:
		case RWP_QUERY:
		case RWP_COEFF:
		default:
			/* write-only or magic */
			continue;
		case RW1:
		case R1:
			value = pmbus_read_byte_data(pmdev->fd, op->cmd);
			if (value < 0) {
				/* FIXME display a diagnostic */
				continue;
			}
			printf("  %-21s %02x: ", name, value);
/* FIXME need per-op (bitmask) decoders... */
			printf("(BITMAP)");
			printf("\n");
			continue;
		case RW2:
		case R2:
			value = pmbus_read_word_data(pmdev->fd, op->cmd);
			if (value < 0) {
				/* FIXME display a diagnostic */
				continue;
			}
			printf("  %-21s %04x: ", name, value);
/* FIXME need decoders */
			if (op->flags == FLG_FORMAT_VOUT && vout_mode_is_linear(pmdev)) {
				printf("%g", pmbus_to_vout_format(pmdev, value));
				break;
			}
			switch ((op->query >> 2) & 7) {
			case 0:
				if (op->units == BITS) {
					printf("(BITMAP)");
				} else {
					double	d = value & 0x03ff;

					/* LINEAR encoding:
					 *  - 11 LSBs are signed mantissa
					 *  - 5 LSBs are signed exponent
					 */
					if (value & 0x0400)
						d = -d;
					if (value & 0x8000)
						d /= 1 <<
							(0x0f + 1 - (value >> 11) & 0x0f);
					else if (value & 0x7100)
						d *= 1 << (value >> 11);
					printf("%g", d);
				}
				break;
			case 1:
				/* 16-bit unsigned */
				printf("%d", value);
				break;
			case 3: {
				double	d;
				int r;

				/* DIRECT encoding:
				 *  X = ((value * (10 ^ -R)) - b) / m
				 */
				d = (s16) value;

				/* ideally:
				 *   d *= exp10((double)-op->c[1].R);
				 * but that, or pow(), can be unavailable
				 */
				r = op->c[1].R;
				if (r < 0) {
					do {
						d *= 10.0;
						r++;
					} while (r < 0);
				} else if (r > 0) {
					do {
						d /= 10.0;
						r--;
					} while (r > 0);
				}
				d -= (double)op->c[1].b;
				d /= (double)op->c[1].m;
				printf("%g", d);
				}
				break;
			case 4:
				/* 8-bit unsigned */
				printf("%d", value & 0xff);
				break;
			case 5:
				printf("u16 (VID)");
				break;
			case 6:
				printf("manufacturer specific");
				break;
			default:
				printf("unknown format");
				break;
			}
			break;
#if 0
//	RWB,		/* read/write block (up to 255 bytes) */
//	RWB14,		/* read/write block (of 14 bytes) */
		case RWB:
		case RWB14:
			format = "block";
#endif
			break;
		}

		name = units(op);
		if (name)
			printf(" %s", name);
		printf("\n");
	}
	printf("\n");
}

static void pmbus_dev_show(struct pmbus_dev *pmdev, bool values, bool cmds)
{
	pmbus_dev_show_p1(pmdev);
	if (values) {
		pmbus_dev_show_status(pmdev);
		pmbus_dev_show_values(pmdev);
	}
	if (cmds)
		pmbus_dev_show_commands(pmdev);
}

/*----------------------------------------------------------------------*/

static void pmbus_clear_fault(struct pmbus_dev *pmdev)
{
	/* if we know we can't clear faults, don't try */
	if (checksupport(pmdev, PMB_CLEAR_FAULT) != 0)
		(void) smbus_write_byte(pmdev->fd, PMB_CLEAR_FAULT);
}

/*----------------------------------------------------------------------*/

static int pmbus_dev_scan(struct pmbus_dev *pmdev)
{
	int			status;

	/* SMBus (hence PMBus) devices must always ack their addresses.  */
	if (pmdev->funcs & I2C_FUNC_SMBUS_QUICK) {
		status = pmbus_quick(pmdev->fd);
		if (status < 0) {
			fprintf(stderr, "No device present? error %d\n",
					status);
			return status;
		}
	}

	/* QUERY lets us see what operations this device supports, which is
	 * the most interesting information available without product docs.
	 *
	 * It also lets us avoid making calls we know will fail, which is
	 * polite since such failures will trigger host notification or
	 * SMBALERT# on some devices.  If we can't query, we'll just hope
	 * those mechanisms aren't in use.
	 */
	checksupport(pmdev, PMB_QUERY);

	if (checksupport(pmdev, PMB_CAPABILITY) != 0) {
		status = pmbus_read_byte_data(pmdev->fd, PMB_CAPABILITY);
		if (status < 0) {
			if (verbose)
				fprintf(stderr, "No PMBus capability support; "
					"assuming no PEC, etc\n");
		} else {
			pmdev->capability = status;

			/* enable PEC if the device supports it */
			if ((status & (1 << 7)) && enable_pec) {
				if (ioctl(pmdev->fd, I2C_PEC, 1) < 0)
					fprintf(stderr, "couldn't "
						"enable PEC\n");
				else
					pmdev->use_pec = 1;
			}
		}
	}

	/* PMBus 1.0 has PMBUS_REVISION too; it's not new to PMBus 1.1 */
	if (checksupport(pmdev, PMB_PMBUS_REVISION) != 0) {
		status = pmbus_read_byte_data(pmdev->fd, PMB_PMBUS_REVISION);
		if (status < 0) {
			if (verbose)
				fprintf(stderr, "No PMBUS_REVISION support; "
					"assuming 1.0\n");
		} else
			pmdev->revision = status;
	}

	/*
	 * We may not find out anything about the device in the code above;
	 * PMBUS 1.1 conformant devices only need to implement ONE command
	 * that's not manufacturer-specific.
	 *
	 * REVISIT support part definition files, probably using XML as a
	 * relatively neutral syntax that's easily extended and manipulated,
	 * and which doesn't need Yet Another Lowlevel Parser.
	 *
	 *  - They should be able to represent all the data we can query
	 *    from fully dynamic PMBus 1.1 devices.
	 *
	 *  - And support lookup by basic product ID strings, for PMBus 1.0
	 *    devices that may expose little more than those strings and
	 *    some sensor/control attributes.
	 *
	 *  - Provide a "dump" option, to generate those files for devices
	 *    which do happen to be fully query-able.
	 *
	 *  - Allow cross-validation (file against device, to find bugs),
	 *    and merging (file adds extra data, e.g. coefficients from
	 *    docs rather than by protocol requests).
	 *
	 *  - File should be able to describe things like manufacturer
	 *    specific commands, format of user data, accuracy, how the
	 *    various controls and sensors work in the local environment,
	 *    what "pages" exist (and what each one does), and so on.
	 *
	 * Doing all that usefully depends on having a variety of PMBus
	 * based products running Linux.  At this writing there aren't
	 * very many of them, although some recent Dell rackmount servers
	 * are described as having PMBus support.
	 */

	return 0;
}

/*----------------------------------------------------------------------*/

static const unsigned long i2c_func_pmbus_min
		= I2C_FUNC_SMBUS_BYTE_DATA
		| I2C_FUNC_SMBUS_WORD_DATA
		| I2C_FUNC_SMBUS_PROC_CALL;

int main(int argc, char **argv)
{
	int			c;
	struct pmbus_dev	dev;
	char			*adapter = "/dev/i2c-0";
	char			*addr_tail;
	int			addr;
	bool			clear = false;
	bool			force = false;
	bool			list = false;
	bool			show = false;
	u8			mfr_cmd = 0;
	char			*page_str = NULL;
	int			page = -1;

	while ((c = getopt(argc, argv, "b:Cfg:lpsv"
#ifdef HACK
			"m:"
#endif
			)) != EOF) {
		switch (c) {
		case 'b':
			adapter = optarg;
			continue;
		case 'C':
			clear = true;
			continue;
		case 'f':
			force = true;
			continue;
		case 'g':
			page_str = optarg;
			continue;
		case 'l':
			list = true;
			continue;
#ifdef HACK
		case 'm':
			c = atoi(optarg);
			if (c < 0 || c > 45) {
				fprintf(stderr, "mfr_specific_%s ??\n",
					optarg);
				goto usage;
			}
			mfr_cmd = PMB_MFR_SPECIFIC(c);
			continue;
#endif
		case 'p':
			enable_pec = 1;
			continue;
		case 's':
			show = true;
			continue;
		case 'v':
			verbose++;
			continue;
		case '?':
		default:
			goto usage;
		}
	}

	if (optind == argc || *argv[optind] == '\0') {
		fprintf(stderr, "missing device address\n");
		goto usage;
	}
	if (optind != (argc - 1)) {
		fprintf(stderr, "too many arguments\n");
		goto usage;
	}
	addr = (int) strtol(argv[optind], &addr_tail, 0);
	if (*addr_tail || addr < 0) {
		fprintf(stderr, "'%s' is not a device address\n", optarg);
		goto usage;
	}

	/* SMBUS 2.0 table 4 lists reserved addresses */
	if (addr < 0x09 || addr > 0x77 || addr == 0x0c || addr == 0x28
				|| addr == 0x37 || addr == 0x61) {
		fprintf(stderr, "%#02x' is a reserved device address\n",
				addr);
		goto usage;
	}

	if (page_str) {
		char *end;
		page = (int)strtol(page_str, &end, 0);
		if (*end || page < 0 || page > 0xff) {
			fprintf(stderr, "'%s' is not a valid PAGE number\n", page_str);
			goto usage;
		}
	}

	/*
	 * Set up a handle for the specified device on its bus.
	 */
	memset(&dev, 0, sizeof dev);
	dev.fd = open(adapter, O_RDWR);
	if (dev.fd < 0) {
		perror(adapter);
		fprintf(stderr, "Couldn't connect to I2C bus %s\n", adapter);
		return 1;
	}
	dev.bus = adapter;

	c = ioctl(dev.fd, I2C_FUNCS, &dev.funcs);
	if (c < 0) {
		perror(adapter);
		fprintf(stderr, "%s: Couldn't get funcs\n", adapter);
		return 1;
	}

	/* Trying for portability here.  We want to support all core PMBus
	 * features.  Minimal SMBus support is almost good enough ... except
	 * for block read/write and block proc calls.  So we insist on I2C
	 * where the SMBus support is weak, and if it's available we also use
	 * it to cope with the annoying "refuse to do 33+ byte blocks" limit.
	 *
	 * NOTE: WRITE_BLOCK isn't currently used -- or required -- so the
	 * pmbus_block_write() method might fail on some systems.  If that
	 * matters in your usage, add another test ...
	 */
	if ((dev.funcs & i2c_func_pmbus_min) != i2c_func_pmbus_min
			|| !(dev.funcs & (I2C_FUNC_SMBUS_READ_BLOCK_DATA
						| I2C_FUNC_I2C))
			|| !(dev.funcs & (I2C_FUNC_SMBUS_BLOCK_PROC_CALL
						| I2C_FUNC_I2C))
			) {
		fprintf(stderr, "%s: Funcs don't support PMBus\n", adapter);
		return 1;
	}

	/* some adapter drivers don't support PEC */
	if (!(dev.funcs & I2C_FUNC_SMBUS_PEC) && enable_pec) {
		fprintf(stderr, "%s: No PEC support\n", adapter);
		enable_pec = 0;
	}

	c = ioctl(dev.fd, force ? I2C_SLAVE_FORCE : I2C_SLAVE, addr);
	if (c < 0) {
		perror(adapter);
		fprintf(stderr, "Couldn't %sattach to device %#02x\n",
				force ? "force " : "", addr);
		return 1;
	}
	dev.addr = addr;

	if (pmbus_dev_scan(&dev) < 0)
		return 1;

	if (page != -1) {
		c = pmbus_write_byte_data(dev.fd, 0x00, page);
		if (c < 0) {
			fprintf(stderr, "PAGE command failed: %s\n", strerror(c));
			return 1;
		}
	}

	if (show || list)
		pmbus_dev_show(&dev, show, list);

	if (clear)
		pmbus_clear_fault(&dev);

#ifdef HACK
	if (mfr_cmd && checksupport(&dev, mfr_cmd) == 0) {
		printf("Unsuppported mfr_specific command: %#02x\n", mfr_cmd);
		mfr_cmd = 0;
	}

	if (mfr_cmd) {
		if (verbose)
			printf("Issuing mfr_specific command, %#02x...\n",
					mfr_cmd);
		/* NOTE: manufacturer commands can be of arbitrary syntax;
		 * the hack here is that we "know" it's write-only, no-data.
		 */
		c = smbus_write_byte(dev.fd, mfr_cmd);
		if (c < 0)
			fprintf(stderr, "Error %d on mfr cmd %#02x\n",
					c, mfr_cmd);
	}
#endif

	return 0;

usage:
	fprintf(stderr,
		"Usage: %s [options] addr\n"
		"  SMBus address may be in hex, decimal, or octal.\n"
		"  Valid addresses include 0x09-0x77, with exceptions\n"
		"\n"
		"Options include:\n"
		"  -b /dev/i2c-X    specify I2C bus adapter for bus X\n"
		"                   (default bus is i2c-0)\n"
		"  -C               clear all status flags\n"
		"  -f               bypass 'address in use' checks\n"
		"                   (needed with new-style I2C systems)\n"
		"  -g 0x01          specify PAGE number to use\n"
		"  -l               list device capabilities\n"
#ifdef HACK
		"  -m NN            issue no-param mfr_specific_NN\n"
#endif
		"  -p               enable PEC, if the device supports it\n"
		"  -s               show device status and attribute values\n"
		"  -v               be more verbose\n"
		, argv[0]);
	return 1;
}
