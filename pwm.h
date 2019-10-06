/*
 * Copyright (C) 2016 Stefan Brüns <stefan.bruens@rwth-aachen.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

/* Set the following three defines to your needs */

#ifndef __pwm_h__
#define __pwm_h__

#ifndef SDK_PWM_PERIOD_COMPAT_MODE
  #define SDK_PWM_PERIOD_COMPAT_MODE 0
#endif
#ifndef PWM_MAX_CHANNELS
  #define PWM_MAX_CHANNELS 8
#endif
#define PWM_DEBUG 0
#define PWM_USE_NMI 0

/*
	https://bbs.espressif.com/viewtopic.php?t=3115
	
	GPIO0: PERIPHS_IO_MUX_GPIO0_U
	GPIO1: PERIPHS_IO_MUX_U0TXD_U
	GPIO2: PERIPHS_IO_MUX_GPIO2_U
	GPIO3: PERIPHS_IO_MUX_U0RXD_U
	GPIO4: PERIPHS_IO_MUX_GPIO4_U
	GPIO5: PERIPHS_IO_MUX_GPIO5_U
	GPIO6: PERIPHS_IO_MUX_SD_CLK_U
	GPIO7: PERIPHS_IO_MUX_SD_DATA0_U
	GPIO8: PERIPHS_IO_MUX_SD_DATA1_U
	GPIO9: PERIPHS_IO_MUX_SD_DATA2_U
	GPIO10: PERIPHS_IO_MUX_SD_DATA3_U
	GPIO11: PERIPHS_IO_MUX_SD_CMD_U
	GPIO12: PERIPHS_IO_MUX_MTDI_U
	GPIO13: PERIPHS_IO_MUX_MTCK_U
	GPIO14: PERIPHS_IO_MUX_MTMS_U
	GPIO15: PERIPHS_IO_MUX_MTDO_U
*/



/* no user servicable parts beyond this point */

#define PWM_MAX_TICKS 0x7fffff
#if SDK_PWM_PERIOD_COMPAT_MODE
#define PWM_PERIOD_TO_TICKS(x) (x * 0.2)
#define PWM_DUTY_TO_TICKS(x) (x * 5)
#define PWM_MAX_DUTY (PWM_MAX_TICKS * 0.2)
#define PWM_MAX_PERIOD (PWM_MAX_TICKS * 5)
#else
#define PWM_PERIOD_TO_TICKS(x) (x)
#define PWM_DUTY_TO_TICKS(x) (x)
#define PWM_MAX_DUTY PWM_MAX_TICKS
#define PWM_MAX_PERIOD PWM_MAX_TICKS
#endif

#include <c_types.h>
#include <pwm.h>
#include <eagle_soc.h>
#include <ets_sys.h>

// from SDK hw_timer.c
#define TIMER1_DIVIDE_BY_16             0x0004
#define TIMER1_ENABLE_TIMER             0x0080

struct pwm_phase {
	uint32_t ticks;    ///< delay until next phase, in 200ns units
	uint16_t on_mask;  ///< GPIO mask to switch on
	uint16_t off_mask; ///< GPIO mask to switch off
};

/* Three sets of PWM phases, the active one, the one used
 * starting with the next cycle, and the one updated
 * by pwm_start. After the update pwm_next_set
 * is set to the last updated set. pwm_current_set is set to
 * pwm_next_set from the interrupt routine during the first
 * pwm phase
 */
typedef struct pwm_phase (pwm_phase_array)[PWM_MAX_CHANNELS + 2];

static struct {
	struct pwm_phase* next_set;
	struct pwm_phase* current_set;
	uint8_t current_phase;
} pwm_state;


// 3-tuples of MUX_REGISTER, MUX_VALUE and GPIO number
typedef uint32_t (pin_info_type)[3];

struct gpio_regs {
	uint32_t out;         /* 0x60000300 */
	uint32_t out_w1ts;    /* 0x60000304 */
	uint32_t out_w1tc;    /* 0x60000308 */
	uint32_t enable;      /* 0x6000030C */
	uint32_t enable_w1ts; /* 0x60000310 */
	uint32_t enable_w1tc; /* 0x60000314 */
	uint32_t in;          /* 0x60000318 */
	uint32_t status;      /* 0x6000031C */
	uint32_t status_w1ts; /* 0x60000320 */
	uint32_t status_w1tc; /* 0x60000324 */
};


struct timer_regs {
	uint32_t frc1_load;   /* 0x60000600 */
	uint32_t frc1_count;  /* 0x60000604 */
	uint32_t frc1_ctrl;   /* 0x60000608 */
	uint32_t frc1_int;    /* 0x6000060C */
	uint8_t  pad[16];
	uint32_t frc2_load;   /* 0x60000620 */
	uint32_t frc2_count;  /* 0x60000624 */
	uint32_t frc2_ctrl;   /* 0x60000628 */
	uint32_t frc2_int;    /* 0x6000062C */
	uint32_t frc2_alarm;  /* 0x60000630 */
};


static void ICACHE_RAM_ATTR
pwm_intr_handler(void);

/**
 * period: initial period (base unit 1us OR 200ns)
 * duty: array of initial duty values, may be NULL, may be freed after pwm_init
 * pwm_channel_num: number of channels to use
 * pin_info_list: array of pin_info
 */
void ICACHE_FLASH_ATTR
pwm_init(uint32_t , uint32_t , uint32_t , uint32_t );

__attribute__ ((noinline))
static uint8_t ICACHE_FLASH_ATTR
_pwm_phases_prep(struct pwm_phase*);

void ICACHE_FLASH_ATTR
pwm_start(void);

void ICACHE_FLASH_ATTR
pwm_set_duty(uint32_t, uint8_t);

uint32_t ICACHE_FLASH_ATTR
pwm_get_duty(uint8_t);

void ICACHE_FLASH_ATTR
pwm_set_period(uint32_t);

uint32_t ICACHE_FLASH_ATTR
pwm_get_period(void);

uint32_t ICACHE_FLASH_ATTR
get_pwm_version(void);

void ICACHE_FLASH_ATTR
set_pwm_debug_en(uint8_t);


#endif