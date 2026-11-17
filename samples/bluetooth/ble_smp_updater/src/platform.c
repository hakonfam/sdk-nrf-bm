/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* The platform on sdk-nrf-bm, without its SoftDevice handler library. A vector table
 * in RAM forwards the stack's exceptions to the stack's own vector table, and the
 * stack's SoC events are polled. NVM writes go through sd_flash_write(), which owns
 * RRAMC while the stack is enabled.
 */

#include "platform.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/reboot.h>

#include <psa/crypto.h>

#include <nrf_error.h>
#include <nrf_sd_isr.h>
#include <nrf_soc.h>

#include <nrfx.h>

#define RAND_SEED_LEN 32U
#define WORD_LEN      4U
/* The bytes one sd_flash_write() call writes at most. */
#define STAGE_LEN     256U

#define SVC_PRIORITY 4U

#define STACK_VECTORS_ADDR ((uint32_t)DT_REG_ADDR(DT_NODELABEL(softdevice_partition)))

#define EXCEPTION(irqn)	   ((uint16_t)(16 + (int32_t)(irqn)))
#define VECTOR_COUNT	   (16U + (uint32_t)CONFIG_NUM_IRQS)
/* VTOR selects a vector by OR, so the table is aligned to its size rounded up to a
 * power of two. */
#define VECTOR_TABLE_ALIGN 2048U

_Static_assert((STAGE_LEN % WORD_LEN) == 0U, "the stage holds whole words");
_Static_assert((PLATFORM_SLOT_ADDR % WORD_LEN) == 0U, "the update slot is word aligned");
_Static_assert((VECTOR_COUNT * sizeof(uint32_t)) <= VECTOR_TABLE_ALIGN,
	       "the vector table outgrows its alignment");
_Static_assert((VECTOR_COUNT * sizeof(uint32_t)) > (VECTOR_TABLE_ALIGN / 2U),
	       "the vector table alignment is larger than it needs to be");

enum flash_state {
	FLASH_IDLE,
	FLASH_BUSY,
	FLASH_FAILED,
};

struct stack_vector {
	uint16_t exception;
	uint16_t offset;
};

/* The exceptions the BLE stack serves, and where its vector table holds each handler. */
static const struct stack_vector stack_vectors[] = {
	{EXCEPTION(HardFault_IRQn), NRF_SD_ISR_OFFSET_HARDFAULT},
	{EXCEPTION(SVCall_IRQn), NRF_SD_ISR_OFFSET_SVC},
	{EXCEPTION(SWI00_IRQn), NRF_SD_ISR_OFFSET_SWI00},
	{EXCEPTION(AAR00_CCM00_IRQn), NRF_SD_ISR_OFFSET_AAR00_CCM00},
	{EXCEPTION(ECB00_IRQn), NRF_SD_ISR_OFFSET_ECB00},
	{EXCEPTION(TIMER10_IRQn), NRF_SD_ISR_OFFSET_TIMER10},
	{EXCEPTION(RADIO_0_IRQn), NRF_SD_ISR_OFFSET_RADIO_0},
	{EXCEPTION(GRTC_3_IRQn), NRF_SD_ISR_OFFSET_GRTC_3},
	{EXCEPTION(CLOCK_POWER_IRQn), NRF_SD_ISR_OFFSET_CLOCK_POWER},
};

static uintptr_t ram_vectors[VECTOR_COUNT] __aligned(VECTOR_TABLE_ALIGN);

static volatile enum flash_state flash_state;

/* sd_flash_write() reads the source until the write completes. */
static uint32_t stage[STAGE_LEN / WORD_LEN];

void platform_halt(void)
{
	for (;;) {
		__WFE();
	}
}

void platform_reset(void)
{
	sys_reboot(SYS_REBOOT_COLD);
	platform_halt();
}

void platform_secret_clear(uint8_t *buf, size_t len)
{
	volatile uint8_t *const p = buf;

	for (size_t i = 0U; i < len; i++) {
		p[i] = 0U;
	}
}

/* Wakes the main loop, which serves the stack's events in thread mode. */
static void stack_event_isr(void)
{
}

void platform_stack_attach(void)
{
	const uint32_t *const stack_table = (const uint32_t *)(uintptr_t)STACK_VECTORS_ADDR;
	void (*const stack_reset)(void) =
		(void (*)(void))stack_table[NRF_SD_ISR_OFFSET_RESET / sizeof(uint32_t)];

	memcpy(ram_vectors, (const void *)(uintptr_t)SCB->VTOR, sizeof(ram_vectors));
	for (size_t i = 0U; i < ARRAY_SIZE(stack_vectors); i++) {
		ram_vectors[stack_vectors[i].exception] =
			stack_table[stack_vectors[i].offset / sizeof(uint32_t)];
	}
	ram_vectors[EXCEPTION(SD_EVT_IRQn)] = (uintptr_t)stack_event_isr;

	SCB->VTOR = (uint32_t)(uintptr_t)ram_vectors;
	__DSB();
	__ISB();

	stack_reset();
	NVIC_SetPriority(SVCall_IRQn, SVC_PRIORITY);
}

bool platform_stack_seed(void)
{
	uint8_t seed[RAND_SEED_LEN];
	const bool ok = (psa_generate_random(seed, sizeof(seed)) == PSA_SUCCESS) &&
			(sd_rand_seed_set(seed) == NRF_SUCCESS);

	platform_secret_clear(seed, sizeof(seed));
	return ok;
}

bool platform_soc_events_serve(void)
{
	uint32_t evt = 0U;

	while (sd_evt_get(&evt) == NRF_SUCCESS) {
		switch (evt) {
		case NRF_EVT_RAND_SEED_REQUEST:
			if (!platform_stack_seed()) {
				return false;
			}
			break;
		case NRF_EVT_FLASH_OPERATION_SUCCESS:
			flash_state = FLASH_IDLE;
			break;
		case NRF_EVT_FLASH_OPERATION_ERROR:
			flash_state = FLASH_FAILED;
			break;
		default:
			/* An event that needs no reply. */
			break;
		}
	}
	return true;
}

/* Writes the whole words at @p word_addr from the stage, and waits for the stack to
 * report the write done. */
static bool stage_write(uint32_t word_addr, uint32_t word_count)
{
	flash_state = FLASH_BUSY;
	if (sd_flash_write((uint32_t *)(uintptr_t)word_addr, stage, word_count) != NRF_SUCCESS) {
		flash_state = FLASH_IDLE;
		return false;
	}
	while (flash_state == FLASH_BUSY) {
		if (!platform_soc_events_serve()) {
			return false;
		}
		if (flash_state == FLASH_BUSY) {
			__WFE();
		}
	}
	return flash_state == FLASH_IDLE;
}

bool platform_nvm_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
	if ((addr < PLATFORM_SLOT_ADDR) || (len > PLATFORM_SLOT_SIZE) ||
	    ((addr - PLATFORM_SLOT_ADDR) > (PLATFORM_SLOT_SIZE - len))) {
		return false;
	}

	uint32_t word_addr = addr - (addr % WORD_LEN);
	uint32_t head = addr - word_addr;

	while (len > 0U) {
		uint32_t take = STAGE_LEN - head;

		if (take > len) {
			take = len;
		}

		const uint32_t span = head + take;
		const uint32_t word_count = (span + WORD_LEN - 1U) / WORD_LEN;

		/* RRAM needs no erase: the bytes around the range are written back as
		 * they are. */
		memcpy(stage, (const void *)(uintptr_t)word_addr, word_count * WORD_LEN);
		memcpy((uint8_t *)stage + head, data, take);
		if (!stage_write(word_addr, word_count)) {
			return false;
		}

		word_addr += word_count * WORD_LEN;
		data += take;
		len -= take;
		head = 0U;
	}
	return true;
}
