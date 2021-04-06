#ifndef __FLASH_H__
#define __FLASH_H__

#ifdef EFM32HG110F64
#define CFGDATA_BASE     (0x0000FC00UL) /* config data page base address: 64K - 1K */
#elif EFM32ZG110F32
#define CFGDATA_BASE     (0x00007C00UL) /* config data page base address: 32K - 1K */
#elif EFM32GG230F512
#define CFGDATA_BASE     (0x0000FC00UL) /* config data page base address: 64K - 1K */
#endif

uint32_t *cfg_addr = ((uint32_t *) CFGDATA_BASE);

typedef struct cfg_data {
	uint32_t init_flag;
	uint32_t epoch;
	uint32_t tx_count;
	uint64_t paired_did;
} cfg_data_t;

cfg_data_t g_cfg __attribute__((aligned(4)));

void flash_init()
{
	uint32_t *p = (uint32_t *)&g_cfg;

	uint32_t flag = *cfg_addr;

	if (0x55aa == flag) {
		/*
		 * cfg in flash is used
		 * need to init the g_cfg
		*/
		memcpy(p, cfg_addr, sizeof(g_cfg));
	}
}

void flash_update()
{
	if (0x55aa == g_cfg.init_flag) {
		MSC_Init();
		MSC_ErasePage(cfg_addr);

		MSC_WriteWord(cfg_addr, &g_cfg, sizeof(g_cfg));
		MSC_Deinit();
	}
}

#endif
