#ifndef __APP_H__
#define __APP_H__

// FLASH MAP
// 0x79: SAVE1 0x7A: SAVE2 0x7B: FLAG
#define APP_START_SEC   	0x79
#define APP_POWER_PERCENT 	100
#define APP_POWER_LIMIT(x) 	((x) * (APP_POWER_PERCENT) / 100)

typedef struct mcu_status_t {
	uint8 s;
	uint8 r;
	uint8 g;
	uint8 b;
	uint8 w;
} __attribute__((aligned(1), packed)) mcu_status_t;

typedef struct system_status_t {
	uint8 init_flag;
	uint16 start_count;
	uint8 start_continue;
	mcu_status_t mcu_status;
	uint8 packed[3];
} __attribute__((aligned(4), packed)) system_status_t;

typedef enum app_state_t {
	APP_STATE_NORMAL,
	APP_STATE_SMART,
	APP_STATE_UPGRADE,
	APP_STATE_RESTORE,
} app_state_t;

void mjyun_receive(const char * event_name, const char * event_data);
void app_apply_settings(mcu_status_t *pst);
void app_load(void);
void app_save(void);
void app_push_status(mcu_status_t *pst);
void app_check_mcu_save(mcu_status_t *st);
void app_start_check(uint32_t system_start_seconds);
void app_set_smart_effect(uint8_t effect);
void ICACHE_FLASH_ATTR
uart_send(const uint8_t * buffer, uint16_t length);
void ICACHE_FLASH_ATTR
uart_receive(const uint8_t * pdata, uint16_t length);

#endif /* __USER_VIRTUAL_MCU_H__ */
