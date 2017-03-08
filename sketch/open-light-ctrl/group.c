#include "user_config.h"

irom void esp_now_receive_callback(uint8_t * macaddr, uint8_t * data, uint8_t length)
{
	INFO("%s: ESPNow receive\r\n", __func__);
}

irom void esp_now_send_callback(uint8_t * mac_addr, uint8_t status)
{
	INFO("%s: ESPNow send\r\n", __func__);
}

irom void espnow_delete(void)
{
	esp_now_unregister_send_cb();
	esp_now_unregister_recv_cb();
	esp_now_deinit();
	INFO("%s: ESPNow delete OK\r\n", __func__);
}

irom void espnow_create(void)
{
	int32_t result;
	uint8_t esp_now_key[16] = {
		0x80, 0x01, 0x52, 0x09, 0xee, 0x5b, 0xed, 0x5f,
		0x58, 0x93, 0x62, 0xfa, 0xf0, 0x70, 0xd3, 0x03,
	};

	uint8_t esp_now_controller_mac[6] = {
		0x00, 0x00, 0x00, 0x11, 0x22, 0x33,
	};

	if (0 == esp_now_init()) {
		INFO("%s: ESPNow create OK\r\n", __func__);
		if (0 == esp_now_set_self_role(ESP_NOW_ROLE_SLAVE)) {
			INFO("%s: ESPNow set role OK\r\n",
					 __func__);
			esp_now_register_recv_cb(esp_now_receive_callback);
			esp_now_register_send_cb(esp_now_send_callback);
			esp_now_set_kok(esp_now_key,
					sizeof(esp_now_key) / sizeof(uint8_t));
			uint8_t channel = wifi_get_channel();
			if (0 ==
			    esp_now_add_peer(esp_now_controller_mac,
					     (uint8_t) ESP_NOW_ROLE_CONTROLLER,
					     channel, esp_now_key,
					     sizeof(esp_now_key) /
					     sizeof(uint8_t))) {
				INFO("%s: ESPNow add peer OK\r\n",
						 __func__);
			} else {
				INFO
				    ("%s: ESPNow add peer Failed\r\n",
				     __func__);
			}
		} else {
			INFO("%s: ESPNow set role Failed\r\n",
					 __func__);
		}
	} else {
		INFO("%s: ESPNow create Failed\r\n", __func__);
	}
}
