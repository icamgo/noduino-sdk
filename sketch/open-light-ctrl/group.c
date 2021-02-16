/*
 *  Copyright (c) 2015 - 2025 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/
#include "user_config.h"

#if 0
irom void esp_now_rx_cb(uint8_t * macaddr, uint8_t * data, uint8_t length)
{
	DEBUG("ESPNow receive\r\n");
}

irom void esp_now_tx_cb(uint8_t * mac_addr, uint8_t status)
{
	DEBUG("ESPNow send\r\n");
}

irom void espnow_stop()
{
	esp_now_unregister_send_cb();
	esp_now_unregister_recv_cb();
	esp_now_deinit();
	DEBUG("ESPNow delete OK\r\n");
}

irom void espnow_start()
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
		INFO("ESPNow create OK\r\n");
		if (0 == esp_now_set_self_role(ESP_NOW_ROLE_SLAVE)) {
			INFO("ESPNow set role OK\r\n");
			esp_now_register_recv_cb(esp_now_rx_cb);
			esp_now_register_send_cb(esp_now_tx_cb);
			esp_now_set_kok(esp_now_key, sizeof(esp_now_key) / sizeof(uint8_t));

			uint8_t channel = wifi_get_channel();
			if (0 == esp_now_add_peer(esp_now_controller_mac,
					     (uint8_t) ESP_NOW_ROLE_CONTROLLER,
					     channel, esp_now_key,
					     sizeof(esp_now_key) / sizeof(uint8_t))) {
				INFO("ESPNow add peer OK\r\n");
			} else {
				INFO("ESPNow add peer Failed\r\n");
			}
		} else {
			INFO("ESPNow set role Failed\r\n");
		}
	} else {
		INFO("ESPNow create Failed\r\n");
	}
}
#endif
