/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
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

#include <SPI.h>
#include <Ethernet.h>
//#include <SPIFlash.h>
#include <EEPROM.h>
#include <DES.h>
#include <TextFinder.h>

#include <avr/wdt.h>

#include "radio.h"

#define		DEBUG_SERVER
#define		DEBUG
#define		DEBUG_EEPROM
#define		DEBUG_TOKEN
#define		NETCORE_ROUTER_FIXUP
//#define		NO_DHCP_ROBUST
//#define		WITH_FLASH

#define		TRYNUM	1
#define		FW_VER	"1.0.0"

byte mac[7];
byte dkey[9];
byte uuid[25];
byte token[25];

IPAddress ip(192,168,1,3);
IPAddress gw_ip5(10,0,0,2);
IPAddress ip9(10,0,0,254);

#ifdef CONFIG_V0
char cos_serv[] = "192.168.1.97";
#else
char cos_serv[] = "api.noduino.org";
#endif

EthernetClient client;

TextFinder finder(client);

//String body = "";

// same sec message interval
const uint16_t msg_group_len = 2000;

unsigned long last_post_time = 0;

int push_data(char *data, char serv[]);
byte wan_ok();
#ifdef NO_DHCP_ROBUST
byte try_static_ip();
#endif

#ifdef WITH_FLASH
SPIFlash flash(5, 0xEF40);
byte buf[8];
#endif

// get the uuid, key and mac from eeprom
void init_devid()
{
	int i;
	// read the uuid
	for (i=0; i<19; i++) {
		uuid[i] = EEPROM.read(0x10 + i);
	}
	uuid[19] = 0x0;

	// read the key
	for (i=0; i<8; i++) {
		dkey[i] = EEPROM.read(0x23 + i);
	}
	dkey[8] = 0;

	// read the mac
	for (i=0; i<6; i++) {
		mac[i] = EEPROM.read(0x30 + i);
	}
}

DES des;

void gen_token(byte *out, byte *uuid, byte *key)
{
	unsigned long seed = millis();
	uuid[20] = seed & 0xff;
	uuid[21] = (seed >> 8) & 0xff;
	uuid[22] = (seed >> 16) & 0xff;
	uuid[23] = (seed >> 24) & 0xff;
#ifdef DEBUG_TOKEN
	Serial.printf("seed = 0x%08X\n", seed);
#endif
	des.encrypt(out+16, uuid, key);
	des.encrypt(out+8, uuid+8, key);
	des.encrypt(out, uuid+16, key);
}

// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
void setup_wdt(int i)
{
	byte bb;

	if (i > 9) i = 9;

	bb = i & 7;

	if (i > 7) bb |= (1 << 5);
	bb |= (1<<WDCE);

	noInterrupts();
	MCUSR &= ~(1<<WDRF);
	// start timed sequence
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	// set new watchdog timeout value
	WDTCSR = bb;
	WDTCSR |= _BV(WDIE);
	interrupts();
}

uint8_t wdt_off()
{
	noInterrupts();
	wdt_reset();

	// clear WDRF in MCUSR
	MCUSR &= ~(1<<WDRF);
	// write logical one to WDCE and WDE
	// keep old prescaler setting to prevent unintentional time-out
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	// turn off wdt
	WDTCSR = 0x00;

	interrupts();
	return 1;
}

void chip_reset()
{
	//wdt_enable(WDTO_250MS);
	setup_wdt(3);
	while(1);
}

void setup() {
	// disable the watchdog
	wdt_disable();

#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("Power On now!");
#endif

	randomSeed(analogRead(14));
	radio_setup();
#if 1

#ifdef WITH_FLASH
	// Pull up pin D5, this is CS signal (active LOW) of SPI flash
	pinMode(5, OUTPUT);
	digitalWrite(5, HIGH);
#endif

#ifdef WITH_FLASH
	// Init SPI flash
	if (flash.initialize()) {
		Serial.println("SPI Flash init OK");

		for(int i=0; i<8; i++) {
			Serial.print(flash.readByte(i), HEX);
		}
		Serial.println();

		byte xx[4] = "xxx";
		flash.blockErase4K(0);
		while(flash.busy());
		flash.writeBytes(0, xx, 3);
		while(flash.busy());
		Serial.println("SPI Write Done");

	} else {
		Serial.println("SPI Flash Init failed");
	}

#ifdef DEBUG
	flash.readBytes(0, buf, 8);
	for(int i=0; i<8; i++) {
		Serial.print(buf[i], HEX);
	}
	Serial.println();
#endif
#endif // FLASH

	init_devid();

#ifdef DEBUG_EEPROM
	Serial.printf("uuid = %s\n", (char *)uuid);
	Serial.printf("dkey = %s\n", (char *)dkey);
	Serial.print("mac = ");
	for(int i=0; i<6; i++) {
		Serial.printf("%02X ", mac[i]);
	}
	Serial.println();
#endif

	//body.reserve(100);

	setup_wdt(9);
	// start the Ethernet connection:
	if (Ethernet.begin(mac) == 0) {
#ifdef DEBUG
		Serial.println("Failed to configure Ethernet using DHCP");
#endif
		// DHCP failed, so try a fixed IP
		// If static ip failed, it's network issue
		// waitting for the network is OK
#ifdef NO_DHCP_ROBUST
		if (try_static_ip() == 0) {
#endif
			chip_reset();
#ifdef NO_DHCP_ROBUST
		}
#endif
	} else {
#ifdef DEBUG
		Serial.println("DHCP is OK");
#endif
	}

	// setup ip must be in 8s otherwise chip would be reset
	wdt_reset();

#ifdef DEBUG
	Serial.print("My address:");
	Serial.println(Ethernet.localIP());
#endif

	// push a message to say "I'm online"
#ifdef NETCORE_ROUTER_FIXUP
	if(wan_ok() == 0) {
#ifdef DEBUG
		Serial.println("wan is offline");
#endif
	}
#endif

#if 0
	body = "{";
	push_data(0x0, cos_serv);
#endif

	wdt_reset();

#endif
}

#ifdef NO_DHCP_ROBUST
byte try_static_ip()
{
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 254;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 111;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	// Try 192.168.0.111/254/3
	ip[2] = 0;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 254;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	ip[3] = 3;
	Ethernet.begin(mac, ip);
	if (wan_ok())
		return 1;

	// Try 192.168.10.3/111/254
	ip[2] = 10;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 111;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 254;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}

	// Try 192.168.18.3/111/254
	ip[2] = 18;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 3;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}
	ip[3] = 111;
	Ethernet.begin(mac, ip);
	if (wan_ok()) {
		return 1;
	}

	// Try 10.0.0.254
	Ethernet.begin(mac, ip9, gw_ip5, gw_ip5);
	if (wan_ok()) {
		return 1;
	}

	return 0;
}
#endif

byte wan_ok()
{
	wdt_off();
	if (client.connect(cos_serv, 80)) {
		client.stop();
		setup_wdt(9);
		return 1;
	} else {
		client.stop();
		setup_wdt(9);
		return 0;
	}
}

uint64_t old_val = 0;

char pbuf[100];

void loop() {

	int i = 0, e = 1;

	int try_num = 0;
	wdt_off();

	e = radio_available(pbuf);

	if (e) {

		//Serial.println("Radio data available");

		// make sure push the data success
		try_num = 0;
#ifdef NETCORE_ROUTER_FIXUP
		if(wan_ok() == 0) {
#ifdef DEBUG
			Serial.println("wan is offline");
#endif
			chip_reset();
		}
		wdt_reset();
#endif
#if 1
		while (push_data(pbuf, cos_serv) == -1) {
			wdt_reset();
			try_num++;
#ifdef DEBUG
			Serial.print("pushed data failed. Try num:");
			Serial.println(try_num);
#endif
			if (try_num >= TRYNUM) break;
			delay(1200);		// delay 1.2s
#ifdef DEBUG
			Serial.println("try next");
#endif
		}
#endif
	}
}


#if 1
/*
 * Pushed Request:
 *
 * POST /dev/quarkx HTTP/1.0
 * Accept: * / *
 * mac: 0008DCF46025
 * Content-Length: 43
 * Content-Type: text/html
 * Connection: close
 *
 * \!U/3.66/P/3727.87/devid/248/snr/7/rssi/-47
 *
 *
*/
int push_data(char *pbuf, char serv[]) {

#ifdef	DEBUG
	//Serial.println("Try push data");
#endif

	// if there's a successful connection:
	if (!client.connected() && wdt_off() && client.connect(serv, 80)) {
		setup_wdt(9);
#ifdef	DEBUG
		//Serial.println("Connecting to Cloud ...");
#endif

		// Send the HTTP PUT request
#ifdef CONFIG_V0
		client.println("POST /dev/t2x HTTP/1.0");
#else
		gen_token(token, uuid, dkey);

		client.println("POST /dev/quarkx HTTP/1.0");
#endif

		client.println("Accept: */*");

#if 0
		client.print("nodid: ");
		for(int i = 0; i < 19; i++)
			client.printf("%c", uuid[i]);
		client.println();

		client.print("token: ");
		for(int i = 0; i < 24; i++)
			client.printf("%02X", token[i]);
		client.println();
#endif

		client.print("mac: ");
		for(int i = 0; i < 6; i++)
			client.printf("%02X", mac[i]);
		client.println();

		client.print("Content-Length: ");
		// calculate the length of the sensor reading in bytes:
		client.println(strlen(pbuf));

		// last pieces of the HTTP PUT request:
		client.println("Content-Type: text/html");
		client.println("Connection: close");
		client.println();

		// here's the actual content of the PUT request:
		client.println(pbuf);
		client.println();

#ifdef DEBUG
#ifdef DEBUG_SERVER
		Serial.println(pbuf);
#endif
		//Serial.println("Messages pushed.");
#endif
		// note the time that the connection was made or attempted:
		last_post_time = millis();

		wdt_reset();

		if(client.connected()) {

			wdt_off();

			if (finder.findUntil("200 OK", "\n\r")) {
				setup_wdt(9);
			#ifdef DEBUG
				Serial.println("Server Response OK");
			#endif
				client.stop();
				return 0;
			} else {
				setup_wdt(9);
			#ifdef DEBUG
				Serial.println("Server Response Failed!");
				int len = client.available();
				if (len > 0) {
					byte buffer[80];
					if (len > 80) len = 80;
					client.read(buffer, len);
					Serial.write(buffer, len);
				}
			#endif
				client.stop();
				return -1;
			}
		} else {
			#ifdef DEBUG
				Serial.println("Can't get the server response. Client is not Connected!");
			#endif
				client.stop();
				return -1;
		}

	} else {
#ifdef DEBUG
		// if you couldn't make a connection:
		Serial.println("Connection failed");
#endif
		setup_wdt(9);
		client.stop();
		return -1;
	}
}
#endif
