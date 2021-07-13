#include "m5311.h"

char str[BUF_LEN] __attribute__((aligned(4)));
char modem_said[MODEM_LEN] __attribute__((aligned(4)));

void M5311::init(HardwareSerial *serial)
{
	MODEM_SERIAL = serial;
}

bool M5311::wait_modem()
{
	char wait_ok[] = "READY";

	if (expect_rx_str(2000, wait_ok, 5) != "") {

		return true;

	} else {

		return false;
	}
}

bool M5311::init_modem()
{
	INFOLN("Initial Modem");

#if 0
	MODEM_SERIAL->println(F("AT+CFUN=1"));
	MODEM_SERIAL->flush();
	delay(1);
#endif

	MODEM_SERIAL->println(F("AT*CMBAND=8"));
	MODEM_SERIAL->flush();
	delay(1);

#if 1
	MODEM_SERIAL->println(F("AT+SM=LOCK"));
	MODEM_SERIAL->flush();
	delay(1);
#endif

	MODEM_SERIAL->clear_rxbuf();
}

bool M5311::disable_deepsleep()
{
	MODEM_SERIAL->println(F("AT+SM=LOCK_FOREVER"));
	MODEM_SERIAL->clear_rxbuf();

	char wait_ok[] = "OK";

	if (expect_rx_str(2000, wait_ok, 2) != "") {

		return true;

	} else {

		return false;
	}
}

#if 0
bool M5311::set_band()
{
}

void M5311::sw_power_off()
{
	MODEM_SERIAL->println(F("AT+CPOF"));
	MODEM_SERIAL->flush();
	delay(10);
}
#endif

void M5311::enter_deepsleep()
{
	MODEM_SERIAL->println(F("AT+SM=UNLOCK"));
	MODEM_SERIAL->flush();
	delay(1);

	MODEM_SERIAL->println(F("AT+CFUN=0"));
	MODEM_SERIAL->flush();
	delay(1);
}

void M5311::clean_net_cache()
{
	INFOLN("cleanup net cache");

	MODEM_SERIAL->println(F("AT+CFUN=0"));
	MODEM_SERIAL->flush();
	delay(1);

	MODEM_SERIAL->println(F("AT+CLPLMN"));
	MODEM_SERIAL->flush();

	MODEM_SERIAL->clear_rxbuf();
}

bool M5311::reboot()
{
	//MODEM_SERIAL->println("AT+COLDRB");

	MODEM_SERIAL->println(F("AT+CMRB"));
	//MODEM_SERIAL->flush();
	delay(1000);

	char wait_str[] = "REBOOT";

	if (expect_rx_str(3000, wait_str, 6) != "") {
		INFOLN("Reboot done.");
		return true;
	} else {
		INFOLN("Reboot failed.");
		return false;
	}
}

String M5311::get_imei()
{
	MODEM_SERIAL->println(F("AT+CGSN=1"));
	MODEM_SERIAL->flush();

	char wait_str[] = "+CGSN:";
	String re_str;

	re_str = expect_rx_str(1000, wait_str, 6);

	if (re_str != "") {
		return re_str;
	}
	return "";
}

String M5311::get_imsi()
{
	MODEM_SERIAL->println(F("AT+CIMI"));
	MODEM_SERIAL->flush();
	char wait_str[] = "\r\n";
	String re_str;

	re_str = expect_rx_str(1000, wait_str, 2);

	if (re_str != "") {
		return re_str;
	}
	return "";
}

String M5311::get_iccid()
{
	MODEM_SERIAL->println(F("AT+ICCID"));
	MODEM_SERIAL->flush();

	char wait_str[] = "+ICCID: ";
	String re_str;

	re_str = expect_rx_str(1000, wait_str, 8);

	if (re_str != "") {
		return re_str;
	}
	return "";
}

/*
 * 0: Failed
 * 1: OK
 * 2: timeout
*/
int M5311::check_boot()
{
	char wait_str[] = "+IP: ";

	String ret_s = expect_rx_str(10000, wait_str, 5);

	if ( ret_s == "") {
		INFOLN("Boot failed");
		return 0;
	} else if (ret_s == "T") {
		return 2;
	} else {
		INFOLN("Start OK, got the ip.");
		return 1;
	}
}

/*
 * 0: Failed
 * 1: OK
 * 2: timeout
*/
int M5311::check_at_ready()
{
	char wait_str[] = "*ATREADY:";

	String ret_s = expect_rx_str(1200, wait_str, 9);

	if ( ret_s == "") {
		return 0;
	} else if (ret_s == "T") {
		return 2;
	} else {
		return 1;
	}
}

/*
 * 0: Failed
 * 1: OK
 * 2: timeout
*/
int M5311::check_modem_status()
{
	MODEM_SERIAL->println(F("AT"));
	MODEM_SERIAL->flush();

	char wait_ok[] = "OK";

	String ret_s = expect_rx_str(1000, wait_ok, 2);

	if ( ret_s == "") {
		return 0;
	} else if (ret_s == "T") {
		return 2;
	} else {
		return 1;
	}
}

/*
 * 0: Failed
 * 1: OK
 * 2: timeout
*/
int M5311::check_network()
{
	char wait_str[] = "+CGATT: 1";

	INFOLN("check network");

	MODEM_SERIAL->println(F("AT+CGATT?"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

#if 1
	String ret_s = expect_rx_str(1000, wait_str, 9);
#else
	String ret_s = find_rxbuf_str(wait_str, 9);
#endif

	if (ret_s == "") {
		INFOLN("network is not ok");
		return 0;
	} else if (ret_s == "T") {
		return 2;
	} else {
		INFOLN("Regiester network Done");
		return 1;
	}
}

String M5311::check_ipaddr()
{
	char wait_str[] = "+CGDCONT: 1,";
	String re_str;

	MODEM_SERIAL->println(F("AT+CGDCONT?"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

	re_str = expect_rx_str(1000, wait_str, 12);

	if (re_str != "") {
		return re_str;
	}
	return "";
}

String M5311::get_net_time()
{
	char wait_str[] = "+CCLK: ";
	String re_str;

	MODEM_SERIAL->println(F("AT+CCLK?"));
	delay(200);

	re_str = expect_rx_str(1000, wait_str, 7);

	if (re_str != "") {
		/* +CCLK: 21/02/26,06:22:38+32 */
		return re_str;
	}
	return "";
}


String M5311::expect_rx_str(unsigned long period, char exp_str[], int len_check)
{
	unsigned long cur_t = millis();
	unsigned long start_t = millis();

	bool time_out = 0;
	bool loop_out = 0;

	int i = 0;
	int found_index = 0;

	char c;
	char *x;

	memset(modem_said, 0, MODEM_LEN);
	memset(str, 0, BUF_LEN);

	INFOLN("expect");

	while (!loop_out) {

		if (MODEM_SERIAL->available()) {
			c = MODEM_SERIAL->read();
			modem_said[i++] = c;
		}

		cur_t = millis();

		if (cur_t - start_t > period) {
			time_out = true;
			start_t = cur_t;
			loop_out = true;
		}

		if (i >= MODEM_LEN-1) {
			loop_out = true;
		}
	}

	modem_said[i] = '\0';

	INFOLN(modem_said);
	INFO("len(modem_said) = ");
	INFOLN(strlen(modem_said));

	x = strstr(modem_said, exp_str);
	found_index = x ? x - modem_said : -1;

	if (found_index >= 0) {
		i = 0;

		while (modem_said[found_index + i + len_check] != 0x0D || i == 0) {
			str[i] = modem_said[found_index + i + len_check];
			i++;
		}
		str[i] = '\0';

		return str;

	} else {

		if (time_out) {
			INFOLN("expect timeout");
			return "T";
		} else {

			INFOLN("expect no str found");
			return "";
		}
	}
}

String M5311::find_rxbuf_str(char exp_str[], int len_check)
{
	char *rxbuf = MODEM_SERIAL->get_rxbuf();

	char *x = strnstr(rxbuf, exp_str, 159);

	int found_index = x ? x - rxbuf : -1;

	if (found_index >= 0) {

		int i = 0;

		while (rxbuf[found_index + i + len_check] != 0x0D || i == 0) {
			str[i] = modem_said[found_index + i + len_check];
			i++;
		}
		str[i] = '\0';

		return str;

	} else {

		return "";
	}
}

bool M5311::check_match(char target[], char pattern[], int len_check)
{
	int target_length = strlen(target);
	int count = 0;

	for (int i = 0; i < target_length; i++) {
		// find the 1st char in target
		if (target[i] == pattern[0]) {
			// start at index of target[index] that match[pattern]
			// loop and count the match char
			for (int j = 0; j < len_check; j++) {
				if (target[i] == pattern[j]) {
					count++;
					i++;
				}
			}
		}
	}
	if (count == len_check) {
		return true;
	}
	return false;
}

int M5311::check_match_index(char target[], char pattern[], int len_check)
{
	int target_length = strlen(target);
	int count = 0;
	int i = 0, index = 0;
	for (i = 0; i < target_length; i++) {
		// find the 1st char in target
		if (target[i] == pattern[0]) {
			// start at index of target[index] that match[pattern]
			// loop and count the match char
			for (int j = 0; j < len_check; j++) {
				if (target[i] == pattern[j]) {
					count++;
					index = i;
					i++;
				}
			}	// loop all char in pattern
		}
	}
	if (count == len_check) {
		return index;
	}
	return -1;
}

bool M5311::check_incoming_msg()
{
	char strCmd[BUF_LEN / 2];
	bool found = false;
	int totalSocket = 1;
	String checkIncmd = "AT+NSORF= ,8\r\n";

	for (int j = 0; j < totalSocket; j++) {
		for (int i = 0; i < 14; i++) {
			if (i == 9) {
				MODEM_SERIAL->print(j);
			} else {
				strCmd[i] = checkIncmd[i];
				MODEM_SERIAL->print(strCmd[i]);
			}
		}		// for i
	}			//for j

	char wait_str[] = "+NSONMI:0,8";

	if (expect_rx_str(1000, wait_str, 11) != "") {
		//INFOLN( " There is a message need to read" );
		return true;
	}
	return false;
}

int M5311::check_modem_signal()
{
	int ssi;
	char ssi_str[3];

	MODEM_SERIAL->println("AT+CSQ");

	char wait_str[] = "+CSQ: ";
	String re_str;
	re_str = expect_rx_str(1000, wait_str, 6);

	if (re_str != "") {
		ssi_str[0] = re_str[0];

		if (re_str[1] != 0x2c) {
			// It's not the "," 
			ssi_str[1] = re_str[1];
			ssi_str[2] = '\0';
			ssi = atoi(ssi_str);
			ssi = -1 * (113 - ssi * 2);
			return ssi;
		}
		// it is single digit
		ssi_str[1] = '\0';
		ssi = atoi(ssi_str);
		ssi = -1 * (113 - ssi * 2);
		return ssi;
	} else {
		return -200;
	}
}

int M5311::get_csq()
{
	int ssi;

	MODEM_SERIAL->println(F("AT+CSQ"));
	MODEM_SERIAL->flush();

	char wait_str[] = "+CSQ:";
	String re_str;

	re_str = expect_rx_str(1000, wait_str, 5);

	memset(str, 0, BUF_LEN);
	re_str.toCharArray(str, BUF_LEN);

	if (re_str != "") {

		sscanf(str, "%d,", &ssi);
		return ssi;

	} else {
		return 99;
	}
}

String M5311::req_srv_ip(char srv[])
{
	MODEM_SERIAL->print(F("AT+CMDNS="));
	MODEM_SERIAL->println(srv);

	char wait_str[] = "+IP:";

	String re_str;
	re_str = expect_rx_str(1000, wait_str, 4);
	
	return re_str;
}

bool M5311::mqtt_begin(char srv[], int port, char client_id[])
{
	//AT+MQTTCFG="mqtt.autoeco.net",1883,"12123089999",10,"test","test",1,0
	MODEM_SERIAL->print(F("AT+MQTTCFG=\""));
	MODEM_SERIAL->print(srv);
	MODEM_SERIAL->print(F("\","));
	MODEM_SERIAL->print(port);
	MODEM_SERIAL->print(F(",\""));
	MODEM_SERIAL->print(client_id);
	MODEM_SERIAL->println(F("\",60,\"test\",\"test\",1,0\r\n"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

#if 0
	INFOLN("mqtt begin");

	char wait_str[] = "OK";
	expect_rx_str(1000, wait_str, 2); 

	INFOLN("mqtt begin over");
#endif

	return true;
}

bool M5311::mqtt_connect()
{
	MODEM_SERIAL->println(F("AT+MQTTOPEN=1,1,1,1,1,\"dev/gws\",\"online\"\r\n"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

	char wait_str[] = "+MQTTOPEN: OK";
	//char wait_str[] = "OK";

	String ret_s = expect_rx_str(6000, wait_str, 13);

	if ( ret_s == "") {
		return false;
	} else if (ret_s == "T") {
		return false;
	} else {
		return true;
	}
}

bool M5311::mqtt_pub(char topic[], char msg[])
{
	MODEM_SERIAL->print(F("AT+MQTTPUB=\""));
	MODEM_SERIAL->print(topic);
	MODEM_SERIAL->print(F("\",1,0,0,0,\""));
	MODEM_SERIAL->print(msg);
	MODEM_SERIAL->println(F("\"\r\n"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

	char wait_str[] = "+MQTTPUBACK: ";

	String ret_s = expect_rx_str(6000, wait_str, 13);

	if (ret_s == "") {
		return false;
	} else if (ret_s == "T") {
		return false;
	} else {
		return true;
	}
}

bool M5311::mqtt_pub_noack(char topic[], char msg[])
{
	MODEM_SERIAL->print(F("AT+MQTTPUB=\""));
	MODEM_SERIAL->print(topic);
	MODEM_SERIAL->print(F("\",1,0,0,0,\""));
	MODEM_SERIAL->print(msg);
	MODEM_SERIAL->println(F("\"\r\n"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

	char wait_str[] = "+MQTTPUBACK: ";

	String ret_s = expect_rx_str(1500, wait_str, 13);

	if (ret_s == "") {
		return false;
	} else if (ret_s == "T") {
		return false;
	} else {
		return true;
	}
}

void M5311::mqtt_end()
{
	char wait_str[] = "OK";

	MODEM_SERIAL->println(F("AT+MQTTDISC\r\n"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

	delay(800);
	//expect_rx_str(200, wait_str, 2);

	MODEM_SERIAL->println(F("AT+MQTTDEL\r\n"));
	MODEM_SERIAL->flush();
	MODEM_SERIAL->clear_rxbuf();

	delay(200);

	//expect_rx_str(200, wait_str, 2);
}
