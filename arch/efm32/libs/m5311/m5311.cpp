#include "M5311.h"

void M5311::init(Stream & serial)
{
	MODEM_SERIAL = &serial;
}

bool M5311::init_modem()
{
	INFOLN("Initial Modem to connect NB-IoT Network");
	MODEM_SERIAL->println(F("AT+CMRB"));
	delay(1000);

	char wait_str[] = "REBOOT";

	if (expect_rx_str(3000, wait_str, 6) != "") {
		INFOLN("Reboot done Connecting to Network");
	}

	//MODEM_SERIAL->println(F("AT+CFUN=1"));
	//delay(2000);
	MODEM_SERIAL->println(F("AT"));
	delay(1000);
}

bool M5311::reboot()
{
	//MODEM_SERIAL->println("AT+CMRB");
	MODEM_SERIAL->println("AT+COLDRB");
}

String M5311::get_imei()
{
	MODEM_SERIAL->println(F("AT+CGSN=1"));
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
	char wait_str[] = "\r\n";
	String re_str;

	re_str = expect_rx_str(1000, wait_str, 2);

	if (re_str != "") {
		return re_str;
	}
	return "";
}

bool M5311::check_modem_status()
{
	MODEM_SERIAL->println(F("AT"));

	char wait_ok[] = "OK";

	if (expect_rx_str(2000, wait_ok, 2) != "") {
		return true;
	}

	return false;
}

bool M5311::check_network()
{
	bool regist = 0;

	MODEM_SERIAL->println(F("AT+CGATT?"));
	delay(2000);

	char wait_str[] = "+CGATT: 1";

	if (expect_rx_str(2000, wait_str, 9) != "") {
		INFOLN("Regiester network Done");
		return true;

	} else {
		INFO(".");
		return false;
	}
}

String M5311::check_ipaddr()
{
	char wait_str[] = "+CGDCONT: 1,";
	String re_str;

	MODEM_SERIAL->println(F("AT+CGDCONT?"));
	delay(1000);

	re_str = expect_rx_str(1000, wait_str, 11);

	if (re_str != "") {
		return re_str;
	}
	return "";
}

String M5311::get_net_time()
{
	char wait_str[] = "+CCLK: 2";
	String re_str;

	MODEM_SERIAL->println(F("AT+CCLK?"));
	delay(1000);

	re_str = expect_rx_str(1000, wait_str, 8);

	if (re_str != "") {
		return re_str;
	}
	return "";
}

char modem_said[MODEM_RESP];
char str[BUF_MAX_SIZE];

String M5311::expect_rx_str(unsigned long period, char exp_str[], int len_check)
{
	unsigned long cur_t = millis();
	unsigned long start_t = millis();
	bool str_found = 0;
	bool time_out = 0;
	bool loop_out = 0;
	int i = 0;
	int found_index = 0, end_index = 0;
	int modem_said_len = 0;
	char c;
	String re_str;
	char *x;

	memset(modem_said, 0, MODEM_RESP);
	memset(str, 0, BUF_MAX_SIZE);

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
	}

	modem_said[i] = '\0';

	INFOLN(modem_said);

	end_index = i;

	x = strstr(modem_said, exp_str);
	found_index = x ? x - modem_said : -1;

	if (found_index >= 0) {
		i = 0;
		while (modem_said[found_index + i + len_check] != 0x0D | i == 0) {
			str[i] = modem_said[found_index + i + len_check];
			re_str += String(str[i]);
			i++;
		}
		str[i] = '\0';
		return re_str;
	}
	return "";
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
	char strCmd[BUF_MAX_SIZE / 2];
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
	char resp_result[BUF_MAX_SIZE];
	int index = 0;
	int ssi;
	char ssi_str[3];

	MODEM_SERIAL->println("AT+CSQ");

	char wait_str[] = "+CSQ:";
	String re_str;
	re_str = expect_rx_str(1000, wait_str, 5);

	if (re_str != "") {
		ssi_str[0] = re_str[0];
		// check the next char is not "," It is not single digit
		if (re_str[1] != 0x2c) {
			//INFOLN( resp_result[index+2]);
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

String M5311::req_srv_ip(char srv[])
{
	MODEM_SERIAL->print(F("AT+CMDNS="));
	MODEM_SERIAL->println(srv);

	char wait_str[] = "+IP:";

	String re_str;
	re_str = expect_rx_str(1000, wait_str, 4);
	
	return re_str;
}

bool M5311::mqtt_connect(int port, char sock_num[])
{

	MODEM_SERIAL->print(F("AT+NSOCR=DGRAM,17,"));
	MODEM_SERIAL->print(port);
	MODEM_SERIAL->println(F(",1"));

	char wait_str[] = "+IP:";

	String re_str;
	re_str = expect_rx_str(1000, wait_str, 4);
}


bool M5311::mqtt_pub(char topic[], char msg[])
{

#if 0
	int str_len = data.length();
	char buffer[str_len + 2];
	data.toCharArray(buffer, str_len + 1);

	/* Start AT command */
	MODEM_SERIAL->print(F("AT+NSOST=0"));
	MODEM_SERIAL->print(F(","));
	MODEM_SERIAL->print(ip);
	MODEM_SERIAL->print(F(","));
	MODEM_SERIAL->print(port);
	MODEM_SERIAL->print(F(","));
	MODEM_SERIAL->print(String(str_len));
	MODEM_SERIAL->print(F(","));

	/* Fetch print data in hex format */
	char *h_buf;
	h_buf = buffer;
	char fetch[3] = "";
	bool chk = false;
	int i = 0;

	while (*h_buf) {
		chk = itoa((int)*h_buf, fetch, 16);
		if (chk) {
			MODEM_SERIAL->print(fetch);
		}
		h_buf++;
	}
	MODEM_SERIAL->print("\r\n");
#endif

}

String M5311::check_udp_incoming_str()
{

	String retNSOMI;
	int indexNSONMI;
	String recvBuf;
	int delim_pos[10];
	String msg[10];
	String msgdata = "";

	if (MODEM_SERIAL->available()) {

		retNSOMI = MODEM_SERIAL->readString();
		//INFOLN(retNSOMI);

		/* Check +NSOMI index */
		indexNSONMI = retNSOMI.indexOf("+NSONMI:");

		if (indexNSONMI > 0) {
			/* Send NSORF to require incoming message size : 100 byte */
			MODEM_SERIAL->println(F("AT+NSORF=0,100"));
			delay(300);

			if (MODEM_SERIAL->available()) {
				recvBuf = MODEM_SERIAL->readString();
				//INFOLN("recvBuf is " + recvBuf);

				/* Parse buffer to message */
				for (int chkDelim = 0; chkDelim <= 5;
				     chkDelim++) {

					if (chkDelim == 0) {
						delim_pos[chkDelim] =
						    recvBuf.indexOf(F(","));
						msg[chkDelim] =
						    recvBuf.substring(0,
								      delim_pos
								      [chkDelim]);
					} else {
						delim_pos[chkDelim] =
						    recvBuf.indexOf(F(","),
								    (delim_pos
								     [chkDelim -
								      1] + 1));
						msg[chkDelim] =
						    recvBuf.
						    substring(delim_pos
							      [chkDelim - 1] +
							      1,
							      delim_pos
							      [chkDelim]);
					}
					//INFOLN("delim_pos[" + String(chkDelim) + "] : " + delim_pos[chkDelim]);
					//INFOLN("msg[" + String(chkDelim) + "]: " + msg[chkDelim]);
				}

				//INFOLN(msg[4]);
				//INFOLN("len : " + String(msg[4].length()));
				msgdata = hex2str(msg[4]);
				//INFOLN("msg data : " + msgdata);
			}
		}

	}
	return msgdata;
}

String M5311::hex2str(String hexData)
{

	String converted;
	char fetchC;
	_hexData = hexData;

	for (int hexCnt = 0; hexCnt < _hexData.length(); hexCnt += 2) {
		fetchC =
		    byte_convert(_hexData[hexCnt]) << 4 |
		    byte_convert(_hexData[hexCnt + 1]);
		converted += fetchC;
	}
	return converted;
}

char M5311::byte_convert(char c)
{

	char _byte;
	char _c = c;

	if ((_c >= '0') && (_c <= '9')) {
		_byte = _c - 0x30;
	} else {
		//..
	}

	if ((_c >= 'A') && (_c <= 'F')) {
		_byte = _c - 55;
	} else {
		//..
	}

	return _byte;
}
