#include <Stream.h>
#include <Arduino.h>

#define	DEBUG_M5311				1

#ifdef DEBUG_M5311
#define INFO(param)				Serial.print(param)
#define INFOLN(param)			Serial.println(param)
#define INFO_S(param)			Serial.print(F(param))
#define INFO_HEX(param)			Serial.print(param,HEX)
#define INFOLN_HEX(param)		Serial.println(param,HEX)
#define FLUSHOUTPUT				Serial.flush();
#else
#define INFO_S(param)
#define INFO(param)
#define INFO_HEX(param)
#define INFOLN_HEX(param)
#define INFOLN(param)
#define FLUSHOUTPUT
#endif

#define BUF_MAX_SIZE			64
#define MODEM_RESP				160

class M5311 {

public:
	void init(Stream & serial);
	bool reboot();

	String get_imsi();
	String get_imei();

	bool closeUDPSocket();
	bool check_match(char target[], char pattern[], int len_check);
	int check_match_index(char target[], char pattern[], int len_check);
	String expect_rx_str(unsigned long period, char exp_str[],
			     int len_check);

	String find_rxbuf_str(char exp_str[], int len_check);

	bool init_modem();

	int check_modem_status();
	int check_network();
	int check_boot();

	void clean_net_cache();

	bool disable_deepsleep();


	String check_ipaddr();
	String get_net_time();

	int check_modem_signal();

	bool check_incoming_msg();

	String req_srv_ip(char srv[]);

	bool mqtt_begin(char srv[], int port, char clientid[]);
	bool mqtt_connect();
	bool mqtt_pub(char topic[], char msg[]);
	void mqtt_end();

	String check_udp_incoming_str();
	String hex2str(String hexData);
	char byte_convert(char c);

private:
	Stream *MODEM_SERIAL;
	String _hexData;
};
