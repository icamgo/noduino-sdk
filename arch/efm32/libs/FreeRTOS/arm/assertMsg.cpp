#include <Arduino.h>

extern "C" {
	/**
	*  Print file and line when configASSERT is defied like this.
	*
	* #define configASSERT( x ) if( ( x ) == 0 ) {assertMsg(__FILE__,__LINE__);}
	*/
	void _Error_Handler(char* file, uint32_t line);
	
	void assertMsg(const char* file, int line) {
#if USE_ERRORBLINK
        _Error_Handler((char *)file, line);
#else
		interrupts();
		Serial.print(file);
		Serial.write('.');
		Serial.println(line);
		Serial.flush();
		noInterrupts();
		for (;;) {}
#endif
	}
}  // extern "C"
