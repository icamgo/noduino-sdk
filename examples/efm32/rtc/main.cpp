#include "Arduino.h"
#include "rtcdriver.h"

RTCDRV_TimerID_t id;

void myCallback(RTCDRV_TimerID_t id, void *user)
{
	(void) id;
	(void) user;

	Serial.println("Check sensor data....");	

	// Restart timer
	RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, 3000, myCallback, NULL);
}

void setup()
{
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	// Initialization of RTCDRV driver
	RTCDRV_Init();

	// Reserve a timer
	RTCDRV_AllocateTimer(&id);

	// Start a oneshot timer with 5000 ms timeout
	RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, 3000, myCallback, NULL);
}

void loop()
{

}
