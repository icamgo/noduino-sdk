/*
 *  Demo of three threads
 *  LED blink thread, print thread, and idle loop
 *
 *  Attention:
 *   To use idle hook, Must set configUSE_IDLE_HOOK to 1 in file HAL_Conf.h or FreeRTOSConfig.h
*/
#include <FreeRTOS.h>

#if configUSE_IDLE_HOOK == 0
# error	"!Use idel hook,The macro variable configUSE_IDLE_HOOK must be set to 1 in file HAL_Conf.h!"
#endif

// The LED is attached to pin 13 on Arduino.
# define LED_PIN     LED_BUILTIN
# define LED_LEVELON (LED_BUILTIN_MASK & 0x01)	/*LOW or HIGH */

volatile uint32_t count = 0;

// handle for blink task
TaskHandle_t blink;

// high priority for blinking LED
static void vLEDFlashTask(void *pvParameters)
{
	UNUSED(pvParameters);
	pinMode(LED_PIN, OUTPUT);

	// Flash led every 200 ms.
	for (;;) {
		// Turn LED on.
		digitalWrite(LED_PIN, LED_LEVELON);

		// Sleep for 20 milliseconds.
		vTaskDelay((20L * configTICK_RATE_HZ) / 1000L);

		// Turn LED off.
		digitalToggle(LED_PIN);

		// Sleep for 230 milliseconds.
		vTaskDelay((230L * configTICK_RATE_HZ) / 1000L);
	}
}

static void vPrintTask(void *pvParameters)
{
	UNUSED(pvParameters);
	while (1) {
		// Sleep for one second.
		vTaskDelay(configTICK_RATE_HZ);
		if (count) {
			// Print count for previous second.
			Serial.print(F("Count per second: "));
			Serial.println(count);
			// Zero count.
			count = 0;
		} else {
			Serial.
			    println
			    (" idle hook is not runed. Please set configUSE_IDLE_HOOK to 1");
		}
	}
}

void setup()
{

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	while (!Serial) {
	}

	// create blink task
	xTaskCreate(vLEDFlashTask,
		    "Task1",
		    configMINIMAL_STACK_SIZE + 50,
		    NULL, tskIDLE_PRIORITY + 2, &blink);

	// create print task
	xTaskCreate(vPrintTask,
		    "Task2",
		    configMINIMAL_STACK_SIZE + 100,
		    NULL, tskIDLE_PRIORITY + 1, NULL);

	//start FreeRTOS
	vTaskStartScheduler();

	// should never return
	Serial.println(F("die"));
	while (1) ;
}

void loop()
{
	while (1) {
		// must insure increment is atomic
		// in case of context switch for print
		noInterrupts();
		count++;
		interrupts();
	}
}
