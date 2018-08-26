#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "SIM7600.h"

void start_lte_tasks(void const * argument) {
	Sim80x_Init(osPriorityLow);
	while (1) {
		osDelay(1000);
	}
	
}



