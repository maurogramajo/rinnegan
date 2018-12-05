/* Copyright 2015, Pablo Ridolfi
 * All rights reserved.
 *
 * This file is part of lpc1769_template.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @brief Blinky using FreeRTOS.
 *
 *
 * NOTE: It's interesting to check behavior differences between standard and
 * tickless mode. Set @ref configUSE_TICKLESS_IDLE to 1, increment a counter
 * in @ref vApplicationTickHook and print the counter value every second
 * inside a task. In standard mode the counter will have a value around 1000.
 * In tickless mode, it will be around 25.
 *
 */

/** \addtogroup rtos_blink FreeRTOS blink example
 ** @{ */

/*==================[inclusions]=============================================*/

#include "board.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "main.h"

/*==================[macros and definitions]=================================*/

#define RPORT	0
#define RPIN	22

#define GPORT	3
#define GPIN	25

#define BPORT	3
#define BPIN	26

/*==================[internal data declaration]==============================*/
xSemaphoreHandle rsignal = 0;
xSemaphoreHandle gsignal = 0;
xSemaphoreHandle bsignal = 0;

typedef struct
{
	int port;
	int pin;
} typedef_portpin;

enum RGB{ R=0, G, B };

typedef_portpin led[3];
/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);
static void Maquina(void *);
static void ToogleLed(void *);
/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
    SystemCoreClockUpdate();

    Board_Init();

    Chip_GPIO_SetPinDIR(LPC_GPIO, RPORT, RPIN, true);
    Chip_GPIO_SetPinDIR(LPC_GPIO, GPORT, GPIN, true);
    Chip_GPIO_SetPinDIR(LPC_GPIO, BPORT, BPIN, true);
}

static void Maquina(void *a)
{
	int switcher = 0;
	while (1) {
		switch(switcher)
		{
		case 0:
			switcher++;
			xSemaphoreGive(rsignal);
			vTaskDelay(1000 / portTICK_RATE_MS);
			break;
		case 1:
			switcher++;
			xSemaphoreGive(gsignal);
			vTaskDelay(1000 / portTICK_RATE_MS);
			break;
		case 2:
			switcher=0;
			xSemaphoreGive(bsignal);
			vTaskDelay(1000 / portTICK_RATE_MS);
			break;
		default:
			break;
		}

	}
}

static void ToogleLed(void *p)
{
	while(1)
	{
		if(xSemaphoreTake(rsignal, 10 / portTICK_RATE_MS))
		{
			Chip_GPIO_SetPinState(LPC_GPIO, led[R].port, led[R].pin, false);
			Chip_GPIO_SetPinState(LPC_GPIO, led[G].port, led[G].pin, true);
			Chip_GPIO_SetPinState(LPC_GPIO, led[B].port, led[B].pin, true);
		}
		if(xSemaphoreTake(gsignal, 10 / portTICK_RATE_MS))
		{
			Chip_GPIO_SetPinState(LPC_GPIO, led[R].port, led[R].pin, true);
			Chip_GPIO_SetPinState(LPC_GPIO, led[G].port, led[G].pin, false);
			Chip_GPIO_SetPinState(LPC_GPIO, led[B].port, led[B].pin, true);
		}
		if(xSemaphoreTake(bsignal, 10 / portTICK_RATE_MS))
		{
			Chip_GPIO_SetPinState(LPC_GPIO, led[R].port, led[R].pin, true);
			Chip_GPIO_SetPinState(LPC_GPIO, led[G].port, led[G].pin, true);
			Chip_GPIO_SetPinState(LPC_GPIO, led[B].port, led[B].pin, false);
		}
	}
}

/*==================[external functions definition]==========================*/

int main(void)
{
	initHardware();

	led[R].port = RPORT;
	led[R].pin = RPIN;

	led[G].port = GPORT;
	led[G].pin = GPIN;

	led[B].port = BPORT;
	led[B].pin = BPIN;

	vSemaphoreCreateBinary(rsignal);
	vSemaphoreCreateBinary(gsignal);
	vSemaphoreCreateBinary(bsignal);

	xTaskCreate(Maquina, (const char *)"MaquinaDeEstados", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);
	xTaskCreate(ToogleLed, (const char *)"ToogleLed", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, 0);

	vTaskStartScheduler();

	while (1) {
	}
}

/** @} doxygen end group definition */

/*==================[end of file]============================================*/
