#include "consoleUtils.h"
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "interrupt.h"
#include "gpio_v2.h"
#include "dmtimer.h"
#include "error.h"

#include "Manager/UARTKISSInterface.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "task.h"

void configure_platform(void);
extern volatile unsigned int cntValue;

#define USR2_LED_GPIO_INSTANCE_ADDRESS	(SOC_GPIO_1_REGS)
#define USR2_LED_GPIO_PIN				(23)

#include "uart_irda_cir.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "consoleUtils.h"
#include "hw_types.h"

void vTaskHeartbeat(void *pvParameters) {

    /* Enable GPIO clocks */
    GPIO1ModuleClkConfig();

    /* Selecting GPIO1[23] pin for use. */
    /* GPIO1[23] Maps to LED USR2 */
    GPIO1Pin23PinMuxSetup();

    /* Enabling the GPIO module. */
    GPIOModuleEnable(USR2_LED_GPIO_INSTANCE_ADDRESS);

    /* Resetting the GPIO module. */
    GPIOModuleReset(USR2_LED_GPIO_INSTANCE_ADDRESS);

    /* Setting the GPIO pin as an output pin. */
    GPIODirModeSet(USR2_LED_GPIO_INSTANCE_ADDRESS,
                   USR2_LED_GPIO_PIN,
                   GPIO_DIR_OUTPUT);

    while(1)
    {

        /* Driving a logic HIGH on the GPIO pin. */
        GPIOPinWrite(USR2_LED_GPIO_INSTANCE_ADDRESS,
                     USR2_LED_GPIO_PIN,
                     GPIO_PIN_HIGH);

        UARTFIFOWrite(SOC_UART_0_REGS, "heart\n", 6);

        vTaskDelay(1000);

        /* Driving a logic LOW on the GPIO pin. */
        GPIOPinWrite(USR2_LED_GPIO_INSTANCE_ADDRESS,
                     USR2_LED_GPIO_PIN,
                     GPIO_PIN_LOW);

        UARTFIFOWrite(SOC_UART_0_REGS, "beat\n", 5);

        vTaskDelay(1000);

    }

}

int main() {

    configure_platform();

    UARTKISSInterface_init();

    xTaskCreate(vTaskHeartbeat, "Heartbeat", 1000, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1);


}
