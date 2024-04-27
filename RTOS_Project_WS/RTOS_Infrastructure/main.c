/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "gpio.h"
#include "tm4c123gh6pm_registers.h"

/* Definitions for the event bits in the event group. */
#define mainSW2_INTERRUPT_BIT ( 1UL << 0UL )  /* Event bit 0, which is set by a SW2 Interrupt. */
#define mainSW1_INTERRUPT_BIT ( 1UL << 1UL )  /* Event bit 1, which is set by a SW1 Interrupt. */

EventGroupHandle_t xEventGroup;

/* The HW setup function */
static void prvSetupHardware( void );

/* FreeRTOS tasks */
void vTask1(void *pvParameters);

int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Before an event group can be used it must first be created. */
    xEventGroup = xEventGroupCreate();

    /* Create Tasks here */
    xTaskCreate(vTask1,         /* Pointer to the function that implements the task. */
                "Task 1",       /* Text name for the task.  This is to facilitate debugging only. */
                256,            /* Stack depth - most small microcontrollers will use much less stack than this. */
                NULL,           /* We are not passing a task parameter in this example. */
                1,              /* This task will run at priority 1. */
                NULL);          /* We are not using the task handle. */

    /* Now all the tasks have been started - start the scheduler */
    vTaskStartScheduler();

    /* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
    for (;;);

}

static void prvSetupHardware( void )
{
    /* Place here any needed HW initialization such as GPIO, UART, etc.  */
    GPIO_BuiltinButtonsLedsInit();
    GPIO_SW1EdgeTriggeredInterruptInit();
    GPIO_SW2EdgeTriggeredInterruptInit();
}

void vTask1(void *pvParameters)
{
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = ( mainSW1_INTERRUPT_BIT | mainSW2_INTERRUPT_BIT);

    for (;;)
    {
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xEventGroup,    /* The event group to read. */
                                                xBitsToWaitFor, /* Bits to test. */
                                                pdTRUE,         /* Clear bits on exit if the unblock condition is met. */
                                                pdFALSE,        /* Don't wait for all bits. */
                                                portMAX_DELAY); /* Don't time out. */

        /* In case PF0 edge triggered interrupt occurred, it will set event 0 bit */
        if ((xEventGroupValue & mainSW2_INTERRUPT_BIT) != 0)
        {
            GPIO_RedLedOn();
        }
        /* In case PF4 edge triggered interrupt occurred, it will set event 1 bit */
        if ((xEventGroupValue & mainSW1_INTERRUPT_BIT) != 0)
        {
            GPIO_RedLedOff();
        }
    }
}

void GPIOPortF_Handler(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    if(GPIO_PORTF_RIS_REG & (1<<0))           /* PF0 handler code */
    {
        xEventGroupSetBitsFromISR(xEventGroup, mainSW2_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG   |= (1<<0);       /* Clear Trigger flag for PF0 (Interrupt Flag) */
    }
    else if(GPIO_PORTF_RIS_REG & (1<<4))      /* PF4 handler code */
    {
        xEventGroupSetBitsFromISR(xEventGroup, mainSW1_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG   |= (1<<4);       /* Clear Trigger flag for PF4 (Interrupt Flag) */
    }
}