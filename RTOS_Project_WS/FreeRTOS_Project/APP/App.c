#include "FreeRTOS.h"
#include "GPIO/gpio.h"
#include "ADC/adc.h"
#include "uart0.h"
#include "App.h"
#include "task.h"
#include "event_groups.h"

/* global variables*/
volatile uint8 g_Button_States[NO_OF_SEATES]= {HEATER_LOW,HEATER_OFF};
sint16 g_Seats_Temp[NO_OF_SEATES] = { 0, 0 };
uint8 g_Heater_intensity[NO_OF_SEATES] = {HEATER_OFF,HEATER_OFF};
EventGroupHandle_t xEventGroup;
EventGroupHandle_t xEventGroup2;

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while (count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n))
        ;
}

void vPeriodic_Task_ReadTemp_Seat(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8 previous_temperature[NO_OF_SEATES] = {0,0};
    for (;;)
    {
        /* Implementation */
        uint32 adc_value = 0;
        uint8 seat_type = (uint8) pvParameters;
        switch (seat_type)
        {
        case Driver_Seat:
            adc_value = ADC_read_PE0();
            g_Seats_Temp[Driver_Seat] = (uint8) (((uint32)adc_value * 40)/ 4095);
            if(previous_temperature[Driver_Seat] < g_Seats_Temp[Driver_Seat] -3 || previous_temperature[Driver_Seat] > g_Button_States[Driver_Seat] +3){
                xEventGroupSetBits(xEventGroup, AppTemp_Driver_Changed_BIT);
            }
            previous_temperature[Driver_Seat] = g_Seats_Temp[Driver_Seat];
            break;
        case Passenger_Seat:
            adc_value = ADC_read_PE1();
            g_Seats_Temp[Passenger_Seat] = (uint8)(148 - ((uint32)(248 * adc_value)/4096));
            if(previous_temperature[Passenger_Seat] < g_Seats_Temp[Passenger_Seat] -3 || previous_temperature[Passenger_Seat] > g_Button_States[Passenger_Seat] +3){
                xEventGroupSetBits(xEventGroup, AppTemp_Passenger_Changed_BIT);
            }
            previous_temperature[Passenger_Seat] = g_Seats_Temp[Passenger_Seat];
            break;
        }

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10000 ) );
    }
}


void vPeriodic_Task_SetIntensity_Seat(void *pvParameters)
{
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = (  AppButton_Driver_Pressed_BIT
                                        | AppButton_Passenger_Pressed_BIT
                                        | AppTemp_Driver_Changed_BIT
                                        | AppTemp_Passenger_Changed_BIT);
    for (;;)
    {
        uint8 current_temp = 0;
        uint8 desiresd_temp = 0;
        uint8 current_intensity = 0;
        uint8 seat_type = (uint8) pvParameters;
        /***************************************************************************************************************************
         *                                                          Wait
         **************************************************************************************************************************/
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xEventGroup,    /* The event group to read. */
                                                xBitsToWaitFor, /* Bits to test. */
                                                pdTRUE,         /* Clear bits on exit if the unblock condition is met. */
                                                pdFALSE,        /* Don't wait for all bits. */
                                                portMAX_DELAY); /* Don't time out. */

        if (((xEventGroupValue & AppTemp_Driver_Changed_BIT) != 0)||(((xEventGroupValue & AppButton_Driver_Pressed_BIT) != 0) && seat_type == Driver_Seat))
        {
            /***************************************************************************************************************************
             *                                                          Driver Pressed
             **************************************************************************************************************************/
            current_temp = g_Seats_Temp[Driver_Seat];
            desiresd_temp = g_Button_States[Driver_Seat];
            current_intensity = g_Heater_intensity[Driver_Seat];

            if (current_intensity != HEATER_OFF)
            {

                if (current_temp < desiresd_temp - 10)
                {

                    g_Heater_intensity[Driver_Seat] = HIGH_INTENSITY;

                }
                else if (current_temp >= desiresd_temp - 10
                        && current_temp < desiresd_temp - 5)
                {

                    g_Heater_intensity[Driver_Seat] = MED_INTENSITY;

                }
                else if (current_temp >= desiresd_temp - 5
                        && current_temp < desiresd_temp - 2)
                {

                    g_Heater_intensity[Driver_Seat] = LOW_INTENSITY;

                }
                else if (current_temp > desiresd_temp)
                {

                    g_Heater_intensity[Driver_Seat] = HEATER_OFF;

                }

            }
            else
            {

                if (current_temp < desiresd_temp - 3)
                {

                    g_Heater_intensity[Driver_Seat] = LOW_INTENSITY;

                }

            }
            xEventGroupSetBits(xEventGroup2, AppIntensity_Driver_Selected_BIT);
            /*End of Driver pressed*/
        }

        if (((xEventGroupValue & AppTemp_Passenger_Changed_BIT) != 0)||(((xEventGroupValue & AppButton_Passenger_Pressed_BIT) != 0) && seat_type == Driver_Seat)){
            /***************************************************************************************************************************
             *                                                          Passenger Pressed
             **************************************************************************************************************************/

            current_temp = g_Seats_Temp[Passenger_Seat];
            desiresd_temp = g_Button_States[Passenger_Seat];
            current_intensity = g_Heater_intensity[Passenger_Seat];

            if (current_intensity != HEATER_OFF)
            {

                if (current_temp < desiresd_temp - 10)
                {
                    // x<15 (cyan)
                    g_Heater_intensity[Passenger_Seat] = HIGH_INTENSITY;

                }
                else if (current_temp >= desiresd_temp - 10
                        && current_temp < desiresd_temp - 5)
                {
                    // 15  -x- 20 (blue)
                    g_Heater_intensity[Passenger_Seat] = MED_INTENSITY;

                }
                else if (current_temp >= desiresd_temp - 5 && current_temp < desiresd_temp - 2)
                {
                   //   20 -x- 23 (green)
                    g_Heater_intensity[Passenger_Seat] = LOW_INTENSITY;

                }
                else if (current_temp > desiresd_temp)
                {
                    // x>25 (off)
                    g_Heater_intensity[Passenger_Seat] = HEATER_OFF;

                    }

                }
            else
            {

                if (current_temp < desiresd_temp - 3)
                {

                    g_Heater_intensity[Passenger_Seat] = LOW_INTENSITY;

                }

            }
            xEventGroupSetBits(xEventGroup2, AppIntensity_Passenger_Selected_BIT);
        /*End of Passenger pressed*/
        }
    }
}


void vPeriodic_Task_ControlHeating_Seat(void *pvParameters)
{
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = ( AppIntensity_Driver_Selected_BIT | AppIntensity_Passenger_Selected_BIT);
    for (;;)
    {
        /***************************************************************************************************************************
         *                                                          Wait
         **************************************************************************************************************************/
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xEventGroup2,    /* The event group to read. */
                                                xBitsToWaitFor, /* Bits to test. */
                                                pdTRUE,         /* Clear bits on exit if the unblock condition is met. */
                                                pdFALSE,        /* Don't wait for all bits. */
                                                portMAX_DELAY); /* Don't time out. */
        /* Implementation */
        uint8 current_intensity = 0;
        uint8 seat_type = (uint8) pvParameters;

        if(((xEventGroupValue & AppIntensity_Driver_Selected_BIT) != 0) && seat_type == Driver_Seat){

            current_intensity =  g_Heater_intensity[Driver_Seat];

            switch (current_intensity)
            {
            case HEATER_OFF:
                GPIO_AllLedOff();
                break;

            case LOW_INTENSITY:
                GPIO_AllLedOff();
                GPIO_GreenLedOn();
                break;

            case MED_INTENSITY:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                break;

            case HIGH_INTENSITY:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                GPIO_GreenLedOn();
                break;
            }

        }
        if(((xEventGroupValue & AppIntensity_Passenger_Selected_BIT) != 0) && seat_type == Passenger_Seat){

            current_intensity = g_Heater_intensity[Passenger_Seat];

            switch (current_intensity)
            {
            case HEATER_OFF:
                GPIO_AllLedOff();
                break;

            case LOW_INTENSITY:
                GPIO_AllLedOff();
                GPIO_GreenLedOn();
                break;

            case MED_INTENSITY:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                break;

            case HIGH_INTENSITY:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                GPIO_GreenLedOn();
                break;
            }

        }
    }
}


void vPeriodic_Task_DisplayTempData_LCD(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Implementation */
        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("===================================== [DASHBOARD] ===========================================\r\n");
        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("---------------------------------------------------------------------------------------------\r\n");
        UART0_SendString("               Driver Seat               |             Passenger Seat                        \r\n");
        UART0_SendString("---------------------------------------------------------------------------------------------\r\n");
        UART0_SendString("Current Temperature:                     |\r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(g_Seats_Temp[Driver_Seat]);
        UART0_SendString("                  |                  ");
        UART0_SendInteger(g_Seats_Temp[Passenger_Seat]);
        UART0_SendString("\r\n");

        UART0_SendString("Desired Temperature:                     |\r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(g_Button_States[Driver_Seat]);
        UART0_SendString("                 |                  ");
        UART0_SendInteger(g_Button_States[Passenger_Seat]);
        UART0_SendString("\r\n");

        UART0_SendString("Heater Intensity Temperature:            |\r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(g_Heater_intensity[Driver_Seat]);
        UART0_SendString("                  |                  ");
        UART0_SendInteger(g_Heater_intensity[Passenger_Seat]);
        UART0_SendString("\r\n");

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10000 ) );
    }
}


void DriverButtonPressed(void){
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    uint8 current = g_Button_States[Driver_Seat];
    uint8 next = 0;
    if(current == HEATER_OFF){
        next = HEATER_LOW;
    }else if(current == HEATER_LOW){
        next = HEATER_MEDIUM;
    }else if(current == HEATER_MEDIUM){
        next = HEATER_HIGH;
    }else if(current == HEATER_HIGH){
        next = HEATER_OFF;
    }
    g_Button_States[Driver_Seat] = next;
    xEventGroupSetBitsFromISR(xEventGroup, AppButton_Driver_Pressed_BIT,&pxHigherPriorityTaskWoken);
}

void PassengerButtonPressed(void){
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    uint8 current = g_Button_States[Driver_Seat];
    uint8 next = 0;
    if(current == HEATER_OFF){
        next = HEATER_LOW;
    }else if(current == HEATER_LOW){
        next = HEATER_MEDIUM;
    }else if(current == HEATER_MEDIUM){
        next = HEATER_HIGH;
    }else if(current == HEATER_HIGH){
        next = HEATER_OFF;
    }
    g_Button_States[Passenger_Seat] = next;
    xEventGroupSetBitsFromISR(xEventGroup, AppButton_Passenger_Pressed_BIT,&pxHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/
