#include "FreeRTOS.h"
#include "GPIO/gpio.h"
#include "ADC/adc.h"
#include "uart0.h"
#include "App.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"

EventGroupHandle_t xEventGroup_Set_Intensity;
EventGroupHandle_t xEventGroup_ControlHeating;

QueueHandle_t xQueue_Button_Driver_State;
QueueHandle_t xQueue_Button_Passenger_State;

QueueHandle_t xQueue_Temp_Driver;
QueueHandle_t xQueue_Temp_Passenger;

QueueHandle_t xQueue_Intensity_Driver;
QueueHandle_t xQueue_Intensity_Passenger;

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while (count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n))
        ;
}

void vPeriodic_Task_ReadTemp_Seat(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    sint8 previous_temperature = 0;
    sint8 current_temperature = 0;
    uint32 adc_value = 0;

    for (;;)
    {
        switch ((uint8) pvParameters)
        {
        case Driver_Seat:
            adc_value = ADC_read_PE0();
            current_temperature = (uint8) (((uint32)adc_value * 45)/ 4095);
            xQueueSend(xQueue_Temp_Driver,&current_temperature,0);
            if(previous_temperature < current_temperature -3 || previous_temperature > current_temperature +3){
                xEventGroupSetBits(xEventGroup_Set_Intensity, AppTemp_Driver_Changed_BIT);
             }
            break;
        case Passenger_Seat:
            adc_value = ADC_read_PE1();
            current_temperature = (uint8)(148 - ((uint32)(248 * adc_value)/4096));
            xQueueSend(xQueue_Temp_Passenger,&current_temperature,0);
            if(previous_temperature < current_temperature -3 || previous_temperature > current_temperature +3){
                xEventGroupSetBits(xEventGroup_Set_Intensity, AppTemp_Passenger_Changed_BIT);
            }
            break;
        }


        previous_temperature = current_temperature;

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10000 ) );

    }

}


void vPeriodic_Task_SetIntensity_Seat(void *pvParameters)
{
    QueueHandle_t xQueue_Temp_Current_Task = 0;
    QueueHandle_t xQueue_Intensity_Current_Task = 0;

    EventBits_t xEventGroupValue;
    uint8 flag = pdFALSE;
    const EventBits_t xBitsToWaitFor = (  AppButton_Driver_Pressed_BIT
            | AppTemp_Driver_Changed_BIT
            | AppButton_Passenger_Pressed_BIT
            | AppTemp_Passenger_Changed_BIT);
    for (;;)
    {

        uint8 current_intensity = 0;
        uint8 current_temp = 0;
        uint8 desiresd_temp = 0;
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xEventGroup_Set_Intensity,    /* The event group to read. */
                                                xBitsToWaitFor, 			  /* Bits to test. */
                                                pdTRUE,         			  /* Clear bits on exit if the unblock condition is met. */
                                                pdFALSE,       			      /* Don't wait for all bits. */
                                                portMAX_DELAY); 			  /* Don't time out. */

        if (((xEventGroupValue & AppTemp_Driver_Changed_BIT) != 0)||(((xEventGroupValue & AppButton_Driver_Pressed_BIT) != 0) && (uint8) pvParameters == Driver_Seat))
        {
            xQueueReceive(xQueue_Button_Driver_State, &desiresd_temp, 0);
            xQueue_Temp_Current_Task = xQueue_Temp_Driver;
            xQueue_Intensity_Current_Task = xQueue_Intensity_Driver;
            xEventGroupSetBits(xEventGroup_ControlHeating,AppIntensity_Driver_Selected_BIT);
            flag = pdTRUE;
        }else if (((xEventGroupValue & AppTemp_Passenger_Changed_BIT) != 0)||(((xEventGroupValue & AppButton_Passenger_Pressed_BIT) != 0) && (uint8) pvParameters == Passenger_Seat)){
            xQueueReceive(xQueue_Button_Passenger_State, &desiresd_temp, 0);
            xQueue_Temp_Current_Task = xQueue_Temp_Passenger;
            xQueue_Intensity_Current_Task = xQueue_Intensity_Passenger;
            xEventGroupSetBits(xEventGroup_ControlHeating,AppIntensity_Passenger_Selected_BIT);
            flag = pdTRUE;
        }

        if(flag){/* Start of Logic */

            xQueueReceive(xQueue_Temp_Current_Task, &current_temp, 0);

            if(current_temp < 5 || current_temp >40){/* Warning */
                xQueueSend(xQueue_Intensity_Current_Task,(void*)TEMP_OUT_OF_RANGE_ERROR,0);
            }/* Warning */

            else{/* No Warning */

                if (current_intensity != HEATER_OFF){ /* Heater ON */

                    if (current_temp < desiresd_temp - 10)
                        xQueueSend(xQueue_Intensity_Current_Task,(void*)HIGH_INTENSITY,0);

                    else if (current_temp >= desiresd_temp - 10 && current_temp < desiresd_temp - 5)
                        xQueueSend(xQueue_Intensity_Current_Task,(void*)MED_INTENSITY,0);

                    else if (current_temp >= desiresd_temp - 5 && current_temp < desiresd_temp - 2)
                        xQueueSend(xQueue_Intensity_Current_Task,(void*)LOW_INTENSITY,0);

                    else if (current_temp > desiresd_temp)
                        xQueueSend(xQueue_Intensity_Current_Task,(void*)HEATER_OFF,0);

                }/* Heater ON */

                else{/* Heater OFF */
                    if (current_temp < desiresd_temp - 3)
                        xQueueSend(xQueue_Intensity_Current_Task,(void*)LOW_INTENSITY, 0);

                }/* Heater OFF */

            }/* End of No Warning */

            flag = pdFALSE;
        }/* End of Logic */
    }/* End of For Loop */

}/* End of Task */

void vPeriodic_Task_ControlHeating_Seat(void *pvParameters)
{
        uint8 current_intensity = 0;
        EventBits_t xEventGroupValue;
        const EventBits_t xBitsToWaitFor = ( AppIntensity_Driver_Selected_BIT | AppIntensity_Passenger_Selected_BIT);
    for (;;)
    {
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xEventGroup_ControlHeating,    /* The event group to read. */
                                                xBitsToWaitFor,                /* Bits to test. */
                                                pdTRUE,                        /* Clear bits on exit if the unblock condition is met. */
                                                pdFALSE,                       /* Don't wait for all bits. */
                                                portMAX_DELAY);                /* Don't time out. */

        if(((xEventGroupValue & AppIntensity_Driver_Selected_BIT) != 0) && (uint8)pvParameters == Driver_Seat)
                xQueueReceive(xQueue_Intensity_Driver, &current_intensity, 0);

        if(((xEventGroupValue & AppIntensity_Passenger_Selected_BIT) != 0) && (uint8)pvParameters == Passenger_Seat)
                xQueueReceive(xQueue_Intensity_Passenger, &current_intensity, 0);

            switch (current_intensity)
            {
            case TEMP_OUT_OF_RANGE_ERROR:
                GPIO_AllLedOff();
                GPIO_RedLedOn();
                break;

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



void vPeriodic_Task_DisplayTempData_LCD(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    sint8 Seats_Temp[NO_OF_SEATES] = {0,0};
    uint8 Button_States[NO_OF_SEATES] = {0,0};
    uint8 Heater_intensity[NO_OF_SEATES] = {0,0};
    for (;;)
    {

        xQueueReceive(xQueue_Button_Driver_State, &Button_States[Driver_Seat], 0);
        xQueueReceive(xQueue_Button_Passenger_State, &Button_States[Passenger_Seat], 0);

        xQueueReceive(xQueue_Temp_Driver, &Seats_Temp[Driver_Seat], 0);
        xQueueReceive(xQueue_Temp_Passenger, &Seats_Temp[Passenger_Seat], 0);

        xQueueReceive(xQueue_Intensity_Driver, &Heater_intensity[Driver_Seat], 0);
        xQueueReceive(xQueue_Intensity_Passenger,&Heater_intensity[Passenger_Seat], 0);



        /* Implementation */
        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("===================================== [DASHBOARD] ===========================================\r\n");
        UART0_SendString("=============================================================================================\r\n");
        UART0_SendString("---------------------------------------------------------------------------------------------\r\n");
        UART0_SendString("               Driver Seat               |             Passenger Seat                        \r\n");
        UART0_SendString("---------------------------------------------------------------------------------------------\r\n");
        UART0_SendString("Current Temperature:                     |\r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(Seats_Temp[Driver_Seat]);
        UART0_SendString("                  |                  ");
        UART0_SendInteger(Seats_Temp[Passenger_Seat]);
        UART0_SendString("\r\n");

        UART0_SendString("Desired Temperature:                     |\r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(Button_States[Driver_Seat]);
        UART0_SendString("                 |                  ");
        UART0_SendInteger(Button_States[Passenger_Seat]);
        UART0_SendString("\r\n");

        UART0_SendString("Heater Intensity Temperature:            |\r\n");
        UART0_SendString("                      ");
        UART0_SendInteger(Heater_intensity[Driver_Seat]);
        UART0_SendString("                  |                  ");
        UART0_SendInteger(Heater_intensity[Passenger_Seat]);
        UART0_SendString("\r\n");

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10000 ) );
    }
}


void DriverButtonPressed(void){
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    static uint8 current = 0;

    switch (current)
    {
    case HEATER_OFF:
        current = HEATER_LOW;
        break;
    case HEATER_LOW:
        current = HEATER_MEDIUM;
        break;
    case HEATER_MEDIUM:
        current = HEATER_HIGH;
        break;
    case HEATER_HIGH:
        current = HEATER_OFF;
        break;
    }
    xQueueSendFromISR (xQueue_Button_Driver_State, &current , &pxHigherPriorityTaskWoken);
    xEventGroupSetBitsFromISR(xEventGroup_Set_Intensity, AppButton_Driver_Pressed_BIT,&pxHigherPriorityTaskWoken);
}

void PassengerButtonPressed(void){
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    static uint8 current = 0;

    switch (current)
    {
    case HEATER_OFF:
        current = HEATER_LOW;
        break;
    case HEATER_LOW:
        current = HEATER_MEDIUM;
        break;
    case HEATER_MEDIUM:
        current = HEATER_HIGH;
        break;
    case HEATER_HIGH:
        current = HEATER_OFF;
        break;
    }

    xQueueSendFromISR (xQueue_Button_Passenger_State, &current , &pxHigherPriorityTaskWoken);
    xEventGroupSetBitsFromISR(xEventGroup_Set_Intensity, AppButton_Passenger_Pressed_BIT,&pxHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/
