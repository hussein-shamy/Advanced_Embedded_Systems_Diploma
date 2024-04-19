/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* MCAL includes. */
#include "gpio.h"
#include "ADC/adc.h"
#include "uart0.h"

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 369

//intensity
#define HEATER_OFF          ((uint8)0)
#define LOW_INTENSITY       ((uint8)1)
#define MED_INTENSITY       ((uint8)2)
#define HIGH_INTENSITY      ((uint8)3)


void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while (count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n))
        ;
}

/* The HW setup function */
static void prvSetupHardware(void);

/* FreeRTOS tasks */
void vPeriodic_Task_ReadTemp_Seat(void *pvParameters);
void vPeriodic_Task_SetIntensity_Seat(void *pvParameters);
void vPeriodic_Task_ControlHeating_Seat(void *pvParameters);
void vPeriodic_Task_DisplayTempData_LCD(void *pvParameters);

/* global variables*/
uint8 g_Seats_Temp[NO_OF_SEATES] = { 0, 0 };
uint8  g_Heater_intensity[NO_OF_SEATES] = {HEATER_OFF,HEATER_OFF};

int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Create Tasks here */
    xTaskCreate(vPeriodic_Task_ReadTemp_Seat,
                "Read Temperature for Driver's Seat",
                64,
                (void*) Driver_Seat,
                4,
                NULL);

    xTaskCreate(vPeriodic_Task_ReadTemp_Seat,
                "Read Temperature for Passenger's Seat",
                64,
                (void*) Passenger_Seat,
                4,
                NULL);

    xTaskCreate(vPeriodic_Task_SetIntensity_Seat,
                "Set Heating Intensity for Driver's Seat",
                64,
                (void*) Driver_Seat,
                3,
                NULL);

    xTaskCreate(vPeriodic_Task_SetIntensity_Seat,
                "Set Heating Intensity for Passenger's Seat",
                64,
                (void*) Passenger_Seat,
                3,
                NULL);

    xTaskCreate(vPeriodic_Task_ControlHeating_Seat,
                "Control Heating for Driver's Seat",
                64,
                (void*) Driver_Seat,
                2,
                NULL);

    xTaskCreate(vPeriodic_Task_ControlHeating_Seat,
                "Control Heating for Passenger's Seat",
                64,
                (void*) Passenger_Seat,
                2,
                NULL);

    xTaskCreate(vPeriodic_Task_DisplayTempData_LCD,
                "Display Temperature Data on LCD Screen",
                128,
                NULL,
                1,
                NULL);


    /* Now all the tasks have been started - start the scheduler.

     NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
     The processor MUST be in supervisor mode when vTaskStartScheduler is
     called.  The demo applications included in the FreeRTOS.org download switch
     to supervisor mode prior to main being called.  If you are not using one of
     these demo application projects then ensure Supervisor mode is used here. */
    vTaskStartScheduler();

    /* Should never reach here!  If you do then there was not enough heap
     available for the idle task to be created. */
    for (;;)
        ;

}

static void prvSetupHardware(void)
{
    /* Place here any needed HW initialization such as GPIO, UART, etc.  */
    //Enable_Exceptions();
   // Enable_Faults();
    /*GPIO*/
    GPIO_BuiltinButtonsLedsInit();
    GPIO_SW1EdgeTriggeredInterruptInit();
    GPIO_SW2EdgeTriggeredInterruptInit();
    GPIO_SW3EdgeTriggeredInterruptInit();
    /*ADC*/
    ADC_PE0_PE1_init();
    /*UART*/
    UART0_Init();
}

void vPeriodic_Task_ReadTemp_Seat(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /* Implementation */
        uint32 adc_value = 0;
        uint8 seat_type = (uint8) pvParameters;
        switch (seat_type)
        {
        case Driver_Seat:
            adc_value = ADC_read_PE0();
            g_Seats_Temp[Driver_Seat] = (uint8) (((uint32)adc_value * 40)
                    / 4095);
            break;
        case Passenger_Seat:
            adc_value = ADC_read_PE1();
            g_Seats_Temp[Passenger_Seat] = (uint8) (((uint32) adc_value * 40)
                    / 4095);
            break;
        }
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1000 ) );
    }
}


void vPeriodic_Task_SetIntensity_Seat(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /* Implementation */
        uint8 current_temp = 0;
        uint8 desiresd_temp = 0;
        uint8 current_intensity = 0;
        uint8 seat_type = (uint8) pvParameters;

        switch (seat_type)
        {
        case Driver_Seat:

            current_temp = g_Seats_Temp[Driver_Seat];
            desiresd_temp = g_Button_States[Driver_Seat];
            current_intensity = g_Heater_intensity[Driver_Seat];

            if (current_intensity != HEATER_OFF)
            {

                if (current_temp < desiresd_temp - 10)
                {

                    g_Heater_intensity[Driver_Seat] = HEATER_HIGH;

                }
                else if (current_temp >= desiresd_temp - 10
                        && current_temp < desiresd_temp - 5)
                {

                    g_Heater_intensity[Driver_Seat] = HEATER_MEDIUM;

                }
                else if (current_temp >= desiresd_temp - 5
                        && current_temp < desiresd_temp - 2)
                {

                    g_Heater_intensity[Driver_Seat] = HEATER_LOW;

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

                    g_Heater_intensity[Driver_Seat] = HEATER_LOW;

                }

            }

            break;

        case Passenger_Seat:

            current_temp = g_Seats_Temp[Passenger_Seat];
            desiresd_temp = g_Button_States[Passenger_Seat];
            current_intensity = g_Heater_intensity[Passenger_Seat];

            if (current_intensity != HEATER_OFF)
            {

                if (current_temp < desiresd_temp - 10)
                {

                    g_Heater_intensity[Passenger_Seat] = HEATER_HIGH;

                }
                else if (current_temp >= desiresd_temp - 10
                        && current_temp < desiresd_temp - 5)
                {

                    g_Heater_intensity[Passenger_Seat] = HEATER_MEDIUM;

                }
                else if (current_temp >= desiresd_temp - 5
                        && current_temp < desiresd_temp - 2)
                {

                    g_Heater_intensity[Passenger_Seat] = HEATER_LOW;

                }
                else if (current_temp > desiresd_temp)
                {

                    g_Heater_intensity[Passenger_Seat] = HEATER_OFF;

                }

            }
            else
            {

                if (current_temp < desiresd_temp - 3)
                {

                    g_Heater_intensity[Passenger_Seat] = HEATER_LOW;

                }

            }

            break;
        }
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 2000 ) );
    }
}


void vPeriodic_Task_ControlHeating_Seat(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Implementation */
        uint8 current_intensity = 0;

        switch ((uint8) pvParameters)
        {
        case Driver_Seat:

            current_intensity = g_Heater_intensity[Driver_Seat];

            switch (current_intensity)
            {
            case HEATER_OFF:
                GPIO_AllLedOff();
                break;

            case HEATER_LOW:
                GPIO_AllLedOff();
                GPIO_GreenLedOn();
                break;

            case HEATER_MEDIUM:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                break;

            case HEATER_HIGH:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                GPIO_GreenLedOn();
                break;
            }

            break;
        case Passenger_Seat:
            current_intensity = g_Heater_intensity[Driver_Seat];

            switch (current_intensity)
            {
            case HEATER_OFF:
                GPIO_AllLedOff();
                break;

            case HEATER_LOW:
                GPIO_AllLedOff();
                GPIO_GreenLedOn();
                break;

            case HEATER_MEDIUM:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                break;

            case HEATER_HIGH:
                GPIO_AllLedOff();
                GPIO_BlueLedOn();
                GPIO_GreenLedOn();
                break;
            }

            break;
        }
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 4000 ) );
    }
}


void vPeriodic_Task_DisplayTempData_LCD(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Implementation */
        UART0_SendString("=============================================================\r\n");
        UART0_SendString("                          DASHBOARD                          \r\n");
        UART0_SendString("=============================================================\r\n");
        UART0_SendString("Driver Seat\r\n");
        UART0_SendString("Current Temperature: ");
        UART0_SendByte(g_Seats_Temp[Driver_Seat]);
        UART0_SendString("\r\n");
        UART0_SendString("Desired Temperature: ");
        UART0_SendByte(g_Button_States[Driver_Seat]);
        UART0_SendString("\r\n");
        UART0_SendString("Heater Intensity Temperature: ");
        UART0_SendByte(g_Heater_intensity[Driver_Seat]);
        UART0_SendString("\r\n");

        UART0_SendString("Passenger Seat\r\n");
        UART0_SendString("Current Temperature: ");
        UART0_SendByte(g_Seats_Temp[Passenger_Seat]);
        UART0_SendString("\r\n");
        UART0_SendString("Desired Temperature: ");
        UART0_SendByte(g_Button_States[Passenger_Seat]);
        UART0_SendString("\r\n");
        UART0_SendString("Heater Intensity Temperature: ");
        UART0_SendByte(g_Heater_intensity[Passenger_Seat]);
        UART0_SendString("\r\n");
        UART0_SendString("=============================================================");
        UART0_SendString("\r\n");

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 8000 ) );
    }
}


/*-----------------------------------------------------------*/
