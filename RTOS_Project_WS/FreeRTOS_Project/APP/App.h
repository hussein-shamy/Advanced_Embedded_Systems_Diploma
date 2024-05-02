/*
 * App.h
 *
 *  Created on: Apr 26, 2024
 *      Author: Hussein El-Shamy
 */

#ifndef APP_H_
#define APP_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "GPIO/gpio.h"

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 369

#define TEMP_OUT_OF_RANGE_ERROR ((uint8)44)
#define LOW_INTENSITY           ((uint8)1)
#define MED_INTENSITY           ((uint8)2)
#define HIGH_INTENSITY          ((uint8)3)

#define NO_OF_SEATES            ((uint8)2)
#define Driver_Seat             ((uint8)0)
#define Passenger_Seat          ((uint8)1)

#define HEATER_OFF              ((uint8)0)
#define HEATER_LOW              ((uint8)25)
#define HEATER_MEDIUM           ((uint8)30)
#define HEATER_HIGH             ((uint8)35)


#define AppButton_Driver_Pressed_BIT        ( 1UL << 0UL )
#define AppButton_Passenger_Pressed_BIT     ( 1UL << 1UL )
#define AppTemp_Driver_Changed_BIT          ( 1UL << 2UL )
#define AppTemp_Passenger_Changed_BIT       ( 1UL << 3UL )

#define AppIntensity_Passenger_Selected_BIT ( 1UL << 0UL )
#define AppIntensity_Driver_Selected_BIT    ( 1UL << 1UL )



//#pragma diag_suppress=770

extern EventGroupHandle_t xEventGroup_Set_Intensity;
extern EventGroupHandle_t xEventGroup_ControlHeating;

extern QueueHandle_t xQueue_Button_Driver_State;
extern QueueHandle_t xQueue_Button_Passenger_State;

extern QueueHandle_t xQueue_Temp_Driver;
extern QueueHandle_t xQueue_Temp_Passenger;

extern QueueHandle_t xQueue_Intensity_Driver;
extern QueueHandle_t xQueue_Intensity_Passenger;




/* FreeRTOS tasks */
void vPeriodic_Task_ReadTemp_Seat(void *pvParameters);
void vPeriodic_Task_SetIntensity_Seat(void *pvParameters);
void vPeriodic_Task_ControlHeating_Seat(void *pvParameters);
void vPeriodic_Task_DisplayTempData_LCD(void *pvParameters);
void DriverButtonPressed(void);
void PassengerButtonPressed(void);


#endif /* APP_H_ */
