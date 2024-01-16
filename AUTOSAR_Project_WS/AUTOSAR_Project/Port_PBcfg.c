/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - PORT Driver
 *
 * Author: Hussein El-Shamy
 ******************************************************************************/

/************************************************************************************
 *                             Module Version 1.0.0
 ************************************************************************************/
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)
/************************************************************************************
 *                             AUTOSAR Version 4.0.3
 ************************************************************************************/
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)
/************************************************************************************
 *                  Version Compatibility Check between included files
 ************************************************************************************/
#include "Port.h"

#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
        ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
        ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Dio_PBcfg.c and Dio.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
        ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
        ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of PBcfg.c does not match the expected version"
#endif

/************************************************************************************
 *                   CONFIGURATION STRUCTURE
 *-----------------------------------------------------------------------------------
 *
 *
 *
 *
 *
 *
 *
 *
 *
 ************************************************************************************/

const Port_ConfigType Port_Configuration = {
        /**************************************************************
         *                          PORT A
         **************************************************************/
        /************************** PIN 0 *****************************/
        PORT_A,
        PORTA_PA0,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 1 *****************************/
        PORT_A,
        PORTA_PA1,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 2 *****************************/
        PORT_A,
        PORTA_PA2,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 3 *****************************/
        PORT_A,
        PORTA_PA3,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 4 *****************************/
        PORT_A,
        PORTA_PA4,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 5 *****************************/
        PORT_A,
        PORTA_PA5,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 6 *****************************/
        PORT_A,
        PORTA_PA6,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 7 *****************************/
        PORT_A,
        PORTA_PA7,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /**************************************************************
         *                          PORT B
         **************************************************************/
        /************************** PIN 0 *****************************/
        PORT_B,
        PORTB_PB0,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 1 *****************************/
        PORT_B,
        PORTB_PB1,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 2 *****************************/
        PORT_B,
        PORTB_PB2,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 3 *****************************/
        PORT_B,
        PORTB_PB3,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 4 *****************************/
        PORT_B,
        PORTB_PB4,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 5 *****************************/
        PORT_B,
        PORTB_PB5,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 6 *****************************/
        PORT_B,
        PORTB_PB6,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 7 *****************************/
        PORT_B,
        PORTB_PB7,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /**************************************************************
         *                          PORT C
         **************************************************************/
        /************************** PIN 0 *****************************/
        PORT_C,
        PORTC_PC0,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 1 *****************************/
        PORT_C,
        PORTC_PC1,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 2 *****************************/
        PORT_C,
        PORTC_PC2,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 3 *****************************/
        PORT_C,
        PORTC_PC3,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 4 *****************************/
        PORT_C,
        PORTC_PC4,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 5 *****************************/
        PORT_C,
        PORTC_PC5,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 6 *****************************/
        PORT_C,
        PORTC_PC6,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 7 *****************************/
        PORT_C,
        PORTC_PC7,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /**************************************************************
         *                          PORT D
         **************************************************************/
        /************************** PIN 0 *****************************/
        PORT_D,
        PORTD_PD0,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 1 *****************************/
        PORT_D,
        PORTD_PD1,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 2 *****************************/
        PORT_D,
        PORTD_PD2,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 3 *****************************/
        PORT_D,
        PORTD_PD3,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 4 *****************************/
        PORT_D,
        PORTD_PD4,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 5 *****************************/
        PORT_D,
        PORTD_PD5,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 6 *****************************/
        PORT_D,
        PORTD_PD6,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 7 *****************************/
        PORT_D,
        PORTD_PD7,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /**************************************************************
         *                          PORT E
         **************************************************************/
        /************************** PIN 0 *****************************/
        PORT_E,
        PORTE_PE0,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 1 *****************************/
        PORT_E,
        PORTE_PE1,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 2 *****************************/
        PORT_E,
        PORTE_PE2,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 3 *****************************/
        PORT_E,
        PORTE_PE3,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 4 *****************************/
        PORT_E,
        PORTE_PE4,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 5 *****************************/
        PORT_E,
        PORTE_PE5,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /**************************************************************
         *                          PORT F
         **************************************************************/
        /************************** PIN 0 *****************************/
        PORT_F,
        PORTF_PF0,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 1 *****************************
         ******************** Configured as LED **********************/
        PORT_F,
        PORTF_PF1,
        PORT_PIN_DEFAULT_MODE,
        OUTPUT,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 2 *****************************/
        PORT_F,
        PORTF_PF2,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 3 *****************************/
        PORT_F,
        PORTF_PF3,
        PORT_PIN_DEFAULT_MODE,
        PORT_PIN_DEFAULT_DIRECTION,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
        /************************** PIN 4 *****************************
         ******************* Configured as BUTTON ********************/
        PORT_F,
        PORTF_PF4,
        PORT_PIN_DEFAULT_MODE,
        INPUT,
        PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE,
        PORT_PIN_DEFAULT_INTERNAL_RESISTOR,
        PORT_PIN_DEFAULT_INITIAL_VALUE,
        PORT_PIN_DEFAULT_MODE_CHANGEABLE,
};
