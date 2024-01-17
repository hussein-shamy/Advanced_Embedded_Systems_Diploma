 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Hussein El-Shamy
 ******************************************************************************/

#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/************************************************************************************
 *                             Module Version 1.0.0
 ************************************************************************************/
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)
/************************************************************************************
 *                             AUTOSAR Version 4.0.3
 ************************************************************************************/
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)
/************************************************************************************
 *                        CONFIGURATION PARAMETERS RANGES
 ************************************************************************************/

/* Used as a parameter value for APIs existence*/
#define ENABLE                  (0x01U)         /*The function is enabled (exist)*/
#define DISABLE                 (0x00U)         /*The function is disabled (not exist)*/

/* Used as a parameter value for Port Pin Value */
#define PORT_PIN_LEVEL_HIGH     (0x01U)         /*Port Pin level is High*/
#define PORT_PIN_LEVEL_LOW      (0x00U)         /*Port Pin level is LOW*/

/* Used as a parameter value for Port Pin Mode */
/* Described in details in target specification (23.3 Signals by Function, Except for GPIO)*/
#define PORT_PIN_MODE_DIO       (0U)            /**Port Pin used by DIO*/
#define PORT_PIN_MODE_ADC       (1U)            /**Port Pin used by ADC*/
#define PORT_PIN_MODE_ACMP      (2U)            /*Port Pin used by Analog Comparators*/
#define PORT_PIN_MODE_CAN       (3U)            /**Port Pin used by Controller Area Network*/
#define PORT_PIN_MODE_CORE      (4U)            /*Port Pin used by Core*/
#define PORT_PIN_MODE_GPT       (5U)            /**Port Pin used by General-Purpose Timers*/
#define PORT_PIN_MODE_Hibernate (6U)            /*Port Pin used by Hibernate*/
#define PORT_PIN_MODE_I2C       (7U)            /**Port Pin used by I2C*/
#define PORT_PIN_MODE_JTAG      (8U)            /*Port Pin used by I2C*/
#define PORT_PIN_MODE_PWM       (9U)            /**Port Pin used by PWM*/
#define PORT_PIN_MODE_POWER     (10U)           /*Port Pin used by Power*/
#define PORT_PIN_MODE_QEI       (11U)           /*Port Pin used by QEI*/
#define PORT_PIN_MODE_SSI       (12U)           /**Port Pin used by SSI*/
#define PORT_PIN_MODE_SYSCLK    (13U)           /*Port Pin used by System Control & Clocks*/
#define PORT_PIN_MODE_UART      (14U)           /**Port Pin used by UART*/
#define PORT_PIN_MODE_USB       (15U)           /**Port Pin used by USB*/

 /*******************************************************************************
 *                                PORTA                                         *
 *******************************************************************************/
#define PORTA_PA0                        (0U)
#define PORTA_PA1                        (1U)
#define PORTA_PA2                        (2U)
#define PORTA_PA3                        (3U)
#define PORTA_PA4                        (4U)
#define PORTA_PA5                        (5U)
#define PORTA_PA6                        (6U)
#define PORTA_PA7                        (7U)
 /*******************************************************************************
 *                                PORTB                                         *
 *******************************************************************************/
#define PORTB_PB0                        (8U)
#define PORTB_PB1                        (9U)
#define PORTB_PB2                        (10U)
#define PORTB_PB3                        (11U)
#define PORTB_PB4                        (12U)
#define PORTB_PB5                        (13U)
#define PORTB_PB6                        (14U)
#define PORTB_PB7                        (15U)
 /*******************************************************************************
 *                                PORTC                                         *
 *******************************************************************************/
#define PORTC_PC0                        (16U) /* JTAG1 */
#define PORTC_PC1                        (17U) /* JTAG2 */
#define PORTC_PC2                        (18U) /* JTAG3 */
#define PORTC_PC3                        (19U) /* JTAG4 */
#define PORTC_PC4                        (20U)
#define PORTC_PC5                        (21U)
#define PORTC_PC6                        (22U)
#define PORTC_PC7                        (23U)
 /*******************************************************************************
 *                                PORTD                                         *
 *******************************************************************************/
#define PORTD_PD0                        (24U)
#define PORTD_PD1                        (25U)
#define PORTD_PD2                        (26U)
#define PORTD_PD3                        (27U)
#define PORTD_PD4                        (28U)
#define PORTD_PD5                        (29U)
#define PORTD_PD6                        (30U)
#define PORTD_PD7                        (31U) /* NMI PIN */
 /*******************************************************************************
 *                                PORTE                                         *
 *******************************************************************************/
#define PORTE_PE0                        (32U)
#define PORTE_PE1                        (33U)
#define PORTE_PE2                        (34U)
#define PORTE_PE3                        (35U)
#define PORTE_PE4                        (36U)
#define PORTE_PE5                        (37U)
 /*******************************************************************************
 *                                PORTF                                         *
 *******************************************************************************/
#define PORTF_PF0                        (38U) /* NMI PIN */
#define PORTF_PF1                        (39U)
#define PORTF_PF2                        (40U)
#define PORTF_PF3                        (41U)
#define PORTF_PF4                        (42U)
 /*******************************************************************************
 *                             LOCKING PINS                                     *
 *******************************************************************************/
#define UNLOCK_VALUE                    0x4C4F434B
#define JTAG_PIN1                       PORTC_PC0
#define JTAG_PIN2                       PORTC_PC1
#define JTAG_PIN3                       PORTC_PC2
#define JTAG_PIN4                       PORTC_PC3
#define NMI_PIN1                        PORTF_PF0
#define NMI_PIN2                        PORTD_PD7
 /*******************************************************************************
 *                              PINS IDs                                        *
 *******************************************************************************/
#define PIN0_PIN_NUM                   (uint8)0
#define PIN1_PIN_NUM                   (uint8)1
#define PIN2_PIN_NUM                   (uint8)2
#define PIN3_PIN_NUM                   (uint8)3
#define PIN4_PIN_NUM                   (uint8)4
#define PIN5_PIN_NUM                   (uint8)5
#define PIN6_PIN_NUM                   (uint8)6
#define PIN7_PIN_NUM                   (uint8)7
/************************************************************************************
 *                                  DEFAULT VALUES
 *                   TO ASSIGN THESES VALUES TO UNCONFIGURED PINS
 ***********************************************************************************/
#define PORT_PIN_DEFAULT_DIRECTION                 (INPUT)
#define PORT_PIN_DEFAULT_MODE                      (PORT_PIN_MODE_DIO)
#define PORT_PIN_DEFAULT_INITIAL_VALUE             (STD_LOW)
#define PORT_PIN_DEFAULT_DIRECTION_CHANGEABLE      (PIN_DIRECTION_CHANGEABLE_OFF)
#define PORT_PIN_DEFAULT_MODE_CHANGEABLE           (PIN_MODE_CHANGEABLE_OFF)
#define PORT_PIN_DEFAULT_INTERNAL_RESISTOR         (OFF)
/************************************************************************************
 *                        CONTAINERS AND CONFIGURATION PARAMETERS
 *                                [CONFIGURED PORT PINS]
 *        NOTE: SIMULATING AS I CONFIGRED PINS FROM AUTOSAR CONFIGURAQTION TOOL
 *        - PORTF-PIN1 for LED
 *        - PORTF-PIN4 for BUTTON
 *        So, I created TWO PortPin Containers
 ************************************************************************************
| Module Name          | Port                                                |
| Module Description   | Configuration of the Port module.                   |
| Included Containers  | - PortConfigSet                                     |
|                      | - PortGeneral                                       |
 ***********************************************************************************/

    /************************************************************************************
    | Container Name       | PortGeneral                                         |
    | Description Module   | Wide configuration parameters of the PORT driver.   |
     ***********************************************************************************/

    #define PORT_DEV_ERROR_DETECT                 		(ENABLE)   				 /*Valid Range: enable or disable*/
    #define PORT_SET_PIN_DIRECTION_API             		(ENABLE)   				 /*Valid Range: enable or disable*/
    #define PORT_SET_PIN_MODE_API                  		(ENABLE)   				 /*Valid Range: enable or disable*/
    #define PORT_VERSION_INFO_API                 		(ENABLE)   				 /*Valid Range: enable or disable*/

    /************************************************************************************
    | Container Name       | PortConfigSet                                       |
    | Description Module   | Base of a multiple configuration set.               |
    | Included Containers  | - PortContainer                                     |
     ***********************************************************************************/

        /************************************************************************************
        | Container Name       | PortContainer                                     |
        | Description Module   | Container collecting the PortPins.                |
        | Included Containers  | - PortPin                                         |
        ***********************************************************************************/

        #define PORT_NUMBER_OF_PORT_PINS              (43U)                     /*Valid Range: 0 to 65535*/

            /************************************************************************************
            | Container Name       | PortPin_LED                                                |
            | Description Module   | Configuration of the individual port pins.                 |
            ************************************************************************************/

            #define PORT_PIN_LED_DIRECTION              (OUTPUT)               /*Valid Range: input or output*/
            #define PORT_PIN_LED_DIRECTION_CHANGEABLE   (DISABLE)              /*Valid Range: enable or disable*/
            #define PORT_PIN_LED_ID                     (PIN1_PIN_NUM)         /*Valid Range: 0 to 65535*/
            #define PORT_PIN_LED_INTIAL_MODE            (PORT_PIN_MODE_DIO)    /*Valid Range: one of supported modes above */
            #define PORT_PIN_LED_LEVEL_VALUE            (PORT_PIN_LEVEL_LOW)   /*Valid Range: High or Low*/
            #define PORT_PIN_LED_MODE                   (PORT_PIN_MODE_DIO)    /*Valid Range: one of supported modes above */
            #define PORT_PIN_LED_CHANGEABLE             (DISABLE)              /*Valid Range: enable or disable*/
            /* Non AUTOSAR Configuration*/
            #define PORT_PIN_LED_PORT_ID                (PORT_F)
            #define PORT_PIN_LED_INTERNAL_RESISTOR      (OFF)

            /************************************************************************************
            | Container Name       | PortPin_BUTTON                                             |
            | Description Module   | Configuration of the individual port pins.                 |
            ************************************************************************************/

            #define PORT_PIN_BUTTON_DIRECTION             (INPUT)               /*Valid Range: input or output*/
            #define PORT_PIN_BUTTON_DIRECTION_CHANGEABLE  (DISABLE)             /*Valid Range: enable or disable*/
            #define PORT_PIN_BUTTON_ID                    (PIN4_PIN_NUM)        /*Valid Range: 0 to 65535*/
            #define PORT_PIN_BUTTON_INTIAL_MODE           (PORT_PIN_MODE_DIO)   /*Valid Range: one of supported modes above */
            #define PORT_PIN_BUTTON_LEVEL_VALUE           (PORT_PIN_LEVEL_LOW)  /*Valid Range: High or Low*/
            #define PORT_PIN_BUTTON_MODE                  (PORT_PIN_MODE_DIO)   /*Valid Range: one of supported modes above */
            #define PORT_PIN_BUTTON_CHANGEABLE            (DISABLE)             /*Valid Range: enable or disable*/
            /* Non AUTOSAR Configuration*/
            #define PORT_PIN_BUTTON_PORT_ID               (PORT_F)
            #define PORT_PIN_BUTTON_INTERNAL_RESISTOR     (PULL_UP)

/************************************************************************************************/


#endif /* PORT_CFG_H_ */
