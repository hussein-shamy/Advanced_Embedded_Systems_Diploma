/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Hussein El-Shamy
 * Co-Author: Mohamed Tarek
 *
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/************************************************************************************
 *                                     Vendor ID
 ************************************************************************************/
#define PORT_VENDOR_ID                          (1000U)
/************************************************************************************
 *                                     Module ID
 ************************************************************************************/
#define PORT_MODULE_ID                          (124U)
/************************************************************************************
 *                                Module Instance ID
 ************************************************************************************/
#define PORT_INSTANCE_ID  	                    (0U)
/************************************************************************************
 *                             Module Version 1.0.0
 ************************************************************************************/
#define PORT_SW_MAJOR_VERSION                	(1U)
#define PORT_SW_MINOR_VERSION                 	(0U)
#define PORT_SW_PATCH_VERSION             	    (0U)
/************************************************************************************
 *                             AUTOSAR Version 4.0.3
 ************************************************************************************/
#define PORT_AR_RELEASE_MAJOR_VERSION     	    (4U)
#define PORT_AR_RELEASE_MINOR_VERSION     	    (0U)
#define PORT_AR_RELEASE_PATCH_VERSION     	    (3U)
/************************************************************************************
 *                             Enum for PORT Status
 ************************************************************************************/
typedef enum {
    PORT_NOT_INITIALIZED,
    PORT_INITIALIZED
} Port_Status;
/************************************************************************************
 *                  Version Compatibility Check between included files
 ************************************************************************************/
#include "Std_Types.h"

#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
        ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
        ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Std_Types.h does not match the expected version"
#endif

#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
        ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
        ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
        ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
        ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR file */
#include "Common_Macros.h"

/******************************************************************************
 *                               API Service IDs                                       *
 ******************************************************************************/
#define PORT_INIT_SID                   (uint8)0x00    /* Port_init(); */
#define PORT_SET_PIN_DIRECTION_SID      (uint8)0x01    /* Port_SetPinDirection(); */
#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0x02    /* Port_RefreshPortDirection(); */
#define PORT_GET_VERSION_INFO_SID       (uint8)0x03    /* Port_GetVersionInfo(); */
#define PORT_SET_PIN_MODE_SID           (uint8)0x04    /* Port_SetPinMode()' */
/************************************************************************************
 *                          AUTOSAR SPECIFICATION ERRORS
 ************************************************************************************/
#define PORT_E_PARAM_PIN                (uint8)0x0A   /* Invalid Port Pin ID requested */
#define PORT_E_DIRECTION_UNCHANGEABLE   (uint8)0x0B   /* Port Pin not configured as changeable */
#define PORT_E_PARAM_CONFIG             (uint8)0x0C   /* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_INVALID_MODE       (uint8)0x0D   /* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE        (uint8)0x0E   /* API service called without module initialization */
#define PORT_E_UNINIT                   (uint8)0x0F   /* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER            (uint8)0x1    /* API Port_GetVersionInfo called with a Null Pointer */
/*******************************************************************************
 *                              MODULE DEFINITIONS                             *
 *******************************************************************************/
/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000
/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C
/* PCMx Register*/
#define PMCx_REGISTER_1            		  0x00000001
#define PMCx_REGISTER_2            		  0x00000002
#define PMCx_REGISTER_3            		  0x00000003
#define PMCx_REGISTER_4             	  0x00000004
#define PMCx_REGISTER_7             	  0x00000007
#define PMCx_REGISTER_8             	  0x00000008

#define SHFITING_VALUE_FOUR               4
/*******************************************************************************
 *                              MODULE DATA TYPES                              *
 *******************************************************************************/

/*
 * Port_PinType:
 * ----------------------------------------------------------------------------
 * | Type             | uint
 * ----------------------------------------------------------------------------
 * | Range            | 0 - <number of port pins:>
 * |                  | Shall cover all available port pins. The type should
 * |                  | be chosen for the specific MCU platform (best
 * |                  | performance).
 * ----------------------------------------------------------------------------
 * | Description      | Data type for the symbolic name of a port pin.
 * ----------------------------------------------------------------------------
 */

typedef uint8   Port_PinType;

/*
 * Port_PinModeType:
 * ----------------------------------------------------------------------------
 * | Type             | uint
 * ----------------------------------------------------------------------------
 * | Range            | As several port pin modes shall be configurable on
 * |                  | one pin, the range shall be determined by the
 * |                  | implementation.
 * ----------------------------------------------------------------------------
 * | Description      | Different port pin modes.
 * ----------------------------------------------------------------------------
 */

typedef uint8   Port_PinModeType;

/*
 * Port_PinDirectionType:
 * ----------------------------------------------------------------------------
 * | Type             | Enumeration
 * ----------------------------------------------------------------------------
 * | Range            | PORT_PIN_IN: Sets port pin as input.
 * |                  | PORT_PIN_OUT: Sets port pin as output.
 * ----------------------------------------------------------------------------
 * | Description      | Possible directions of a port pin.
 * ----------------------------------------------------------------------------
 */

typedef enum
{
    INPUT, OUTPUT
} Port_PinDirectionType;

/*
 * Port_InternalResistor:
 * ----------------------------------------------------------------------------
 * | Type             | Enumeration
 * ----------------------------------------------------------------------------
 * | Range            | OFF: No internal resistor.
 * |                  | PULL_UP: Internal pull-up resistor.
 * |                  | PULL_DOWN: Internal pull-down resistor.
 * ----------------------------------------------------------------------------
 * | Description      | Enumeration for different internal resistor options.
 * ----------------------------------------------------------------------------
 */

typedef enum
{
    OFF, PULL_UP, PULL_DOWN
} Port_InternalResistor;

/*
 * Pin_directionChangeable:
 * ----------------------------------------------------------------------------
 * | Type             | Enumeration
 * ----------------------------------------------------------------------------
 * | Range            | PIN_DIRECTION_CHANGEABLE_OFF: Pin direction is not
 * |                  | changeable.
 * |                  | PIN_DIRECTION_CHANGEABLE_ON: Pin direction is
 * |                  | changeable.
 * ----------------------------------------------------------------------------
 * | Description      | Enumeration to indicate whether the pin direction
 * |                  | is changeable or not.
 * ----------------------------------------------------------------------------
 */

typedef enum
{
    PIN_DIRECTION_CHANGEABLE_OFF,
    PIN_DIRECTION_CHANGEABLE_ON
}Pin_directionChangeable;

/*
 * Pin_modeChangeable:
 * ----------------------------------------------------------------------------
 * | Type             | Enumeration
 * ----------------------------------------------------------------------------
 * | Range            | PIN_MODE_CHANGEABLE_OFF: Pin mode is not changeable.
 * |                  | PIN_MODE_CHANGEABLE_ON: Pin mode is changeable.
 * ----------------------------------------------------------------------------
 * | Description      | Enumeration to indicate whether the pin mode is
 * |                  | changeable or not.
 * ----------------------------------------------------------------------------
 */

typedef enum
{
    PIN_MODE_CHANGEABLE_OFF,
    PIN_MODE_CHANGEABLE_ON
}Pin_modeChangeable;

/*
 * Port_ID:
 * ----------------------------------------------------------------------------
 * | Type             | Enumeration
 * ----------------------------------------------------------------------------
 * | Range            | PORT_A, PORT_B, PORT_C, PORT_D, PORT_E, PORT_F
 * ----------------------------------------------------------------------------
 * | Description      | Enumeration representing different port identifiers.
 * ----------------------------------------------------------------------------
 */

typedef enum
{
  PORT_A,PORT_B,PORT_C,PORT_D,PORT_E,PORT_F
}Port_ID;

/*
 * Port_ConfigChannel:
 * ----------------------------------------------------------------------------
 * | Field                     | Type
 * ----------------------------------------------------------------------------
 * | port_num                  | Port_ID
 * ----------------------------------------------------------------------------
 * | pin_num                   | Port_PinType
 * ----------------------------------------------------------------------------
 * | pin_mode                  | Port_PinModeType
 * ----------------------------------------------------------------------------
 * | direction                 | Port_PinDirectionType
 * ----------------------------------------------------------------------------
 * | initial_value             | uint8
 * ----------------------------------------------------------------------------
 * | pin_direction_change      | Pin_directionChangeable
 * ----------------------------------------------------------------------------
 * | pin_mode_change           | Pin_modeChangeable
 * ----------------------------------------------------------------------------
 * | resistor                  | Port_InternalResistor
 * ----------------------------------------------------------------------------
 * | Description               | Configuration structure for a specific port pin.
 * |                           | Contains information such as port identifier, pin
 * |                           | number, pin mode, pin direction, initial value,
 * |                           | changeability of pin direction and mode, and the
 * |                           | internal resistor option.
 * ----------------------------------------------------------------------------
 */

typedef struct
{
    Port_ID port_num;
    Port_PinType pin_num;
    Port_PinModeType pin_mode;
    Port_PinDirectionType direction;
    Pin_directionChangeable pin_direction_change;
    Port_InternalResistor resistor;
    uint8 initial_value;
    Pin_modeChangeable pin_mode_change;
} Port_ConfigChannel;
/*
 * Port_ConfigType:
 * ----------------------------------------------------------------------------
 * | Type             | Structure
 * ----------------------------------------------------------------------------
 * | Dependency       | Hardware Dependent Structure
 * ----------------------------------------------------------------------------
 * | Range            | The contents of the initialization data structure
 * |                  | are specific to the microcontroller.
 * ----------------------------------------------------------------------------
 * | Description      | Represents the external data structure containing
 * |                  | the initialization data for the Port module. It
 * |                  | consists of an array of Port_ConfigChannel
 * |                  | structures, where each element corresponds to a
 * |                  | configured channel for the Port module.
 * ----------------------------------------------------------------------------
 */
typedef struct
{
    Port_ConfigChannel Channel[PORT_NUMBER_OF_PORT_PINS];
} Port_ConfigType;


/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
 * Service Name:     Port_Init
 * Sync/Async:       Synchronous
 * Reentrancy:       Non Reentrant
 * Parameters (in):  - ConfigPtr -> Pointer to the configuration set.
 * Parameters (inout): None
 * Parameters (out):   None
 * Return value:       None
 * Description:        Initializes the Port Driver module.
 ************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr);

#if (PORT_SET_PIN_DIRECTION_API ==STD_ON)
/************************************************************************************
 * Service Name:     Port_SetPinDirection
 * Sync/Async:       Synchronous
 * Reentrancy:       Reentrant
 * Parameters (in):  - Pin       -> Port Pin ID number.
 *                   - Direction -> Port Pin Direction.
 * Parameters (inout): None
 * Parameters (out):   None
 * Return value:       None
 * Description:        Sets the port pin direction.
 ************************************************************************************/
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction );
#endif


/************************************************************************************
 * Service Name:     Port_RefreshPortDirection
 * Sync/Async:       Synchronous
 * Reentrancy:       Non Reentrant
 * Parameters (in):  None
 * Parameters (inout): None
 * Parameters (out):   None
 * Return value:       None
 * Description:        Refreshes port direction.
 ************************************************************************************/
void Port_RefreshPortDirection(void);


#if (PORT_VERSION_INFO_API==STD_ON)
/************************************************************************************
 * Service Name:     Port_GetVersionInfo
 * Sync/Async:       Synchronous
 * Reentrancy:       Non Reentrant
 * Parameters (in):  None
 * Parameters (inout): None
 * Parameters (out):   versioninfo -> Pointer to where to store the version information
 * Return value:       None
 * Description:        Returns the version information of this module.
 ************************************************************************************/
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif


#if (PORT_SET_PIN_MODE_API==STD_ON)
/************************************************************************************
* Service Name:     Port_SetPinMode
* Sync/Async:       Synchronous
* Reentrancy:       Reentrant
* Parameters (in):  - Pin  -> Port Pin ID number.
*                   - Mode -> New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out):   None
* Return value:       None
* Description:        This function sets the port pin mode:
*                     - It modifies the mode of the specified port pin during runtime.
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode);
#endif

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
