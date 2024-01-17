/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author : Hussein El-Shamy
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"
#include "Det.h"

/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
        || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
        || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

STATIC Port_Status PORT_Status = PORT_NOT_INITIALIZED;

STATIC const Port_ConfigChannel *Port_PortChannels = NULL_PTR;

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

void Port_Init(const Port_ConfigType *ConfigPtr)
{
    /************************************************************************************
     *                      [0] THE DECLERATION OF LOCAL VARIABLES
     * [A]Declaring the local variables that used in this function
     ***********************************************************************************/

    uint8 pinIndex = PORTA_PA0; /* used as index in the loop, starting with the first Pin*/

    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

    /************************************************************************************
     *                          [1] DEVELOPMENT EEROR CHECKING
     * [A]Checking the development error: PORT_E_PARAM_CONFIG
     * [B]And if the error is not exist, change the status of module to initialized status
     * [C]Also use global pointer 'Port_PortChannels' to store the address of the first channel configuration structure
     ***********************************************************************************/

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
        PORT_E_PARAM_CONFIG);
    }
    else
#endif
    {
        /*
         * Set the module state to initialized and point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures
         */
        PORT_Status = PORT_INITIALIZED;
        Port_PortChannels = ConfigPtr->Channel; /* address of the first Channels structure --> Channels[0] */
    }

    /************************************************************************************
     *                          [2] LOOPING TO ALL CHANNELS
     * [A] LOOPING TO ALL CHANNELS to assign the Post-build configurations
     ***********************************************************************************/

    for (pinIndex = PORTA_PA0; pinIndex < PORT_NUMBER_OF_PORT_PINS; pinIndex++)
    {

        /*************************** START OF LOOPING TO ALL CHANNELS **********************/

        /************************************************************************************
         *                          [3] BASE ADDRESS ASSIGNMENT
         * [A] Assigning the base address of the GPIO port to use in manipulating the registers, lately
         ***********************************************************************************/

        switch (Port_PortChannels[pinIndex].port_num)
        {
        case PORT_A:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
        case PORT_B:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
        case PORT_C:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
        case PORT_D:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
        case PORT_E:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
        case PORT_F:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
        }

        /************************************************************************************
         *                              [4] LOCKING AND JTAG PINs
         * [A] Checking the LOCK and JTAG Pins
         ***********************************************************************************/

        if (((Port_PortChannels[pinIndex].port_num == PORT_D)
                && (Port_PortChannels[pinIndex].pin_num == PIN7_PIN_NUM ))
                || ((Port_PortChannels[pinIndex].port_num == PORT_F)
                        && (Port_PortChannels[pinIndex].pin_num == PIN0_PIN_NUM ))) /* PD7 or PF0 */
        {
            /* Unlock the GPIOCR register */
            *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                    + PORT_LOCK_REG_OFFSET) = UNLOCK_VALUE;
            /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);
        }
        else if ((Port_PortChannels[pinIndex].port_num == PORT_C)
                && (Port_PortChannels[pinIndex].pin_num <= PIN3_PIN_NUM )) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
            /* Do Nothing ... No need to unlock the commit register for this pin */
        }

        /************************************************************************************
         *                          [5] DIRECTION AND INTERNAL RESISTORS
         * [A] Configure the Direction of the pin
         * [B] if the direction configuration is INPUT, Configure the INTERNAL RESISTORS (UP,DOWN,OFF)
         ***********************************************************************************/

        if (Port_PortChannels[pinIndex].direction == OUTPUT)
        {
            /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);

            if (Port_PortChannels[pinIndex].initial_value == STD_HIGH)
            {
                /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num);
            }
            else
            {
                /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                CLEAR_BIT(
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num);
            }
        }
        else if (Port_PortChannels[pinIndex].direction == INPUT)
        {
            /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
            CLEAR_BIT(
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);

            if (Port_PortChannels[pinIndex].resistor == PULL_UP)
            {
                /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num);
            }
            else if (Port_PortChannels[pinIndex].resistor == PULL_DOWN)
            {
                /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num);
            }
            else
            {
                /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                CLEAR_BIT(
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num);
                /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                CLEAR_BIT(
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num);
            }
        }
        else
        {
            /* Do Nothing */
        }

        /************************************************************************************
         *                      [6] ANALOG AND DIGITAL FUNCTIONALITY OF THE PIN
         * [A] ENABLE or DISABLE the analog and digital functionality of the pin based on the PB configuration
         ***********************************************************************************/

        if (PORT_PIN_MODE_ADC == Port_PortChannels[pinIndex].pin_mode)
        {
            /************************************ INCASE OF THE ADC  *****************************************/

            /************************** ENABLE THE ANALOG FUNCTIONALITY **************************************
             * Set the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin*
             *************************************************************************************************/
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);

            /************************** DISABLE THE DIGITAL FUNCTIONALITY *************************************
             *  Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin  *
             ***************************************************************************************************/
            CLEAR_BIT(
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);
        }
        else
        {
            /************************** DISABLE THE ANALOG FUNCTIONALITY **************************************
             *Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin*
             ***************************************************************************************************/
            CLEAR_BIT(
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);

            /************************** ENABLE THE DIGITAL FUNCTIONALITY *************************************
             * Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin  *
             **************************************************************************************************/
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);
        }

        /************************************************************************************
         *          [7] ALTERNATIVE FUNCTIONALITY AND CONTROL REGISTER PCMx OF THE PIN
         * [A] ENABLE or DISABLE the alternative functionality of the pin based on the PB configuration
         * [B] IF ENABLE, So Assigning the value of PCMx REGISTER based on the mode of the pin configured in PB structure
         ***********************************************************************************/

        if (PORT_PIN_MODE_DIO == Port_PortChannels[pinIndex].pin_mode)
        {
            /************************************* DISABLE ***************************************/
            /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            CLEAR_BIT(
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);
        }
        else
        {
            /************************************* ENABLE ***************************************/
            /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET),
                    Port_PortChannels[pinIndex].pin_num);
            /************************************************************************************
             * [B] Assigning the value of PCMx register based on the mode of the pin configured in PB structure
             ***********************************************************************************/

            switch (Port_PortChannels[pinIndex].pin_mode)
            {

            /****************************************** CAN *************************************/
            case PORT_PIN_MODE_CAN:

                if (Port_PortChannels[pinIndex].port_num == PORT_F
                        && (Port_PortChannels[pinIndex].pin_num == PIN0_PIN_NUM
                                || Port_PortChannels[pinIndex].pin_num
                                        == PIN3_PIN_NUM ))
                {
                    /************************************************************************
                     *                         with PF0 & PF3 >>> PCMx = 3
                     ***********************************************************************/

                    *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                            + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_3
                            << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));

                }
                else
                {
                    /************************************************************************
                     *          with PA0 & PA1 & PB4 & PB5 & PE4 & PE5 >>> PCMx = 8
                     ***********************************************************************/

                    *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                            + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_8
                            << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));

                }

                break;
                /****************************************** GPT *************************************/
            case PORT_PIN_MODE_GPT:

                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_7
                        << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));

                break;
                /****************************************** I2C *************************************/
            case PORT_PIN_MODE_I2C:

                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_3
                        << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));

                break;

                /****************************************** PWM *************************************/
            case PORT_PIN_MODE_PWM:

                /* NOTE: The SW only support the Motion Control Module 0 */

                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_4
                        << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));

                break;

                /****************************************** SSI *************************************/
            case PORT_PIN_MODE_SSI:

                if (Port_PortChannels[pinIndex].port_num
                        == PORT_D&& Port_PortChannels[pinIndex].pin_num <=PIN3_PIN_NUM)
                {
                    /************************************************************************
                     *                         with PD0 & PD1 & PD2 & PD3 >>> PCMx = 1
                     ***********************************************************************/

                    *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                            + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_1
                            << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));

                }
                else
                {
                    /************************************************************************
                     *                         with other pins
                     ***********************************************************************/
                    *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                            + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_2
                            << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));
                }

                break;

                /****************************************** UART *************************************/
            case PORT_PIN_MODE_UART:

                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_1
                        << (Port_PortChannels[pinIndex].pin_num * SHFITING_VALUE_FOUR));

                break;

            default:

                /* Do nothing */

                break;

                /**************************END OF SWITCH **********************************************/
            }

            /**************************END OF IF CONDITION **********************************************/
        }

        /*************************** END OF LOOPING TO ALL CHANNELS ************************/
    }

    /*************************** END OF THE INTIALZATION FUNCTION **********************/
}

#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
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
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{

    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)

    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == PORT_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
        PORT_SET_PIN_DIRECTION_SID,
                        PORT_E_UNINIT);

    }
    else
    {
        /* No Action Required */
    }

    /* Check if incorrect Port Pin ID passed before using this function */
    if (Pin >= PORT_NUMBER_OF_PORT_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
        PORT_SET_PIN_DIRECTION_SID,
                        PORT_E_PARAM_PIN);

    }
    else
    {
        /* No Action Required */
    }

    /* Check if Port Pin not configured as changeable before using this function */
    if (Port_PortChannels[Pin].pin_direction_change == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
        PORT_SET_PIN_DIRECTION_SID,
                        PORT_E_DIRECTION_UNCHANGEABLE);

    }
    else
    {
        /* No Action Required */
    }

#endif

    switch (Port_PortChannels[Pin].port_num)
    {
    case PORT_A:
        /* PORTA Base Address */
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTA_BASE_ADDRESS;
        break;
    case PORT_B:
        /* PORTB Base Address */
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTB_BASE_ADDRESS;
        break;
    case PORT_C:
        /* PORTC Base Address */
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTC_BASE_ADDRESS;
        break;
    case PORT_D:
        /* PORTD Base Address */
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTD_BASE_ADDRESS;
        break;
    case PORT_E:
        /* PORTE Base Address */
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTE_BASE_ADDRESS;
        break;
    case PORT_F:
        /* PORTF Base Address */
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTF_BASE_ADDRESS;
        break;
    }

    if (Port_PortChannels[Pin].direction == OUTPUT)
    {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),
                Port_PortChannels[Pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

    }
    else if (Port_PortChannels[Pin].direction == INPUT)
    {
        CLEAR_BIT(
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),
                Port_PortChannels[Pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
    }
    else
    {
        /* Do Nothing */
    }
}
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
void Port_RefreshPortDirection(void)
{

    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    uint8 pinIndex = PORTA_PA0;

#if (PORT_DEV_ERROR_DETECT == STD_ON)

    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == PORT_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
        PORT_REFRESH_PORT_DIRECTION_SID,
                        PORT_E_UNINIT);

    }
    else
    {
        /* No Action Required */
    }

#endif

    for (pinIndex = PORTA_PA0; pinIndex <= PORT_NUMBER_OF_PORT_PINS; pinIndex++)
    {

        switch (Port_PortChannels[pinIndex].port_num)
        {
        case 0:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
        case 1:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
        case 2:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
        case 3:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
        case 4:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
        case 5:
            PortGpio_Ptr = (volatile uint32*) GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
        }

        if ((Port_PortChannels[pinIndex].port_num == PORT_C)
                && (Port_PortChannels[pinIndex].pin_num <= PIN3_PIN_NUM )) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */

        }

        if (Port_PortChannels[pinIndex].pin_direction_change == STD_OFF)
        {
            if (OUTPUT == Port_PortChannels[pinIndex].direction)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

            }
            else if (INPUT == Port_PortChannels[pinIndex].direction)
            {
                CLEAR_BIT(
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET),
                        Port_PortChannels[pinIndex].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

            }
            else
            {
                /* Do Nothing */
            }
        }
        else
        {
            /* Do Nothing */

        }

    }

}

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
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if (NULL_PTR == versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
        PORT_GET_VERSION_INFO_SID,
                        PORT_E_PARAM_POINTER);
    }
    else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16) PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16) PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8) PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8) PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8) PORT_SW_PATCH_VERSION;
    }
}

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
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{
    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)

    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == PORT_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
        PORT_E_UNINIT);

    }
    else
    {
        /* No Action Required */
    }

    /* Check if incorrect Port Pin ID passed before using this function */
    if (Pin >= PORT_NUMBER_OF_PORT_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
        PORT_E_PARAM_PIN);

    }
    else
    {
        /* No Action Required */
    }

    /* Check if Port Pin not configured as changeable before using this function */
    if (Port_PortChannels[Pin].pin_direction_change == STD_OFF)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
        PORT_E_MODE_UNCHANGEABLE);

    }
    else
    {
        /* No Action Required */
    }

#endif

    switch (Port_PortChannels[Pin].port_num)
    {
    case 0:
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
        break;
    case 1:
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        break;
    case 2:
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        break;
    case 3:
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        break;
    case 4:
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        break;
    case 5:
        PortGpio_Ptr = (volatile uint32*) GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        break;
    }

    if ((Port_PortChannels[Pin].port_num == PORT_C)
            && (Port_PortChannels[Pin].pin_num <= PIN3_PIN_NUM )) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */

    }

    /************************************************************************************
     *                      [6] ANALOG AND DIGITAL FUNCTIONALITY OF THE PIN
     * [A] ENABLE or DISABLE the analog and digital functionality of the pin based on the PB configuration
     ***********************************************************************************/

    if (PORT_PIN_MODE_ADC == Port_PortChannels[Pin].pin_mode)
    {
        /************************************ INCASE OF THE ADC  *****************************************/

        /************************** ENABLE THE ANALOG FUNCTIONALITY **************************************
         * Set the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin*
         *************************************************************************************************/
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET),
                Port_PortChannels[Pin].pin_num);

        /************************** DISABLE THE DIGITAL FUNCTIONALITY *************************************
         *  Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin  *
         ***************************************************************************************************/
        CLEAR_BIT(
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET),
                Port_PortChannels[Pin].pin_num);
    }
    else
    {
        /************************** DISABLE THE ANALOG FUNCTIONALITY **************************************
         *Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin*
         ***************************************************************************************************/
        CLEAR_BIT(
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET),
                Port_PortChannels[Pin].pin_num);

        /************************** ENABLE THE DIGITAL FUNCTIONALITY *************************************
         * Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin  *
         **************************************************************************************************/
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET),
                Port_PortChannels[Pin].pin_num);
    }

    /************************************************************************************
     *          [7] ALTERNATIVE FUNCTIONALITY AND CONTROL REGISTER PCMx OF THE PIN
     * [A] ENABLE or DISABLE the alternative functionality of the pin based on the PB configuration
     * [B] IF ENABLE, So Assigning the value of PCMx REGISTER based on the mode of the pin configured in PB structure
     ***********************************************************************************/

    if (PORT_PIN_MODE_DIO == Port_PortChannels[Pin].pin_mode)
    {
        /************************************* DISABLE ***************************************/
        /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET),
                Port_PortChannels[Pin].pin_num);
    }
    else
    {
        /************************************* ENABLE ***************************************/
        /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET),
                Port_PortChannels[Pin].pin_num);
        /************************************************************************************
         * [B] Assigning the value of PCMx register based on the mode of the pin configured in PB structure
         ***********************************************************************************/

        switch (Port_PortChannels[Pin].pin_mode)
        {

        /****************************************** CAN *************************************/
        case PORT_PIN_MODE_CAN:

            if (Port_PortChannels[Pin].port_num == PORT_F
                    && (Port_PortChannels[Pin].pin_num == PIN0_PIN_NUM
                            || Port_PortChannels[Pin].pin_num == PIN3_PIN_NUM ))
            {
                /************************************************************************
                 *                         with PF0 & PF3 >>> PCMx = 3
                 ***********************************************************************/

                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_3
                        << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));

            }
            else
            {
                /************************************************************************
                 *          with PA0 & PA1 & PB4 & PB5 & PE4 & PE5 >>> PCMx = 8
                 ***********************************************************************/

                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_8
                        << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));

            }

            break;
            /****************************************** GPT *************************************/
        case PORT_PIN_MODE_GPT:

            *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                    + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_7
                    << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));

            break;
            /****************************************** I2C *************************************/
        case PORT_PIN_MODE_I2C:

            *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                    + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_3
                    << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));

            break;

            /****************************************** PWM *************************************/
        case PORT_PIN_MODE_PWM:

            /* NOTE: The SW only support the Motion Control Module 0 */

            *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                    + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_4
                    << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));

            break;

            /****************************************** SSI *************************************/
        case PORT_PIN_MODE_SSI:

            if (Port_PortChannels[Pin].port_num
                    == PORT_D&& Port_PortChannels[Pin].pin_num <=PIN3_PIN_NUM)
            {
                /************************************************************************
                 *                         with PD0 & PD1 & PD2 & PD3 >>> PCMx = 1
                 ***********************************************************************/

                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_1
                        << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));

            }
            else
            {
                /************************************************************************
                 *                         with other pins
                 ***********************************************************************/
                *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                        + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_2
                        << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));
            }

            break;

            /****************************************** UART *************************************/
        case PORT_PIN_MODE_UART:

            *(volatile uint32*) ((volatile uint8*) PortGpio_Ptr
                    + PORT_CTL_REG_OFFSET) |= (PMCx_REGISTER_1
                    << (Port_PortChannels[Pin].pin_num * SHFITING_VALUE_FOUR));

            break;

        default:

            /* Do nothing */

            break;

            /**************************END OF SWITCH **********************************************/
        }

        /**************************END OF IF CONDITION **********************************************/
    }

}
#endif
