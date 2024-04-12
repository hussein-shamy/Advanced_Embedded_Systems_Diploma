 /******************************************************************************
 *
 * Module: ADC
 *
 * File Name: adc.c
 *
 * Description: Source file for the TivaC ADC driver
 *
 * Author: Hussein El-Shamy
 *
 *******************************************************************************/

#include "adc.h"
#include "common_macros.h"
#include "tm4c123gh6pm_registers.h"

/*
 * Description :
 * Function responsible for initialize the ADC driver.
 */
void ADC_init(void){

    //PLL

    /*Enable the ADC clock using the RCGCADC register */
    SET_BIT(SYSCTL_RCGCADC_REG,0);

    /* Enable the clock to the appropriate GPIO modules via the RCGCGPIO register [PORTE] */
    SET_BIT(SYSCTL_RCGCGPIO_REG,4);
    while(!(SYSCTL_PRGPIO_REG & 0x10));

    /* Set the GPIO AFSEL bits for the ADC input pins */
    /* PE0, AIN3 */
    SET_BIT(GPIO_PORTE_AFSEL_REG,0);
    /* PE1, AIN2 */
    SET_BIT(GPIO_PORTE_AFSEL_REG,1);

    /* Configure the AINx signals to be analog inputs by clearing the corresponding DEN bit in the
     *GPIO Digital Enable (GPIODEN) register */
    /* PE0 */
    SET_BIT(GPIO_PORTE_DEN_REG,0);
    /* PE1 */
    SET_BIT(GPIO_PORTE_DEN_REG,1);

    /* Disable the analog isolation circuit for all ADC input pins */
    /* PE0 */
    SET_BIT(GPIO_PORTE_AMSEL_REG,0);
    /* PE1 */
    SET_BIT(GPIO_PORTE_AMSEL_REG,1);


}
/*
 * Description :
 * Function responsible for read analog data from a certain ADC channel
 * and convert it to digital using the ADC driver.
 */
uint16 ADC_readChannel(uint8 channel_num){




}
