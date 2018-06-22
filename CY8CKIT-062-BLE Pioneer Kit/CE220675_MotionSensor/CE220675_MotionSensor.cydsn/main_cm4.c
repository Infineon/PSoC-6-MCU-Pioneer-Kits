/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.10
*
* Description: This project demonstrates interfacing PSoC® 6 MCU with a 
*  BMI160 motion sensor. 
*
* Related Document: CE220675_MotionSensor.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
/******************************************************************************
* The CY8CKIT-028-EPD E-INK display Shield contains a BMI160 motion sensor.
* This example interfaces to the sensor to read motion data. 
*
* The example reads steps counted by the sensor to emulate a pedometer. 
* Interrupts are used to detect and read the step count.
* Raw motion data is read and converted to estimate the orientation of the board.

*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "motion_sense.h"
#include "display.h"
#include "stdio.h"
#include "stdio_user.h"

/* Flag set by motion sensor's step detection interrupt */
uint8_t motionStepsChanged = 0;

/*******************************************************************************
* Function Name: MotionSensor_Interrupt
********************************************************************************
* Summary:
*  This function is registered to be called when a rising edge is detected on 
*  motion interrupt pin (Pin_MotionInterrupt_INT1). This is connected to the 
*  INT1 pin of BMI160 which is configured to trigger an interrupt with a change
*  in step counts. Use MotionSensor_ReadSteps function to read the current 
*  step count when this interrupt is triggered.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void MotionSensor_Interrupt(void)
{
    /* Clear the source of interrupt (GPIO) */
    Cy_GPIO_ClearInterrupt(Pin_MotionInterrupt_INT1_PORT, Pin_MotionInterrupt_INT1_NUM);
    
    /* Set flag to indicate that steps have changed */
    motionStepsChanged = 1;
    
    /* Clear pending IRQ status on CPU */
    NVIC_ClearPendingIRQ(SysInt_MotionINT_cfg.intrSrc);
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  Main function for the project.
*
* Parameters:
*  None
*
* Return:
*  int
*
* Theory:
* The main function for the Cortex-M0+ CPU does the following:
*  Initialization:
*   - Initialize E-INK display interface
*   - Initialize BMI160 motion sensor (I2C)
*   - Initialize debug output (UART)
*   - Configure BMI160 motion sensor and enable step detector interrupt
*   
*  Do forever loop: 
*   - Check step detection interrupt flag and update step counts
*   - Read motion data (accelerometer) and compute orientation
*   - Update the E-INK display with the motion information
*
*******************************************************************************/
int main(void)
{
    /* Return status for motion sensor function calls */
    int8 motionStatus = 0;
    
    /* Variables used for display information */
    displayInfoType motionInfo;
    
    /* Initial orientation state: Top edge of the board points towards the ceiling */
    motionInfo.orientation = ORIENTATION_TOP_EDGE;   
    
    /* Reset initial step counts to 0 */
    motionInfo.stepCount = 0;
        
    /* Enable global interrupts. */
    __enable_irq(); 
    
    /* Start the UART debug output */
    UART_DEBUG_Start();
    printf("\x1b[2J\x1b[;H"); /* Clear terminal display */
    printf("PSoC 6 MCU: Motion Sensor\r\n");
        
    /* Initialize the motion sensor */
    motionStatus = MotionSensor_Init();
    if (motionStatus != STATUS_SUCCESS)
    {
        printf("Failed to initialize motion sensor. Check connection. \r\n");    
        CY_ASSERT(0u);  /* Halt the program */
    }
    
    /* Configure the motion sensor */
    motionStatus = MotionSensor_ConfigureStepCounter();
    
    if (motionStatus != STATUS_SUCCESS)
    {
        printf("Failed to configure motion sensor\r\n");    
        CY_ASSERT(0u);  /* Halt the program */
    }
    
    /* Enable motion sensor interrupt */
    Cy_SysInt_Init(&SysInt_MotionINT_cfg, MotionSensor_Interrupt);
    NVIC_EnableIRQ(SysInt_MotionINT_cfg.intrSrc);
    
    /* Initialize the E-INK display */
    InitDisplay();
    
    for(;;) 
    {   
        /* Check for step detection interrupt from motion sensor and update */
        if (motionStepsChanged)
        {
            /* Reset the steps changed flag */
            motionStepsChanged = 0;
            
            /* Read the current value of step counter from the motion sensor register */
            motionStatus = MotionSensor_ReadSteps(&motionInfo.stepCount);
            printf("Steps = %d \r\n", motionInfo.stepCount);
            
            /* Toggle green LED with every step */
            Cy_GPIO_Inv(StepDetected_LED_PORT, StepDetected_LED_NUM);
        }

        /* Read the motion sensor and compute the current orientation */
        MotionSensor_UpdateOrientation(&motionInfo.orientation);
        
        /* Call function to refresh the E-INK display. The refresh happens only if there is a change in orientation / step count */
        RefreshDisplay(&motionInfo);
	}   
} 

/* [] END OF FILE */
