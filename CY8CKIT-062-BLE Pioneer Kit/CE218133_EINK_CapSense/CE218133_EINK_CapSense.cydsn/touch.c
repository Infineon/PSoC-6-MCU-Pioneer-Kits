/******************************************************************************
* File Name: touch.c
*
* Version: 1.10
*
* Description: This file contains the functions used to initialize and read touch
*              information from the CapSense sensors 
*
* Related Document: CE218133_EINK_CapSense.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
*******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation. All rights reserved.
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
* This file contains the functions used to initialize and read touch
* information from the CapSense sensors 
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "touch.h"

/* Threshold used for detecting swipes on the slider */
#define SLIDER_SWIPE_THRESHOLD  (uint8_t)(20u)

/* Macro used to clear the variables that track finger position on the slider */
#define CLEAR_POSITION          (uint8_t)(0x00u)

/*******************************************************************************
* Function Name: void InitTouch(void)
********************************************************************************
*
* Summary:
*  Initializes the CapSense touch sensing 
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
void InitTouch(void)
{
    /* Start and initialize the CapSense touch sensing */
    CapSense_Start();
}

/*******************************************************************************
* Function Name: touch_data_t GetTouch(void)
********************************************************************************
*
* Summary:
* Scans the CapSense widgets, performs analysis and returns the touch information
*
* Parameters:
*  None
*
* Return:
*  touch_data_t :  slider swipe / button touched / no touch
*
* Side Effects :
*  None
*
*******************************************************************************/
touch_data_t GetTouch(void)
{
    /* Variables that store the current and the previous slider positions
       and the incremental values of slider position */
    uint8_t static      currSliderPos    = CLEAR_POSITION;
    uint8_t static      prevSliderPos    = CLEAR_POSITION;
    uint8_t static      leftSliderValue  = CLEAR_POSITION;
    uint8_t static      rightSliderValue = CLEAR_POSITION;
    
    /* Variable that stores the status of touch on the slider */
    bool    static      sliderTouched       = false;
    
    /* Flags used to initiate swipe detection */
    bool    static      checkRightSwipe     = false;
    bool    static      checkLeftSwipe      = false;
    
    /* Variable that stores the return data */
    touch_data_t        touchInformation =
    {
        .touchType =    NO_TOUCH,
        .scanBusy  =    false
    };

    /* Do this only when the CapSense isn't busy with a previous operation*/
    if (CapSense_NOT_BUSY == CapSense_IsBusy())
    {
        /* Process data from all the sensors and find out the touch 
           information */
        CapSense_ProcessAllWidgets();
        
        /* Check if the select button (button 0) is touched */
        if (CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID))
        {
            /* Send the select button (button 0)touch detection */
            touchInformation.touchType = SELECT_BUTTON;
        }
        /* Check if the back button (button 1) is touched */
        else if (CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID))
        {
            /* Send the back button (button 1) touch detection */
             touchInformation.touchType = BACK_BUTTON;
        }
        /* Check if the slider is touched */
        else if (CapSense_IsWidgetActive(CapSense_LINEARSLIDER0_WDGT_ID))
        {
            /* If the slider was touched during the previous scan, 
               find out the direction of finger swipe */
            if (sliderTouched)
            {
                /* Backup the slider position from the previous scan and 
                   read the new slider position */
                prevSliderPos = currSliderPos;
                currSliderPos = CapSense_GetCentroidPos
                                (CapSense_LINEARSLIDER0_WDGT_ID);
                
                /* Right swipe detected */
                if ((currSliderPos > prevSliderPos))
                {
                    /* Accumulate the finger travel distance */
                    leftSliderValue += (currSliderPos - prevSliderPos);
                    rightSliderValue = CLEAR_POSITION;
                    
                    /* Report the swipe if the finger travel distance is 
                       above the threshold */
                    if (leftSliderValue > SLIDER_SWIPE_THRESHOLD)
                    {
                        /* Clear the accumulated slider value */
                        leftSliderValue = CLEAR_POSITION;
                        
                        /* If the swipe detection is enabled, store the 
                           detected swipe */
                        if(checkLeftSwipe)
                        {
                            touchInformation.touchType = SLIDER_RIGHT;
                            checkLeftSwipe  = false;
                        }
                        /* Report a "no touch" otherwise */
                        else
                        {
                            touchInformation.touchType = NO_TOUCH;
                            checkRightSwipe  = true;
                        }
                    }
                }
                /* Left swipe detected*/
                else if ((currSliderPos < prevSliderPos))
                {
                    /* Accumulate the finger travel distance */
                    rightSliderValue += (prevSliderPos - currSliderPos);
                    leftSliderValue   = CLEAR_POSITION;
                    
                    /* Report the swipe if the finger travel distance is 
                       above the threshold */
                    if (rightSliderValue > SLIDER_SWIPE_THRESHOLD)
                    {
                        /* Clear the accumulated slider value */
                        rightSliderValue = CLEAR_POSITION;
                        
                        /* If the swipe detection is enabled, store the 
                           detected swipe */
                        if(checkRightSwipe)
                        {
                            touchInformation.touchType = SLIDER_LEFT;
                            checkRightSwipe = false;
                        }
                        /* Report a "no touch" otherwise */
                        else
                        {
                            touchInformation.touchType = NO_TOUCH;
                            checkLeftSwipe  = true;
                        }
                    }
                }
                /* Report that slider is touched */
                sliderTouched = true;
            }
            /* If no previous touch */
            else
            {
                /* Get the current slider position and update the slider touch
                   flag */
                currSliderPos       = CapSense_GetCentroidPos
                                     (CapSense_LINEARSLIDER0_WDGT_ID);
                leftSliderValue     = CLEAR_POSITION;
                rightSliderValue    = CLEAR_POSITION;
                                
                /* Report that slider is touched */
                sliderTouched       = true;
                                
                /* Set the flag to check swipe for the next scan */
                checkLeftSwipe      = true;                
                checkRightSwipe     = true;  
                                
                /* Send a "no touch" for this scan */
                touchInformation.touchType = NO_TOUCH;
            }
        }
        /* No widgets are active */
        else
        {
            /* Clear the slider touch flag and the variables that store the 
               finger position */
            sliderTouched       = false;
            leftSliderValue     = CLEAR_POSITION;
            rightSliderValue    = CLEAR_POSITION;
            
            /* Send a "no touch" for this scan */
            touchInformation.touchType = NO_TOUCH;
        }
        /* Start CapSense scan */
        CapSense_ScanAllWidgets();
    }
    else
    {
        /* Send a "no touch" as CapSense is busy with a previous operation */
         touchInformation.touchType = NO_TOUCH;
         touchInformation.scanBusy  = true;
    }
    
    /* Return the detected touch information */
    return(touchInformation);
}

/* [] END OF FILE */
