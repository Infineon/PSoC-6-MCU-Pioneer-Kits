/******************************************************************************
* File Name: display.c
*
* Version: 1.10
*
* Description: This file contains the functions that initialize the E-INK display
*              and update the display with current time and date
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
/***************************************************************************
* This file contains the functions that initialize the E-INK display and show
* the current time and date.
*
* For the details of the E-INK library functions, see Appendix A of the
* CE218133 - PSoC 6 MCU E-INK Display with CapSense code example document.
*
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following location:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Header file includes */
#include <string.h>
#include "display.h"
#include "screen_contents.h"
#include "motion_sense.h"

/* Definition of ambient temperature at 25 degree C */
#define AMBIENT_TEMPERATURE             (int8_t) (25)

/* Two frame buffers are used in this project since the E-INK display requires
   the storage of the previous as well as the current frame. See Appendix A
   of the CE218133 - PSoC 6 MCU E-INK Display with CapSense code example document for the details 
   of frame buffers */
#define NUMBNER_OF_FRAME_BUFFERS        (uint8_t) (0x02u)

/* Enumerated data type used to identify the frame buffers */
typedef enum frameBuffers
{
    BUFFER0 = (0x00u),
    BUFFER1 = (0x01u)
}   frameBufferType;

/* Variable that stores the current frame buffer being used */
frameBufferType currentFrameBuffer  = BUFFER0;

/* Frame buffers used by the display update functions. See Appendix A of the 
   CE218133 - PSoC 6 MCU E-INK Display with CapSense code example document for details of frame 
   buffers */
cy_eink_frame_t      frameBuffer[NUMBNER_OF_FRAME_BUFFERS]
                           [CY_EINK_FRAME_SIZE];
/* Variable that stores the pointer to the current frame being displayed */
cy_eink_frame_t*     currentFrame        = CY_EINK_WHITE_FRAME;
 
/*******************************************************************************
* Function Name: void InitDisplay(void)
********************************************************************************
*
* Summary:
*  Initializes the E-INK display hardware and clears the display
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
void InitDisplay(void)
{ 
    /* Initialize the E-INK display hardware with the given ambient temperature */
    Cy_EINK_Start(AMBIENT_TEMPERATURE);
    
    /* Power on the display and check if the operation is successful */
    if (Cy_EINK_Power(CY_EINK_ON) == CY_EINK_OPERATION_SUCCESS)
    {
        /* Clear the display to white background */
        Cy_EINK_Clear(CY_EINK_WHITE_BACKGROUND, CY_EINK_POWER_MANUAL);
        Cy_EINK_Power(CY_EINK_OFF);
    }
    /* If the power on operation has failed */
    else
    {
        /* Halt the CPU */
        CY_ASSERT(0u != 0u);
    }
}

/*******************************************************************************
* Function Name: void RefreshDisplay (void)
********************************************************************************
*
* Summary:
*  Updates the E-INK display with current orientation and step count
*
* Parameters:
*  displayInfoType
*
* Return:
*  None
*
* Side Effects:
*  This function is hard-coded to work with the images, text, and font used in 
*  this code example. Since this is NOT A GENERIC FUNCTION, it NOT RECOMMENDED 
*  to copy/paste this function into a different project to display text and 
*  images on the E-INK display. It is recommended to use the E-INK library functions 
*  for such use cases. See Appendix A of the CE218133 - PSoC 6 MCU E-INK Display with CapSense code 
*  example document for the details of E-INK library functions.
*
*******************************************************************************/
void RefreshDisplay (displayInfoType* currentDisplayInfo)
{   
    /* Text fields used in this project start printing from these coordinates. See
       Appendix A of the CE218133 - PSoC 6 MCU E-INK Display with CapSense code example document for the 
       details of font size and text coordinate details */
    uint8 const  stepOrigin[]           =   {0x0Au, 0x03u};
    
    /* Coordinates at which the background image is cropped before loading to the 
       frame buffers. In this project, the entire frame is copied to initialize the
       frame buffers */
    uint8 const  fullFrameCoordinates[]     =   {00, 33, 00, 175};
    
    uint8 const  orientationCoordinates[]   =   {02, 14, 42, 144}; 
    
    /* Variable that counts the number of frame buffers initialized */
    uint8 static buffersFilled  = 0u;
    
    char  static stepCountString[6];
    
    /* Variable used for the unconditional refresh at startup */
    bool static firstTime       = true;

    displayInfoType static prevDisplayInfo = {ORIENTATION_BOTTOM_EDGE, 0x0000};
            
    if ((currentDisplayInfo->orientation != prevDisplayInfo.orientation) ||
        (currentDisplayInfo->stepCount   != prevDisplayInfo.stepCount) || firstTime == true)
    {
       
        /* Two frame buffers are used in this project since the E-INK display requires
           the retention of the previous as well as the current frame. The code below
           finds the frame buffer used in the previous update and selects the other
           frame buffer to be overwritten by the current operation */
        if (currentFrameBuffer == BUFFER0)
        {
            currentFrameBuffer = BUFFER1;
        }
        else
        {
            currentFrameBuffer = BUFFER0;
        }
        
        /* Initialize the current frame buffer being used with the background image
        and the default step count */
        if (buffersFilled < NUMBNER_OF_FRAME_BUFFERS)
        {
            Cy_EINK_ImageToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                        (uint8*)background, 
                                        (uint8*)fullFrameCoordinates); 
            buffersFilled++;
        } 
        
        prevDisplayInfo.orientation = currentDisplayInfo->orientation;
        prevDisplayInfo.stepCount   = currentDisplayInfo->stepCount;
          
        /* Load the frame buffer with the current orientation and stepcount */
        Cy_EINK_ImageToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                (cy_eink_image_t*)orientationImages[currentDisplayInfo->orientation],
                                (uint8_t*)orientationCoordinates);
        sprintf (stepCountString,"%05u",currentDisplayInfo->stepCount);
                
        Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], 
                               stepCountString,
                               CY_EINK_FONT_16X16BLACK,
                               (uint8_t*)stepOrigin);

        /* Update the display with the orientation and step count*/
        Cy_EINK_ShowFrame(currentFrame, frameBuffer[currentFrameBuffer],
                       CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);
        
        /* Store the pointer to the current frame, which is required for subsequent 
           updates */
        currentFrame = frameBuffer[currentFrameBuffer];
        
        firstTime = false;
    }
}

/* [] END OF FILE */
