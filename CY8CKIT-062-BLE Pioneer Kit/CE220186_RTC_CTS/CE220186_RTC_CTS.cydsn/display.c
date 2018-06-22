/******************************************************************************
* File Name: display_interface.c
*
* Version: 1.10
*
* Description: This file contains the functions that initialize the E-INK 
*              display and update the display with current time and date
*
* Related Document: CE220186_RTC_CTS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD EINK Display Shield
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
* This file contains the functions that initialize the E-INK display and show
* the current time and date.
*
* For the details of the EINK library functions, see Appendix A of the
* code example document of CE218133 - PSoC 6 MCU E-INK Display with CapSense
*
* For the details of EINK display hardware and driver interface, see the 
* documents available at the following location:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Header file includes */
#include <string.h>
#include "display.h"
#include "screen_contents.h"
#include "real_time_clock.h"

/* Definition of ambient temperature at 25 degree C */
#define AMBIENT_TEMPERATURE             (int8_t) (25)

/* Two frame buffers are used in this project since the E-INK display requires
   the storage of the previous as well as the current frame. See Appendix A
   of the the code example document of CE218133 - PSoC 6 MCU E-INK Display with 
   CapSense for details */
#define NUMBNER_OF_FRAME_BUFFERS        (uint8_t) (0x02u)

/* Enumerated data type used to identify the frame buffers */
typedef enum 
{
    BUFFER0 = (0x00u),
    BUFFER1 = (0x01u)
}   frame_buffer_t;

/* Variable that stores the current frame buffer being used */
frame_buffer_t      currentFrameBuffer  = BUFFER0;

/* Frame buffers used by the display update functions. See Appendix A of the 
   the code example document of CE218133 - PSoC 6 MCU E-INK Display with 
   CapSense for details of frame buffers */
cy_eink_frame_t     frameBuffer[NUMBNER_OF_FRAME_BUFFERS]
                               [CY_EINK_FRAME_SIZE];
/* Variable that stores the pointer to the current frame being displayed */
cy_eink_frame_t*    currentFrame =  CY_EINK_WHITE_FRAME;
 
/* Flag that indicates if a display refresh is required */
bool displayRefreshFlag  = false;

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
    if ( Cy_EINK_Power(CY_EINK_ON)== CY_EINK_OPERATION_SUCCESS)
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
*  Updates the E-INK display with current time and date if conditions permit
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  This function is hard-coded to work with the images, text, and font used in 
*  this code example. Since this is NOT A GENERIC FUNCTION, it NOT RECOMMENDED 
*  to copy/paste this function into a different project to display text and 
*  images on the E-INK display. It is recommended to use the EI-NK library functions 
*  for such use cases. See Appendix A of the the code example document of 
*  CE218133 - PSoC 6 MCU E-INK Display with CapSense for the details of E-INK 
*  library functions.
*******************************************************************************/
void RefreshDisplay (void)
{   
    /* Text fields used in this project start printing from these coordinates. See
       Appendix A of the Cof the the code example document of CE218133 - PSoC 6 
       MCU E-INK Display with CapSense for the details of font size and text 
       coordinate details */
    uint8 const  timeOrigin[]           =   {0x05u, 0x01u};
    uint8 const  dateOrigin[]           =   {0x05u, 0x03u};
    uint8 const  instructionOrigin[]    =   {0x00u, 0x06u};
    
    /* Coordinates at which the background image is cropped before loading to the 
       frame buffers. In this project, the entire frame is copied to initialize the
       frame buffers with calender and clock icons and to clear the text area with
       white pixels */
    uint8 const  fullFrameCoordinates[] =   {0, 33, 0, 175};
    
   /* Variable that stores the time and date strings */
    time_date_strings_t*          timeAndDateBuffer; 

    /* Variable that counts the number of frame buffers initialized */
    uint8 static buffersFilled  = 0u;
    
    /* Variable used for the unconditional refresh at startup */
    bool static firstTime       = true;

    /* Refresh the display if the display refresh flag is set. Also, refresh the 
       display unconditionally at startup */
    if((displayRefreshFlag == true)||(firstTime == true))
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
           and the instructions (text), if both the frame buffers are not initialized
           already */
        if (buffersFilled < NUMBNER_OF_FRAME_BUFFERS)
        {
            Cy_EINK_ImageToFrameBuffer(frameBuffer[currentFrameBuffer],(uint8*)background,
                                       (uint8*)fullFrameCoordinates); 
            Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], (char*)instructions,
                                      CY_EINK_FONT_8X12BLACK, (uint8*)instructionOrigin); 
            buffersFilled++;
        }    
            
        /* Read the strings that store current time and date */
        timeAndDateBuffer = GetRtcStrings();
            
        /* Load the frame buffer with the strings that store current time and date */
        Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                  timeAndDateBuffer->timeSting,
                                  CY_EINK_FONT_16X16BLACK, (uint8_t*)timeOrigin);
        Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                  timeAndDateBuffer->dateString,
                                  CY_EINK_FONT_16X16BLACK, (uint8_t*)dateOrigin);

        /* Update the display with the current date and time */
        Cy_EINK_ShowFrame(currentFrame, frameBuffer[currentFrameBuffer],
                          CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);
        
        /* Store the pointer to the current frame, which is required for subsequent 
           updates */
        currentFrame = frameBuffer[currentFrameBuffer];
        
        /* Clear the conditional flags now that display update is finished */
        displayRefreshFlag = false;
        firstTime = false;
    }
}

/*******************************************************************************
* Function Name: void EnableDisplayRefresh(void)
********************************************************************************
*
* Summary:
*  Function that schedules a display refresh at the next available slot
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
void EnableDisplayRefresh(void)
{
    /* Schedule a display refresh at the next available slot */
    displayRefreshFlag = true;
}

/* [] END OF FILE */
