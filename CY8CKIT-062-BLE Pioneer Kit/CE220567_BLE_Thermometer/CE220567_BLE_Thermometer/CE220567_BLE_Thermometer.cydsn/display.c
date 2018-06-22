/******************************************************************************
* File Name: display.c
*
* Version: 1.00
*
* Description: This file contains the functions that initialize the E-INK 
*              display and update the display with temperature data
*
* Related Document: CE220567_BLE_Thermometer.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress source code and derivative works for the sole purpose of creating
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited
* without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice.
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products
* where a malfunction or failure may reasonably be expected to result in
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/***************************************************************************
* This file contains the functions that initialize the E-INK display and show
* the current temperature.
*
* For the details of the EINK library functions, see Appendix A of the
* code example document of CE218133 - PSoC 6 MCU E-INK Display with CapSense
*
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following location:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Header file includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "temperature.h"
#include "display.h"
#include "screen_contents.h"
#include "ble_application.h"

/* Two frame buffers are used in this project since the E-INK display requires
   the storage of the previous as well as the current frame. See Appendix A
   of the the code example document of CE218133 - PSoC 6 MCU E-INK Display with 
   CapSense for details */
#define NUMBER_OF_FRAME_BUFFERS        (uint8_t) (2u)
/* Character string size used to store temperature value */
#define TEMPERATURE_STRING_SIZE        (uint8_t) (10u)
/* Macro used to convert the temperature to two digit decimal values */
#define DECIMAL_2DIGIT_SCALER          (uint8_t) (100u)

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
cy_eink_frame_t     frameBuffer[NUMBER_OF_FRAME_BUFFERS]
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
    /* Initialize the E-INK display hardware with the read 
       ambient temperature */
    Cy_EINK_Start(25u);
    
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
        CY_ASSERT(0u);
    }
}

/*******************************************************************************
* Function Name: void RefreshDisplay (void)
********************************************************************************
*
* Summary:
*  Updates the E-INK display with current temperature value if conditions permit
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
*  images on the E-INK display. It is recommended to use the EI-NK library 
*  functions for such use cases. See Appendix A of the the code example document 
*  of CE218133 - PSoC 6 MCU E-INK Display with CapSense for the details of E-INK 
*  library functions.
*******************************************************************************/
void RefreshDisplay (void)
{   
    /* Text fields used in this project start printing from these coordinates. 
       See Appendix A of the code example document of CE218133 - PSoC 6 MCU  
       E-INK Display with CapSense for the details of font size and text 
	   coordinate formats */
    uint8 const  tempOrigin[]           =   {0x02u, 0x01u};
    uint8 const  note1Origin[]          =   {0x15u, 0x00u};
    uint8 const  note2Origin[]          =   {0x15u, 0x01u};
    uint8 const  note3Origin[]          =   {0x15u, 0x02u};
    uint8 const  instructionOrigin[]    =   {0x00u, 0x04u};
    
    /* Coordinates at which the background image is cropped before loading to 
       the frame buffers. In this project, the entire frame is copied to 
       initialize the frame buffers with calender and clock icons and to clear 
       the text area with white pixels */
    uint8 const  fullFrameCoordinates[] =   {0, 33, 0, 175};

    /* Variable that counts the number of frame buffers initialized */
    uint8 static buffersFilled  = 0u;
    
    /* Variable used for the unconditional refresh at startup */
    bool static firstTime       = true;

    /* Refresh the display if the display refresh flag is set. Also, refresh the 
       display unconditionally at startup */
    if(((displayRefreshFlag == true)&&(IsBleconnected()==false))||
        (firstTime == true))
    {
        /* Variable that stores temperature value */
        float temperature;
        /* Variable that store text to be printed */
        char static tempSting[TEMPERATURE_STRING_SIZE];
        /* Variables used to convert the string to printable format */
        int8  decimal;
        int8  fraction;
        int16 temp;
        
        /* Two frame buffers are used in this project since the E-INK display 
           requires the retention of the previous as well as the current frame. 
           The code below finds the frame buffer used in the previous update 
           and selects the other frame buffer to be overwritten by the current
           operation */
        if (currentFrameBuffer == BUFFER0)
        {
            currentFrameBuffer = BUFFER1;
        }
        else
        {
            currentFrameBuffer = BUFFER0;
        }
        
        /* Initialize the current frame buffer being used with the background 
           image and the instructions (text), if both the frame buffers are not 
           initialized already */
        if (buffersFilled < NUMBER_OF_FRAME_BUFFERS)
        {
            char note1[] = "(updated at";
            char note2[] = " one minute";
            char note3[] = " intervals)";
            
            Cy_EINK_ImageToFrameBuffer(frameBuffer[currentFrameBuffer],
                                       (uint8*)background,
                                       (uint8*)fullFrameCoordinates); 
            Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                      (char*)note1, CY_EINK_FONT_8X12BLACK,
                                      (uint8*)note1Origin); 
            Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer],
                                      (char*)note2, CY_EINK_FONT_8X12BLACK,
                                      (uint8*)note2Origin);
            Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer],
                                      (char*)note3, CY_EINK_FONT_8X12BLACK,
                                      (uint8*)note3Origin); 
            Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer],
                                      (char*)instructions, 
                                      CY_EINK_FONT_8X12BLACK,
                                      (uint8*)instructionOrigin); 
            buffersFilled++;
        }    
        
        /* Read the temperature */
        temperature = GetTemperature();
        
        /* Covert temperature data to string */
        temp  = roundf(temperature*DECIMAL_2DIGIT_SCALER);
        decimal  = temp/DECIMAL_2DIGIT_SCALER;
        fraction = (int8_t)(abs(temp%DECIMAL_2DIGIT_SCALER));
        sprintf (tempSting,"%3d.%02d",decimal,fraction);
        
        /* Load the frame buffer with the string that stores temperature data */
        Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], 
                                  tempSting,
                                  CY_EINK_FONT_16X16BLACK, (uint8_t*)tempOrigin);

        /* Update the display with the current frame */
        Cy_EINK_ShowFrame(currentFrame, frameBuffer[currentFrameBuffer],
                          CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);
        
        /* Store the pointer to the current frame, which is required for 
           subsequent updates */
        currentFrame = frameBuffer[currentFrameBuffer];
        
        /* Clear the conditional flags now that display update is finished */
        displayRefreshFlag = false;
        firstTime = false;
    }
}

/*******************************************************************************
* Function Name: void RequestDisplayRefresh(void)
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
void RequestDisplayRefresh(void)
{
    /* Schedule a display refresh at the next available slot */
    displayRefreshFlag = true;
}

/* [] END OF FILE */
