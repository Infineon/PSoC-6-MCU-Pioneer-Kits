/******************************************************************************
* File Name: display.c
*
* Version: 1.10
*
* Description: This file contains the functions that initialize the E-INK display
* and show the startup/menu images and text pages
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
* This file contains the functions that initialize the E-INK display and show
* the startup/menu images and text pages. This file is an adaptation layer
* between the the screen.c and the low-level E-INK library
*
* For the details of the E-INK library functions, see Appendix A of the
* code example document.
*
* For the details of E-INK display hardware and driver interface, see the 
* documents available at the following location:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Header file includes */
#include "display.h"

/* Definition of ambient temperature at 25 degree C */
#define AMBIENT_TEMPERATURE         (int8_t) (25)

/* Macros used to control the red and blue LEDs */
#define LED_ON                      (uint8_t) (0x00u)
#define LED_OFF                     (uint8_t) (0x01u)

/* Two frame buffers are used in this project since the E-INK display requires
   the storage of the previous as well as the current frame. See Appendix A
   of the code example document for the details of frame buffers */
#define NUMBER_OF_FRAME_BUFFERS     (uint8_t) (2u)

/* Number of startup screens before the main menu image is displayed */
#define NUMBER_OF_STARTUP_SCREENS   (uint8_t) (1u)

/* Macro used to clear firmware counters */
#define CLEAR_COUNTER               (uint8_t) (0x00u)

/* Text pages used in this project start printing from these coordinates. See
   Appendix A of the code example document for the details of text and font */
#define TEXT_PAGE_ORIGIN            {0x00u, 0x00u}

/* Coordinates at which the text page background image is cropped before loading 
   to the frame buffers. This cropping speeds up the initialization of frame 
   buffers by not initializing the area to which the text is written */
#define CROP_BACKGROUND_TOP         {0, 33, 0, 4}
#define CROP_BACKGROUND_BOTTOM      {0, 33, 148, 176}

/* Enumerated data type used to identify the frame buffers */
typedef enum
{
    BUFFER0 = 0x00,
    BUFFER1 = 0x01
}   frame_buffers_t;

/* Enumerated data type used to identify the content of the screen */
typedef enum
{
    MAIN_MENU_CONTENT,
    TEXT_PAGE_CONTENT,
    STARTUP_CONTENT
}   screen_content_t;

/* Variable that stores the current screen content being displayed */
screen_content_t    currentScreenContent = STARTUP_CONTENT;

/* Variable that stores the current frame buffer being used */
frame_buffers_t     currentFrameBuffer   = BUFFER0;

/* Frame buffers used by the display update functions. See Appendix A of the code
   example document for details of frame buffers */
cy_eink_frame_t     frameBuffer[NUMBER_OF_FRAME_BUFFERS][CY_EINK_FRAME_SIZE];

/* Variable that stores the pointer to the current frame being displayed */
cy_eink_frame_t*    currentFrame = CY_EINK_WHITE_FRAME;

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
    if (Cy_EINK_Power(CY_EINK_ON)== CY_EINK_OPERATION_SUCCESS)
    {
        /* Clear the display to white background */
        Cy_EINK_Clear(CY_EINK_WHITE_BACKGROUND, CY_EINK_POWER_MANUAL);
        Cy_EINK_Power(CY_EINK_OFF);
    }
    /* If the power on operation has failed */
    else
    {
        /* Turn the red LED on to indicate that the E-INK display is not 
           detected. Check the connection between the E-INK shield and the 
           Pioneer Baseboard if this happens, and then reset the PSoC 6 BLE */
        Cy_GPIO_Write(LED_Red_PORT, LED_Red_NUM, LED_ON);
        
        /* Halt the CPU */
        CY_ASSERT(0u != 0u);
    }
}

/*******************************************************************************
* Function Name: InitializeFrameBuffers(image* imagePointer)
********************************************************************************
*
* Summary:
*  Initialize the frame buffers with a background image
*
* Parameters:
*  image* imagePointer : pointer to the image
*
* Return:
*  None
*
* Side Effects:
*  This function is hard-coded to work with the images and text used in this code
*  example. The background image is loaded only once to the frame buffer
*  since only one type of background is used in this project. Similarly, the images
*  are cropped at predesignated coordinates before loading to the frame buffers.
*  This cropping speeds up the initialization of frame buffers by not initializing
*  the area to which the text is written. Since this is NOT A GENERIC FUNCTION,
*  it NOT RECOMMENDED to copy/paste this function into a different project to
*  initialize the frame buffer with an image. It is recommended to use the E-INK 
*  library functions for such use cases. See Appendix A of the code example 
*  document for the details of E-INK library functions.
*
*******************************************************************************/
void InitializeFrameBuffers(image* imagePointer)
{
    /* Coordinates at which the text menu images cropped before loading to the
       frame buffers. This cropping speeds up the initialization of the frame 
       buffers by not initializing the area to which the text is written */
    const uint8_t   cropTop[]    = CROP_BACKGROUND_TOP;
    const uint8_t   cropBottom[] = CROP_BACKGROUND_BOTTOM;
    
    /* Flag used to determine when this function is called for the first time */
    bool static     firstTime = true;
    
    /* The background image is loaded only once to the frame buffer since only
       one type of background is used in this project. */
    if (firstTime)
    {
        /* Crop the background image at the predesignated coordinates and  then 
           load to the frame buffers. This cropping speeds up the initialization 
           of the frame buffers by not initializing the area to which the text 
           is written. */
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER0], imagePointer,
                                   (uint8_t*) cropTop);
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER0], imagePointer,
                                   (uint8_t*) cropBottom);
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER1], imagePointer,
                                   (uint8_t*) cropTop);
        Cy_EINK_ImageToFrameBuffer(frameBuffer[BUFFER1], imagePointer,
                                   (uint8_t*) cropBottom);
                                
        /* Clear the first time flag */
        firstTime = false;
    }
}

/*******************************************************************************
* Function Name: void DisplayImage(image* imagePointer)
********************************************************************************
*
* Summary:
*  Displays an image on the E-INK display.
*
* Parameters:
*  image* imagePointer : pointer to the image to be displayed
*
* Return:
*  None
*
* Side Effects:
*  This function selects an update type (partial/full) based on the current
*  screen type. This function also stores the pointer to the previous frame,
*  which is used for subsequent updates. This function is hard-coded to work 
*  with the menu types and image types used in this code example. Since this is 
*  NOT A GENERIC FUNCTION, it NOT RECOMMENDED to copy/paste this function into a 
*  different project to display an image on the E-INK display. It is recommended 
*  to use the Cy_EINK_ShowFrame() library function for such use cases. See 
*  Appendix A of the code example document for the details of E-INK library 
*  functions.
*
*******************************************************************************/
void DisplayImage(image* imagePointer)
{
    /* Counter variable for the startup screen */
    uint8_t static  startupScreenCounter = CLEAR_COUNTER;
    
    /* Turn on the yellow LED to indicate a display update in progress */
    Cy_GPIO_Write(LED_Orange_PORT, LED_Orange_NUM, LED_ON);
    
    /* If the existing image on the display belong to the startup type */
    if (currentScreenContent == STARTUP_CONTENT)
    {
        /* Perform a full update to avoid ghosting as the startup images differ
           significantly from one another */
        Cy_EINK_ShowFrame(currentFrame, imagePointer, CY_EINK_FULL_2STAGE,
                          CY_EINK_POWER_AUTO);
        
        /* Check if the end of startup screen is reached */
        if (startupScreenCounter >= NUMBER_OF_STARTUP_SCREENS)
        {
            /* Change the current screen content to main menu type */
            currentScreenContent = MAIN_MENU_CONTENT;
        }
        else
        {
            /* Increment the startup counter */
            startupScreenCounter++;
        }
    }
    /* If the existing image on the display belong to the startup type */
    else if (currentScreenContent == TEXT_PAGE_CONTENT)
    {
        /* Perform a full update to avoid ghosting as the text pages differ
           significantly from the main menu images */
        Cy_EINK_ShowFrame(currentFrame, imagePointer, CY_EINK_FULL_2STAGE,
                          CY_EINK_POWER_AUTO);
        
        /* Change the current screen content to main menu type */
        currentScreenContent = MAIN_MENU_CONTENT;
    }
    /* If the existing image on the display belong to the main menu type */
    else
    {
        /* Perform a partial update for a fast refresh as the main menu images 
           are similar */
        Cy_EINK_ShowFrame(currentFrame, imagePointer, CY_EINK_PARTIAL, 
                          CY_EINK_POWER_AUTO);
        
        /* Change the current screen content to main menu type */
        currentScreenContent = MAIN_MENU_CONTENT;
    }
    /* Store the pointer to the current image, which is required for subsequent 
       updates */
    currentFrame = imagePointer;
    
    /* Turn off the yellow LED to indicate that display update is finished */
    Cy_GPIO_Write(LED_Orange_PORT, LED_Orange_NUM, LED_OFF);
}

/*******************************************************************************
* Function Name: void DisplayImageAndText(char* text, image* backgroundImage)
********************************************************************************
*
* Summary:
*  Displays a string of text with a background image on the E-INK display
*
* Parameters:
*  char* text                   : pointer to the character string
*  image* backgroundImage       : pointer to the background image
*
* Return:
*  None
*
* Side Effects:
*  This function is hard-coded to work with the images, text, and font used in 
*  this code example. Since this is NOT A GENERIC FUNCTION, it NOT RECOMMENDED 
*  to copy/paste this function into a different project to display text on the 
*  E-INK display. It is recommended to use the E-INK library functions for such 
*  use cases. See Appendix A of the code example document for the details of 
*  E-INK library functions.
*
*******************************************************************************/
void DisplayImageAndText(char* text, image* backgroundImage)
{
    /* Text pages used in this project start printing from these coordinates.
       See Appendix A of the code example document for the details of text and 
       font */
    const uint8_t   textOrigin[] = TEXT_PAGE_ORIGIN;
    
    /* Turn on the Yellow LED to indicate a display update in progress */
    Cy_GPIO_Write(LED_Orange_PORT, LED_Orange_NUM, LED_ON);
    
    /* Initialize the frame buffers with the background image. See the function
       definition for more details */
    InitializeFrameBuffers(backgroundImage);
    
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
    /* Load the frame buffer with the current text Page content */
    Cy_EINK_TextToFrameBuffer(frameBuffer[currentFrameBuffer], text,
                              CY_EINK_FONT_8X12BLACK, (uint8_t*) textOrigin);
    
    /* Perform a full update to avoid ghosting as the text pages differ
       significantly from one another and also from the main menu images */
    Cy_EINK_ShowFrame(currentFrame, frameBuffer[currentFrameBuffer],
                      CY_EINK_FULL_2STAGE, CY_EINK_POWER_AUTO);
    
    /* Store the pointer to the current frame, which is required for subsequent 
       updates */
    currentFrame = frameBuffer[currentFrameBuffer];
    
    /* Change the current screen content to text page type */
    currentScreenContent = TEXT_PAGE_CONTENT;
    
    /* Turn off the yellow LED to indicate that display update is finished */
    Cy_GPIO_Write(LED_Orange_PORT, LED_Orange_NUM, LED_OFF);
}

/* [] END OF FILE */
