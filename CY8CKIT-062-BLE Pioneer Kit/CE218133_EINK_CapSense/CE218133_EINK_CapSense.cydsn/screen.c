/******************************************************************************
* File Name: screen.c
*
* Version: 1.10
*
* Description: This file contains the functions used to update the screen 
*              according to a touch input.
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
* This file contains the functions used to update the screen according to a
* touch input.
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "screen.h"
#include "screen_contents.h"
#include "display.h"

/* Time in mS during which the logo will be displayed during startup */
#define LOGO_DELAY                      (uint16_t) (3000u)

/* Start and maximum indexes of the main menu items */
#define MAIN_MENU_INDEX_START           (uint8_t) (0x00u)
#define MAIN_MENU_MAX_INDEX             (NUMBER_OF_MAIN_MENU_ITEMS - 1)

/* Start page number of the text pages */
#define TEXT_PAGE_INDEX_START           (uint8_t) (0x00u)

/* Number of screen types consisting of the main menu and text pages */
#define NUMBER_OF_SCREEN_TYPES          (uint8_t) (0x02u)

/* Enumerated data-type for the screen type, consists of main menu and the
   text page */
typedef enum
{
    MAIN_MENU   = 0x00u,
    TEXT_PAGE   = 0x01u
}   screen_type_t;

/* Datatype for the screen information */
typedef struct
{
    screen_type_t   screen;
    uint8_t         menuItem;
    uint8_t         textPage;
}   screen_t;

/* Variable that stores the details of the current screen */
screen_t  currentScreen =
{
    .screen     = MAIN_MENU,
    .menuItem   = MAIN_MENU_INDEX_START,
    .textPage   = TEXT_PAGE_INDEX_START
};

/*******************************************************************************
* Function Name: void InitScreen(void)
********************************************************************************
*
* Summary:
*  Initializes the display and clears the screen
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
void InitScreen(void)
{
    /* Initialize the display hardware and clear to screen to white */
    InitDisplay();
}

/*******************************************************************************
* Function Name: void ShowStartupScreen(void)
********************************************************************************
*
* Summary:
*  Shows the Cypress logo for a certain period of time and then transitions to
*  the default main menu screen
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
void ShowStartupScreen(void)
{
    /* Show the logo */
    DisplayImage((image*) logo);

    /* Keep the logo on for a specific time*/
    Cy_SysLib_Delay(LOGO_DELAY);

    /* Show the default main menu image*/
    DisplayImage((image*) mainMenuImage[MAIN_MENU_INDEX_START]);
}

/*******************************************************************************
* Function Name: void NoScreenChange(void)
********************************************************************************
*
* Summary:
*  Null function that is called by the "ScreenUpdatePointer" function pointer when 
*  no screen change is required
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
void NoScreenChange(void)
{
}

/*******************************************************************************
* Function Name: void GoToTextPage(void)
********************************************************************************
*
* Summary:
*  Performs transition from the main menu to the text page
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
void GoToTextPage(void)
{
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* Change the current screen type to text page */
    currentScreen.screen = TEXT_PAGE;

    /* Re-initialize the text page index to point to the start page */
    currentScreen.textPage = TEXT_PAGE_INDEX_START;
    
    /* Access the index array and fetch the index of the character array that 
      stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
        /* Display the text page with background image */
        DisplayImageAndText((char*) textPage[currentPageIndex],
                            (image*) textPageBackground);
    }
}

/*******************************************************************************
* Function Name: void BackToMainMenu(void)
********************************************************************************
*
* Summary:
*  Returns to the main menu from a text page
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
void BackToMainMenu(void)
{
    /* Change the current screen type to main menu */
    currentScreen.screen = MAIN_MENU;
    
    /* Display the current main menu image */
    DisplayImage((image*) mainMenuImage[currentScreen.menuItem]);
}

/*******************************************************************************
* Function Name: void MoveArrowUp(void)
********************************************************************************
*
* Summary:
*  Moves the selection arrow of the main menu upwards
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
void MoveArrowUp(void)
{
    
    /* If the beginning of the main menu is reached, then move the arrow to the
       final item of the main menu by selecting the maximum index */
    if (currentScreen.menuItem == MAIN_MENU_INDEX_START)
    {
        currentScreen.menuItem = MAIN_MENU_MAX_INDEX;
    }
    /* Otherwise, decrement the index to move the arrow up */
    else
    {
        currentScreen.menuItem--;
    }
    
    /* Display the current main menu image */
    DisplayImage((image*) mainMenuImage[currentScreen.menuItem]);
}

/*******************************************************************************
* Function Name: void MoveArrowDown(void)
********************************************************************************
*
* Summary:
*  Moves the selection arrow of the main menu downwards
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
void MoveArrowDown(void)
{
    
    /* If the final item of the main menu is reached, then move the arrow to the
       beginning of the main menu by selecting the starting index */
    if (currentScreen.menuItem >= MAIN_MENU_MAX_INDEX)
    {
        currentScreen.menuItem = MAIN_MENU_INDEX_START;
    }
    /* Otherwise, increment the index to move the arrow down */
    else
    {
        currentScreen.menuItem++;
    }
    
    /* Display the current main menu image */
    DisplayImage((image*) mainMenuImage[currentScreen.menuItem]);
}

/*******************************************************************************
* Function Name: void PreviousTextPage(void)
********************************************************************************
*
* Summary:
*  Performs the transition to the previous text page
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
void PreviousTextPage(void)
{
    
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* If the start page is reached, then go to the final page by selecting the 
       maximum index of the text pages */
    if (currentScreen.textPage == TEXT_PAGE_INDEX_START)
    {
        currentScreen.textPage = maxTextPageIndexes[currentScreen.menuItem];
    }
    /* Otherwise, select the previous page by decrementing the index */
    else
    {
        currentScreen.textPage--;
    }

    /* Access the index array and fetch the index of the character array that
        stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
        /* Display the text page with background image */
        DisplayImageAndText((char*) textPage[currentPageIndex],
                            (image*) textPageBackground);
    }
}

/*******************************************************************************
* Function Name: void NextTextPage(void)
********************************************************************************
*
* Summary:
*  Performs the transition to the next text page
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
void NextTextPage(void)
{
    /* Variable that stores the index of the character array, which in turn 
       stores the current text page as a string */
    uint8_t currentPageIndex;
    
    /* If the final page is reached, then go to the start page by selecting 
       the starting index of the text pages */
    if (currentScreen.textPage >= maxTextPageIndexes[currentScreen.menuItem])
    {
        currentScreen.textPage = TEXT_PAGE_INDEX_START;
    }
     /* Otherwise, select the next page by incrementing the index */
    else
    {
        currentScreen.textPage++;
    }

    /* Access the index array and fetch the index of the character array that
        stores the current text page as a string */
    currentPageIndex = textPageIndex[currentScreen.menuItem]
                                    [currentScreen.textPage];
    
    /* Check if the fetched index is valid */
    if (currentPageIndex != INVALID_PAGE_INDEX)
    {
        /* Display the text page with background image */
        DisplayImageAndText((char*) textPage[currentPageIndex],
                            (image*) textPageBackground);
    }
}

/*******************************************************************************
* Function Name: void UpdateScreen (touch_data_t touchInfo)
********************************************************************************
*
* Summary:
*  Updates the screen according to the touch_data_t input
*
* Parameters:
*  touch_data_t :  slider swipe / button touched / no touch
*
* Return:
*  None
*
* Side Effects:
*  None
*
*******************************************************************************/
void UpdateScreen(touch_data_t touchInfo)
{
    /* Function pointer that selects a screen update function based on the 
      current screen type and the touch input:
    ________________________________________________________
    |                               |                       |
    |          Parameters           |   Selected Function   |
    |_______________________________|_______________________|
    |                               |                       |
    |   [MAIN_MENU][NO_TOUCH])      |   NoScreenChange      |
    |   [MAIN_MENU][SELECT_BUTTON]) |   GoToTextPage        |
    |   [MAIN_MENU][BACK_BUTTON])   |   NoScreenChange      |
    |   [MAIN_MENU][SLIDER_LEFT])   |   MoveArrowUp         |
    |   [MAIN_MENU][SLIDER_RIGHT])  |   MoveArrowDown       |
    |                               |                       |
    |   [TEXT_PAGE][NO_TOUCH])      |   NoScreenChange      |
    |   [TEXT_PAGE][SELECT_BUTTON]) |   NoScreenChange      |
    |   [TEXT_PAGE][BACK_BUTTON])   |   BackToMainMenu      |
    |   [TEXT_PAGE][SLIDER_LEFT])   |   PreviousTextPage    |
    |   [TEXT_PAGE][SLIDER_RIGHT])  |   NextTextPage        |
    |_______________________________|_______________________|*/
    static void (*ScreenUpdatePointer[NUMBER_OF_SCREEN_TYPES]
                                     [NUMBER_OF_TOUCH_TYPES]) (void) =
    {
        { NoScreenChange, GoToTextPage, NoScreenChange, MoveArrowUp, MoveArrowDown },
        { NoScreenChange, NoScreenChange, BackToMainMenu, PreviousTextPage, NextTextPage }
    };

    /* Flag used to determine when this function is called for the first time */
    bool static firstTime = true;
    
    /* If this function  is called for the first time, show the startup screen,
       and then clear the firstTime flag */
    if (firstTime)
    {
        ShowStartupScreen();
        firstTime = false;
    }
    else
    {
        /* Enter sleep mode if CapSense is busy with a scan. CapSense interrupt 
           will automatically wake up the system */
        if (touchInfo.scanBusy == true)
        {
            Cy_SysPm_Sleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
        }
        /* Otherwise, call the function pointer that selects a screen update function
        according to the touch input */     
        else
        { 
            /* See the function pointer definition for details */
            (*ScreenUpdatePointer[currentScreen.screen][touchInfo.touchType])();
        }
    }
}

/* [] END OF FILE */
