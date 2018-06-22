/******************************************************************************
* File Name: cy_eink_psoc_encapsulation.c
*
* Version: 1.10
*
* Description: This file contains functions that encapsulate PSoC Component APIs
*              or Peripheral Driver Library APIs.
*
* Hardware Dependency: CY8CKIT-028-EPD E-INK Display Shield
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
* This file contains functions that encapsulate PSoC Component APIs or Peripheral
* Driver Library APIs. Functions defined in this file are used by the 
* cy_eink_hardware_driver.c.
*
* For the details of the E-INK display and library functions, see the code  
* example document of CE218133 - PSoC 6 MCU E-INK Display with CapSense
*
* For the details of E-INK display control and communication protocols, see the
* driver document available at the following location:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Header file includes */
#include "cy_eink_psoc_interface.h"

/*******************************************************************************
* Function Name: void Cy_EINK_TimerInit(void)
********************************************************************************
*
* Summary:
*  This function clears and initializes the E-INK Timer
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_TimerInit(void)
{   
    /* Clear the counter value and the counter variable */
    CY_EINK_Timer_SetCounter(0);  
    
    /* Initialize the Timer */
    CY_EINK_Timer_Start();
}

/*******************************************************************************
* Function Name: uint32_t Cy_EINK_GetTimeTick(void)
********************************************************************************
*
* Summary:
*  This function returns the value of current time tick of E-INK Timer.
*
* Parameters:
*  None
*
* Return:
*  uint32_t : current value of time tick
*
* Side Effects:
*  None
*******************************************************************************/
uint32_t Cy_EINK_GetTimeTick(void)
{
    /* Variable used to store the time tick */
    uint32_t timingCount;
    
    /* Read the current time tick from the E-INK Timer */
    timingCount = CY_EINK_Timer_GetCounter();

    /* Return the current value of time tick */
    return(timingCount);
}

/*******************************************************************************
* Function Name: void Cy_EINK_TimerStop(void)
********************************************************************************
*
* Summary:
*  Stops the E-INK Timer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_TimerStop(void)
{
    /* Stop the E-INK Timer */
    CY_EINK_Timer_Disable();
}

/*******************************************************************************
* Function Name: void Cy_EINK_InitSPI(void)
********************************************************************************
*
* Summary:
*  Initializes the SPI block that communicates with the E-INK display.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_InitSPI(void)
{
    /* Start the SPI master */
    Cy_SCB_SPI_Init(CY_EINK_SPIM_HW, &CY_EINK_SPIM_config, &CY_EINK_SPIM_context);
    Cy_SCB_SPI_Enable(CY_EINK_SPIM_HW);
    
    /* Make the chip select HIGH */
    CY_EINK_CsHigh;
}

/*******************************************************************************
* Function Name: void Cy_EINK_AttachSPI(void)
********************************************************************************
*
* Summary:
*  Attaches the SPI master to the E-INK display driver.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_AttachSPI(void)
{
    /* Make the chip select HIGH */
    CY_EINK_CsHigh;
    
    /* Start the SPI block */
    Cy_SCB_SPI_Enable(CY_EINK_SPIM_HW);
}

/*******************************************************************************
* Function Name: void Cy_EINK_DetachSPI(void)
********************************************************************************
*
* Summary:
*  Detaches the SPI master from the E-INK display driver.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_DetachSPI(void)
{
    /* Stop the SPI master */
    Cy_SCB_SPI_Disable(CY_EINK_SPIM_HW, &CY_EINK_SPIM_context);
}

/*******************************************************************************
* Function Name: void Cy_EINK_WriteSPI(uint8_t data)
********************************************************************************
*
* Summary:
*  Send a byte of data to the E-INK display driver via SPI.
*
* Parameters:
*  uint8_t data : data byte that need to be transmitted
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_WriteSPI(uint8_t data)
{
    /* Send one byte of data */
    Cy_SCB_SPI_Write(CY_EINK_SPIM_HW, data);
    
    /* Wait for RX buffer to get filled with the dummy data from E-INK driver */
    while ( CY_SCB_SPI_RX_NOT_EMPTY != (Cy_SCB_SPI_GetRxFifoStatus
           (CY_EINK_SPIM_HW) & CY_SCB_SPI_RX_NOT_EMPTY))
    {
    }
    /* Clear the TX and RX buffers */
    Cy_SCB_SPI_ClearTxFifo(CY_EINK_SPIM_HW);
    Cy_SCB_SPI_ClearRxFifo(CY_EINK_SPIM_HW);
    Cy_SCB_SPI_ClearRxFifoStatus(CY_EINK_SPIM_HW, CY_SCB_SPI_RX_NOT_EMPTY);
}

/*******************************************************************************
* Function Name: Cy_EINK_ReadSPI(uint8_t data)
********************************************************************************
*
* Summary:
*  Read a byte of data from the E-INK display driver via SPI.
*
* Parameters:
*  uint8_t data : command that need to be transmitted
*
* Return:
*  uint8_t : received data
*
* Side Effects:
*  None
*******************************************************************************/
uint8_t Cy_EINK_ReadSPI(uint8_t data)
{
    /* Variable used to store the return data*/
    uint8_t readData;
    
    /* Send a command to the E-INK driver */
    Cy_SCB_SPI_Write(CY_EINK_SPIM_HW, data);
    
    /* Wait for RX buffer to get filled with a byte of data */
    while ( CY_SCB_SPI_RX_NOT_EMPTY != (Cy_SCB_SPI_GetRxFifoStatus
           (CY_EINK_SPIM_HW) & CY_SCB_SPI_RX_NOT_EMPTY))
    {
    }
    /* Read one byte of the RX data */
    readData = Cy_SCB_SPI_Read(CY_EINK_SPIM_HW);
    
    /* Clear the RX and TX buffers */
    Cy_SCB_SPI_ClearTxFifo(CY_EINK_SPIM_HW);
    Cy_SCB_SPI_ClearRxFifo(CY_EINK_SPIM_HW);
    Cy_SCB_SPI_ClearRxFifoStatus(CY_EINK_SPIM_HW, CY_SCB_SPI_RX_NOT_EMPTY);
    
    /* Return received byte */
    return(readData);
}

/*******************************************************************************
* Function Name: uint8_t Cy_EINK_WriteReadSPI(uint8_t data)
********************************************************************************
*
* Summary:
*  Write a byte of data to the E-INK driver and read a return byte.
*
* Parameters:
*  uint8_t data : data byte that need to be transmitted
*
* Return:
*  uint8_t: received data byte
*
* Side Effects:
*  None
*******************************************************************************/
uint8_t Cy_EINK_WriteReadSPI(uint8_t data)
{
    /* Send a byte of data to the E-INK driver */
    Cy_SCB_SPI_Write(CY_EINK_SPIM_HW, data);
    
    /* Wait for the SPI to finish the operation */
    while (Cy_SCB_SPI_IsBusBusy(CY_EINK_SPIM_HW));
    
    /* Clear the TX buffer */
    Cy_SCB_SPI_ClearTxFifo(CY_EINK_SPIM_HW);
    
    /* Check for received data */
    if (CY_SCB_SPI_RX_NOT_EMPTY == (Cy_SCB_SPI_GetRxFifoStatus(CY_EINK_SPIM_HW)
                                    & CY_SCB_SPI_RX_NOT_EMPTY))
    {
        /* Read one byte of the RX data */
        data = Cy_SCB_SPI_Read(CY_EINK_SPIM_HW);
        
        /* Clear the RX buffer */
        Cy_SCB_SPI_ClearRxFifo(CY_EINK_SPIM_HW);
        Cy_SCB_SPI_ClearRxFifoStatus(CY_EINK_SPIM_HW, CY_SCB_SPI_RX_NOT_EMPTY);
    }
    /* Return received byte */
    return(data);
}

/*******************************************************************************
* Function Name: bool CY_EINK_IsBusy(void)
********************************************************************************
*
* Summary:
*  Check if the E-INK display is busy.
*
* Parameters:
*  None
*
* Return:
*  bool : True if the E-INK display is buy, False otherwise
*
* Side Effects:
*  None
*******************************************************************************/
bool Cy_EINK_IsBusy(void)
{
    /* Return the status of CY_EINK_DispBusy pin */
    if (Cy_GPIO_Read(CY_EINK_DispBusy_PORT, CY_EINK_DispBusy_NUM))
    {
        return(true);
    }
    else
    {
        return(false);
    }
}

/* [] END OF FILE */
