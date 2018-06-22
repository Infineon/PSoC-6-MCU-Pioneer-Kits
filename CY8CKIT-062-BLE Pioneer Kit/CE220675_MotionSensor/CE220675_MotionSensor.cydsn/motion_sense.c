/******************************************************************************
* File Name:  motion_sense.c
*
* Version: 1.10
*
* Description: This file includes the functions used to initialize and configure 
*  the motion sensor, setup interrupts and compute motion outputs like orientation 
*  from raw accelerometer data
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
/*****************************************************************************
* This file implements the functions related to initializing, configuring and
* reading outputs from BMI160 motion sensor. This file implements application
* on top of the BMI160 driver provided by Bosch Sensortec GmbH, see bmi160.c 
* for details.
*******************************************************************************/

/* Header file includes */
#include "motion_sense.h"
#include "bmi160.h"
#include "stdlib.h"

/* Instance of BMI160 structure */
struct bmi160_dev sensor;

/*******************************************************************************
* Function Name: int8_t MotionSensor_Init(void)
********************************************************************************
*
* Summary: 
*  Initializes the motion sensor. This initializes the driver, resets the chip
*  and checks connection with the sensor.
*
* Parameters:
*  None
*
* Return:
*  Returns the error status, a non-zero value indicates an error. An error indicates 
*  failure in communicating with the motion sensor.
*
* Side Effects:
*  None
*
*******************************************************************************/
int8_t MotionSensor_Init(void)   
{
    uint8_t rslt        = BMI160_OK;
    /* Start the I2C master interface for BMI160 motion sensor */
    I2Cm_Start();
    sensor.id           = BMI160_I2C_ADDR;
    sensor.interface    = BMI160_I2C_INTF;
    sensor.read         = (bmi160_com_fptr_t)I2C_ReadBytes;
    sensor.write        = (bmi160_com_fptr_t)I2C_WriteBytes;
    sensor.delay_ms     = Cy_SysLib_Delay;
    
    /* Initialize BNI160 sensor */
    rslt = bmi160_init(&sensor);
    
    if(rslt == BMI160_OK)
    {
        /* Select the Output data rate, range of accelerometer sensor */
        sensor.accel_cfg.odr    = BMI160_ACCEL_ODR_100HZ;
        sensor.accel_cfg.range  = BMI160_ACCEL_RANGE_2G;
        sensor.accel_cfg.bw     = BMI160_ACCEL_BW_NORMAL_AVG4;

        /* Select the power mode of accelerometer sensor */
        sensor.accel_cfg.power  = BMI160_ACCEL_NORMAL_MODE;

        /* Select the Output data rate, range of Gyroscope sensor */
        sensor.gyro_cfg.odr     = BMI160_GYRO_ODR_100HZ;
        sensor.gyro_cfg.range   = BMI160_GYRO_RANGE_250_DPS;
        sensor.gyro_cfg.bw      = BMI160_GYRO_BW_NORMAL_MODE;

        /* Select the power mode of Gyroscope sensor */
        sensor.gyro_cfg.power   = BMI160_GYRO_NORMAL_MODE; 

        /* Set the sensor configuration */
        rslt = bmi160_set_sens_conf(&sensor);
    }
    return rslt;
}

/*******************************************************************************
* Function Name: int8_t MotionSensor_ConfigureStepCounter(void)
********************************************************************************
*
* Summary:
*  Configures the motion sensor to count steps. This functions maps INT1 to 
*  provide a pulse on step detection and configures the active level and pulse
*  width. The motion sensor's step counter is enabled and steps can be read
*  using function MotionSensor_ReadSteps.
* 
* Parameters:
*  None
*
* Return:
*  Returns the error status, a non-zero value indicates an error. 
*
* Side Effects:
*  None
*
*******************************************************************************/
int8_t MotionSensor_ConfigureStepCounter(void)
{
    struct bmi160_int_settg int_config;
    uint8_t rslt = BMI160_OK;
    uint8_t step_enable = 1;
    
    /* Map the step interrupt to INT1 pin */
    int_config.int_channel = BMI160_INT_CHANNEL_1;

    /* Select the Interrupt type Step Detector interrupt */
    int_config.int_type = BMI160_STEP_DETECT_INT;
    
    /* Enabling interrupt pins to act as output pin */
    int_config.int_pin_settg.output_en = BMI160_ENABLE;
    
    /*Choosing push-pull mode for interrupt pin */
    int_config.int_pin_settg.output_mode = BMI160_DISABLE;
    
    /* Choosing active High output */
    int_config.int_pin_settg.output_type = BMI160_ENABLE;
    
    /* Choosing edge triggered output */
    int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;
    
    /* Disabling interrupt pin to act as input */
    int_config.int_pin_settg.input_en = BMI160_DISABLE;
    
    /* non-latched output */
    int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;

    /* Select the Step Detector interrupt parameters */
    int_config.int_type_cfg.acc_step_detect_int.step_detector_mode = BMI160_STEP_DETECT_SENSITIVE;
    
    /* Enable step detector */
    int_config.int_type_cfg.acc_step_detect_int.step_detector_en = BMI160_ENABLE;

    /* Set the Step Detector interrupt */
    rslt = bmi160_set_int_config(&int_config, &sensor);
    
    if(rslt == BMI160_OK)
    {
        /* enable the step counter */
        rslt = bmi160_set_step_counter(step_enable,  &sensor);
    }
    
    return rslt;
}


/*******************************************************************************
* Function Name: int8_t MotionSensor_ReadSteps(uint16_t* steps)
********************************************************************************
*
* Summary:
*  Read steps from the step counter. The motion sensor has a filter built-in
*  for counting steps. When steps are detected from an idle condition, the filter 
*  starts counting but starts reporting a change in steps only after 4 consecutive 
*  steps are detected. After 4 consecutive steps are reported, the following steps 
*  are reported normally as they are detected (increments by 1 step).
*
* Parameters:
*  Address of the variable that stores step count
*
* Return:
*  Returns the error status, a non-zero value indicates an error. 
*
* Side Effects:
*  None
*
*******************************************************************************/
int8_t MotionSensor_ReadSteps(uint16_t* steps)
{
    return bmi160_read_step_counter(steps,  &sensor);
}

/*******************************************************************************
* Function Name: int8_t MotionSensor_UpdateOrientation(orientation_t *orientationResult)
********************************************************************************
*
* Summary:
*  Updates orientation status to one of the 6 types, see 'orientation_t'.
*  This functions detects which axis is most perpendicular to ground by
*  looking at the absolute value of acceleration in that axis. The sign
*  of the acceleration signifies whether the axis is facing the ground 
*  or the opposite. 
*
* Parameters:
*  Address of the variable that stores orientation information
*
* Return:
*  Returns the error status, a non-zero value indicates an error. 
*
* Side Effects:
*  None
*
*******************************************************************************/
int8_t MotionSensor_UpdateOrientation(orientation_t *orientationResult)
{
    
    struct bmi160_sensor_data accel;
    int8_t rslt = BMI160_OK; 
    uint16_t absX, absY, absZ;
    
    /* Read x, y, z components of acceleration */
    rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);
    
    /* Get the absolute value of accelerations */
    absX = abs(accel.x);
    absY = abs(accel.y);
    absZ = abs(accel.z);
        
    /* Z axis (perpendicular to face of the display) is most aligned with gravity */
    if ((absZ > absX) && (absZ > absY))
    {
        if (accel.z > 0) 
        {
            /* Display faces down (towards the ground) */
            *orientationResult = ORIENTATION_DISP_DOWN;  
        }
        else 
        {
            /* Display faces up (towards the sky/ceiling) */
            *orientationResult = ORIENTATION_DISP_UP;
        }
    }
    /* Y axis (parallel with shorter edge of board) is most aligned with gravity */
    else if ((absY > absX) && (absY > absZ))
    {
        if (accel.y > 0)
        {
            /* Display has an inverted landscape orientation */
            *orientationResult = ORIENTATION_BOTTOM_EDGE;
        }
        else
        {
            /* Display has landscape orientation */
            *orientationResult = ORIENTATION_TOP_EDGE;
        }
    }
    else /* X axis (parallel with longer edge of board) is most aligned with gravity */
    {
        if (accel.x < 0) 
        {
            /* Display has an inverted portrait orientation */
            *orientationResult = ORIENTATION_RIGHT_EDGE;
        }
        else 
        {
            /* Display has portrait orientation */
            *orientationResult = ORIENTATION_LEFT_EDGE;
        }
    }
    
	return rslt;
}


/* [] END OF FILE */
