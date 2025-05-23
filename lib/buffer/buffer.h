/**
  ***************************************************************************************
  * @file    buffer.cpp
  * @author  lijihu
  * @version V1.0.0
  * @date    2025/05/10
  * @brief   Implements buffer functionality
              *Buffer description
                Photosensor: Blocked 1, Unblocked 0;
                Material switch: Material present 0, No material 1;
                Button: Pressed 0, Released 1;

                Pins:
                HALL1 --> PB2 (Photosensor 3)
                HALL2 --> PB3 (Photosensor 2)
                HALL3 --> PB4 (Photosensor 1)
                ENDSTOP_3 --> PB7(Material switch)
                KEY1 --> PB13(Back)
                KEY2 --> PB12(Forward)
  *
  * @note    
  ***************************************************************************************
  * Copyright COPYRIGHT 2024 xxx@126.com
  ***************************************************************************************
**/

#ifndef __BUFFER_H__
#define __BUFFER_H__

#include <TMCStepper.h>
#include <Arduino.h>
#include <EEPROM.h>

#define HALL1       PB2 // Photosensor 3
#define HALL2       PB3 // Photosensor 2   
#define HALL3       PB4 // Photosensor 1

#define ENDSTOP_3   PB7 // Material switch

#define KEY1        PB13 // Back
#define KEY2        PB12 // Forward

#define EN_PIN      PA6 // Enable
#define DIR_PIN     PA7 // Direction
#define STEP_PIN    PC13 // Step
#define UART        PB1 // Software serial

#define DUANLIAO    PB15 // Material break
#define ERR_LED     PA15 // Indicator light
#define START_LED   PA8  // Indicator light

#define DRIVER_ADDRESS 0b00 // TMC Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

#define SPEED       300  // Speed (unit: r/min)
#define Move_Divide_NUM            ((int32_t)(64))        // (Microsteps per step for flexible part control)
#define VACTRUAL_VALUE (uint32_t)(SPEED*Move_Divide_NUM*200/60/0.715)   // VACTRUAL register value

#define STOP 0                // Stop
#define I_CURRENT (600)        // Current
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN,x)// Enable EN pin
#define FORWARD        1// Material direction
#define BACK        0

#define DEBUG 0

// Define struct to store the state of each sensor in the buffer
typedef struct Buffer
{
    // buffer1
    bool buffer1_pos1_sensor_state;    
    bool buffer1_pos2_sensor_state;        
    bool buffer1_pos3_sensor_state;        
    bool buffer1_material_swtich_state;    
    bool key1;
    bool key2;
    
}Buffer;

// Motor state control enum
typedef enum
{
    Forward=0,// Forward
    Stop,        // Stop
    Back        // Back
}Motor_State;

extern void buffer_sensor_init();
extern void buffer_motor_init();

extern void read_sensor_state(void);
extern void motor_control(void);

extern void buffer_init();
extern void buffer_loop(void);
extern void timer_it_callback();
extern void buffer_debug(void);

extern bool is_error;
extern uint32_t front_time;// Forward time
extern uint32_t timeout;
extern bool is_front;
extern TMC2209Stepper driver;


#endif