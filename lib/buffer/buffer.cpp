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
  * Copyright COPYRIGHT 2025 xxx@126.com
  ***************************************************************************************
**/


#include "buffer.h"
TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer = {0}; // Store the state of each sensor
Motor_State motor_state = Stop;

bool is_front = false; // Forward flag
uint32_t front_time = 0; // Forward time
const int EEPROM_ADDR_TIMEOUT = 0;
const uint32_t DEFAULT_TIMEOUT = 30000;
uint32_t timeout = 30000; // Timeout, unit: ms
bool is_error = false; // Error flag, if continuously pushing material for 30s without stopping, consider it an error
String serial_buf;

static HardwareTimer timer(TIM6); // Timeout error

void buffer_init()
{
    buffer_sensor_init();
    buffer_motor_init();
    delay(1000);

    EEPROM.get(EEPROM_ADDR_TIMEOUT, timeout);
    // Determine if the read value is valid (e.g., before first write it's 0xFFFFFFFF or 0)
    if (timeout == 0xFFFFFFFF || timeout == 0)
    {
        timeout = DEFAULT_TIMEOUT;
        EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
        Serial.println("EEPROM is empty");
    }
    else
    {
        Serial.print("read timeout: ");
        Serial.println(timeout);
    }

    timer.pause();
    timer.setPrescaleFactor(48); // 48 prescaler  48000000/48=1000000
    timer.setOverflow(1000); // 1ms
    timer.attachInterrupt(&timer_it_callback);
    timer.resume();
}

void buffer_loop()
{
    uint32_t lastToggleTime = millis(); // Record the last toggle time
    while (1)
    {
        if (millis() - lastToggleTime >= 500) // Every 500ms
        {
            lastToggleTime = millis(); // Record current time
            digitalToggle(ERR_LED);
        }
        // 1. Read the value of each sensor
        read_sensor_state();

#if DEBUG
        buffer_debug();
        while(Serial.available()>0){
        char c=Serial.read();
        serial_buf+=c;
        int pos_enter = -1;
        pos_enter = serial_buf.indexOf("\n");
        if(pos_enter != -1){
            String str=serial_buf.substring(0,pos_enter);
            serial_buf=serial_buf.substring(pos_enter+1);
            if(strstr(str.c_str(),"gconf")!=NULL){
                TMC2208_n::CHOPCONF_t gconf{0};

                // Extract the hexadecimal string after "gconf"
                int pos = str.indexOf("gconf");
                if (pos != -1) {
                    String hexPart = str.substring(pos + 5); // Skip "gconf"
                    hexPart.trim(); // Remove leading and trailing whitespace

                    // Convert string to 32-bit unsigned integer
                    uint32_t hexValue = strtoul(hexPart.c_str(), NULL, 16);

                    // Assign to struct (assign according to your struct definition)
                    gconf.sr = hexValue; // Assume sr is the raw register value field in the struct
                }
                driver.GCONF(gconf.sr);
                Serial.print("write GCONF:0x");
                Serial.println(gconf.sr,HEX);
                Serial.print("read GCONF: 0x");    
                Serial.println(driver.GCONF(),HEX);
                
            }

        }
    }
#else
        motor_control();

        while (Serial.available() > 0)
        {
            char c = Serial.read();
            serial_buf += c;
        }
        if (serial_buf.length() > 0)
        {
            if (serial_buf == "rt")
            {
                Serial.print("read timeout=");
                Serial.println(timeout);
                serial_buf = "";
            }
            else if (serial_buf.startsWith("set"))
            {
                serial_buf.remove(0, 3);
                int64_t num = serial_buf.toInt();
                if (num < 0 || num > 0xffffffff)
                {
                    serial_buf = "";
                    Serial.println("Error: Invalid timeout value.");
                    continue;
                }
                timeout = num;
                EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
                serial_buf = "";
                Serial.print("set succeed! timeout=");
                Serial.println(timeout);
            }
            else
            {
                Serial.println(serial_buf.c_str());
                Serial.println("command error!");
                serial_buf = "";
            }
        }


#endif
    }
}


void buffer_sensor_init()
{
    // Sensor initialization
    pinMode(HALL1,INPUT);
    pinMode(HALL2,INPUT);
    pinMode(HALL3,INPUT);
    pinMode(ENDSTOP_3,INPUT);
    pinMode(KEY1,INPUT);
    pinMode(KEY2,INPUT);

    // Material indicator light initialization
    pinMode(DUANLIAO,OUTPUT);
    pinMode(ERR_LED,OUTPUT);
    pinMode(START_LED,OUTPUT);
}

void buffer_motor_init()
{
    // Motor driver pin initialization
    pinMode(EN_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable driver in hardware

    // Motor driver initialization
    driver.begin(); // UART: Init SW UART (if selected) with default 115200 baudrate
    driver.beginSerial(9600);
    driver.I_scale_analog(false);
    driver.toff(5); // Enables driver in software
    driver.rms_current(I_CURRENT); // Set motor RMS current
    driver.microsteps(Move_Divide_NUM); // Set microsteps to 1/16th
    driver.VACTUAL(STOP); // Set velocity
    driver.en_spreadCycle(true);
    driver.pwm_autoscale(true);
}

/**
  * @brief  Read the state of each sensor
  * @param  NULL
  * @retval NULL
**/
void read_sensor_state(void)
{
    buffer.buffer1_pos1_sensor_state = digitalRead(HALL3);
    buffer.buffer1_pos2_sensor_state = digitalRead(HALL2);
    buffer.buffer1_pos3_sensor_state = digitalRead(HALL1);
    buffer.buffer1_material_swtich_state = digitalRead(ENDSTOP_3);
    buffer.key1 = digitalRead(KEY1);
    buffer.key2 = digitalRead(KEY2);
}

/**
  * @brief  Motor control
  * @param  NULL
  * @retval NULL
**/
void motor_control(void)
{
    static Motor_State last_motor_state = Stop;

    // Button controls motor
    // Button 1 pressed
    if (!digitalRead(KEY1))
    {
        WRITE_EN_PIN(0); // Enable
        driver.VACTUAL(STOP); // Stop

        driver.shaft(BACK);
        driver.VACTUAL(VACTRUAL_VALUE);
        while (!digitalRead(KEY1)); // Wait for release


        driver.VACTUAL(STOP); // Stop
        motor_state = Stop;

        is_front = false;
        front_time = 0;
        is_error = false;
        WRITE_EN_PIN(1); // Disable
    }
    else if (!digitalRead(KEY2)) // Button 2 pressed
    {
        WRITE_EN_PIN(0);
        driver.VACTUAL(STOP); // Stop

        driver.shaft(FORWARD);
        driver.VACTUAL(VACTRUAL_VALUE);
        while (!digitalRead(KEY2));


        driver.VACTUAL(STOP); // Stop
        motor_state = Stop;

        is_front = false;
        front_time = 0;
        is_error = false;
        WRITE_EN_PIN(1);
    }

    // Determine material
    if (digitalRead(ENDSTOP_3))
    {
        // No material, stop motor
        driver.VACTUAL(STOP); // Stop
        motor_state = Stop;

        // Material break pin outputs low level
        digitalWrite(DUANLIAO, 0);

        // Turn off indicator light
        digitalWrite(START_LED, 0);

        is_front = false;
        front_time = 0;
        is_error = false;
        WRITE_EN_PIN(1);


        return; // No material, end
    }

    // Material present, material break pin outputs high level
    digitalWrite(DUANLIAO, 1);

    // Turn on indicator light
    digitalWrite(START_LED, 1);

    // Determine if there is an error
    if (is_error)
    {
        // Stop motor
        driver.VACTUAL(STOP); // Stop
        motor_state = Stop;
        WRITE_EN_PIN(1);
        return;
    }

    // Buffer position record
    if (buffer.buffer1_pos1_sensor_state) // Buffer position is 1, push material forward
    {
        last_motor_state = motor_state; // Record last state
        motor_state = Forward;
        is_front = true;
    }
    else if (buffer.buffer1_pos2_sensor_state) // Buffer position is 2, motor stops rotating
    {
        last_motor_state = motor_state; // Record last state
        motor_state = Stop;
        is_front = false;
        front_time = 0;
    }
    else if (buffer.buffer1_pos3_sensor_state) // Buffer position is 3, retract material
    {
        last_motor_state = motor_state; // Record last state
        motor_state = Back;
        is_front = false;
        front_time = 0;
    }

    if (motor_state == last_motor_state)
        // If last state is the same as this state, no need to send control command again, end this function
        return;

    // Motor control
    switch (motor_state)
    {
    case Forward: // Forward
        {
            WRITE_EN_PIN(0);
            if (last_motor_state == Back) driver.VACTUAL(STOP); // Last was back, stop first then forward
            driver.shaft(FORWARD);
            driver.VACTUAL(VACTRUAL_VALUE);
        }
        break;
    case Stop: // Stop
        {
            WRITE_EN_PIN(1);
            driver.VACTUAL(STOP);
        }
        break;
    case Back: // Back
        {
            WRITE_EN_PIN(0);
            if (last_motor_state == Forward) driver.VACTUAL(STOP); // Last was forward, stop first then back
            driver.shaft(BACK);
            driver.VACTUAL(VACTRUAL_VALUE);
        }
        break;
    }
}

void timer_it_callback()
{
    if (is_front)
    {
        // If pushing forward
        front_time++;
        if (front_time > timeout)
        {
            // If timeout
            is_error = true;
        }
    }
}

void buffer_debug(void)
{
    // Serial.print("buffer1_pos1_sensor_state:");Serial.println(buffer.buffer1_pos1_sensor_state);
    // Serial.print("buffer1_pos2_sensor_state:");Serial.println(buffer.buffer1_pos2_sensor_state);
    // Serial.print("buffer1_pos3_sensor_state:");Serial.println(buffer.buffer1_pos3_sensor_state);
    // Serial.print("buffer1_material_swtich_state:");Serial.println(buffer.buffer1_material_swtich_state);
    // Serial.print("key1:");Serial.println(buffer.key1);
    // Serial.print("key2:");Serial.println(buffer.key2);
    static int i = 0;
    if (i < 0x1ff)
    {
        Serial.print("i:");
        Serial.println(i);
        driver.GCONF(i);
        driver.PWMCONF(i);
        i++;
    }
    uint32_t gconf = driver.GCONF();
    uint32_t chopconf = driver.CHOPCONF();
    uint32_t pwmconf = driver.PWMCONF();
    if (driver.CRCerror)
    {
        Serial.println("CRCerror");
    }
    else
    {
        Serial.print("GCONF():0x");
        Serial.println(gconf,HEX);
        Serial.print("CHOPCONF():0x");
        char buf[11]; // "0x" + 8 digits + null terminator
        sprintf(buf, "%08lX", chopconf); // %08lX -> 8-digit uppercase hexadecimal (long unsigned)
        Serial.println(buf);
        Serial.print("PWMCONF():0x");
        sprintf(buf, "%08lX", pwmconf); // %08lX -> 8-digit uppercase hexadecimal (long unsigned)
        Serial.println(buf);
        Serial.println("");
    }
    delay(1000);
}
