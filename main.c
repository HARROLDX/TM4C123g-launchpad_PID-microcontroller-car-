/**
 * main.c
 */
#define MAX_DUTY 2000
#include <accel_6050.h>
#include <stdint.h>
#include "tm4c123.h"
#include "serial.h"
#include "i2c.h"
//************************************************************
//  GLOBAL VARS
//************************************************************
int16_t accel_x,accel_y,accel_z,temp,gyro_x,gyro_y,gyro_z;//int 16 variables to read accelxyz,temperature,gyroxyz
volatile uint32_t milliseconds=0;//systickhandler use it to count time

#define DUTYSCALER 1000
#define PWM_MIN                 60
#define PWM_MAX                 95
// 主电机PID控制器状态变量
int32_t MainDuty = 0;                   // 输出的PWM占空比
int32_t d_lastError_main = 0;           // 上一次的误差值（用于微分计算）
int32_t i_sum_main = 0;                 // 误差累积和（用于积分计算）
int32_t PWM_last_main = 0;              // 上一次的PWM输出值

// 尾电机PID控制器状态变量
int32_t TailDuty = 0;                   // 输出的PWM占空比
int32_t d_lastError_tail = 0;           // 上一次的误差值
int32_t i_sum_tail = 0;                 // 误差累积和
int32_t PWM_last_tail = 0;              // 上一次的PWM输出值
//  GPIO
//************************************************************
void enableGPIO(unsigned int PortNumber);//use GPIO
void pinMode(unsigned int PortNumber, unsigned int bit, unsigned int mode);//configure pin
//PWM
void initMotorPWM(void);//init PWM
//************************************************************
//  CARS
//************************************************************
void stop(void);//stop car
void forwards(uint32_t duty_cycle);//forward car
void turnright(uint32_t duty_cycle);//turnright
void turnleft(uint32_t duty_cycle);//turnleft
void backwards(uint32_t duty_cycle);//backwards
void setLeft(uint32_t duty_cycle);//only left dutycycle
void setRight(uint32_t duty_cycle);//only right dutycycle
//************************************************************
//  SYSTICK
//************************************************************
void delay(uint32_t ms);
void initSysTick()//tm4c123gh!!!!!!!!!!!!!!!!!!!!!!!
{
    // Default power up clock status is as follows:
    // PIOSC runs at 16MHz.  This supplies the system clock
    // The systick timer is driven by PIOSC/4 (4MHz)
    // enable systick and its interrupts
    SYS_STCTRL = SYS_STCTRL | ( (1 << 1) | (1 << 0)); // enable and allow interrupts
    SYS_STRELOAD=4000-1; // generate 1 millisecond time base
    SYS_STCURRENT=10;    // Start the counter close to zero so we get an interrupt early
    enable_interrupts(); // Global interrupt enable

}
void SysTick_Handler();
//************************************************************
//  ENCODER(QEI)
//************************************************************
void initQEI0(void);
void initQEI1(void);
int getQEI0Position();
int getQEI1Position();
int getQEI0Velocity();
int getQEI1Velocity();
//********************************************************
// PID controller for the main motor
//************************************************************
uint32_t Leftcontroller(int32_t, int32_t);
uint32_t Rightcontroller(int32_t,int32_t);
//************************************************************
//  MAIN
//************************************************************


int main(void)
{

    initSysTick();//init systick
    enable_interrupts();//enable interrupts to make a clock(systick timeout interrupt)
    initSerial(9600);//set baudrates
       initQEI0();
       initQEI1();
       initMotorPWM();
       stop();
       while(1)
       {
           eputs("Velocity0 : ");//9600---编码器测速
           eputd(getQEI0Velocity());
           eputs("   Position0 : ");
           eputd(getQEI0Position());
           eputs("\n\r");
           eputs("Velocity1 : ");
           eputd(getQEI1Velocity());
           eputs("   Position1 : ");
           eputd(getQEI1Position());
           eputs("\n\r");

           MainDuty = Leftcontroller(2000, getQEI1Velocity());
//         TailDuty = Rightcontroller(2000, getQEI0Velocity());
           forwards(MainDuty);
           delay(5000);

           forwards(80);
           delay(3000);
           setLeft(MainDuty*0.8);
           delay(3000);

           turnleft(80);
           delay(3000);

           turnright(80);
           delay(3000);

           backwards(80);
           delay(3000);
           delay(500);
       }
}


void SysTick_Handler()
{
    milliseconds++;//ms++  毫秒
}

void delay(uint32_t ms)
{
    uint32_t end_time=milliseconds + ms;
    while(milliseconds < end_time)
    {
        asm(" wfi "); // break！
    }
}

void enableGPIO(unsigned int PortNumber)
{
    if (PortNumber < 6)
    {
        SYSCTL_RCGC2 = SYSCTL_RCGC2 | (1 << PortNumber); // turn on GPIO
        SYSCTL_GPIOHBCTL = SYSCTL_GPIOHBCTL | (1 << PortNumber); // turn on AHB access to GPIO
    }
}


void stop(void)
{
    GPTM0_32_TAMATCHR = 0;
    GPTM0_32_TBMATCHR = 0;
    GPTM1_32_TAMATCHR = 0;
    GPTM1_32_TBMATCHR = 0;
}
void forwards(uint32_t duty_cycle)
{

    GPTM0_32_TAMATCHR = 0;
        GPTM0_32_TBMATCHR = duty_cycle*MAX_DUTY/100;
        GPTM1_32_TAMATCHR = duty_cycle*MAX_DUTY/100;
    GPTM1_32_TBMATCHR = 0;
}
void turnleft(uint32_t duty_cycle)
{
    GPTM0_32_TAMATCHR = 0;
            GPTM0_32_TBMATCHR = 0;
            GPTM1_32_TAMATCHR = duty_cycle*MAX_DUTY/100;
        GPTM1_32_TBMATCHR = 0;
}
void turnright(uint32_t duty_cycle)
{
    GPTM0_32_TAMATCHR = 0;
            GPTM0_32_TBMATCHR = duty_cycle*MAX_DUTY/100;
            GPTM1_32_TAMATCHR = 0;
        GPTM1_32_TBMATCHR = 0;
}
void backwards(uint32_t duty_cycle)
{
    GPTM0_32_TAMATCHR = duty_cycle*MAX_DUTY/100;
    GPTM0_32_TBMATCHR = 0;
    GPTM1_32_TAMATCHR = 0;
    GPTM1_32_TBMATCHR = duty_cycle*MAX_DUTY/100;
}
void setRight(uint32_t duty_cycle)
{
    GPTM0_32_TAMATCHR = 0;
    GPTM0_32_TBMATCHR = duty_cycle*MAX_DUTY/100;
}
void setLeft(uint32_t duty_cycle)
{
    GPTM1_32_TAMATCHR = duty_cycle*MAX_DUTY/100;
    GPTM1_32_TBMATCHR = 0;
}
void initQEI1(void)
{
    // Quadrature encoder is connected to PC5 and PC6
    // These correspond to PhaseA1 and PhaseB1
    SYSCTL_RCGCQEI |= (1 << 1); // enable the clock for QEI1
    SYSCTL_RCGCGPIO |= (1 << 2); // enable GPIOC
    SYSCTL_GPIOHBCTL |= (1 << 2); // enable AHB access to GPIOC
    GPIOC_DEN |= (1 << 5) + (1 << 6); // digital mode for bits 5 and 6 of GPIOC
    GPIOC_AFSEL |= (1 << 5) + (1 << 6); // alternate function mode for bits 5 and 6
    GPIOC_PCTL &= ~((0x0f << 20) + (0x0f << 24)); // zero out pin control value for bits 5 and 6
    GPIOC_PCTL |= ((6 << 20) + (6 << 24)); // zero out pin control value for bits 5 and 6

    QEI1_CTL = 0x00000020;
    QEI1_LOAD = 8000000; // This sets the timing window to 1 second when system clock is 16MHz
                         // If you need to sample the speed more quickly then set this to
                         // a smaller value (shorter window) and adjust speed reading accordingly
    QEI1_MAXPOS = 1000;
    QEI1_CTL |= 1;

}
void initQEI0(void)
{
    // Quadrature encoder is connected to PA7 and PD6
    // These correspond to PhaseA1 and PhaseB1
    SYSCTL_RCGCQEI |= (1 << 0); // enable the clock for QEI0
    SYSCTL_RCGCGPIO |= (1 << 0); // enable GPIOA
    SYSCTL_RCGCGPIO |= (1 << 3); // enable GPIOD
    SYSCTL_GPIOHBCTL |= (1 << 0); // enable AHB access to GPIOA
    SYSCTL_GPIOHBCTL |= (1 << 3); // enable AHB access to GPIOD

    GPIOA_DEN |= (1 << 7); // digital mode for bits 7 of GPIOA
    GPIOD_DEN |= (1 << 6); // digital mode for bits 6 of GPIOD

    GPIOA_AFSEL |= (1 << 7); // alternate function mode for PA7
    GPIOD_AFSEL |= (1 << 6); // alternate function mode for PD6

    GPIOA_PCTL &= ~(0x0f << 28); // zero out pin control value for bits PA7
    GPIOA_PCTL |= (6 << 28); // zero out pin control value for bits PA7

    GPIOD_PCTL &= ~(0x0f << 24); // zero out pin control value for bits PD6
    GPIOD_PCTL |= (6 << 24); // zero out pin control value for bits PD6

    QEI0_CTL = 0x00000020;
    QEI0_LOAD = 8000000; // This sets the timing window to 1 second when system clock is 16MHz
                         // If you need to sample the speed more quickly then set this to
                         // a smaller value (shorter window) and adjust speed reading accordingly
    QEI0_MAXPOS = 1000;
    QEI0_CTL |= 1;

}

int getQEI1Position()
{
    return QEI1_POS;
}
int getQEI0Position()
{
    return QEI0_POS;
}
int getQEI1Velocity()
{
    int speed = QEI1_SPEED;
    int direction = QEI1_STAT >> 1;
    if (direction)
        speed = -speed;
    return speed;
}
int getQEI0Velocity()
{
    int speed = QEI0_SPEED;
    int direction = QEI0_STAT >> 1;
    if (direction)
        speed = -speed;
    return speed;
}


uint32_t Leftcontroller(int32_t target, int32_t current){

    // Scales the values up by a constant so for example 1000 * 0.01 can be 1010 instead of 1 * 0.01 getting 1.01
    // because decimals are inacurate and even small changes from rounding could be a problem
    target = target * DUTYSCALER;
    current = current * DUTYSCALER;
    int32_t i_Max = 10000 * DUTYSCALER;
    int32_t i_Min = 0;
    float Kp = 0.1;
//    float Ki = 1.6;
    float Ki = 0.01;
    float Kd = 0.01;
    int32_t error = target - current;

    // Proportional: The error times the proportional coefficent (Kp)
    int32_t P = error * Kp;

    // Add the current Error to the error sum
    i_sum_main += error / DUTYSCALER;

    // Limit the summed error to between i_max and i_min
    if (i_sum_main > i_Max) i_sum_main = i_Max;
    else if (i_sum_main < i_Min) i_sum_main = i_Min;

    // Integral: Multiply the sum by the integral coefficent (Ki)
    int32_t I = Ki * i_sum_main;

    // Derivative: Calculate change in error between now and last time through the controller
    // then multiply by the differential coefficent (Kd)
    int32_t D = Kd * (d_lastError_main - error);

    // Store error to be used to calculate the change next time
    d_lastError_main = error;

    // Combine the proportional, intergral and dirrivetave components and then scales back down.
    //      (looking at it again im not sure why this isn't just "P + I + D" as the previous
    //      duty cycle shouldn't matter, I'll test changing this)
    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit the duty cycle to between 95 and 5
    if (PWM_Duty > 95) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < 60) PWM_Duty = PWM_MIN;

    PWM_last_main = PWM_Duty;
    return PWM_Duty;
}
uint32_t Rightcontroller(int32_t target, int32_t current){

    target = target * DUTYSCALER;
    current = current * DUTYSCALER;
    int32_t i_Max = 10000 * DUTYSCALER;
    int32_t i_Min = 0;
    float Kp = 0.04;
    float Ki = 0.00001;
    float Kd = 0.01;
    int32_t error = target - current;

    // Proportional
    int32_t P = error * Kp;

    // Integral
    i_sum_tail += error / DUTYSCALER;

    // Limit sum
    if (i_sum_tail > i_Max) i_sum_tail = i_Max;
    else if (i_sum_tail < i_Min) i_sum_tail = i_Min;

    // Integral
    int32_t I = Ki * i_sum_tail;

    // Derivative
    int32_t D = Kd * (d_lastError_tail - error);
    d_lastError_tail = error;

    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit PWM to specification
    if (PWM_Duty > 95) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < 60) PWM_Duty = PWM_MIN;

    PWM_last_tail = PWM_Duty;
    return PWM_Duty;
}

void pinMode(unsigned int PortNumber, unsigned int bit, unsigned int mode)
{
    if (PortNumber < 6)
    {
        enableGPIO(PortNumber); // make sure the port is enabled
        volatile uint32_t *Port = (volatile uint32_t *)(GPIOA_BASE + 0x1000*PortNumber);
        if (PortNumber==GPIOF)
        {
            // Bit 0 of GPIOF is locked at boot.  Need to unlock first
            GPIOF_LOCK = 0x4C4F434B;    // password
            GPIOF_CR = 1; // Commit changes for Bit 0
        }
        switch (mode)
        {
            case INPUT: {
                *(Port+(GPIODEN>>2)) |= (1 << bit); // enable digital mode
                *(Port+(GPIOAFSEL>>2)) &= ~(1 << bit); // disable alternative function
                *(Port+(GPIODIR>>2)) &= ~(1 << bit); // configure the bit as an input
                break;
            }
            case OUTPUT: {
                *(Port+(GPIODEN>>2)) |= (1 << bit); // enable digital mode
                *(Port+(GPIOAFSEL>>2)) &= ~(1 << bit); // disable alternative function
                *(Port+(GPIODIR>>2)) |= (1 << bit); // configure the bit as an output
                break;
            }
            case ALTERNATE: {
                *(Port+(GPIODEN>>2)) |= (1 << bit); // enable digital mode (assumption)
                *(Port+(GPIOAFSEL>>2)) |= (1 << bit); // enable alternative function
                break;
            }
            case ANALOG: {
                *(Port+(GPIODEN>>2)) &= ~(1 << bit); // disable digital mode (assumption)
                *(Port+(GPIOAFSEL>>2)) &= ~(1 << bit); // disable alternative function
                *(Port+(GPIOAMSEL>>2)) |= (1 << bit); // enable analog mode
                break;
            }
            case INPUT_PULLUP: {
                *(Port+(GPIODEN>>2)) |= (1 << bit); // enable digital mode
                *(Port+(GPIOAFSEL>>2)) &= ~(1 << bit); // disable alternative function
                *(Port+(GPIODIR>>2)) &= ~(1 << bit); // configure the bit as an input
                *(Port+(GPIOPUR>>2)) |= (1 << bit); // enable pull-up resistor
                break;
            }
        }
    }
}

void initMotorPWM(void)
{
    enableGPIO(GPIOB);
    pinMode(GPIOB,4,ALTERNATE);  // alternate function for Timer (AF 7)
    pinMode(GPIOB,5,ALTERNATE);  // alternate function for Timer (AF 7)
    pinMode(GPIOB,6,ALTERNATE);  // alternate function for Timer (AF 7)
    pinMode(GPIOB,7,ALTERNATE);  // alternate function for Timer (AF 7)
    GPIOB_PCTL &= ~(0xf << 4*4);
    GPIOB_PCTL &= ~(0xf << 4*5);
    GPIOB_PCTL &= ~(0xf << 4*6);
    GPIOB_PCTL &= ~(0xf << 4*7);
    GPIOB_PCTL |= (0x07 << 4*4);
    GPIOB_PCTL |= (0x07 << 4*5);
    GPIOB_PCTL |= (0x07 << 4*6);
    GPIOB_PCTL |= (0x07 << 4*7);
    // Turn on Timer 0 and Timer 1
    SYSCTL_RCGCTIMER |= BIT0 + BIT1;
    // Configure the Timers
    GPTM0_32_CTL = 0; // start by disabling the timers
    GPTM0_32_CFG = 4; // select 16 bit mode
    GPTM1_32_CTL = 0; // start by disabling the timers
    GPTM1_32_CFG = 4; // select 16 bit mode

    // Set bit 10 to force reload of register to align with
    // end of PWM period (avoids glitches)
    // Set T0BMS=1, T0BCMR = 0; T0BMR=2;
    // This puts the timer into PWM mode
    GPTM0_32_TAMR = BIT3+BIT1+BIT10;
    // Set T0BMS=1, T0BCMR = 0; T0BMR=2;
    GPTM0_32_TBMR = BIT3+BIT1+BIT10;
    // Set T1AMS=1, T1ACMR = 0; T1AMR=2;
    GPTM1_32_TAMR = BIT3+BIT1+BIT10;;
    // Set T1BMS=1, T1BCMR = 0; T1BMR=2;
    GPTM1_32_TBMR = BIT3+BIT1+BIT10;

// Set period and initial duty for all 4 channels
    GPTM0_32_TAILR = MAX_DUTY+1;
    GPTM0_32_TAMATCHR = 0;

    GPTM0_32_TBILR = MAX_DUTY+1;
    GPTM0_32_TBMATCHR = 0;

    GPTM1_32_TAILR = MAX_DUTY+1;
    GPTM1_32_TAMATCHR = 0;

    GPTM1_32_TBILR = MAX_DUTY+1;
    GPTM1_32_TBMATCHR = 0;


    // enable T0A and T0B with output inversion
    GPTM0_32_CTL |= BIT0+BIT8+BIT6+BIT14;
    // enable T1A and T1B with output inversion
    GPTM1_32_CTL |= BIT0+BIT8+BIT6+BIT14;
}


