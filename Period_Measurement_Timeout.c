// Period_Measurement.c
// Runs on TM4C123
// Use Timer0A in input edge mode 16-bits count.  
// Timer0A use PB6 as imput.
// PB7 connects to Ultrasonic sensor trigger pin.
// By Min He
// March 30th, 2018

/* This example used the following book example, Example 8.3, as reference
   "Embedded Systems: Real Time Operating Systems for Arm Cortex M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2014

 Copyright 2018 by Min He, min.he@csulb.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Nokia5110.h"
#include "ADCSWTrigger.h"
#include "states.h"

#define MC_LEN 0.0625      // length of one machine cyce in microsecond for 16MHz clock
#define SOUND_SPEED 0.0343 // centimeter per micro-second
#define MAX_DURATION 0xFFFF

void Timer0_Init(void);
void Timer1_Init(void);
void WaitForInterrupt(void);  // low power mode
void EnableInterrupts(void);
void PortD_Init(void);
void init_PortF(void); 
void SysTick_Init(void); 
void forwardDirection(void);
void SysTick_Wait(unsigned long delay);
void Delay(unsigned int t);
void SysTick_Wait1us(unsigned long delay); 

	unsigned int testa, testb, test1, test2; 
	unsigned char testButton; 

uint32_t period=0;
uint8_t done=0, timeout=0;
uint32_t first = 0;
char o;

// the following variables are for testing purpose, need to move inside main after testing
uint32_t distance=0;
uint32_t first_time = 0;
uint32_t second_time = 0;
uint32_t first_read=0, second_read=0;
uint8_t OutOfRange = 0;

// following variables are for for the ain use for the ir
unsigned long ain1, ain2, ain3, dutyCycle;
char sample=0;  

///////////////////ADC Stuff///////////////////
int adcTable[] = {4095, 3050, 1980, 1370, 950, 830, 730, 650, 570, 530, 460, 390, 330, 300, 0};
int distTable[] = {0, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 999};
float distance_ADC = 0;  //  <---- THIS USE TO BE CALLed distance but is now changed to distance_ADC so be aware
float calibration = 0;
float a = 0;
float b = 0;
int ia = 0;
int ib = 0;
float m = 0;
float l = 0;
float lm;
int i;
int f;
float dist1, dist2;
float DC;

unsigned int period_pwm = 5000; 
//unsigned int lDuty, rDuty; 
unsigned int left = 4960, right  = 4960; // initially moving wheels 
int array2[] = {0,0}; 

//======================================================================
//DEFINITIONS FOR PWM
#define SYSCTL_RCC_USEPWMDIV  0x00100000 // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M   0x000E0000 // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2   0x00000000 // PWM clock /2

#define LEFTFORWARD 					(*((volatile unsigned long *)0x40007010)) //PD2
#define LEFTBACKWARD 			 		(*((volatile unsigned long *)0x40007020)) //PD3
#define RIGHTFORWARD			   	(*((volatile unsigned long *)0x40007100)) //Pd6
#define RIGHTBACKWARD 				(*((volatile unsigned long *)0x40007200)) //Pd7

#define LED										(*((volatile unsigned long*)0x40025038)) // PF3-1

#define RED 0x02;
#define BLUE 0x04;
#define GREEN 0x08;
	
// DEFINITIONS FOR THE FSM
#define TURN_RIGHT 0x4
#define TURN_LEFT 0x2
#define GO_STRAIGHT 0x0

#define IR_ON 0x1
#define IR_OFF 0x0

#define leftWheel   0x00
#define rightWheel  0x01

/*
char direction;
unsigned char s;


struct State{
	unsigned long LPWM;
	unsigned long RPWM;
	
	uint32_t NS[4];
};

typedef const struct State stateType;
#define STRAIGHT 0
#define RIGHT 1
#define LEFT 2
#define STOP 3

stateType fsm[4]={
	// Straight
	{1250, 1250, {STRAIGHT, RIGHT, LEFT, STOP}},
	
	// Right
	{29000, 1250, {STRAIGHT, RIGHT, LEFT, STOP}},
	
	// Left
	{1250, 4998, {STRAIGHT, RIGHT, LEFT, STOP}},
	
	// Stop
	{35000, 35000, {STRAIGHT, RIGHT, LEFT, STOP}}
};
*/

int main(void){

  //PLL_Init();               // 80 MHz clock
	ADC_Init298();
	EnableInterrupts();

	Nokia5110_Init();
	Nokia5110_Clear();
	SysTick_Init();         // use default 16MHz clock
  init_PortF();
	PortD_Init();
	forwardDirection();
	
  Timer0_Init();          // initialize timer0A
  Timer1_Init();
	
  //ain3 = 4998;
	LED = BLUE; 
	
	while(1){
		GPIO_PORTB_DATA_R &= ~0x80; // send low to trigger
		//Delay(2);
		SysTick_Wait1us(2);
		GPIO_PORTB_DATA_R |= 0x80; // send high to trigger
	//	Delay(10);
		SysTick_Wait1us(10);
		GPIO_PORTB_DATA_R &= ~0x80; // send low to trigger

    // start timer 0 capture mode
    TIMER0_IMR_R = 0x00000004;    // enable capture mode event 
    TIMER0_TAILR_R = MAX_DURATION;    // reload start value
	  TIMER0_CTL_R = 0x0000000D;    // Enable TIMER0A capture mode: both edges
		
		// start timer 1 periodic mode
    TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
    TIMER1_TAILR_R = MAX_DURATION;// reload value
		TIMER1_CTL_R = 0x00000001;    // enable TIMER1A
		
		while ((!done)&&(!timeout)){
	  TIMER0_CTL_R = 0x00000000;    // disable TIMER0A 
	  TIMER1_CTL_R = 0x00000000;    // disable TIMER1A 
    TIMER0_IMR_R = 0x00000000;    // disable interrupt
    TIMER1_IMR_R = 0x00000000;    // disable interrupt
		
		if (done) {
			// The speed of sound is approximately 340 meters per second, 
			// or  .0343 c/µS.
      // Distance = (duration * 0.0343)/2;
		  distance = (period*MC_LEN*SOUND_SPEED)/2;	
			OutOfRange = 0;
			//Nokia5110_Clear();
			Nokia5110_SetCursor(0,0);
			Nokia5110_OutUDec(distance);
		}
		else { // out of range			
		  distance = 0;
			OutOfRange = 1;
			Nokia5110_SetCursor(0,0);
			Nokia5110_OutString(" OOR ");
		}
		//Nokia5110_OutString("test");
		first = 0;
		done = 0;
    timeout	= 0;		

		
		if(dist1 <20){
		left = 	Right(leftWheel);
		right =  Right(rightWheel); 

		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		LED = RED; 
		}
		
		else if (dist2 <20){
		left = 	Left(leftWheel);
		right =  Left(rightWheel); 

		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		LED = GREEN; 
		}
		
		else if(dist2 > 20 && dist1 > 20){
		left = 	Forward(leftWheel);
		right =  Forward(rightWheel);
		
		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		
		LED = 0x0C; 
		}
		
		
	  if(dist2 < 20 && dist1 <20) {
		left = 	Stop(leftWheel);
		right =  Stop(rightWheel);
		
		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		
		LED = 0x0A; 
		}
		
		
		/******************************/
		
		/*
		Nokia5110_SetCursor(0,0);
	 
		
		
		GPIO_PORTB_DATA_R &= ~0x80; // send low to trigger
		SysTick_Wait1us(2);
		GPIO_PORTB_DATA_R |= 0x80; // send high to trigger
		SysTick_Wait1us(10);
		GPIO_PORTB_DATA_R &= ~0x80; // send low to trigger

    // start timer 0 capture mode
    TIMER0_IMR_R = 0x00000004;    // enable capture mode event 
    TIMER0_TAILR_R = MAX_DURATION;    // reload start value
	  TIMER0_CTL_R = 0x0000000D;    // Enable TIMER0A capture mode: both edges
		
		// start timer 1 periodic mode
    TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
    TIMER1_TAILR_R = MAX_DURATION;// reload value
		TIMER1_CTL_R = 0x00000001;    // enable TIMER1A
		
		 Nokia5110_OutString("testtest");
		 
		// Use general purpose timer input edge mode 16 bits count, 
		// detectable range: (65535*62.58*10^(-3)*0.0343)/2=70.2cm
		// Notice that the detect range for HC - SR04 ultrasonic sensor is 400cm
    // Since our application only need to detect obstcle within 70cm, 
    // 16 bits count is good enough for us.		
		
    while ((!done)&&(!timeout)){
	  TIMER0_CTL_R = 0x00000000;    // disable TIMER0A 
	  TIMER1_CTL_R = 0x00000000;    // disable TIMER1A 
    TIMER0_IMR_R = 0x00000000;    // disable interrupt
    TIMER1_IMR_R = 0x00000000;    // disable interrupt
		
		if (done) {
			// The speed of sound is approximately 340 meters per second, 
			// or  .0343 c/µS.
      // Distance = (duration * 0.0343)/2;
		  distance = (period*MC_LEN*SOUND_SPEED)/2;	
			OutOfRange = 0;
			//Nokia5110_Clear();
			Nokia5110_SetCursor(0,0);
			Nokia5110_OutUDec(distance);
		}
		*/
		/*
		else { // out of range			
		  distance = 0;
			OutOfRange = 1;
			Nokia5110_SetCursor(0,0);
			Nokia5110_OutString(" OOR ");
		}*/
		//Nokia5110_OutString("test");		
}
//=========================================================================================		
/*
    if(sample){
			sample = 0;	
			ADC_In298(&ain1, &ain2, &ain3); // Ensure sampler works
			GPIO_PORTF_DATA_R ^= 0x04;
			
			// Update Motors
			//dutyCycle = ain3-1;
			//PWM1_0_CMPA_R = dutyCycle;
			//PWM1_0_CMPB_R = dutyCycle;
			
			//Update Sensors
			// Find distance
		for(i = 0; i < 15; i = i + 1){
			if(ain1 > adcTable[i]){
				break;
			}
			else{
				a = adcTable[i+1];
				ia = i+1;
			}
		}
		
		for(f = 0; f < 15; f = f + 1){
			if(ain1 < adcTable[f]){
				b = adcTable[f];
				ib = f;
			}
			else {
				break;
			}
		}
		 m = b - a;
		 l = b - ain1;
		lm = l / m ;
		
		dist1 = distTable[ib] + (lm * 5);
		
		// Find distance
		for(i = 0; i < 15; i = i + 1){
			if(ain2 > adcTable[i]){
				break;
			}
			else{
				a = adcTable[i+1];
				ia = i+1;
			}
		}
		
		for(f = 0; f < 15; f = f + 1){
			if(ain2 < adcTable[f]){
				b = adcTable[f];
				ib = f;
			}
			else {
				break;
			}
		}
		 m = b - a;
		 l = b - ain2;
		lm = l / m ;
		
		dist2 = distTable[ib] + (lm * 5);
		
		DC = (ain3*100)/4095;
		DC = 100 - DC; 
		
			//Update LCD
			Nokia5110_SetCursor(0,0);
			Nokia5110_OutString("Duty Cycle:");
			Nokia5110_SetCursor(0,1);
			Nokia5110_OutUDec(DC);
			Nokia5110_SetCursor(0,2);
			Nokia5110_OutString("Sensor 1:");
			Nokia5110_SetCursor(0,3);
			if(dist1 > 60) Nokia5110_OutString("  OOR");
			else Nokia5110_OutUDec(dist1);
			Nokia5110_SetCursor(0,4);
			Nokia5110_OutString("Sensor 2:");
			Nokia5110_SetCursor(0,5);
			if (dist2 >60) Nokia5110_OutString("  OOR");
			else Nokia5110_OutUDec(dist2);

		}
		
	}*/
}}
// ***************** Timer0_Init ****************
// Activate TIMER0 interrupts to capture 
// the period between a rising edge and a falling edge
// to be used to calculate distance detected by
// an ultrasonic sensor.
void Timer0_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  SYSCTL_RCGCGPIO_R |= 0x0002;     // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
  GPIO_PORTB_DEN_R |= 0x40;        // enable digital I/O on PB6
                                   // configure PB6 as T0CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;     // disable analog functionality on PB6
		
	// PB7 connects to Ultrasonic sensor trigger pin
  GPIO_PORTB_AFSEL_R &= ~0x80;      // disable alt funct on PB7
  GPIO_PORTB_DEN_R |= 0x80;        // enable digital I/O on PB7
                                   // configure PB7 as GPIO
  GPIO_PORTB_PCTL_R &= ~0xF0000000;
  GPIO_PORTB_AMSEL_R &= ~0x80;     // disable analog functionality on PB7
	GPIO_PORTB_DIR_R |= 0x80;        // PB7 is output

  TIMER0_CTL_R &= ~0x0000000F;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000004;    // 2) configure for 16-bit timer mode
	TIMER0_TAMR_R = 0x00000007;   // 3) edge time capture mode: count down
  TIMER0_TAILR_R = MAX_DURATION;    // 4) start value
  TIMER0_ICR_R = 0x00000004;    // 6) clear TIMER0A capture and timeout flag
  TIMER0_IMR_R = 0x00000000;    // 7) disable capture mode event interrupt
  
	NVIC_PRI4_R = (NVIC_PRI4_R&0x1FFFFFFF)|0x80000000; // 8) priority 2
  // interrupts enabled in the main program after all devices initialized
  // vector number 35, interrupt number 19
  NVIC_EN0_R |= 0x80000;           // 9) enable IRQ 19 in NVIC
}

void Timer0A_Handler(void)
{
	TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowledge TIMER0A capture interrupt
	if ((GPIO_PORTB_DATA_R & 0x40)==0x40) { //rising edge
		first = TIMER0_TAR_R;  
		first_time = first; // this line of code is for debugging purpose, can be removed
		done = 0;
	}
	else if (first != 0){
		period = (first - TIMER0_TAR_R)&0x00FFFFFF; // 24 bits counter
		second_time = TIMER0_TAR_R; // this line of code is for debugging purpose, can be removed
		done = 1;
	} 
  	
}

// Use TIMER1 in 32-bit periodic mode to request interrupts at a periodic rate
void Timer1_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
  while((SYSCTL_RCGCTIMER_R&0x02) == 0){};// ready?
		
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAILR_R = MAX_DURATION;// 4) reload value
  TIMER1_TAPR_R = 0xFF;            // 5) bus clock resolution
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
  TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00008000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 37, interrupt number 21
  NVIC_EN0_R |= 0x200000;           // 9) enable IRQ 21 in NVIC
}

void Timer1A_Handler(void){
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER1A timeout
	timeout = 1;
	TIMER1_CTL_R = 0x00000000;    // disable TIMER1A
}


// INITIALIZAITONS FOR THE PWM OF THE 2 DC MOTORS
void PortD_Init(void){ 
	volatile unsigned long delay;	
  SYSCTL_RCGC2_R 		 |= 0x00000008;  	// (a) activate clock for port D
	delay = SYSCTL_RCGC2_R;	
	
	
	GPIO_PORTD_LOCK_R = 0x4C4F434B; 
  GPIO_PORTD_CR_R = 0xCF;           // allow changes to PD0   
  GPIO_PORTD_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTD_DIR_R = 0xCF;          // 5) PD0-1 output    1100.1111
  GPIO_PORTD_DEN_R = 0xCF;          // 7) enable digital pins PD0-1   	
	
	SYSCTL_RCGCPWM_R 	 |= 0x00000002;		// STEP 1: activate clock for PWM Module 1
	SYSCTL_RCGCGPIO_R  |= 0x00000008;   // STEP 2: enable GPIO clock
	
  GPIO_PORTD_AFSEL_R |= 0x03;   			// STEP 3: enable alt function on   PD0-1
	
		// STEP 4: configure alt funt PF4-0
	GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~0x000000FF)| 0x00000055;
	
	// STEP 5: configure the use of PWM divide
	SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;  // PWM divider
	SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;  // clear the PWM divider field
	SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;   // configure for /2 diveder

	// STEP 6: confiure genertor countdown mode PWM1_0 is for PD0
	PWM1_0_CTL_R  &= ~0xFFFFFFFF;
	PWM1_0_GENA_R |= 0x0000008C;
	PWM1_0_GENB_R |= 0x0000080C;
	
	
	PWM1_0_LOAD_R = period_pwm - 1;    // STEP 7 : set period
	
  PWM1_0_CMPA_R =  right - 1;    	     // STEP 8: set duty cycle right
  PWM1_0_CMPB_R =  left - 1; 				//left 
	
	testa = PWM1_0_CMPA_R; 
	testb = PWM1_0_CMPB_R; 
	
  PWM1_0_CTL_R  |= 0x00000001;   	  // STEP 9: start the M1PWM5 generator
  PWM1_ENABLE_R |= 0x00000003;			// STEP 10: enable   M1PWM0-1 outputs
}

void init_PortF(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF3-0 
	GPIO_PORTF_DIR_R = 0x0E;          // 5)PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4  
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}


void GPIOPortF_Handler(void){
	
	if ((GPIO_PORTF_RIS_R & 0x10)){ // sw1 is pressed
		GPIO_PORTF_ICR_R = 0x10;
	left = 	Left(leftWheel);
	right =  Left(rightWheel); 

	PWM1_0_CMPA_R = right - 1;
	PWM1_0_CMPB_R = left  - 1;
		LED = RED; 
		

	}
	
	if ((GPIO_PORTF_RIS_R & 0x01)){ // sw2 is pressed
		GPIO_PORTF_ICR_R = 0x01;
	left = 	Right(leftWheel);
	right =  Right(rightWheel);

	PWM1_0_CMPA_R = right - 1;
	PWM1_0_CMPB_R = left  - 1;
			
		LED = GREEN; 
	}
}

// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 62.5 nsec for 80 MHz clock)
void SysTick_Wait(unsigned long delay){
  volatile unsigned long elapsedTime;
  unsigned long startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}

// Time delay using busy wait.
// This assumes 16 MHz system clock.
void SysTick_Wait1us(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(16);  // wait 1us (assumes 16 MHz clock)
  }
}

// Can sample at 20 or 10 Hz also
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}

void SysTick_Handler(){
	sample = 1;
}

void forwardDirection(void){
	LEFTFORWARD   = 0xFF;																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																															
	LEFTBACKWARD  = 0x00;					
	RIGHTFORWARD  = 0xFF;
	RIGHTBACKWARD = 0x00; 
}

void Delay(unsigned int t){unsigned long volatile time;
  time = (727240*200/91)/100000;  // 0.1sec
	for(o = 0; o < t; o++){
  while(time){
		time--;
  }}
} 

