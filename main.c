#include "Nokia5110.c"
#include "PLL.c"
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "SysTick.h"
#include "states.h"
#include "ADCSWTrigger.h"

/*                  	 	GOALS                             */
// PWM ADJUSTMENT BETWEEN TWO WHEELS
// ADC SAMPLING FOR IR SENSOR
// GENERAL PURPOSE TIMER FOR ACTIVATING DIGITAL ULTRA SONIC SENSOR
// SSI INTERFACE FOR NOKIA LCD

// NOTE: ON TM4C PINS B6/D0 AND B7/D1 ARE INTERCONNECTED
/*********************************************************/

/*                     TM4C PIN INTERFACE                 */
// PORT - PIN - MODE
//  D   -  0  - PWM
//  D   -  1  - PWM
//  D   -  2  - GPIO
//  D   -  3  - GPIO
//  D   -  6  - GPIO
//  D   -  7  - GPIO

//  E   -  0  - GPIO
//  F   -  0  - GPIO
//  E   -  0  - GPIO
//  F   -  0  - GPIO
//  E   -  0  - GPIO
//  F   -  0  - GPIO

/***********************************************************/

/***********************************************************/




/*                     DEFINE CONSTANTS                   */
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


// DEFINITIONS FOR ULTRA SONIC
#define MC_LEN 0.0625      // length of one machine cyce in microsecond for 16MHz clock
#define SOUND_SPEED 0.0343 // centimeter per micro-second
#define MAX_DURATION 0xFFFF

/***********************************************************/




/*                     DEFINE FUNCTION PROTOTYPES         */

/***********************************************************/
// PWM PROTOTYPES
void PortD_Init(void);
void init_PortF(void); 
void forwardDirection(void);

// GENERAL PURPOSE TIMER PROTOTYPES
void Timer0_Init(void);
void Timer1_Init(void);

// OTHER PROTOTPYES
void WaitForInterrupt(void);  // low power mode
void EnableInterrupts(void);
void Delay(unsigned int t);




/*                  DEFINE VARIABLES                     */
// PWM VARIABLES
unsigned int period_pwm = 5000; 
//unsigned int lDuty, rDuty; 
unsigned int left = 4960, right  = 4960; // initially moving wheels
unsigned int testa, testb, test1, test2; 
unsigned char testButton; 

// SENSOR VARIABLES
float dist1, dist2;
unsigned long ain1, ain2, ain3, dutyCycle;
char sample=0;
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

// ULTRA SONIC VARIABLES
uint32_t period=0;
uint8_t done=0, timeout=0;
uint32_t first = 0;

// the following variables are for testing purpose, need to move inside main after testing
uint32_t distance=0;
uint32_t first_time = 0;
uint32_t second_time = 0;
uint32_t first_read=0, second_read=0;
uint8_t OutOfRange = 0;


/********************************************************/



/*                  MAIN                              */
int main(void){
	//////////////////////////////////////
	//dist1 = 21;
	//dist2 = 21;
	/////////////////////////////////////
	PortD_Init();
	init_PortF();
	forwardDirection();
	
	SysTick_Init();
	ADC_Init298();
	
	Nokia5110_Init();
	Nokia5110_Clear();
	
	Timer0_Init();
	Timer1_Init();
	while(1){	
		// GENERAL PURPOSE TIMER LOGIC
		GPIO_PORTB_DATA_R &= ~0x02; // send low to trigger
		Delay(2);//SysTick_Wait1us(2);
		GPIO_PORTB_DATA_R |= 0x02; // send high to trigger
		Delay(10);//SysTick_Wait1us(10);
		GPIO_PORTB_DATA_R &= ~0x02; // send low to trigger
    // start timer 0 capture mode
    TIMER2_IMR_R = 0x00000004;    // enable capture mode event 
    TIMER2_TAILR_R = MAX_DURATION;    // reload start value
	  TIMER2_CTL_R = 0x0000000D;    // Enable TIMER2A capture mode: both edges
		
		// start timer 1 periodic mode
    TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
    TIMER1_TAILR_R = MAX_DURATION;// reload value
		TIMER1_CTL_R = 0x00000001;    // enable TIMER1A
		
		// Use general purpose timer input edge mode 16 bits count, 
		// detectable range: (65535*62.58*10^(-3)*0.0343)/2=70.2cm
		// Notice that the detect range for HC - SR04 ultrasonic sensor is 400cm
    // Since our application only need to detect obstcle within 70cm, 
    // 16 bits count is good enough for us.		
		
    while ((!done)&&(!timeout));
	  TIMER2_CTL_R = 0x00000000;    // disable TIMER2A 
	  TIMER1_CTL_R = 0x00000000;    // disable TIMER1A 
    TIMER2_IMR_R = 0x00000000;    // disable interrupt
    TIMER1_IMR_R = 0x00000000;    // disable interrupt
		
		if (done) {
			// The speed of sound is approximately 340 meters per second, 
			// or  .0343 c/µS.
      // Distance = (duration * 0.0343)/2;
		  distance = (period*MC_LEN*SOUND_SPEED)/2;	
			Nokia5110_SetCursor(0,0);
			Nokia5110_OutUDec(distance);
			OutOfRange = 0;
		}
		else { // out of range			
		  distance = 0;
			Nokia5110_SetCursor(0,0);
			Nokia5110_OutString(" OOR ");
			OutOfRange = 1;
		}
		first = 0;
		done = 0;
    timeout	= 0;		
		
		
		// ADC PART OF LOOP	
		if(sample){
			sample = 0;	
			ADC_In298(&ain1, &ain2, &ain3); // Ensure sampler works
			
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
		Nokia5110_SetCursor(0,2);
		Nokia5110_OutUDec(dist1);
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
		Nokia5110_SetCursor(0,3);
		Nokia5110_OutUDec(dist2);
	}
		
		
		// END ADC PART OF LOOP
	
		// PWM PART OF LOOP	
		if(dist1 < 50 && dist2 > 50){
		left = 	Right(leftWheel);
		right =  Right(rightWheel); 

		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		LED = RED; 
		}
		
		else if (dist2 < 50 && dist1 > 50){
		left = 	Left(leftWheel);
		right =  Left(rightWheel); 

		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		LED = GREEN; 
		}
		
		else if(dist2 <= 50 && dist1 <= 50){
		left = 	Forward(leftWheel);
		right =  Forward(rightWheel);
		
		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		
		LED = 0x07; 
		}

	  else if(dist2 > 55 && dist1 > 55 && OutOfRange == 1) {
		left = 	Stop(leftWheel);
		right =  Stop(rightWheel);
		
		PWM1_0_CMPA_R = right - 1;
		PWM1_0_CMPB_R = left  - 1;
		
		LED = BLUE; 
		}
		
		// END PWM LOGIC
		
		
		
		
	} // END MASTER LOOP
	
	
} // END MAIN

/*****************************************************/



/*                      FUNCTIONS                      */
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

void forwardDirection(void){
	LEFTFORWARD   = 0xFF;																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																															
	LEFTBACKWARD  = 0x00;					
	RIGHTFORWARD  = 0xFF;
	RIGHTBACKWARD = 0x00; 
}


void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = 2000000-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
}

void SysTick_Handler(){
	sample = 1;
}

/*
void Timer0_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x08;      // activate timer3
  SYSCTL_RCGCGPIO_R |= 0x0002;     // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_AFSEL_R |= 0x04;      // enable alt funct on PB2
  GPIO_PORTB_DEN_R |= 0x04;        // enable digital I/O on PB2
                                   // configure PB2 as T2CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFF0FF)+0x00000700;
  GPIO_PORTB_AMSEL_R &= ~0x04;     // disable analog functionality on PB2
		
	// PB1 connects to Ultrasonic sensor trigger pin
  GPIO_PORTB_AFSEL_R &= ~0x08;      // disable alt funct on PB3
  GPIO_PORTB_DEN_R |= 0x08;        // enable digital I/O on PB3
                                   // configure PB3 as GPIO
  GPIO_PORTB_PCTL_R &= ~0x0000F000;
  GPIO_PORTB_AMSEL_R &= ~0x08;     // disable analog functionality on PB3
	GPIO_PORTB_DIR_R |= 0x08;        // PB3 is output

  TIMER3_CTL_R &= ~0x0000000F;    // 1) disable TIMER2A during setup
  TIMER3_CFG_R = 0x00000004;    // 2) configure for 16-bit timer mode
	TIMER3_TAMR_R = 0x00000007;   // 3) edge time capture mode: count down
  TIMER3_TAILR_R = MAX_DURATION;    // 4) start value
  TIMER3_ICR_R = 0x00000004;    // 6) clear TIMER2A capture and timeout flag
  TIMER3_IMR_R = 0x00000000;    // 7) disable capture mode event interrupt
  
	NVIC_PRI8_R = (NVIC_PRI8_R&0x1FFFFFFF)|0x80000000; // 8) priority 2
  // interrupts enabled in the main program after all devices initialized
  // vector number 51, interrupt number 35
  NVIC_EN1_R |= 0x00008;           // 9) enable IRQ 23 in NVIC
}

void Timer3A_Handler(void)
{
	TIMER2_ICR_R = TIMER_ICR_CAECINT;// acknowledge TIMER0A capture interrupt
	if ((GPIO_PORTB_DATA_R & 0x01)==0x01) { //rising edge
		first = TIMER2_TAR_R;  
		first_time = first; // this line of code is for debugging purpose, can be removed
		done = 0;
	}
	else if (first != 0){
		period = (first - TIMER2_TAR_R)&0x00FFFFFF; // 24 bits counter
		second_time = TIMER2_TAR_R; // this line of code is for debugging purpose, can be removed
		done = 1;
	} 
  	
}

*/





void Timer0_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x04;      // activate timer2
  SYSCTL_RCGCGPIO_R |= 0x0002;     // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_AFSEL_R |= 0x01;      // enable alt funct on PB0
  GPIO_PORTB_DEN_R |= 0x01;        // enable digital I/O on PB0
                                   // configure PB0 as T2CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFFF0)+0x00000007;
  GPIO_PORTB_AMSEL_R &= ~0x01;     // disable analog functionality on PB0
		
	// PB1 connects to Ultrasonic sensor trigger pin
  GPIO_PORTB_AFSEL_R &= ~0x02;      // disable alt funct on PB1
  GPIO_PORTB_DEN_R |= 0x02;        // enable digital I/O on PB1
                                   // configure PB1 as GPIO
  GPIO_PORTB_PCTL_R &= ~0x000000F0;
  GPIO_PORTB_AMSEL_R &= ~0x02;     // disable analog functionality on PB7
	GPIO_PORTB_DIR_R |= 0x02;        // PB7 is output

  TIMER2_CTL_R &= ~0x0000000F;    // 1) disable TIMER2A during setup
  TIMER2_CFG_R = 0x00000004;    // 2) configure for 16-bit timer mode
	TIMER2_TAMR_R = 0x00000007;   // 3) edge time capture mode: count down
  TIMER2_TAILR_R = MAX_DURATION;    // 4) start value
  TIMER2_ICR_R = 0x00000004;    // 6) clear TIMER2A capture and timeout flag
  TIMER2_IMR_R = 0x00000000;    // 7) disable capture mode event interrupt
  
	NVIC_PRI5_R = (NVIC_PRI5_R&0x1FFFFFFF)|0x80000000; // 8) priority 2
  // interrupts enabled in the main program after all devices initialized
  // vector number 39, interrupt number 23
  NVIC_EN0_R |= 0x800000;           // 9) enable IRQ 23 in NVIC
}

void Timer2A_Handler(void)
{
	TIMER2_ICR_R = TIMER_ICR_CAECINT;// acknowledge TIMER0A capture interrupt
	if ((GPIO_PORTB_DATA_R & 0x01)==0x01) { //rising edge
		first = TIMER2_TAR_R;  
		first_time = first; // this line of code is for debugging purpose, can be removed
		done = 0;
	}
	else if (first != 0){
		period = (first - TIMER2_TAR_R)&0x00FFFFFF; // 24 bits counter
		second_time = TIMER2_TAR_R; // this line of code is for debugging purpose, can be removed
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

void Delay(unsigned int t){unsigned long volatile time;
  time = (727240*200/91)/100000;  // 0.1sec
  while(time){
		time--;
  }
}


/******************************************************/





