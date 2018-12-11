#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "states.h"
	
char direction;
unsigned char s;
unsigned int lDuty, rDuty; 
int array[] = {0,0}; 

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
	{30, 45, {STRAIGHT, RIGHT, LEFT, STOP}},
	
	// Right
	{100, 2800, {STRAIGHT, RIGHT, LEFT, STOP}}, // 2800 w/ fresh batteries
	
	// Left
	{2060, 100, {STRAIGHT, RIGHT, LEFT, STOP}},
	
	// Stop
	{4960, 4960, {STRAIGHT, RIGHT, LEFT, STOP}}
};

int Forward(char wheel){
		s = fsm[s].NS[STRAIGHT];
		lDuty = fsm[s].LPWM; 
		rDuty = fsm[s].RPWM; 
		array[0] = lDuty;
		array[1] = rDuty; 

		return array[wheel]; 
}
	
int Right(char wheel){
		s = fsm[s].NS[RIGHT];
		lDuty = fsm[s].LPWM; 
		rDuty = fsm[s].RPWM; 
		array[0] = lDuty;
		array[1] = rDuty; 

		return array[wheel]; 
}
	
int Left(char wheel){
		s = fsm[s].NS[LEFT];
		lDuty = fsm[s].LPWM; 
		rDuty = fsm[s].RPWM; 
		array[0] = lDuty;
		array[1] = rDuty; 

		return array[wheel]; 
}
	
int Stop(char wheel){
		s = fsm[s].NS[STOP];
		lDuty = fsm[s].LPWM; 
		rDuty = fsm[s].RPWM; 
		array[0] = lDuty;
		array[1] = rDuty; 

		return array[wheel]; 
}
	


	