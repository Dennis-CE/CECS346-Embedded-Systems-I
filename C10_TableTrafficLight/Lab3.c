// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
//#include "TExaS.h"
#include "SysTick.h"
#include "tm4c123gh6pm.h"

//PORT B LEDs Red, green, yellow
#define GPIO_PORTB_DATA_R 		(*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R 			(*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R 		(*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_PUR_R 			(*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_DEN_R 			(*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_CR_R 			(*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R 		(*((volatile unsigned long *)0x40005528))

//PORT E switches
#define GPIO_PORTE_DATA_R 		(*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R 			(*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R 		(*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R 			(*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R 			(*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_CR_R 			(*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R 		(*((volatile unsigned long *)0x40024528))
#define RED 		0x04
#define GREEN		0x01
#define YELLOW  0x02
#define SWITCH1 1 //switch 1
#define SWITCH2 2 // switch 2
#define SWITCH3 3 //Both switches


// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
//void DisableInterrupts(void); // Disable interrupts
//void EnableInterrupts(void);  // Enable interrupts
void PortB_Init(void);
void PortE_Init(void);
void nextLight(void);
void Delay(float seconds);

// Global Variables
unsigned long in;


// ***** 3. Subroutines Section *****

int main(void){ 
	unsigned long mask = 0x0;
	
  //TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	PortB_Init();
	PortE_Init();
  GPIO_PORTB_DATA_R = RED;
	
  //EnableInterrupts();
  while(1){
		in = GPIO_PORTE_DATA_R & 0x03; // read PB4/PB0 into In
		
		switch(in){
			case SWITCH1:
				nextLight();
				Delay(1);
				break;
			case SWITCH2:
				mask = GPIO_PORTB_DATA_R;
				GPIO_PORTB_DATA_R = 0x0;
				Delay(0.5);
				GPIO_PORTB_DATA_R = mask;
				Delay(0.5);
				//nextLight();
				break;
			case SWITCH3:
				mask = GPIO_PORTB_DATA_R;
				GPIO_PORTB_DATA_R = 0x0;
				Delay(1);
				GPIO_PORTB_DATA_R = mask;
				Delay(1);
				nextLight();
				break;
			default:
				Delay(1);
				break;
		}
  }
}

void nextLight(void)
{
	if(GPIO_PORTB_DATA_R == RED)
		GPIO_PORTB_DATA_R = GREEN;
	else if(GPIO_PORTB_DATA_R == GREEN)
		GPIO_PORTB_DATA_R = YELLOW;
	else
		GPIO_PORTB_DATA_R = RED;
}

// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
void PortB_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000002;     // - 1) B clock
  delay = SYSCTL_RCGC2_R;           // - delay   
  //GPIO_PORTB_LOCK_R = 0x4C4F434B;   //- 2) unlock PortB PB0  
  GPIO_PORTB_CR_R = 0xFF;           //  - allow changes to PB3-1       
  GPIO_PORTB_AMSEL_R = 0x00;        //  - 3) disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // - 4) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R = 0x07;          //- 5) PB3,PB2,PB1 output  PB4, PB0 not used
  GPIO_PORTB_AFSEL_R = 0x00;        //- 6) no alternate function
	//internal resistor
  //GPIO_PORTB_PUR_R = 0x11;          // enable pullup resistors on PB4,PB0       
  GPIO_PORTB_DEN_R = 0x07;          // - 7) enable digital pins PB3-PB1        
}

// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
void PortE_Init(void){ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000010;     // -1) E clock
  delay = SYSCTL_RCGC2_R;           // -delay   
  //GPIO_PORTE_LOCK_R = 0x4C4F434B;   //- 2) unlock PortE PE0  
  GPIO_PORTE_CR_R = 0x3F;           // -allow changes to PE4 and PE0       
  GPIO_PORTE_AMSEL_R = 0x00;        // - 3) disable analog function
  GPIO_PORTE_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTE_DIR_R = 0xFC;          // 5) PE4,PE0 input    PE3,PE1 not used
  GPIO_PORTE_AFSEL_R = 0x00;        // 6) no alternate function
  //GPIO_PORTE_PUR_R = 0x11;          // enable pullup resistors on PE4,PE0       
  GPIO_PORTE_DEN_R = 0x03;          // 7) enable digital pins PE 4 and PE0        
}
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06

// Subroutine to wait 0.N sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(float seconds){
	unsigned long volatile time;
  time = 727240*seconds*100/91;  // seconds* 0.1sec
  while(time){
		time--;
  }
}
