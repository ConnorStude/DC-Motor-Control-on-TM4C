#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include "delay.h"

#define TIMESLICE              32000  // thread switch time in system time units
																			// clock frequency is 16 MHz, switching time is 2ms	

int32_t LCD;
uint32_t data;
extern int32_t CurrentSize;
uint32_t Counter = 0;
uint32_t Key_ASCII;

char character1;
char character2;
char character3;
char character4;
char character1_T;
char character2_T;
char character3_T;
char character4_T;
char character1_C;
char character2_C;
char character3_C;
char character4_C;
volatile uint8_t  asciiArray[4];
volatile uint8_t  targetSpeedArray[4];
volatile uint8_t 	currentSpeedArray[4];
volatile uint8_t 	targetSpeedDisplay[4];
uint32_t ASCII2Hex(uint8_t*);
void Hex2ASCII(uint8_t*, uint32_t);
void Timer0_Config(uint32_t Reload);
//For LCD
void Init_LCD_Ports(void);
void Init_Keypad(void);
void Scan_Keypad(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Char(char);
void Display_Msg(char *Str);
void OS_Init(void);
void OS_AddThreads(void f1(void), void f2(void), void f3(void));
void OS_Launch(uint32_t);
void OS_InitSemaphore(int32_t *s, int32_t init_val);
void OS_Sleep(uint32_t SleepCtr);
void OS_Wait(int32_t *s);
void OS_Signal(int32_t *s);
void OS_FIFO_Init(void);
void First_Line_Display(uint8_t keyValue[], uint8_t indexArray);
int OS_Fifo_Put(uint32_t data);
void PWM1C_Duty(uint16_t duty);
uint32_t OS_Fifo_Get(void);
int32_t Current_speed(int32_t Avg_volt);
//uint32_t Get_Next(void);
uint8_t indexArray = 0;
char motorAscii;
uint8_t keyValue[5];
uint32_t Time;
int32_t currentSpeed;
uint32_t targetSpeed;
uint32_t speedError;
uint32_t U,I,P; // actuator duty cycle, 100 to 19900 cycles
uint32_t count;
int8_t adc_Code;
uint8_t voltageIndex = 0;
int32_t averageVoltage = 0;
int32_t measuredVoltage = 0;
uint32_t Reload;
void ADC_Port_Init(void);
void PWM_Init(uint16_t period, uint16_t duty);
uint16_t actuator;
void MOT12_Dir_Set(void);
int32_t voltageArray[99];

void Task1(void){//read data from ADC and gets measured speed
	while(1){

	}
}

void Task2(void){//controls keypad input
	while (1){
		// reset index
		indexArray = 0;
		
		// clear array
		for(int i = 0; i < 4; i++){
			asciiArray[i] = 0x00;
		}
		OS_Wait(&LCD);
		Set_Position(0x0c);
		Display_Msg("    ");
		OS_Signal(&LCD);
		
		// store keypresses in temporary array
		while(indexArray < 4){
			Scan_Keypad();
			OS_Sleep(100);
			if(Key_ASCII == 0x23){
				if(indexArray == 0){
					// ignore
				}
				else if(indexArray == 1){
						asciiArray[indexArray + 1] = 0x00;
						asciiArray[indexArray + 2] = 0x00;
						asciiArray[indexArray + 3] = 0x00;
						break;
				}
				else if(indexArray == 2){
						asciiArray[indexArray + 1] = 0x00;
						asciiArray[indexArray + 2] = 0x00;
						break;
				}
				else if(indexArray == 3){
						asciiArray[indexArray + 1] = 0x00;
						break;
				}
			}
			else if(Key_ASCII > 0x2F && Key_ASCII < 0x3A){
				asciiArray[indexArray] = Key_ASCII;
				indexArray++;
			}
			// Display input
			if(asciiArray[0] != 0x00){
				character1 = (char)asciiArray[0];				
				OS_Wait(&LCD);
				Set_Position(0x0c);
				Display_Char(character1);
				OS_Signal(&LCD);
			}
			if(asciiArray[1] != 0x00){
				character2 = (char)asciiArray[1];				
				OS_Wait(&LCD);
				Set_Position(0x0d);
				Display_Char(character2);
				OS_Signal(&LCD);
			}
			if(asciiArray[2] != 0x00){
				character3 = (char)asciiArray[2];				
				OS_Wait(&LCD);
				Set_Position(0x0e);
				Display_Char(character3);
				OS_Signal(&LCD);
			}
			if(asciiArray[3] != 0x00){
				character4 = (char)asciiArray[3];				
				OS_Wait(&LCD);
				Set_Position(0x0e);
				Display_Char(character4);
				OS_Signal(&LCD);
			}
		}
		
		
		// store key presses in array for target speed
		for(int i = 0; i < 4; i++){
			targetSpeedArray[i] = 0x00;
		}
		for(int i = 0; i < 4; i++){
			targetSpeedArray[i] = asciiArray[i];
		}
		
		//convert ascii characters for target speed in to target speed rpm
		targetSpeed = ASCII2Hex(targetSpeedArray);
		if (targetSpeed > 2400){
			targetSpeed = 2400;
		}
		else if (targetSpeed < 400 && targetSpeed > 0){
			targetSpeed = 400;
		}
		else{
			targetSpeed = targetSpeed;
		}
		Hex2ASCII(targetSpeedDisplay, targetSpeed);
	}
}

void Task3(void){//controls LCD
	while(1){
		OS_Sleep(10);
		if(targetSpeedDisplay[0] == 0x00){
				OS_Wait(&LCD);
				Set_Position(0x43);
				Display_Char(' ');
				OS_Signal(&LCD);
		}
		else if(targetSpeedDisplay[0] != 0x00){
				character1_T = (char)targetSpeedDisplay[0];				
				OS_Wait(&LCD);
				Set_Position(0x43);
				Display_Char(character1_T);
				OS_Signal(&LCD);
		}
		if(targetSpeedDisplay[1] == 0x00){
				OS_Wait(&LCD);
				Set_Position(0x44);
				Display_Char(' ');
				OS_Signal(&LCD);
		}
		else if(targetSpeedDisplay[1] != 0x00){
				character2_T = (char)targetSpeedDisplay[1];				
				OS_Wait(&LCD);
				Set_Position(0x44);
				Display_Char(character2_T);
				OS_Signal(&LCD);
		}
		if(targetSpeedDisplay[2] == 0x00){
				OS_Wait(&LCD);
				Set_Position(0x45);
				Display_Char(' ');
				OS_Signal(&LCD);
		}
		else if(targetSpeedDisplay[2] != 0x00){
				character3_T = (char)targetSpeedDisplay[2];				
				OS_Wait(&LCD);
				Set_Position(0x45);
				Display_Char(character3_T);
				OS_Signal(&LCD);
		}
		if(targetSpeedDisplay[3] == 0x00){
				OS_Wait(&LCD);
				Set_Position(0x46);
				Display_Char(' ');
				OS_Signal(&LCD);
		}
		else if(targetSpeedDisplay[3] != 0x00){
				character4_T = (char)targetSpeedDisplay[3];				
				OS_Wait(&LCD);
				Set_Position(0x46);
				Display_Char(character4_T);
				OS_Signal(&LCD);
		}
	}
}

int main(void){
  OS_Init();  
	Reload = 1599;
	
	Init_LCD_Ports();
  Init_LCD();
	Set_Position(0x00);
	Display_Msg("Input RPM: ");
	Set_Position(0x40);
	Display_Msg("T: ");
	Set_Position(0x48);
	Display_Msg("C: ");
	Init_Keypad();
	PWM_Init(2500,1250);
	MOT12_Dir_Set();
	PWM1C_Duty(1250);
	ADC_Port_Init();
	Timer0_Config(Reload);
  OS_InitSemaphore(&LCD, 1);
  OS_FIFO_Init();
  OS_AddThreads(&Task1, &Task2, &Task3);
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

void TIMER0A_Handler(void){
	
	// Take ADC reading when entering Handler
	GPIO_PORTC_DATA_R &= ~0x80; //PC7 to low
			
	int8_t busy = GPIO_PORTE_DATA_R & 0x02;
	while (busy == 0x0){//conversion done when busy != 0
		busy = GPIO_PORTE_DATA_R & 0x02;		
	}
	
	GPIO_PORTC_DATA_R |= 0x80; //PC7 to high
	//8 MSB is valid data should be latched
	//8 MSB and give to measuredVoltage in 8 bit signed integer
	
	adc_Code = ((GPIO_PORTA_DATA_R & 0x80) + (GPIO_PORTA_DATA_R & 0x40) + (GPIO_PORTB_DATA_R & 0x20) + (GPIO_PORTB_DATA_R & 0x10) + (GPIO_PORTB_DATA_R & 0x08) + (GPIO_PORTB_DATA_R & 0x04) + (GPIO_PORTF_DATA_R & 0x02) + (GPIO_PORTF_DATA_R & 0x01)); 
	
	// Convert ADC reading to voltage (map ADC code to voltage from -10 to 10 V)
	measuredVoltage = (((adc_Code + 128) / 255) * (10 - (-10)) + (-10));
	
	// Convert voltage from V to mV
	measuredVoltage = measuredVoltage * 1000;
	
	// Store measured voltage in array
	voltageArray[voltageIndex] = measuredVoltage;
	voltageIndex++;
	
	// Find average voltage
	if(voltageIndex > 99){
		for(int i = 0; i < 100; i++){
			averageVoltage += voltageArray[i];
		}
		voltageIndex = 0; // reset voltage index
	}
	// Convert average voltage to RPM (currentSpeed);
	currentSpeed = Current_speed(averageVoltage);
	U = currentSpeed/targetSpeed*100;
	
	speedError = targetSpeed-currentSpeed;
	if(speedError < -10){
		 U--;// decrease if too fast
	}
	else if(speedError > 10){
		U++;// increase if too slow
	}
		// leave as is if close enough
	if(U<2)
		U=2; // underflow (minimum PWM)
	if(U>249)
		U=249; // overflow (maximum PWM)
	PWM1C_Duty(U); // output to actuator
	TIMER0_ICR_R = 0x01;
// acknowledge timer0A periodic timer
	return;
}

void PWM_Init(uint16_t period, uint16_t duty) {
	SYSCTL_RCGCPWM_R |= 0x02; // activate pwm module 1
  SYSCTL_RCGCGPIO_R |= 0x20; // activate port f
	SYSCTL->RCGCGPIO |= 0x02;
  delayMs(1); // wait for PWM1 to start
  GPIO_PORTF_AFSEL_R |= 0x04; // choose alternate function for PF2
  GPIOF->DIR |= 0x04;
	GPIOF->PCTL &= ~0x00000F00; // clear PF2 alternate function
  GPIOF->PCTL |= 0x00000500; // set PF2 alternate function to PWM
  GPIO_PORTF_AMSEL_R &= ~0x04;// disable analog for pf2
  GPIO_PORTF_DEN_R |= 0x04; // enable pf2 as digital
  SYSCTL->RCC &= ~0x00100000;
  SYSCTL->RCC |= 0x00170000;
  PWM1->_3_CTL = 0; // disable PWM1_3 during configuration
  PWM1->_3_GENA = 0x000000C8; // output low for load, high for match
  PWM1->_3_LOAD = period-1;
  PWM1->_3_CMPA = duty-1;
  PWM1->_3_CTL = 1; // enable PWM1_3
  PWM1->ENABLE |= 0x40; // enable PWM1M6
	GPIOB->DEN |=0x03;
	GPIOB->DIR |=0x03;
}

void MOT12_Dir_Set(void){
	GPIOB->DATA &= ~0x01;
	GPIOB->DATA |=  0x02;
}

void PWM1C_Duty(uint16_t duty){
      PWM1_3_CMPA_R = duty - 1;
}

void ADC_Port_Init (void){
	SYSCTL_RCGCGPIO_R |= 0x37;            // activate clock for Ports A & B
  while((SYSCTL_RCGCGPIO_R&0x37) == 0){} // allow time for clock to stabilize
  GPIO_PORTA_DIR_R &= ~0xC0;             // make PA7/6 input
  GPIO_PORTA_DEN_R |= 0xC0;             // enable digital I/O on PA7/6
	GPIO_PORTB_DIR_R &= ~0x3C;							//make PB5-2 input  
	GPIO_PORTB_DEN_R |= 0x3C;              // enable digital I/O on PB5-0	
	GPIO_PORTC_DIR_R |= 0x80;								//make PC7 output
	GPIO_PORTC_DEN_R |= 0x80;
	GPIO_PORTE_DIR_R &= ~0x02;							//make PE1 input for BUSY bit (pin24)
	GPIO_PORTE_DEN_R |= 0x02;
	GPIO_PORTF_LOCK_R = 0x4c4f434b;					// Unlock PF
	GPIO_PORTF_DIR_R &= ~0x03;
	GPIO_PORTF_DEN_R |= 0x03;
}

void Timer0_Config(uint32_t Reload){
	// Reload = Clock Frequency * Periodic Time - 1
	__disable_irq();// disable interrupts globally
	SYSCTL_RCGCTIMER_R |= 0x01;	// Activate timer by setting bit 0
	TIMER0_CTL_R &= ~0x01;			// Clear bit 0 in timer control regsiter to disable counting
	TIMER0_CFG_R = 0x00000000;		// write 0x00000000 to timer configuration register.
	TIMER0_TAMR_R = (TIMER0_TAMR_R & ~0x03) | 0x02; //
	TIMER0_TAILR_R = Reload;			// write Reload value in interval load register
	TIMER0_TAPR_R = 0x00;				// no prescale;
	TIMER0_IMR_R |= 0x01;					// Arm interrupt bit of Timer0A
	NVIC_PRI4_R = (NVIC_PRI4_R&0x0FFFFFFF)|0x40000000; // priority value 2;
	NVIC_EN0_R |= 0x80000;			// enable Timer0A_Handler;
	TIMER0_CTL_R |= 0x01;
	__enable_irq();//enable interrupts globally
}
