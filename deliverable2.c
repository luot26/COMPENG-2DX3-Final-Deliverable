// Written by Timothy Luo - 400460844
// Demoed on April 5th, 2024
// Measurement LED - PF4/D3
// Additional Status LED - PN1/D1
// Assigned Bus Speed - 24MHz


#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5          // number of receive attempts before giving up

int on = 0;              // Optional on variable for stepper motor, set to 1 by default

int stepCount = 0;                          // Step Count for stepper motor
uint32_t delay = 2;
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
int data[96];
int index = 0;
uint16_t wordData;
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;
uint8_t dataReady;
uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
	
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    //FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void PortF_Init(void){
	//Use PortF onboard LEDs
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;						// Activate clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0x03;        										
	GPIO_PORTF_AFSEL_R &= ~0x03;     										
  GPIO_PORTF_DEN_R |= 0x03;        										

  GPIO_PORTF_AMSEL_R &= ~0x03;     										
	return;
}
// Initialize Port H for Stepper Motor
void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x0F;        								// configure Port M pins (PM0-PM3) as output
  GPIO_PORTM_AFSEL_R &= ~0x0F;     								// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x0F;        								// enable digital I/O on Port M pins (PM0-PM3)
																									// configure Port M as GPIO
  GPIO_PORTM_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	return;
}

void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};				// Allow time for clock to stabilize
		
	GPIO_PORTN_DIR_R=0b00000011;															// Enable PN0 and PN1 as outputs													
	GPIO_PORTN_DEN_R=0b00000011;															// Enable PN0 and PN1 as digital pins
	return;
}

void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;						// Activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTL_DIR_R &= 0xFFFFFFF0;        							// Make PM0 - PM3 in 
	GPIO_PORTL_AFSEL_R &= ~0x0F;     										// Disable alt funct on PM[0:3]
	GPIO_PORTL_DEN_R |= 0x0F;        										// Enable digital I/O on PM[0:3]
  GPIO_PORTL_AMSEL_R &= ~0x0F;     										// Disable analog functionality on PN[0:1]
	return;
}

void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;						// Activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        										// Make PH0 - PH3 out 
	GPIO_PORTH_AFSEL_R &= ~0x0F;     										// Disable alt funct on PH[0:3]
	GPIO_PORTH_DEN_R |= 0x0F;        										// Enable digital I/O on PH[0:3]
																									
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTH_AMSEL_R &= ~0x0F;     										// Disable analog functionality on PN[0:1]
	FlashLED1(1);																				// Flash LED D1 (Hello World)
	return;
}


void spin_Motor(int delay){
	int handleError;
	for (int i=0; i<512; i++){							
		if((i%(512/16)) == 0){ // if at 11.25 degrees, measure distance
			FlashLED3(1);
			GPIO_PORTN_DATA_R = 0b00000010; // Turn on PN0 LED
			
			while(sensorState==0){
				status = VL53L1X_BootState(dev, &sensorState);
				SysTick_Wait10ms(10);
			}
				
			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
			
			/* 2 Initialize the sensor with the default setting  */
			status = VL53L1X_SensorInit(dev);
			Status_Check("SensorInit", status);

			status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
									
		while (dataReady == 0){
			status = VL53L1X_CheckForDataReady(dev, &dataReady);
			FlashLED3(1);
			VL53L1_WaitMs(dev, 5);
		}
		dataReady = 0;
			
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
		if (status != 0){
			handleError = 0;
		}
		status = VL53L1X_GetDistance(dev, &Distance);					//7 The Measured Distance value
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);
		
		VL53L1X_ClearInterrupt(dev); // 8 clear interrupt has to be called to enable next interrupt
		SysTick_Wait10ms(delay);
		
		// error handling
		if (handleError == 0){
			Distance = 9999;
		}
		
		sprintf(printf_buffer,"%u\r\n", Distance);
		UART_printf(printf_buffer);
		SysTick_Wait10ms(1);
		index++;
		}
		
		GPIO_PORTM_DATA_R = 0b00000011;
		SysTick_Wait10ms(delay);											// What if we want to reduce the delay between steps to be less than 10 ms?
		GPIO_PORTM_DATA_R = 0b00000110;
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00001100;
		SysTick_Wait10ms(delay);
		GPIO_PORTM_DATA_R = 0b00001001;
		SysTick_Wait10ms(delay);
		stepCount++;
		
		}

		VL53L1X_StopRanging(dev);
		
		SysTick_Wait10ms(10);
	
		GPIO_PORTN_DATA_R = 0b00000000;
	
		for(int i = 0; i < 512; i++)
		{
			GPIO_PORTM_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			
			GPIO_PORTM_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			
			GPIO_PORTM_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);	

			GPIO_PORTM_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);		
		}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
  PortM_Init();
	PortN_Init();
	PortF_Init();
	PortG_Init();
	PortL_Init();
	PortH_Init();
	uint8_t scan_num = 0;
	FlashAllLEDs();

			SysTick_Wait10ms(500);
			FlashLED1(1);
			SysTick_Wait10ms(500);
			FlashLED1(1);
			
	// Wait for device ToF booted
	while(1){
		int button_press = GPIO_PORTL_DATA_R &= 0xF;
		
		if (button_press == 0x1){
			SysTick_Wait10ms(1);
			while((GPIO_PORTL_DATA_R &= 0xF) != 0){}
			on = 1;
			SysTick_Wait10ms(1);
		}
		
		if ((on == 1)){
			spin_Motor(2);
			scan_num++;
			on = 0;
			}
		if (scan_num >= 3){
			break;
		}
		SysTick_Wait10ms(500);
		GPIO_PORTH_DATA_R ^= 1;
	}	
	
}

