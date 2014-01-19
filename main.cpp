extern "C" {
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include <driverlib/adc.h>
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>

#include "rc_cmds.h"
#include "timer.h"
#include "servo.h"
#include "INA226.h"
#include "Encoder.h"
#include "libraries/SoftPWM/soft_pwm.h"

#define SYSTICKS_PER_SECOND     1000

// HEARTBEAT
#define TICKS_PER_SECOND 		1000

// servo and drive
#define MAX_PWM_STEER			100
#define MAX_PWM_DRIVE			10000

// debug
#define DEBUG
//#define DEBUG_CMD
// use sensors
//#define USE_I2C
//#define USE_INA226
#define INA226_ALERT_PIN		GPIO_PIN_0
#define INA226_ALERT_PORT		GPIO_PORTE_BASE

#define MOSFET_PIN				GPIO_PIN_1
#define MOSFET_PORT				GPIO_PORTE_BASE

void configureGPIO(void);
uint32_t millis();

static unsigned long milliSec = 0;

void SysTickHandler()
{
	milliSec++;
}

uint32_t millis()
{
	return milliSec;
}

void InitConsole(void)
{
	//
	// Enable GPIO port A which is used for UART0 pins.
	// TODO: change this to whichever GPIO port you are using.
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Select the alternate (UART) function for these pins.
	// TODO: change this to select the port/pin you are using.
	//
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioInit(0);
}


}
// testes
int min1 = 1023, max1 = 0, min2 = 1023, max2 = 0;



#include "rf24/RF24.h"
#include "libraries/AMC6821/AMC6821.h"
#include "remote_defines.h"
static unsigned long ulClockMS=0;

unsigned long last_dongle_millis = 0, last_car_param_millis = 0;


double map_value(double x, double in_min, double in_max, double out_min, double out_max, bool trunc = false);

int main(void)
{
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_12MHZ); //50MHZ

	//
	// Enable peripherals to operate when CPU is in sleep.
	//
	MAP_SysCtlPeripheralClockGating(true);

	//
	// Configure SysTick to occur 1000 times per second, to use as a time
	// reference.  Enable SysTick to generate interrupts.
	//
	MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	MAP_SysTickIntEnable();
	MAP_SysTickEnable();

	//
	// Get the current processor clock frequency.
	//
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);

	InitConsole();

	RC_remote mini_rally;
	mini_rally.linear = 0;
	mini_rally.steer = 0;
	mini_rally.buttons = 0;


#ifdef DEBUG
	UARTprintf("Setting up PWM ... ");
#endif
	initSoftPWM(500,40);
	servo_init();
	enablePWM();
#ifdef DEBUG
	UARTprintf("Done!\n");
#endif

#ifdef DEBUG
	UARTprintf("Setting up ESC and Servo ... ");
#endif

	servo_setPosition(90);
	esc_setPosition(90);
#ifdef DEBUG
	UARTprintf("Done!\n");
#endif

	uint32_t last_millis = millis();
	uint32_t mosfet_millis = millis();
	bool mosfet_state = false;

#ifdef DEBUG
	UARTprintf("Setting up GPIO ... ");
#endif
	configureGPIO();
#ifdef DEBUG
	UARTprintf("Done!\n");
#endif

#ifdef USE_I2C
#ifdef DEBUG
	UARTprintf("Setting up I2C\n");
#endif
	//I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_3);
	I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),false);  //false = 100khz , true = 400khz
	I2CMasterTimeoutSet(I2C0_MASTER_BASE, 1000);
#ifdef DEBUG
	UARTprintf("Done!\n");
#endif
#endif

#ifdef USE_I2C
#ifdef USE_INA226
#ifdef DEBUG
	UARTprintf("Setting up INA226 ...");
#endif
	INA226 power_meter = INA226(0x45);
	power_meter.set_sample_average(4);
	power_meter.set_calibration_value(445);
	power_meter.set_bus_voltage_limit(7.0);
	power_meter.set_mask_enable_register(BUS_UNDER_LIMIT);
#ifdef DEBUG
	UARTprintf("Done!");
#endif
	AMC6821 cena;
#endif
#endif


#ifdef USE_NRF24
	RF24 radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[3] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL};

	// Setup and configure rf radio
	radio.begin();

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	radio.setPayloadSize(sizeof(RC_remote));

	radio.setDataRate(RF24_250KBPS);

	// Open pipes to other nodes for communication
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);

	// Start listening
	radio.startListening();

#ifdef DEBUG
	// Dump the configuration of the rf unit for debugging
	radio.printDetails();
#endif

#endif
	while (1)
	{
		// if there is data ready
		if ( radio.available() )
		{
			bool done = false;
			while (!done)
			{

				// Fetch the payload, and see if this was the last one.
				done = radio.read( &mini_rally, sizeof(RC_remote));

				if(done)
				{
					esc_setPosition(map_value(mini_rally.linear, -127, 127, ESC_MIN, ESC_MAX));
					servo_setPosition(map_value(mini_rally.steer, -127, 127, SERVO_MIN, SERVO_MAX));
					last_millis = millis();

					if((mini_rally.buttons & ASK_BIT) == ASK_BIT)
					{
						uint8_t temp;
						temp = GPIOPinRead(INA226_ALERT_PORT, INA226_ALERT_PIN);

#ifdef DEBUG
						UARTprintf("portE1 = %x\n", temp);
#endif
						temp = (temp & INA226_ALERT_PIN) == INA226_ALERT_PIN ? 0 : 1;

#ifdef DEBUG
						UARTprintf("sent = %d\n", mini_rally.buttons);
#endif
						radio.stopListening();
						radio.write(&temp, sizeof(uint8_t));
						radio.startListening();
					}
					//					//SysCtlDelay(50*ulClockMS);
				}
			}
		}
		// timeout case
		if(millis() - last_millis > THRESHOLD_BETWEEN_MSG)
		{
			esc_setPosition(ESC_ZERO);
			servo_setPosition(SERVO_ZERO);
		}

		// Switch ON/OFF mosfet
		if((mini_rally.buttons & BUTTONS_MOSFET_STATE) == BUTTONS_MOSFET_STATE)
		{
			if(millis() - mosfet_millis > 2000)
			{
				if(!mosfet_state)
				{
					GPIOPinWrite(MOSFET_PORT, MOSFET_PIN, MOSFET_PIN);
				}
				else
				{
					GPIOPinWrite(MOSFET_PORT, MOSFET_PIN, ~MOSFET_PIN);
				}
				mosfet_state = !mosfet_state;
				mosfet_millis = millis();
			}
		}
		else
		{
			mosfet_millis = millis();
		}


//		if(millis() - last_car_param_millis > CAR_PARAM_MILLIS)
//
		//		if(millis() - last_car_param_millis > CAR_PARAM_MILLIS)
		//		{
		//			last_car_param_millis = millis();
		//
		//			int16_t l_vel, r_vel;
		//			encoder_get_velocity(&l_vel, &r_vel, millis());
		//			car_param.velocity = (l_vel + r_vel)/2;
		//			car_param.batery_level = power_meter.get_bus_voltage();
		//			car_param.x = 0;
		//			car_param.y = 0;
		//		}
		//
		//		if(millis() - last_dongle_millis > DONGLE_MILLIS)
		//		{
		//			last_dongle_millis = millis();
		//
		//			radio.stopListening();
		//			radio.openWritingPipe(pipes[2]);
		//
		//			radio.write(&car_param, sizeof(RC_Param));
		//			radio.openWritingPipe(pipes[1]);
		//			radio.startListening();
		//		}
	}
}



void configureGPIO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//MAP_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// PE0 = Alert ina226
	// PE1 = MOSTFEt

	GPIOPinTypeGPIOInput(INA226_ALERT_PORT, INA226_ALERT_PIN);
	GPIOPinTypeGPIOOutput(MOSFET_PORT, MOSFET_PIN);



}

double map_value(double x, double in_min, double in_max, double out_min, double out_max, bool trunc)
{
	if(trunc)
	{
		double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		if(temp > out_max) temp = out_max;
		if(temp < out_min) temp = out_min;
		return temp;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
