#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "adc_ctrl.h"
#include "pins.h"
#include "ui.h"
#include "ui_spi.h"
#include "irrigation.h" 


int g_ulIrrigationCurrent;
int g_ulIrrigationEnable = 0;
int g_ulRelayEnable = 0;
int g_ulIrrigationLevel;

void InitExpandedIO()
{ 
	// set CS as output and set to disable
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
	
	//enable the CS and setup porta as output
	//enable the CS and setup porta as output

	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
	SpiWrite(EXPANDEDIO_DIRA);
	SpiWrite(EXPANDEDIO_IOA);

	SysCtlDelay(EXPANDEDIO_CS_DELAY);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
	
	// enable the relay and disable the irrigation, clear watchdog latch
	ExpandedIOUpdate(EXPANDEDIO_PORTA,EXPANDEDIO_RELAY_ENABLE | EXPANDEDIO_IRRIGATION_DISABLE |
	        EXPANDEDIO_CUTTER_DISABLED | EXPANDEDIO_WD_CLEAR);

	// release handpiece from reset

	//enable the CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
	SpiWrite(EXPANDEDIO_DIRB);
	SpiWrite(EXPANDEDIO_IOB);
	SysCtlDelay(EXPANDEDIO_CS_DELAY);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);

	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
	SpiWrite(EXPANDEDIO_WRITE_PORTB);
	SpiWrite(EXPANDEDIO_RELEASE_HANDPIECE);
	SysCtlDelay(EXPANDEDIO_CS_DELAY);

	// disable the CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
}

void ExpandedIOUpdate(int port, unsigned int ioStatus)
{    
    //enable the CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);	
	
	//write io status
	if(port == EXPANDEDIO_PORTA)
		SpiWrite(EXPANDEDIO_WRITE_PORTA);
	else
		SpiWrite(EXPANDEDIO_WRITE_PORTB);

	SpiWrite(ioStatus);	
	
	// update the flags for external use,
	// notice the relay status might be changed after this point
	// so use this flag with caution
	if(port == EXPANDEDIO_PORTA)
	{
		if(ioStatus & EXPANDEDIO_IRRIGATION_ENABLE)
		{
			g_ulIrrigationEnable =1;
		}
		else
		{
			g_ulIrrigationEnable =0;
		}

		if(ioStatus & EXPANDEDIO_RELAY_DISABLE)
		{
			g_ulRelayEnable =0;
		}
		else
		{
			g_ulRelayEnable =1;
		}
	}
	SysCtlDelay(EXPANDEDIO_CS_DELAY);
	
	// disable the CS	
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);

}



int IrrInit(void)
{	
	//
    // Configure the irrigation control pin output
    //
    // set up CS pin as output, and disable it
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);    
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3); 
    
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);    
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6); 
    
    //
    //init spi interface
    //
    SpiInit(); 
    
    //init the expanded io port for irrigation and relay control    
    InitExpandedIO();

    // set current limit, this is roughly (2.9-1.2)/4/0.015 = 28 Amps
    // if consider a 10% error, the low limit will be 26 Amps
    IrrSetCurrentLevel(14);

    //set default irrigation voltage to ~ 7.3 V
    IrrSetLevel(255);
     
    //
    // intialize the ADC
    //
    
    // data line as input
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);                  
                     
    // CS and CLK line as output
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 );
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 , GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
    
    return 1;
}

int IrrSetLevel(int level)
{
	unsigned short cmd = 0x0000;
	unsigned short rev = 0x00; 
	
	//send command    
    cmd += level;
    
    //enable the CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
	
	if(SpiWrite(cmd) == -1)
	{
		return -1;
	}
	
    if(SpiRead(&rev) == -1)
	{
		return -1;
	}
	SysCtlDelay(EXPANDEDIO_CS_DELAY);
	
	//disable the CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	
	if( rev != 0xFFFF)
	{
		return -1;
	}
	return 1;
}


int IrrSetCurrentLevel(int level)
{
	unsigned short cmd = 0x1000;
	unsigned short rev = 0x00; 
	
	//send command    
    cmd += level * 0x0010;
	
	//enable CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
	
	if(SpiWrite(cmd) == -1)
	{
		return -1;
	}
	
    if(SpiRead(&rev) == -1)
	{
		return -1;
	}
	
	SysCtlDelay(EXPANDEDIO_CS_DELAY);
	
	//disable CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	
	if( rev != 0xFFFF)
	{
		return -1;
	}
	return 1;
}

int IrrGetLevel(int level)
{
	unsigned short cmd = 0x0C00;
	unsigned short rev = 0x00;
	
	//enable CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
	
    //send the read command
	if(SpiWrite(cmd) == -1)
	{
		return -1;
	}
	
	//read the output
	if(SpiRead(&rev) == -1)
	{
		return -1;
	}
	
	SysCtlDelay(EXPANDEDIO_CS_DELAY);
	
	//disable CS
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	
	//compare the result unless level is zero
	if ( !level )
	{
		if (level != rev & 0x1FF ) 
		{
		    return -1;
		}
	}
	return 1;
}

int IrrReadCurrent()
{
	unsigned int temp = 0; 
	int i,j;
	static int ioStatus = 0;
	// enable chip select
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
	
	for( i =0, j =0; i< 32; i++)
	{
		if(ioStatus)
		{
		    temp |= ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)>> 7) & 0x01) << (15-j);
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
            SysCtlDelay(500);
            j += 1;
		}
		else
		{
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
			SysCtlDelay(500);
		}       
        ioStatus ^= 1;     
	}
	
	// reset clock
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
	
	// disable chip select
	SysCtlDelay(EXPANDEDIO_CS_DELAY);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);

	return temp;	
}




