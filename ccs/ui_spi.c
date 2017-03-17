#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "ui_spi.h"

#define TIMEOUTCNT 1000

int SpiInit()
{
	unsigned long tempData;
	
    //
    // The SSI0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    //
//    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
//    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
//    GPIOPinConfigure(GPIO_PA4_SSI0RX);
//    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    //
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);

                   
   //
    // Disable the SSI0 module.
    //
    SSIDisable(SSI0_BASE);

    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 16-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 4000000, 16);

    //
    // Enable the SSI0 module.
    //
    SSIEnable(SSI0_BASE);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while (SSIDataGetNonBlocking(SSI0_BASE, &tempData)){}
    
    return 1;    
}
//
//int SpiWriteByte(unsigned char writeByte)
//{
//    int timeoutCnt=0;
//    //
//    // Send the data using the "blocking" put function.  This function
//    // will wait until there is room in the send FIFO before returning.
//    // This allows you to assure that all the data you send makes it into
//    // the send FIFO.
//    //
//    SSIDataPut(SSI0_BASE, (unsigned long)writeByte);    
//
//    //
//    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
//    //
//    while(!SSIBusy(SSI0_BASE))
//    {
//    	SysCtlDelay(1000);
//    	if(timeoutCnt++ > TIMEOUTCNT) return 0;
//    }
//    
//    return 1;
//}
//
//int SpiReadByte(unsigned char *readByte)
//{
//    
//    int ne, cnt=0;
//    unsigned long rData;
//    //
//    // Receive the data using the "non blocking" Get function. 
//    //
//    do
//    {
//    	ne= SSIDataGetNonBlocking(SSI0_BASE, &rData);  	
//    	if(cnt++ > TIMEOUTCNT) return 0;
//    }while(ne == 0);
//
//    //
//    // Mask off the MSB.
//    //
//    *readByte = (char)(rData & 0xFF);
//    
//    return 1;
//}


int SpiWrite(unsigned short wData)
{
    int timeoutCnt=0;
    //
    // Send the data using the "blocking" put function.  This function
    // will wait until there is room in the send FIFO before returning.
    // This allows you to assure that all the data you send makes it into
    // the send FIFO.
    //
    SSIDataPut(SSI0_BASE, (unsigned long)wData);    

    //
    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    //
    while(!SSIBusy(SSI0_BASE))
    {
    	SysCtlDelay(1000);
    	if(timeoutCnt++ > TIMEOUTCNT) return 0;
    }
    
    return 1;
}

int SpiRead(unsigned short *rData)
{
    
    int ne, cnt=0;
    unsigned long rDataTemp;
    //
    // Receive the data using the "non blocking" Get function. 
    //
    do
    {
    	ne= SSIDataGetNonBlocking(SSI0_BASE, &rDataTemp);  	
    	if(cnt++ > TIMEOUTCNT) return 0;
    }while(ne == 0);

    //
    // Mask off the MSB.
    //
    *rData = (char)(rDataTemp & 0xFFFF);
    
    return 1;
}

