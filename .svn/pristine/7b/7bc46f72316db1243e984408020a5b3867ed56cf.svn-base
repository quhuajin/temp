#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "string.h"
#include "faults.h"
#include "main.h"
#include "ui.h"
#include "stdio.h"

#define UART_IDLE  0
#define UART_READ  1
#define UART_WRITE 2


//header length = 1(0xff) + header string length + 1(number of byte)
#define UART_HEADER_LENGTH 9
#define UART_HEADER_STRING_LENGTH 7
#define UART_WREAD_LENGTH 7
#define UART_SREAD_LENGTH 24
#define UART_CREAD_LENGTH 64
#define UART_MAX_WRITE_LENGTH 128
#define UART_MAX_RECV_LENGTH 128

volatile int uartState = UART_IDLE;

char uartCmdHeader[] = "CommRpl";
char uartStrmHeader[]= "StrmRpl";

typedef struct
{
	char rcvBuff[UART_MAX_RECV_LENGTH];
	char rcvBuffStrm[UART_SREAD_LENGTH];
    char rcvBuffCmd[UART_CREAD_LENGTH];
    char rcvBuffHeader[UART_HEADER_LENGTH];
    char hTop;     //header string current position
    char sPos;     //stream receive data current postion
    char cPos;     //command receive buffer current position
    char slength;  //stream receive data section length
    char clength;  //command receive data section length
    char hType;    //hearder type
    volatile char sReadDone;//flag for completed stream read
    volatile char cReadDone;//flag for compeleted command read
    char needNewHeader;
    char head;
    char tail;
}uartRecvBufStruct;

uartRecvBufStruct uartRecvBuf;

//*****************************************************************************
//
// The UART receiving processor.
//
//*****************************************************************************
int uart_process_rcv(void)
{
	char tChar;

    //check if buffer is empty

    while (uartRecvBuf.head != uartRecvBuf.tail)
    {

    	if(uartRecvBuf.head >= UART_MAX_RECV_LENGTH)
    	{
    		uartRecvBuf.head -= UART_MAX_RECV_LENGTH;
    	}

    	// get one byte from the buffer
    	tChar = uartRecvBuf.rcvBuff[uartRecvBuf.head++];

    	// shift the char to appropriate buffer
    	if(uartRecvBuf.hType ==1)
    	{
    		//copy over the last byte
    		uartRecvBuf.rcvBuffCmd[uartRecvBuf.cPos] = tChar;

    		if(uartRecvBuf.cPos == uartRecvBuf.clength -1)
    		{
    			// done reading, reset flags
    			uartRecvBuf.cReadDone =1;
    			uartRecvBuf.cPos =0;
    			uartRecvBuf.hTop =0;
    			uartRecvBuf.hType =0;
    			uartRecvBuf.needNewHeader =1;
    			break;
    		}
    		else
    		{
    			uartRecvBuf.cPos++;

    		}
    	}
    	else if(uartRecvBuf.hType ==2)
    	{
    		//copy over the last byte
    		uartRecvBuf.rcvBuffStrm[uartRecvBuf.sPos] = tChar;

    		if(uartRecvBuf.sPos == uartRecvBuf.slength-1)
    		{
    			// done reading, reset flags
    			uartRecvBuf.sReadDone =1;
    			uartRecvBuf.sPos =0;
    			uartRecvBuf.hTop =0;
    			uartRecvBuf.hType =0;
    			uartRecvBuf.needNewHeader =1;
    			break;
    		}
    		else
    		{
    			uartRecvBuf.sPos++;
    		}
    	}

    	// start accumulate the header by reset header top position.
    	if( uartRecvBuf.needNewHeader )
    	{
    		if(tChar == 0xfe)
    		{
    			uartRecvBuf.hTop =0;
    		}

    		if(uartRecvBuf.hTop < UART_HEADER_LENGTH)
    		{

    			// collect header
    			uartRecvBuf.rcvBuffHeader[uartRecvBuf.hTop] = tChar;

    		}

    		if(uartRecvBuf.hTop == UART_HEADER_LENGTH -1)
    		{
    			//decide which buffer to use
    			if(strncmp((char *)(&uartRecvBuf.rcvBuffHeader[1]),uartCmdHeader,7)==0)
    			{
    				//last byte in the header is the data length
    				uartRecvBuf.clength = (int)uartRecvBuf.rcvBuffHeader[uartRecvBuf.hTop];
    				uartRecvBuf.hType =1;
    				memcpy(uartRecvBuf.rcvBuffCmd,uartRecvBuf.rcvBuffHeader,UART_HEADER_LENGTH);
    				uartRecvBuf.cPos = UART_HEADER_LENGTH;
    				uartRecvBuf.needNewHeader =0;
    			}
    			else if(strncmp((char *)(&uartRecvBuf.rcvBuffHeader[1]),uartStrmHeader,7)==0)
    			{
    				uartRecvBuf.slength = (int)uartRecvBuf.rcvBuffHeader[uartRecvBuf.hTop];
    				if(uartRecvBuf.slength != UART_SREAD_LENGTH)
    				{
    					uartRecvBuf.hTop =0;
    					uartRecvBuf.slength = 0;
    				}
    				else
    				{
    					uartRecvBuf.hType =2;
    					memcpy(uartRecvBuf.rcvBuffStrm,uartRecvBuf.rcvBuffHeader,UART_HEADER_LENGTH);
    					uartRecvBuf.sPos = UART_HEADER_LENGTH;
    					uartRecvBuf.needNewHeader =0;
    				}
    			}
    			else //wrong header, keep collecting
    			{
    				uartRecvBuf.hTop =0;
    			}

    		}
    		else
    		{
    			uartRecvBuf.hTop++;
    		}
    	}

    }

    //check if a read is complete, and return accordingly
    if(uartRecvBuf.cReadDone || uartRecvBuf.sReadDone)
    {
    	return 1;
    }
    else
    {
    	return -1;
    }
}

	
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    unsigned long ulStatus;
    char tempChar;

    uartState = UART_READ;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Check interrupt type and process accordingly
    //
    if(ulStatus & (UART_INT_RX | UART_INT_RT))
    {
    	//
    	// Loop while there are characters in the receive FIFO.
    	//
    	while(UARTCharsAvail(UART0_BASE))
    	{
    		tempChar = UARTCharGetNonBlocking(UART0_BASE);

    		if(uartRecvBuf.tail >= UART_MAX_RECV_LENGTH)
    		{
    			uartRecvBuf.tail = uartRecvBuf.tail - UART_MAX_RECV_LENGTH;
    		}

    		uartRecvBuf.rcvBuff[uartRecvBuf.tail++] = tempChar;

    	}
    }
    else if(ulStatus)
    {
    	//clear the error
    	UARTRxErrorClear(UART0_BASE);
    }
    uartState = UART_IDLE;   
}

/*
 * Function to initialized the uart buffer
 */

void ui_uart_init_buffer(void)
{
    //
    // Init Receive buffer
    //
    uartRecvBuf.hTop = 0;
    uartRecvBuf.hType =0; //default is streaming
    uartRecvBuf.sReadDone =0;
    uartRecvBuf.cReadDone =0;
    uartRecvBuf.needNewHeader =1;
    uartRecvBuf.slength =0;
    uartRecvBuf.clength =0;
    uartRecvBuf.head =0;
    uartRecvBuf.tail =0;
}
/*
 * Function to initialize uart
 */

int ui_uart_init(void)
{

    //
    // Enable the peripherals used by this example.
    // The UART itself needs to be enabled, as well as the GPIO port
    // containing the pins that will be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the GPIO pin muxing for the UART function.
    // This is only necessary if your part supports GPIO pin function muxing.
    // Study the data sheet to see which functions are allocated per pin.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Since GPIO A0 and A1 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    // This function uses SysCtlClockGet() to get the system clock
    // frequency.  This could be also be a variable or hard coded value
    // instead of a function call.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    //
    // set Tx and Rx buffer depth
    //
	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX2_8);

	//
	//Clear the Read buffer
	//
	while (UARTCharsAvail(UART0_BASE))
	{
		UARTCharNonBlockingGet(UART0_BASE);
	}

    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    IntPrioritySet(INT_UART0, 0x10);
    UARTIntEnable(UART0_BASE, UART_INT_RX |UART_INT_RT | UART_INT_FE | UART_INT_BE | UART_INT_PE | UART_INT_OE);
    
    //
    // Init Receive buffer
    //
    ui_uart_init_buffer();
    
    return 1;
}
/*
 * Function to calculate checksum of a char
 */
unsigned char crc8_add( unsigned char inCrc, unsigned char inData )
{
    int i;
    unsigned int  data;

    data = (unsigned int)inCrc ^ (unsigned int)inData;
    data = data<<8;
  
    for ( i = 0; i < 8; i++ ) 
    {
        if (( data & 0x8000 ) != 0 )
        {
            data = data ^ (0x1070U << 3);
        }
        data = data << 1;
    }
    return (unsigned char)( data >> 8 );

} // Crc8

/*
 * function to send uart command,
 */

int ui_uart_send( char *charCmd, int length)
{
	int i;
	unsigned char crc;	
	
	//update state
	uartState = UART_WRITE;
	
	//check length
	if(length > UART_MAX_WRITE_LENGTH)
	{ 
	    return(-1);
	}
	
	//calculate and append checksum
	crc = 0;
    for (i =0; i< length; i++)
    {
        crc = crc8_add(crc, charCmd[i]);
    }
    
    //append crc and add proper termination
    charCmd[length]= crc;
	
	//now start writing
	for(i=0; i<= length; i++)
	{
		UARTCharPutNonBlocking(UART0_BASE, charCmd[i]);
		// give a little delay ~ 3 us
		SysCtlDelay(500);
	}

    return(1);
}
/*
 * Function to receive uart reply
 */

int ui_uart_receive(char *rxData, char *cmdData)
{
	//unsigned short crc;
	int i;
	unsigned char crc;	
	int len;
	char *rcvPnt = NULL;
	char *uBufPnt = NULL;
	
	// this is a blocking call;
	if(uart_process_rcv() ==-1)
	{
		return -1;
    }
	
	//now copy the data over
	if(uartRecvBuf.sReadDone)
	{
		len = uartRecvBuf.slength;
        uartRecvBuf.sReadDone =0;
        uartRecvBuf.sPos =0;
        //check if this is a stream read
        if(rxData != NULL)
        {
        	//copy data over
        	rcvPnt = uartRecvBuf.rcvBuffStrm;
        	uBufPnt = rxData;
        }
	}

	if(uartRecvBuf.cReadDone)
    {   
    	len =uartRecvBuf.clength;
        uartRecvBuf.cReadDone =0;
        uartRecvBuf.cPos =0;
        //check if this is a user read
        if(cmdData != NULL)
        {
        	//copy data over
        	rcvPnt = uartRecvBuf.rcvBuffCmd;
        	uBufPnt = cmdData;
        }
    }    

	//this should not happen, just to make sure
	if(rcvPnt == NULL) return (-1);
    
	//check sum
	crc = 0;
    for (i =0; i< len-1; i++)
    {
        crc = crc8_add(crc, rcvPnt[i]);
    }
    
    //compare checksum
    if(crc != rcvPnt[len-1])
    {
    	return(-1);
    }
    
    //only return the data
    memcpy(uBufPnt, &rcvPnt[UART_HEADER_LENGTH], len-UART_HEADER_LENGTH);

	return(len-UART_HEADER_LENGTH);
}

/*
 * Function to process user command
 */

int ui_uart_ucmd(char *buf, int len)
{
	int rLen;
    unsigned long tStart = UIGetTicks();


	//send command
    if(ui_uart_send(buf,len) ==-1) return -1;

    //wait for reply
    do
    {
    	//time out in ~0.5 seconds
    	if(UIGetTicks() - tStart > 25000000)
    	{
			g_ucHPInitDone = 0x00;
    		return(-1);
    	}
    	rLen = ui_uart_receive(NULL,buf);

    	// give a little delay ~1 ms
    	SysCtlDelay(20000);

    }while(rLen ==-1);

	return rLen;
}

