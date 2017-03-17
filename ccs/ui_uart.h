#ifndef __UI_UART_H__
#define __UI_UART_H__

int ui_uart_init(void);
int ui_uart_receive(char *, char *);
int ui_uart_send(char *,int);
int ui_uart_ucmd(char *, int );
unsigned char crc8_add( unsigned char, unsigned char );

#endif /*__UI_UART_H__*/
