#ifndef UI_SPI_H_
#define UI_SPI_H_

int SpiInit(void);
int SpiRead(unsigned short *rData);
int SpiWrite(unsigned short wData);

#endif /*UI_SPI_H_*/
