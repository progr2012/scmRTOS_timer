#include <avr/io.h>

#define SPI_PORTX PORTB
#define SPI_DDRX DDRB

#define SPI_MISO 4 
#define SPI_MOSI 3
#define SPI_SCK 5
#define SPI_SS 2
#define DP 128


void SPI_SendByte (char byte)

{

  SPDR = byte;

  while (!(SPSR & (1 << SPIF)));

}

void SPI_Init(void)
{

  
   SPI_DDRX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS)|(0<<SPI_MISO);
   SPI_PORTX |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS)|(1<<SPI_MISO);

   
   SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(1<<SPR0);
   SPSR = (0<<SPI2X);

}