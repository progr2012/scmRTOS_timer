//******************************************************************************
//*
//*     FULLNAME:  Single-Chip Microcontroller Real-Time Operating System
//*
//*     NICKNAME:  scmRTOS
//*
//*     PROCESSOR: AVR (Atmel)
//*
//*     TOOLKIT:   avr-gcc (GNU)
//*
//*     PURPOSE:   avr-gcc Port Test File
//*
//*     Version: 5.0.0
//*
//*
//*     Copyright (c) 2003-2015, scmRTOS Team
//*
//*     Permission is hereby granted, free of charge, to any person
//*     obtaining  a copy of this software and associated documentation
//*     files (the "Software"), to deal in the Software without restriction,
//*     including without limitation the rights to use, copy, modify, merge,
//*     publish, distribute, sublicense, and/or sell copies of the Software,
//*     and to permit persons to whom the Software is furnished to do so,
//*     subject to the following conditions:
//*
//*     The above copyright notice and this permission notice shall be included
//*     in all copies or substantial portions of the Software.
//*
//*     THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//*     EXPRESS  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//*     MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//*     IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
//*     CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
//*     TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
//*     THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//*
//*     =================================================================
//*     Project sources: https://github.com/scmrtos/scmrtos
//*     Documentation:   https://github.com/scmrtos/scmrtos/wiki/Documentation
//*     Wiki:            https://github.com/scmrtos/scmrtos/wiki
//*     Sample projects: https://github.com/scmrtos/scmrtos-sample-projects
//*     =================================================================
//*
//******************************************************************************
//*     avr-gcc port by Oleksandr O. Redchuk, Copyright (c) 2007-2015

//---------------------------------------------------------------------------
#include <avr/io.h>
#include <stdio.h>

#include "pin_macros.h"

#include "scmRTOS.h"
#include <avr/eeprom.h>

#include "SPI.h"
#include "max7219.h"
//#include "thermistor.h"

uint16_t data EEMEM;
uint8_t inv_time[4] EEMEM;
volatile uint16_t time;

//---------------------------------------------------------------------------
//      Sample target
//  The sample is intended for following AVR microcontrollers:
//      atmega48..atmega328
//      atmega64, atmega128
//      atmega640..atmega2561
//  Some changes in register names may be needed for other AVRs.

#if defined(TIMSK1)
#  define TIMER1_IE_REG TIMSK1
#elif defined(TIMSK)
#  define TIMER1_IE_REG TIMSK
#else
#  error "Timer1 interrupt mask register not defined"
#endif

#define BAUD 19200
#define MYUBRR F_CPU/16/BAUD-1



void USART_Init(unsigned int ubrr) {
   UBRR0H = (unsigned char)(ubrr>>8);
   UBRR0L = (unsigned char)ubrr;
   UCSR0B = (1<<RXEN0) | (1<<RXCIE0)|(1 << TXEN0)|(1 << TXCIE0);
   UCSR0C |= (1<<UCSZ00 | (1 << UCSZ01));
}




typedef OS::process<OS::pr0, 200> TTimer;
typedef OS::process<OS::pr1, 200> Tusart;
typedef OS::process<OS::pr2, 200> TNTC;



//---------------------------------------------------------------------------
//
//      Process objects
//
TTimer Proc1;
Tusart Proc2;
TNTC Proc3;

//---------------------------------------------------------------------------
tick_count_t tick_count;        // global variable for OS::GetTickCount testing

OS::TEventFlag Timer1_Ovf;  // set in TIMER1_COMPA_vect(), waited in Proc1

OS::TEventFlag input_timer;
OS::TEventFlag setting_timer;

// Объявляем FIFO-буферы для считывания и отправки данных в UART
#define TXFIFO_SIZE 128

struct TData
{
 uint8_t *p;
};
OS::channel<TData, 10> uart_tx_fifo;

//uint8_t tx_buf[TXFIFO_SIZE];
//static OS::TChannel uart_tx_fifo = OS::TChannel(tx_buf, TXFIFO_SIZE);

#define RXFIFO_SIZE 128
uint8_t rx_buf[RXFIFO_SIZE];
static OS::TChannel uart_rx_fifo = OS::TChannel(rx_buf, RXFIFO_SIZE);
uint8_t cur,s_timer;
// Аналогично объявляется буфер на прием

void Setting_timer(){
    s_timer=1;
    if(cur==3){cur=0;}
    
    Clear_7219();
	Send_7219(0x01, 14); /// первый знак P
    Send_7219(0x02, cur); /// параметр
    if(cur==0){Send_7219(0x04,eeprom_read_byte(&inv_time[0]));} /// параметр 0
    if(cur==1){Send_7219(0x04,eeprom_read_byte(&inv_time[1]));} /// параметр 1
    if(cur==2){Send_7219(0x04,eeprom_read_byte(&inv_time[2]));} /// параметр 2
    cur++;
    //Send_7219(0x01, 0); 
}

void Update(){
    PORTD |=(eeprom_read_byte(&inv_time[0])<<PD6);
}


//---------------------------------------------------------------------------
int main()
{
    DDRD |=(1<<PD6)|(1<<PD7);
	PORTD |=(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD2);

    TCCR1B = (1 << WGM12) | (1 << CS10);    // CTC mode, clk/1
    OCR1A  = 40000U;
    TIMER1_IE_REG = (1 << OCIE1A); // Timer1 OC interrupt enable

    // Start System Timer
    TIMER0_CS_REG  = (1 << CS01) | (1 << CS00); // clk/64
    TIMER0_IE_REG |= (1 << TOIE0);


    PCICR |= (0 << PCIE0)|(0 << PCIE1)|(0 << PCIE2);

    //EIMSK |=(1<<INT0); //��������� ���������� int0 -  ������
	//EICRA |=(1<<ISC11)|(0<<ISC10); // ������������ �� ���������� ������ -  ��� ������ 
	//MCUCR |=(1<<IVCE);

    USART_Init(MYUBRR);

    SPI_Init();
	Init_7219();

    time = eeprom_read_word(&data);
    //eeprom_write_word(&data,0x10);
    PORTD |=(eeprom_read_byte(&inv_time[0])<<PD6);
    
    
    OS::run();
}

//---------------------------------------------------------------------------
namespace OS {

template<> void TTimer::exec()
{
    
	Number_7219(eeprom_read_word(&data));
    uint8_t state;
    for(;;) {
        
        sleep(150);

    if (!((1 << PD5) & PIND))
			{
				//сценарий для SET
                Setting_timer();
                setting_timer.signal();
				//if(time==0){time=255;}
			}

	if (!((1 << PD4) & PIND))
			{//сценарий для UP
                if(s_timer){
                    setting_timer.signal();
                    eeprom_write_byte(&inv_time[cur-1],0x01);
                    //PORTD |=(eeprom_read_byte(&inv_time[cur-1])<<PD6); 
                    Send_7219(0x04, 1);  
                }else{
                    time++;
				    eeprom_write_word(&data,time);
				    Number_7219(time);
				    if(time==0){time=255;}
                }
				
			}
	if (!((1 << PD3) & PIND))
			{
				//сценарий для DOWN
                if(s_timer){
                    setting_timer.signal();
                    eeprom_write_byte(&inv_time[cur-1],0);
                    //PORTD |=(eeprom_read_byte(&inv_time[cur-1])<<PD6); 
                    Send_7219(0x04, 0);  
                }else{
				time--;
                if(time==0){time=1;}
				eeprom_write_word(&data,time);
				Number_7219(time);
                }
			}
    if (!((1 << PD2) & PIND))
			{
                state++;
                //input_timer.signal();
                if(state==1){
                    Send_7219(0x01, 14); /// первый знак P
                    //PORTD ^=(1<<PD6);
                    sleep(time*100);
                    PORTD |=(1<<PD6);
                }
                
                
			}else{
                //Send_7219(0x01, 14); /// первый знак P
                state=0;
                if(state==0){Send_7219(0x01, 0x0f);}
                PORTD &=(0<<PD6);
            }
    
    }
}

} // namespace OS


//---------------------------------------------------------------------------
namespace OS {

template<> void Tusart::exec()
{
    for(;;) {
        
        if(setting_timer.wait(5000)){
            
        }else{
            Number_7219(time);
            setting_timer.clear();
            cur=0;
            s_timer=0;
            }
        


        //TData dt;
        //uint8_t i=0;
        //if( uart_tx_fifo.pop(dt, 100) ) // wait for data, timeout 10 system ticks
        //{
         //   while(dt.p[i] != 0)
         //   {
         //       while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/
         //       UDR0 = dt.p[i];            /* Put data into buffer, sends the data */
         //       i++;                             /* increment counter           */
         //   }
       // }
       // else
       // {
        // timeout expires, do something else
        //UDR0 = '0';
       // }
        
        
        
    }
}

} // namespace OS


//---------------------------------------------------------------------------
namespace OS {

    template<> void TNTC::exec()
    {
        for(;;) {
            
            //if(input_timer.wait()){
              //  PORTD ^=(1<<PD6);
                //sleep(time*100);
               // PORTD ^=(1<<PD6);
               // input_timer.clear();
                
                sleep(1000);
           // }
        }
    }
} 



//---------------------------------------------------------------------------
OS_INTERRUPT void TIMER1_COMPA_vect()
{
    OS::TISRW_SS ISRW;
    ENABLE_NESTED_INTERRUPTS();
    Timer1_Ovf.signal_isr();
}


//
//ISR(INT0_vect){
		//sig=1;

  //      OS::TISRW_SS ISRW;
        //ENABLE_NESTED_INTERRUPTS();
    //    input_timer.signal_isr();
		//time = eeprom_read_byte(&data);
//}



ISR( USART_RX_vect )
{
OS::TISRW tisrw;
unsigned char rxbyte = UDR0;
// Необходима проверка на заполненность входящего буфера, так как поведение самого класса в этой ситуации нас немного не устраивает
if (uart_rx_fifo.get_count() < RXFIFO_SIZE)
uart_rx_fifo.push(rxbyte);
}

//------    end of file  main.cpp   -------------------------------------------
