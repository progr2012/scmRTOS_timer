
    
    void Send_7219(char rg, unsigned char dt)

	{
		SPI_PORTX &= ~(1<<SPI_SS);
		SPI_SendByte(rg);
		SPI_SendByte(dt);
		SPI_PORTX |= (1<<SPI_SS);

	}

void Clear_7219(void){
	unsigned char x;
		for(x=0;x<5;x++)
		{
		Send_7219(x, 0x0f); 
		}
	}





// инициализация max7219
void Init_7219(void){
	Send_7219(0x09, 0xFF); 
	Send_7219(0x0B, 3); 
	Send_7219(0x0A, 0x01); //яркость
	Send_7219(0x0C, 1); 
	Clear_7219();
}


void Number_7219(unsigned int num){
		//char ng = 0; //?????????? ??? ??????
	//if (num < 0) 
	//{
//		ng = 1;
//		num *= -1;
//	}
	Clear_7219();
    if (num == 0) {
	    Send_7219(0x01, 0); 
	    return;
    }
	Send_7219(0x04, num%10);//еденицы
	Send_7219(0x03, (num/10)%10|DP);//десятки
	
	if(!((num/100)%10)){
		if(num>100){Send_7219(0x02, (num/100)%10);} ///сотни
		else{Send_7219(0x02, 0x0f);}
	}else{
		Send_7219(0x02, (num/100)%10); //сотни
	}



	if(!((num/1000)%10)){
		Send_7219(0x01, 0x0f); ///тысячи
	}else{
		Send_7219(0x01, (num/1000)%10); ///тысячи
	}
	
	//char i = 0;
	//do {
		
	//	Send_7219(i++, num%10);
	//	if(i==2){Send_7219(0x02, num%10|DP);}
	//	num /= 10;
		//i++;
		
	//} while (num>5);

	//if (ng) {Send_7219(i+1, 0xA);}

	}


	void Set_7219(unsigned char cur,unsigned int time){

		
			Send_7219(cur, 0x0f);
			//_delay_ms(500);
			Number_7219(time);
			//_delay_ms(500);

			
		
		
		

	}