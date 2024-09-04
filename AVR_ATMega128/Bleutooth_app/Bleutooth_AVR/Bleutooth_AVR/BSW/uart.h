
#ifndef UART_H_
#define UART_H_

void uart0_init(unsigned int baud)
{// EX] 보오레이트를 9600 설정할라면 UBRR1L = 95; // 9600 (14745600 / 16 * 9600) -1 = 95
	// 보오레이트 설정
	switch(baud)
	{
		case 115200 : UBRR0L = 7;  break;
		case 38400  : UBRR0L = 23; break;
		case 9600   : UBRR0L = 95; break;
		default: break;
	}
	
	//송수신 설정 , 8비트 데이터 , 1비트 스탑비트
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) ;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void InterruptInit0_Rx()
{
	// RXCIE bit 7번을 1로 바꾼다.
	// UCSR1B |= 0b10000000 ;
	UCSR0B |= (1<<RXCIE);
	sei();
}

void uart0_transmit(unsigned char data)
{
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void uart0_transmit_string(const char *str)
{
	while (*str)
	{//주소를 하나씩 증가시킴 , null이 나오면 중단
		uart0_transmit(*str++);
	}
}

unsigned char uart0_receive()
{//화면에 띄워지는 값들은 ASCII이다. 받을 때 char 형태로 받는다.
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void uart1_init(unsigned int baud)
{
	//보오레이트 설정
	switch(baud)
	{
		case 115200 : UBRR1L = 7;  break;
		case 38400  : UBRR1L = 23; break;
		case 9600   : UBRR1L = 95; break;
		default: break;
	}
	
	
	//송수신 설정 , 8비트 데이터 , 1비트 스탑비트
	UCSR1B = (1 << RXEN1) | (1 << TXEN1) ;
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

void uart1_transmit(unsigned char data)
{
	while(!(UCSR1A & (1 << UDRE1)));
	UDR1 = data;
}

void uart1_transmit_string(const char *str)
{
	while (*str)
	{//주소를 하나씩 증가시킴 , null이 나오면 중단
		uart1_transmit(*str++);
	}
}

unsigned char uart1_receive()
{//화면에 띄워지는 값들은 ASCII이다. 받을 때 char 형태로 받는다.
	while(!(UCSR1A & (1 << RXC1)));
	return UDR1;
}



#endif /* UART_H_ */