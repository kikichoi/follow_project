
#define F_CPU 14745600

#include <avr/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "BSW/uart.h"
#include "BSW/io.h"
#include "BSW/timer.h"
#include "BTCom/hc05.h"
#include "def.h"

char test ;
int on_off , speed , steering ;
int timer_count ;

uint8_t TX_flag = 0;
uint8_t steering_flag = 0;
uint8_t steering_TX[5] = {0,};

ISR(TIMER1_COMPA_vect)
{
	timer_count++;
	
	if (timer_count % 1 == 0)
	{//1ms
		
	}
	else{
		
	}
	
	if (timer_count % 2 == 0)
	{//2ms
		
	}
	else{
		
	}
	
	if (timer_count % 100 == 0)
	{//100ms
		timer_100ms();		
	}
	else{
		
	}
	
	if (timer_count >= 100)
	{
		timer_count = 0;
	}
	else{
		
	}		
}

ISR(USART0_RX_vect)
{
	char rx_data;
	rx_data = UDR0;
	//uart1_transmit(rx_data);
	
	Steering S_data = rx_data;	
	switch(S_data)
	{
		case ON         : steering_flag = 1;  break;
		case OFF        : steering_flag = 2;  break;
		case SPEED_UP   : steering_flag = 3;  break;
		case SPEED_DOWN : steering_flag = 4;  break;		
		case FORWARD    : steering_flag = 5;  break;
		case TRUN_LEFT  : steering_flag = 6;  break;
		case TRUN_RIGHT : steering_flag = 7;  break;
		case BACKWARD   : steering_flag = 8;  break;
			
		default : steering_flag = 0; break;
	}
	TX_Packet();		
}

//**********TX_Packet**********//
//   []       []       []      []   //
// ON,OFF   SPEED   STEERING  0x77  //  
//값들이 유지가 되어서 터치할 시에만 변경되게끔하고 off 누르면 모든 동작이 정지되게끔 강력하게 만든다.
//스피드 업,다운을 한 번 누를 때마다 50씩 업,다운
//방향키도 한 번 누를 때마다 변경 되게끔 설정한다. 즉 누를 때마다 변경 되게끔 한다.  

void TX_Packet()
{		
	if(steering_flag == 1) on_off = 1;	
	else if(steering_flag == 2) on_off = 2;
	else if(steering_flag == 3) speed = 1;
	else if(steering_flag == 4) speed = 2;
	else if(steering_flag == 5) steering = 1;
	else if(steering_flag == 6) steering = 2;
	else if(steering_flag == 7) steering = 3;
	else if(steering_flag == 8) steering = 4;
	else on_off = 2;
	
	if(on_off == 1) steering_TX[0] = steering_ON;
	else steering_TX[0] = steering_OFF;
	
	if(steering_TX[0] == steering_ON)
	{
		if (speed == 1) steering_TX[1] = steering_SPEED_UP;
		else if (speed == 2) steering_TX[1] = steering_SPEED_DOWN;
		else steering_TX[1] = steering_WAIT;
		
		if (steering == 1) steering_TX[2] = steering_FORWARD;
		else if (steering == 2) steering_TX[2] = steering_TRUN_LEFT;
		else if (steering == 3) steering_TX[2] = steering_TRUN_RIGHT;
		else if (steering == 4) steering_TX[2] = steering_BACKWARD;
		else steering_TX[2] = steering_FORWARD;
		
		speed = 0;
		steering = 0;		
	}			
	else
	{
		 steering_TX[1] = steering_RESET;
		 steering_TX[2] = steering_FORWARD;	 
	}	
	
	steering_TX[3] = 0x77; //address 
	uart0_transmit_string(steering_TX);
	uart1_transmit_string(steering_TX); //test
}

void timer_100ms()
{
	// do nothing		
}

int main(void)
{	
	DDRB = 0xff;
	
	//UART
	uart1_init(115200);
	uart0_init(115200);
	InterruptInit0_Rx();
	ISR_CTC_Init();
	//uart0_init(38400); // AT모드 설정시 HC05 진입
	
	//BT
	hc05_init();
	hc05_command();
	
	//test
	//uart1_transmit_string("##ATmega128 Ready \r\n");
	
	/*//통신 에코백//
	test = uart1_receive();
	uart1_transmit(test);*/		
	
    while (1) 
    {
						
    }
}

