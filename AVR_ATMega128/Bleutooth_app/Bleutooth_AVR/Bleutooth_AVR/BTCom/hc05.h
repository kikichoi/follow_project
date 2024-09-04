

#ifndef HC05_H_
#define HC05_H_

char response[100];
char AT_Buff ;

void hc05_init()
{
	DDRE = 0XFF; //  출력 설정 (KEY 핀)
	PORTE &= ~(1 << PE4); // KEY 핀을 LOW 설정 (AT 모드 비활성화)
}

void hc05_enter_at_mode()
{
	PORTE |= (1 << PE4);  // KEY 핀을 high 설정 (AT 모드 활성화)
	//_delay_ms(100);
}

void hc05_exit_at_mode()
{
	PORTE &= ~(1 << PE4); // KEY 핀을 LOW 설정 (AT 모드 활성화)
	//_delay_ms(100);
}

void hc05_send_AT(const char *cmd, char *response, uint8_t max_length)
{//hc05 AT Command TX
	uart0_transmit_string(cmd);
	//uart0_transmit('\r');
	//uart0_transmit('\n');
	
	uint8_t i = 0 ;
	while(i < max_length - 1)
	{
		if (UCSR0A & (1 << RXC0))
		{
			response[i] = uart0_receive();
			if (response[i] == '\r' || response[i] == '\n')
			{
				break;
			}
			i++;
		}
	}
	response[i] = '0' ; //문자열 끝에 null 추가
}

void hc05_command()
{
	hc05_enter_at_mode(); // EN HIGH
	
	uart0_transmit_string("AT+NAME=HC05choi\r\n");
	
	//HC05 모듈 응답 터미널로 띄우기
	//AT_Buff = uart0_receive();
	//uart1_transmit(AT_Buff);
	
	//HC05 모듈의 응답이 줄바꿈 형태로 나와서 잘릴 경우 아래 함수 사용
	//hc05_send_AT("AT+UART?\r\n", response, sizeof(response));
	//uart1_transmit_string(response);
	//uart1_transmit('\n');
	
	hc05_exit_at_mode(); // EN LOW
	
	//블루투스랑 핸드폰끼리 신호는 주고 받는데 글자가 깨질 경우
	//블루투스 uart체크하기 스탑비트,페리티비트가 다를 경우 존재
}



#endif /* HC05_H_ */