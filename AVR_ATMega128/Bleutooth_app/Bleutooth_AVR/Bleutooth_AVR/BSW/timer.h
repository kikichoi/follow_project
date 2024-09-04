
//1ms , 2ms 등등을 이용하여 uart송신을 보낼까? -> 고민

#ifndef TIMER_H_
#define TIMER_H_

void ISR_CTC_Init()
{// interrupt every 1ms
	
	// CTC 모드로 설정 (Clear Timer on Compare Match)
	TCCR1B |= (1 << WGM12);
	
	// 비교일치 OCR1A 설정
	// 14745600 클럭 사용시 (14745600) / (1000 * 64) - 1 = 249
	OCR1A = 229;
	
	// 분주율 64
	TCCR1B |= (1 << CS11) | (1 << CS10);
	
	//output compare match A
	TIMSK |= (1 << OCIE1A);
	
	// 전역 인터럽트 허가
	sei();
}




#endif /* TIMER_H_ */