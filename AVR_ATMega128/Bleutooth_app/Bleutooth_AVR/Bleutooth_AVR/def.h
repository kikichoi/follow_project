
#ifndef DEF_H_
#define DEF_H_

typedef enum
{	
	ON         = '0',
	OFF        = '1',
	SPEED_UP   = '2',
	SPEED_DOWN = '3',	
	FORWARD    = '4',
	TRUN_LEFT  = '5',
	TRUN_RIGHT = '6',
	BACKWARD   = '7'		
}Steering;

typedef enum
{
	steering_ON           = 0xaa,
	steering_OFF          = 0xbb,
	steering_SPEED_UP     = 0xab,
	steering_SPEED_DOWN   = 0xac,
	steering_FORWARD      = 0xcc,	
	steering_TRUN_LEFT    = 0xdd,
	steering_TRUN_RIGHT   = 0xee,
	steering_BACKWARD     = 0xfa,
	steering_WAIT         = 0xfb,
	steering_RESET        = 0xfc
}Steering_Packet;

#endif /* DEF_H_ */