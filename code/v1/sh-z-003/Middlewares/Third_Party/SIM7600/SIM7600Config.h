#ifndef	_SIM7600CONF_H
#define	_SIM7600CONF_H


//	0: No DEBUG				1:High Level Debug .Use printf		2:All RX Data.Use printf

#define	_SIM80X_DEBUG				        1					

#define	_SIM80X_USART				        huart1

#define	_SIM80X_USE_POWER_KEY   	  0

#define	_SIM80X_BUFFER_SIZE			    512

#define _SIM80X_DMA_TRANSMIT        1

#define _SIM80X_USE_BLUETOOTH       0
#define _SIM80X_USE_GPRS            1


//#define	_SIM80X_POWER_KEY_GPIO		  LTE_PWR_GPIO_Port
//#define	_SIM80X_POWER_KEY_PIN		    LTE_PWR_Pin

#endif
