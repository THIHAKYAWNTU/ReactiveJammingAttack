/**
  ******************************************************************************
  * File Name          : common.h
  * Description        : This file contains commonly used define parameters
	* Software Engineer	 : THIHA KYAW
  ******************************************************************************
  */
	
	
typedef struct
{
    uint32_t status;      	//status of program
		uint32_t Frame_Period;
} app_data;


//DW1000 interrupt events
#define APP_RX_MEASURE          0x00000000          // Measure RX signal
#define APP_RX            			0x00000002          // Receiving State
#define APP_JM            			0x00000004          // Jammer State
#define APP_TX            			0x00000008          // Transmit

	
	/**********************************END*******************************************/
	
	
	
	