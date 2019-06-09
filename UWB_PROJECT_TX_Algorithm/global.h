/**
  ******************************************************************************
  * File Name          : global.h
  * Description        : This file contains lists of commands which can be used.
	* Software Engineer	 : THIHA KYAW
  ******************************************************************************
  */
	
#ifndef GLOBAL_H
#define GLOBAL_H
typedef enum 
{
	cmd_Thread    	=	0x01U,
}cmdthread_TypeDef;

typedef struct
{
	//Transmitter
	uint32_t TXEN;			//Enable and Disable
	uint32_t TXDUMMY;			//Enable and Disable
	uint32_t TP;				//time between dummy packet and actual packet
	uint32_t microTP;				//time between dummy packet and actual packet
	uint32_t TP2P;			//time between each packet
	uint32_t NP;				//Number of bytes for Transmitting
	uint32_t NPackets;	//Number of Packets for Transmitting
	
	
	//Jammer
	uint32_t NJ;				//Number of bytes for jamming
                   
}var_DataTypeDef;

#endif /* GLOBAL_H */
/************************ (C) COPYRIGHT THIHA KYAW************************/