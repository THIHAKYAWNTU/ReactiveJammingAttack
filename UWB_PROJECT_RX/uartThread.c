
#include "cmsis_os.h"                   // CMSIS RTOS header file
#include "stm32f7xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "commands.h"


/*************UART*****************/
extern UART_HandleTypeDef huart2;
uint8_t Rx_indx, Rx_data[2], Rx_Buffer[100], Transfer_cplt;

typedef enum 
{
	cmd_Thread    	=	0x01U,
}cmdthread_TypeDef;

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void uartThread (void const *argument);                             // thread function
osThreadId tid_uartThread;                                          // thread id
osThreadDef (uartThread, osPriorityNormal, 1, 0);                   // thread object

int Init_uartThread (void) {

  tid_uartThread = osThreadCreate (osThread(uartThread), NULL);
  if (!tid_uartThread) return(-1);
  
  return(0);
}

void uartThread (void const *argument) {

  Transfer_cplt = 0;
	HAL_UART_Receive_IT(&huart2, Rx_data, 1);
	while (1) 
	{
		osSignalWait (cmd_Thread, osWaitForever);
		
		execute_CMD((char *)Rx_Buffer);
		
		Transfer_cplt = 0;						// Ready to receive again
		HAL_UART_Receive_IT(&huart2, Rx_data, 1);   //activate UART receive interrupt every time
		

  }
}


/**************************UART Receive*************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i;
    if (huart->Instance == USART2)  //current UART
		{
			if(Transfer_cplt == 0)
			{
				if (Rx_indx==0) {for (i=0;i<100;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data 
				if (Rx_data[0]!=13) //if received data different from ascii 13 (enter)
				{
					Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
				}
        else            //if received data = 13
				{
					Rx_indx=0;
					//transfer complete, data is ready to read
					Transfer_cplt = 1; //	Don't receive any data
					osSignalSet(tid_uartThread, cmd_Thread);										
				}
        HAL_UART_Receive_IT(&huart2, Rx_data, 1);   //activate UART receive interrupt every time
			}
		}
}

