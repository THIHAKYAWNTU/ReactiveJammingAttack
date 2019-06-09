
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "stm32f7xx_hal.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "commands.h"
#include "stm32f7xx_nucleo_144.h"
#include "global.h"
/* --------------------  Private variables -------------------------------------*/
extern osThreadId tid_uartThread; 
extern char cmd_Buffer[100];
extern var_DataTypeDef txSettings;

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
/* Recommended TX power and Pulse Generator delay values for the mode defined above. */
static dwt_txconfig_t txconfig = {
    0xC2,            /* PG delay. */
    0x67676767,      /* TX power. */
};

/* The frame sent in this example is a data frame encoded as per the IEEE 802.15.4-2011 standard. It is a 21-byte frame composed of the following
 * fields:
 *     - byte 0/1: frame control (0x8861 to indicate a data frame using 16-bit addressing and requesting ACK).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA)
 *     - byte 5/6: destination address, see NOTE 2 below.
 *     - byte 7/8: source address, see NOTE 2 below.
 *     - byte 9 to 18: MAC payload, see NOTE 1 below.
 *     - byte 19/20: frame check-sum, automatically set by DW1000. */
static uint8 tx_msg[] = {0x61, 0x88, 0, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',						//20
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 							//20
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 							//20
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 							//20
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 							//20
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 							//20
													'v','w', 'x', 'y', 0, 0};																								

static uint8 tx_msg_dmy[] = {0x61, 0x88, 0, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 'a', 0, 0};
	
/* Index to access the sequence number and frame control fields in frames sent and received. */
#define FRAME_FC_IDX 0
#define FRAME_SN_IDX 2
/* ACK frame control value. */
#define ACK_FC_0 0x02
#define ACK_FC_1 0x00
extern UART_HandleTypeDef huart2;	
/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 5000
	
	/* Inter-frame delay period in case of RX timeout, in milliseconds.
 * In case of RX timeout, assume the receiver is not present and lower the rate of blink transmission. */
#define RX_TO_TX_DELAY_MS 5000
/* Inter-frame delay period in case of RX error, in milliseconds.
 * In case of RX error, assume the receiver is present but its response has not been received for any reason and retry blink transmission immediately. */
#define RX_ERR_TX_DELAY_MS 5000
	
	/* Receive response timeout, expressed in UWB microseconds (UUS, 1 uus = 512/499.2 µs). See NOTE 3 below. */
#define RX_RESP_TO_UUS 30000
//11000
/* Buffer to store received frame. See NOTE 4 below. */
#define ACK_FRAME_LEN 5
static uint8 rx_buffer[ACK_FRAME_LEN];

	
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/* ACK status for last transmitted frame. */
static int tx_frame_acked = 0;

/* Current inter-frame delay period.
 * This global static variable is also used as the mechanism to signal events to the background main loop from the interrupt handler callbacks,
 * which set it to positive delay values. */
static int32 tx_delay_ms = -1;

/* Counters of frames sent, frames ACKed and frame retransmissions. See NOTE 1 below. */
static uint32 tx_frame_nb = 0;
uint16_t tx_frame_ack_nb = 0;
uint16_t tx_frame_retry_nb = 0;
/*--------------------------------------------------------------------------*/
uint32_t max_microTP = 16000;
uint32_t step_microTP = 100;
uint32_t max_TP = 36;


/* Declaration of static functions. */
static void transmit_ack(void);
static void transmit_NOack(void);
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);
static void DATA_INIT(void);
static void delayMicrosec(uint32_t microsec);
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void TXThread (void const *argument);                             // thread function
osThreadId tid_TXThread;                                          // thread id
osThreadDef (TXThread, osPriorityRealtime, 1, 5120);                   // thread object

int Init_TXThread (void) {
	
	/* Reset and initialise DW1000. See NOTE 2 below.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	 * performance. */
	 	led_off(LED_ALL);
		reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();

		if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        //Error
				led_on(LED_ALL); //to display error....
        while (1)
        { };
    }
	
		port_set_dw1000_fastrate();
		
		/* Configure GPIOs to show TX/RX activity. See NOTE 6 below. */
    dwt_setlnapamode(1, 1);

    /* Configure LEDs management. See NOTE 6 below. */
    dwt_setleds(DWT_LEDS_ENABLE);
		
		/* Configure DW1000. See NOTE 3 below. */
    dwt_configure(&config);
		dwt_configuretxrf(&txconfig);
		/* Register RX call-back. */
    dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_ARFE, 1);
		
		/* Set delay to turn reception on immediately after transmission of the frame. See NOTE 6 below. */
    dwt_setrxaftertxdelay(0);
		
		/* Set RX frame timeout for the response. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);
		
		/**/
		DATA_INIT();
		
		
		
		/****************************START THREAD**********************************************/

  tid_TXThread = osThreadCreate (osThread(TXThread), NULL);
  if (!tid_TXThread) return(-1);
  
  return(0);
}

void TXThread (void const *argument) {

  while (1) {
		
		if(txSettings.TXEN==1)
		{
			/*------------------Transmitter Enable and Disable-----------------------*/
				if(txSettings.TP<max_TP)
				{
					if((tx_frame_retry_nb+tx_frame_ack_nb)<txSettings.NPackets)
					{
//						if(txSettings.TXDUMMY==1)
//						{
							transmit_NOack();
//						}

						osDelay(txSettings.TP);				//delay in millisec
						
//						delayMicrosec(txSettings.microTP);  //Delay in microsec
						transmit_ack();
					}
					else
					{
						//Results
						sprintf(cmd_Buffer,"Results-> Millisec: %u, ACK:  %d, RETRY:  %d\n\r",txSettings.TP, tx_frame_ack_nb, tx_frame_retry_nb);
						
						//
						//Send data
						osSignalSet(tid_uartThread, cmd_Thread);
					
						/* Re-activate reception immediately. */
						
						BSP_LED_Toggle(LED3);
						osDelay(500);
						tx_frame_retry_nb=0;
						tx_frame_ack_nb=0;
						//txSettings.microTP += step_microTP;
						txSettings.TP++;
						BSP_LED_Toggle(LED3);
					}
			}
			
		}
		else
		{
			//set to zero when disable
			tx_frame_retry_nb=0;
			tx_frame_ack_nb = 0;
			led_off(LED_1);
			led_off(LED_2);
			led_off(LED_3);
			//sprintf(cmd_Buffer,"Disable Operation\n\r");
			
			//Send data
			osSignalSet(tid_uartThread, cmd_Thread);
			osDelay(3000);
		}
		/*------------------Transmitter Enable and Disable END-----------------------*/
  }
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn transmit_ack()
 *
 * @brief Transmit and Get ACK from receiver
 *
 * @param  none
 *
 * @return  none
 */
char TCP_Msg[50]={'\0'};
static void transmit_ack(void)
{
		
		/* Write frame data to DW1000 and prepare transmission. See NOTE 7 below.*/
		dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

		/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
		
		osSignalWait (0x01, osWaitForever);

		/* Execute the defined delay before next transmission. */

		
		if (tx_frame_acked)
		{
				/* Execute a delay between transmissions. See NOTE 1 below. */
				osDelay(txSettings.TP2P);
				led_on(LED_1);
				led_off(LED_2);

				/* Increment the sent frame sequence number (modulo 256). */
				tx_msg[FRAME_SN_IDX]++;

				/* Update number of frames acknowledged. */
				tx_frame_ack_nb++;
//				tx_frame_retry_nb = 0;
//				sprintf(cmd_Buffer,"    ACK\n\r");
//				osSignalSet(tid_uartThread, cmd_Thread);

		}
		else
		{
			led_off(LED_1);
			led_on(LED_2);
			if (tx_delay_ms > 0)
			{
				osDelay(tx_delay_ms);
			}	


			/* Update number of retransmissions. */
			tx_frame_retry_nb++;


		}

		/* Reset acknowledged frame flag. */
		tx_frame_acked = 0;

		/* Reset the TX delay and event signalling mechanism ready to await the next event. */
    tx_delay_ms = -1;


}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn transmit_NOack()
 *
 * @brief Transmit and Get ACK from receiver
 *
 * @param  none
 *
 * @return  none
 */

static void transmit_NOack(void)
{
		char TCP_Msg[50]={'\0'};
		/* Write frame data to DW1000 and prepare transmission. See NOTE 7 below.*/
		dwt_writetxdata(sizeof(tx_msg_dmy), tx_msg_dmy, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(tx_msg_dmy), 0, 0); /* Zero offset in TX buffer, no ranging. */

		/* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
		dwt_starttx(DWT_START_TX_IMMEDIATE);

}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    int i;
	

    /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
     * buffer. */
    for (i = 0 ; i < ACK_FRAME_LEN; i++ )
    {
        rx_buffer[i] = 0;
    }

    /* A frame has been received, copy it to our local buffer. */
    if (cb_data->datalength == ACK_FRAME_LEN)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
			
				/* Check if it is the expected ACK. */
				if ((rx_buffer[FRAME_FC_IDX] == ACK_FC_0) && (rx_buffer[FRAME_FC_IDX + 1] == ACK_FC_1)
						&& (rx_buffer[FRAME_SN_IDX] == tx_msg[FRAME_SN_IDX]))
				{
						tx_frame_acked = 1;
						
				}
    }

    /* Set corresponding inter-frame delay. */
    tx_delay_ms = txSettings.TP2P;
		osSignalSet (tid_TXThread, 0x01);

    /* TESTING BREAKPOINT LOCATION #1 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    /* Set corresponding inter-frame delay. */
    tx_delay_ms = txSettings.TP2P;
		osSignalSet (tid_TXThread, 0x01);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    
		uint32 isrStatus;
		isrStatus  = cb_data->status;
		if(isrStatus & DWT_INT_ARFE) // Frame Filter Rejection
		{
			uart_sendmsg((char*) "Jamming Detected\n\r");
		}

		/* Set corresponding inter-frame delay. */
    tx_delay_ms = txSettings.TP2P;
		osSignalSet (tid_TXThread, 0x01);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

    /* TESTING BREAKPOINT LOCATION #4 */
}

static void DATA_INIT(void)
{
	txSettings.TXEN=1;
	txSettings.TXDUMMY=1;
	txSettings.microTP = 15000;
	txSettings.TP=8;
	txSettings.NPackets=100;
	txSettings.TP2P=500;
}

static void delayMicrosec(uint32_t microsec)
{
	uint32_t tick, timeout;

	tick = osKernelSysTick(); 
	timeout = osKernelSysTickMicroSec(microsec);

	while((osKernelSysTick()-tick)< timeout)
	{
		//Do Nothing
	}
	//led_on(LED_2);
}


