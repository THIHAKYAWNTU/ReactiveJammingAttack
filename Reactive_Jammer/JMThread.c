
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "stm32f7xx_hal.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "commands.h"
#include "stm32f7xx_nucleo_144.h"
#include "common.h"

/* --------------------  Private variables -------------------------------------*/

//#define continuousWave
#define NormalTx

/* Start-to-start delay between frames, expressed in quarters of the 499.2 MHz fundamental frequency (around 8 ns). See NOTE 1 below. */
//#define CONT_FRAME_PERIOD 124800 // 8 nsec * 124800 = 0.99 msec

uint16_t MCU_CLK_MHz = 144;

#define CONT_FRAME_PERIOD 124800000 // 8 nsec * 124800000 = 1sec

/* Continuous frame duration, in milliseconds. */
#define CONT_FRAME_DURATION_MS 10000

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


/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, put to 0.
 *     - byte 2 -> 9: device ID, hard coded constant in this example for simplicity.
 *     - byte 10/11: frame check-sum, automatically set by DW1000 in a normal transmission and set to 0 here for simplicity.
 * See NOTE 1 below. */
static uint8 tx_msg[] = {0x01, 0x18, 0, 0x1A, 0x1E, 'X', 'R', 'X', 'T', 'm', 'a', 'c', 'p', 'a', 'y', 'l', 'o', 'a', 'd', 
												'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', //26
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z', 0, 0};


/* Buffer to store received frame. See NOTE 3 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];
char no_MSG[21]={"No Message Received\n\r"};
uint8_t BlinkSpeed = 0;
char uart_Buffer[50]={'\0'};

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

//To calculate Time Difference between each frame
uint32_t TimeDiff_frame=0x00; 
uint8_t startCount=0;

app_data* APP_DATA;

/********************* Declaration of static functions. ***************************/

static void activate_frameTx(uint32_t frameDurationMs);
static void measureTxframeInterval(void);
static void rx_process(void);
static void init_DWM(void);
/*----------------------------------------------------------------------------
 *      Thread for Jamming
 *---------------------------------------------------------------------------*/
void JMThread (void const *argument);                             // thread function
osThreadId tid_JMThread;                                          // thread id
osThreadDef (JMThread, osPriorityHigh, 1, 10000);                // thread object

int Init_JMThread (void) {
	
	/* Reset and initialise DW1000. See NOTE 2 below.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	 * performance. */
	 	led_off(LED_ALL);
		
		init_DWM();

		//Initially state is to meaure TX signal from source.
		
	
		/****************************START THREAD**********************************************/

  tid_JMThread = osThreadCreate (osThread(JMThread), NULL);
  if (!tid_JMThread) return(-1);
  
  return(0);
}

void JMThread (void const *argument) {
	
  while (1) {
		
		#ifdef NormalTx  
		/* Write response frame data to DW1000 and prepare transmission.*/
		dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */


		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		/* Poll until a frame is properly received or an error occurs. See NOTE 5 below.
		* STATUS register is 5 bytes long but, as the events we are looking at are in the lower bytes of the register, we can use this simplest API
		* function to access it. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXPRD |  SYS_STATUS_ALL_RX_ERR)))
		{

		};
		if (status_reg & SYS_STATUS_RXPRD)
		{

		/* Send the response. */
		dwt_starttx(DWT_START_TX_IMMEDIATE);
		BSP_LED_Off(LED3);
		BSP_LED_On(LED1);	
		/* Poll DW1000 until TX frame sent event set. */
		while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
		{
						
		};
					/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPRD);

		/* Clear TX frame sent event. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		}
		else
		{
		/* Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		}		

		#endif
		
		#ifdef continuousWave 
		/* Configure DW1000. */
    dwt_configure(&config);
    dwt_configuretxrf(&txconfig);

    /* Activate continuous wave mode. */
    dwt_configcwmode(config.chan);
		
		led_on(LED_ALL);

    /* Wait for the wanted duration of the continuous wave transmission. */
    osDelay(CONT_FRAME_DURATION_MS);
		
		led_off(LED_ALL);
		
	

    /* Software reset of the DW1000 to deactivate continuous wave mode and go back to default state. Initialisation and configuration should be run
     * again if one wants to get the DW1000 back to normal operation. */
    dwt_softreset();
		
			init_DWM();
		#endif
	
	}		
		

}


/*************************************************************************
*	Function	: Activate continuous frame Transmission
*	Input			:	This is a 32-bit value that is used to set the interval 
*							between transmissions. 
*/

static void activate_frameTx(uint32_t frameDurationMs)
{
	    /* Activate continuous frame mode. */
    dwt_configcontinuousframemode(frameDurationMs);

    /* Once configured, continuous frame must be started like a normal transmission. */
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    /* Wait for the required period of repeated transmission. */
    osDelay(CONT_FRAME_DURATION_MS);
		
		/* Software reset of the DW1000 to deactivate continuous frame mode and go back to default state. Initialisation and configuration should be run
		* again if one wants to get the DW1000 back to normal operation. */
    dwt_softreset();


}

///*! ------------------------------------------------------------------------------------------------------------------
// * @fn rx_ok_cb()
// *
// * @brief Callback to process RX good frame events
// *
// * @param  cb_data  callback data
// *
// * @return  none
// */
//static void rx_ok_cb(const dwt_cb_data_t *cb_data)
//{
//	
//		int i;

//    /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
//     * buffer. */
//    for (i = 0 ; i < FRAME_LEN_MAX; i++ )
//    {
//        rx_buffer[i] = 0;
//    }	
//		
//		if(APP_DATA ->status == APP_RX_MEASURE)
//		{
//					 measureTxframeInterval();

//		}

//    /* TESTING BREAKPOINT LOCATION #1 */

//    /* A frame has been received, copy it to our local buffer. See NOTE 6 below. */
//    if (cb_data->datalength <= FRAME_LEN_MAX)
//    {
//        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);

//				uart_sendmsg((char*)rx_buffer, sizeof(rx_buffer));
//    }
//		/* Clear good RX frame event in the DW1000 status register. */
//		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
//		
//		if(APP_DATA ->status == APP_RX_MEASURE)
//		{
//			/* Perform manual RX re-enabling. See NOTE 5 below. */
//			dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);
//		}

//    /* TESTING BREAKPOINT LOCATION #2 */
//}

///*! ------------------------------------------------------------------------------------------------------------------
// * @fn rx_err_cb()
// *
// * @brief Callback to process RX error events
// *
// * @param  cb_data  callback data
// *
// * @return  none
// */
//static void rx_err_cb(const dwt_cb_data_t *cb_data)
//{
//	uint32 isrStatus;
//	isrStatus  = cb_data->status;
//	if(isrStatus & DWT_INT_ARFE) // Frame Filter Rejection
//	{
//		uart_sendmsg((char*) "Jamming Detected\n\r", 17);
//	}
//	else
//	{
//		uart_sendmsg(no_MSG, sizeof(no_MSG));
//	}
//	
//		/* Clear RX error events in the DW1000 status register. */
//   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
//		
//	if(APP_DATA ->status == APP_RX_MEASURE)
//	{
//			/* Re-activate reception immediately. */
//			dwt_rxenable(DWT_START_RX_IMMEDIATE);
//	}
//	

//	


//    /* TESTING BREAKPOINT LOCATION #3 */
//}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn uint32_t measureTxframeInterval(void)
 *
 * @brief Measure Time difference between two transmit frame
 *
 * @param  void
 *
 * @return  Frame Period of transmitter
 */
uint32_t timeStart = 0 ;
uint32_t time_difffff = 0;
static void measureTxframeInterval(void)
{
	
	uint32_t tx_FramePeriod = 0 ;

	if(startCount == 0)
		{
			timeStart = osKernelSysTick();
			startCount = 1;
		}
		else if (startCount == 1)
		{
			
			TimeDiff_frame = osKernelSysTick() - timeStart;
			
			time_difffff = TimeDiff_frame / 144 ;
			
			tx_FramePeriod = TimeDiff_frame * 1000 / (8*MCU_CLK_MHz);
			APP_DATA->Frame_Period = tx_FramePeriod;				//THIHA KYAW Recheck again
			APP_DATA->status = APP_JM;									//State change to Jammer Mode

			startCount = 0;
		}

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn uint32_t avgTxframeInterval(uint32_t)
 *
 * @brief Averaging time period
 *
 * @param  void
 *
 * @return  none
 */

static void avgTxframeInterval(uint32_t timeperiod)
{

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn uint32_t rx_process(void)
 *
 * @brief Receive and measure time framee
 *
 * @param  void
 *
 * @return  void
 */

static void rx_process(void)
{
	int i;
	static uint16 frame_len = 0;
	/* TESTING BREAKPOINT LOCATION #1 */

	/* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
	 * the RX buffer.
	 * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
	 * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
	for (i = 0 ; i < FRAME_LEN_MAX; i++ )
	{
			rx_buffer[i] = 0;
	}

	/* Activate reception immediately. See NOTE 3 below. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & ( SYS_STATUS_RXSFDD)))
		{
				BSP_LED_On(LED1);
				BSP_LED_Off(LED3);
		};

		APP_DATA->Frame_Period = 200;				//THIHA KYAW Recheck again
		APP_DATA->status = APP_JM;					//State change to Jammer Mode
		BSP_LED_On(LED3);
		BSP_LED_Off(LED1);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn void init_DWM(void)
 *
 * @brief Initialization DWM
 *
 * @param  void
 *
 * @return  void
 */

static void init_DWM(void)
{
		reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
		port_set_dw1000_slowrate();

		if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
		{
				//Error
				led_on(LED_ALL); //to display error....
				while (1)
				{ };
		}


		/* Configure GPIOs to show TX/RX activity. See NOTE 6 below. */
		dwt_setlnapamode(1, 1);

//		/* Configure LEDs management. See NOTE 6 below. */
		dwt_setleds(DWT_LEDS_ENABLE);
		
		#ifdef NormalTx 
		port_set_dw1000_fastrate();
		/* Configure DW1000. See NOTE 3 below. */
		dwt_configure(&config);

		dwt_configuretxrf(&txconfig);

		#endif


}





