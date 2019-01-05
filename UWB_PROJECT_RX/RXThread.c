
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "stm32f7xx_hal.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "commands.h"
#include "common.h"
#include "stm32f7xx_nucleo_144.h"
/* --------------------  Private variables -------------------------------------*/

uint16_t MCU_CLK_MHz = 144;

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

static dwt_txconfig_t txconfig = {
    0xC2,            /* PG delay. */
    0x67676767,      /* TX power. */
};


/* PAN ID/EUI/short address. See NOTE 1 and 2 below. */
static uint16 pan_id = 0xDECA;
static uint8 eui[] = {'A', 'C', 'K', 'D', 'A', 'T', 'R', 'X'};
static uint16 short_addr = 0x5258; /* "RX" */

static uint8 tx_msg[] = {0x61, 0x88, 0, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', //26
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z',  'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
													'l','m', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v','w', 'x', 'y', 'z', 0, 0};

/* Buffer to store received frame. See NOTE 3 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* ACK request bit mask in DATA and MAC COMMAND frame control's first byte. */
#define FCTRL_ACK_REQ_MASK 0x20

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
//static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
//static uint16 frame_len = 0;

//To calculate Time Difference between each frame
uint32_t TimeDiff_frame=0x00; 
uint8_t startCount=0;
uint32_t timeStart = 0 ;
uint32_t time_difffff = 0;


app_data* APP_DATA;

/*********************APPLICATION Variable**************************************/

char no_MSG[21]={"No Message Received\n\r"};
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void RXThread (void const *argument);                             // thread function
osThreadId tid_RXThread;                                          // thread id
osThreadDef (RXThread, osPriorityNormal, 1, 5120);                   // thread object


/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void measureTxframeInterval(void);

int Init_RXThread (void) {

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
		/* Set PAN ID, EUI and short address. See NOTE 2 below. */
    dwt_setpanid(pan_id);
    dwt_seteui(eui);
    dwt_setaddress16(short_addr);

    /* Configure frame filtering. Only data frames are enabled in this example. Frame filtering must be enabled for Auto ACK to work. */
    dwt_enableframefilter(DWT_FF_DATA_EN);

    /* Activate auto-acknowledgement. Time is set to 0 so that the ACK is sent as soon as possible after reception of a frame. */
    dwt_enableautoack(0);
		
		/* Activate double buffering. */
    dwt_setdblrxbuffmode(1);
		
		/* Register RX call-back. */
    dwt_setcallbacks(NULL, &rx_ok_cb, NULL, &rx_err_cb);

    /* Enable wanted interrupts (RX good frames and RX errors). */
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL| DWT_INT_RXOVRR | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_SFDT | DWT_INT_ARFE, 1);

    /* Activate reception immediately. See NOTE 3 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

  
	tid_RXThread = osThreadCreate (osThread(RXThread), NULL);
  if (!tid_RXThread) return(-1);
  
  return(0);
}

void RXThread (void const *argument) {

  while (1) {
		
  
  }
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
int j;
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    /* Perform manual RX re-enabling. See NOTE 5 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE | DWT_NO_SYNC_PTRS);
	
		int i,  no_error=0;

    /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading the RX
     * buffer. */
    for (i = 0 ; i < FRAME_LEN_MAX; i++ )
    {
        rx_buffer[i] = 0;
    }

    /* TESTING BREAKPOINT LOCATION #1 */

    /* A frame has been received, copy it to our local buffer. See NOTE 6 below. */
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
			
				//Check if receive message is correct
			
//				for (j = 0 ; j < (cb_data->datalength-2); j++ )
//				{
//						if(rx_buffer[j] != tx_msg[j])
//						{
//							BSP_LED_Toggle(LED3);

//						}				
//				}
//				uart_sendmsg((char*)rx_buffer, sizeof(rx_buffer));
				
    }

    /* TESTING BREAKPOINT LOCATION #2 */
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
	
	//DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL| DWT_INT_RXOVRR | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_SFDT 
	
		if(isrStatus & DWT_INT_RPHE) // receiver PHY header error
		{
			uart_sendmsg((char*) "\n\r receiver PHY header error\n\r", 30);
		}
		if(isrStatus & DWT_INT_RFCE) // receiver CRC error
		{
			uart_sendmsg((char*) "\n\rreceiver CRC error\n\r", 30);
		}
		if(isrStatus & DWT_INT_RFSL) // receiver sync loss error
		{
			uart_sendmsg((char*) "\n\rreceiver sync loss error\n\r", 30);
		}
		if(isrStatus & DWT_INT_RXOVRR) // receiver overrun
		{
			uart_sendmsg((char*) "\n\rreceiver overrun\n\r", 30);
		}
		if(isrStatus & DWT_INT_RFTO) // frame wait timeout
		{
			uart_sendmsg((char*) "\n\rframe wait timeout\n\r", 30);
		}
		if(isrStatus & DWT_INT_RXPTO) // preamble detect timeout
		{
			uart_sendmsg((char*) "\n\rpreamble detect timeout\n\r", 30);
		}
		if(isrStatus & DWT_INT_ARFE) // Frame Filter Rejection
		{
			uart_sendmsg((char*) "\n\rFrame Filter Rejection\n\r", 30);
		}
//		if(isrStatus & DWT_INT_SFDT) // SFD timeout
//		{
//			uart_sendmsg((char*) "\n\rSFD timeout\n\r", 15);
//		}
//		else
//		{
//			uart_sendmsg((char*) "\n\rUnknown Error\n\r", 30);
//		}


	/* Re-activate reception immediately. */
 dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* TESTING BREAKPOINT LOCATION #3 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn uint32_t measureTxframeInterval(void)
 *
 * @brief Measure Time difference between two transmit frame
 *
 * @param  void
 *
 * @return  Frame Period of transmitter
 */

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
			
//			time_difffff = TimeDiff_frame / 144 ;
			
			tx_FramePeriod = TimeDiff_frame * 1000 / (8*MCU_CLK_MHz);
			APP_DATA->Frame_Period = tx_FramePeriod;				//THIHA KYAW Recheck again
			APP_DATA->status = APP_JM;									//State change to Jammer Mode

			startCount = 0;
		}

}
