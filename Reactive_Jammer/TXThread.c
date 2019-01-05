
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "stm32f7xx_hal.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "commands.h"

/* --------------------  Private variables -------------------------------------*/

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
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
static uint8 tx_msg[] = {0x61, 0x88, 0, 0xCA, 0xDE, 'X', 'R', 'X', 'T', 'm', 'a', 'c', 'p', 'a', 'y', 'l', 'o', 'a', 'd', 0, 0};
	
/* Index to access the sequence number and frame control fields in frames sent and received. */
#define FRAME_FC_IDX 0
#define FRAME_SN_IDX 2
/* ACK frame control value. */
#define ACK_FC_0 0x02
#define ACK_FC_1 0x00
	
/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000
	
	/* Receive response timeout, expressed in UWB microseconds (UUS, 1 uus = 512/499.2 µs). See NOTE 3 below. */
#define RX_RESP_TO_UUS 2200
	
/* Buffer to store received frame. See NOTE 4 below. */
#define ACK_FRAME_LEN 5
static uint8 rx_buffer[ACK_FRAME_LEN];

	
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/* ACK status for last transmitted frame. */
static int tx_frame_acked = 0;

/* Counters of frames sent, frames ACKed and frame retransmissions. See NOTE 1 below. */
static uint32 tx_frame_nb = 0;
static uint32 tx_frame_ack_nb = 0;
static uint32 tx_frame_retry_nb = 0;


/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
void TXThread (void const *argument);                             // thread function
osThreadId tid_TXThread;                                          // thread id
osThreadDef (TXThread, osPriorityNormal, 1, 5120);                   // thread object

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
		
		/* Set delay to turn reception on immediately after transmission of the frame. See NOTE 6 below. */
    dwt_setrxaftertxdelay(0);
		
		/* Set RX frame timeout for the response. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);
		
		
		/****************************START THREAD**********************************************/

  tid_TXThread = osThreadCreate (osThread(TXThread), NULL);
  if (!tid_TXThread) return(-1);
  
  return(0);
}

void TXThread (void const *argument) {
	
	

  while (1) {
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below.*/
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

			/* We assume that the transmission is achieved normally, now poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, check frame length is correct for ACK, then read and verify the ACK. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len == ACK_FRAME_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Check if it is the expected ACK. */
                if ((rx_buffer[FRAME_FC_IDX] == ACK_FC_0) && (rx_buffer[FRAME_FC_IDX + 1] == ACK_FC_1)
                    && (rx_buffer[FRAME_SN_IDX] == tx_msg[FRAME_SN_IDX]))
                {
                    tx_frame_acked = 1;
                }
            }
        }
        else
        {
            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* Update number of frames sent. */
        tx_frame_nb++;

        if (tx_frame_acked)
        {
            /* Execute a delay between transmissions. See NOTE 1 below. */
            osDelay(TX_DELAY_MS);

            /* Increment the sent frame sequence number (modulo 256). */
            tx_msg[FRAME_SN_IDX]++;

            /* Update number of frames acknowledged. */
            tx_frame_ack_nb++;
        }
        else
        {
            /* Update number of retransmissions. */
            tx_frame_retry_nb++;
        }

        /* Reset acknowledged frame flag. */
        tx_frame_acked = 0;

  }
}
