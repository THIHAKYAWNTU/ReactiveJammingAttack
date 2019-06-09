/**
  ******************************************************************************
  * File Name          : commands.c
  * Description        : This file contains lists of commands which can be used.
	* Software Engineer	 : THIHA KYAW
  ******************************************************************************
  */

#include "commands.h"
#include "port.h"
#include "stm32f7xx_nucleo_144.h"
#include "global.h"
#include "cmsis_os.h" 
extern UART_HandleTypeDef huart2;

var_DataTypeDef txSettings;

static int8_t getline (char *line, int32_t n);

/* Software Reset function prototypes */
static void cmd_TXEN    (char *par);
static void cmd_TXDUMMY(char *par);
static void cmd_reset    (char *par);
static void cmd_TimeP(char *par);
static void cmd_TimeMicroP(char *par);
static void cmd_NumPackets(char *par);
static void cmd_TimeP2P(char *par);
/* Command function prototypes */

static void cmd_ledallON(char *par);
static void cmd_ledallOFF(char *par);

static char *get_entry (char *cp, char **pNext);


 SCMD cmd[] = {
	 
	 "RESET",			cmd_reset,		 
	 /*******Commands for LED*********/
	 "LEDALL1",   cmd_ledallON,
	 "LEDALL0",   cmd_ledallOFF,
	 /*******Commands for Transmitter*********/
	 "TXEN",			cmd_TXEN,
	 "TXDMY",			cmd_TXDUMMY,
	 "TIMEP",   	cmd_TimeP,
	 "TIMEUP",   	cmd_TimeMicroP,
	 "TIMEP2P",   cmd_TimeP2P,
	 "NUMP",   		cmd_NumPackets,
	 
};

/* Local variables */
char in_line[160];
char uart_OK[4]={"OK\n\r"};
char cmd_error[15]={"Command Error\n\r"};
char Buf[10]={'\0'};
/* Local variables */
//static char in_line[160];

/*-----------------------------------------------------------------------------
 *        Process input string for long or short name entry
 *----------------------------------------------------------------------------*/
static char *get_entry (char *cp, char **pNext) {
  char *sp, lfn = 0, sep_ch = ' ';

  if (cp == NULL) {                           /* skip NULL pointers           */
    *pNext = cp;
    return (cp);
  }

  for ( ; *cp == ' ' || *cp == '\"'; cp++) {  /* skip blanks and starting  "  */
    if (*cp == '\"') { sep_ch = '\"'; lfn = 1; }
    *cp = 0;
  }
 
  for (sp = cp; *sp != CR && *sp != LF && *sp != 0; sp++) {
    if ( lfn && *sp == '\"') break;
    if (!lfn && *sp == ' ' ) break;
  }

  for ( ; *sp == sep_ch || *sp == CR || *sp == LF; sp++) {
    *sp = 0;
    if ( lfn && *sp == sep_ch) { sp ++; break; }
  }

  *pNext = (*sp) ? sp : NULL;                 /* next entry                   */
  return (cp);
}

/*------------------------------------------------------------------------------
 *      Line Editor
 *----------------------------------------------------------------------------*/
static int8_t getline (char *line, int32_t n)  {
  int32_t cnt = 0;
  char c;

  do {
    c = getchar ();
    switch (c) {
      case CNTLQ:                          /* ignore Control S/Q             */
      case CNTLS:
        break;
      case BACKSPACE:
      case DEL:
        if (cnt == 0) {
          break;
        }
        cnt--;                             /* decrement count                */
        line--;                            /* and line pointer               */
        putchar (BACKSPACE);               /* echo backspace                 */
        putchar (' ');
        putchar (BACKSPACE);
        fflush (stdout);
        break;
      case ESC:
        *line = 0;                         /* ESC - stop editing line        */
        return 0;
      case CR:                             /* CR - done, stop editing line   */
        *line = c;
        line++;                            /* increment line pointer         */
        cnt++;                             /* and count                      */
        c = LF;
      default:
        putchar (*line = c);               /* echo and store character       */
        fflush (stdout);
        line++;                            /* increment line pointer         */
        cnt++;                             /* and count                      */
        break;
    }
  } while (cnt < n - 2  &&  c != LF);      /* check limit and CR             */

  *line = 0;                               /* mark end of string             */
  
  return 1;
}


/************************************************************************************/
static void cmd_ledallON(char *par)
{

	led_on(LED_ALL);
}

static void cmd_ledallOFF(char *par)
{
	led_off(LED_ALL);
}

void execute_CMD(char *par)
{
		char *sp,*cp,*next;
		uint32_t i;
	
	  sp = get_entry (par, &next);

    for (cp = sp; *cp && *cp != ' '; cp++) {
      *cp = toupper (*cp);                    /* command to upper-case      */
    }
    for (i = 0; i < CMD_COUNT; i++) {
			//if Return value = 0 then it indicates str1 is equal to str2.
      if (strcmp (sp, (const char *)&cmd[i].val)) {	
				//Skip For loop
        continue;
      }
      cmd[i].func (next);                     /* execute command function   */
      break;
    }
		
    if (i == CMD_COUNT) {
			
			uart_sendmsg(cmd_error);
    }
}

/*-----------------------------------------------------------------------------
 * Software Reset Function
 *----------------------------------------------------------------------------*/
static void cmd_reset    (char *par)
{
	HAL_NVIC_SystemReset();
}

/*-----------------------------------------------------------------------------
 *   Enable And Disable Transmission
 *	"TXEN w value"					           
	*	"TXEN r value"					
 *----------------------------------------------------------------------------*/

static void cmd_TXEN(char *par)
{
	char *cmd, *next;
	char *writePtr = "w";
	char *readPtr = "r";
	int value;

	//Check the Command
  cmd = get_entry (par, &next);
//		
	//If cmd is write
	if(strcmp(cmd, writePtr) == 0)	//it is write command
	{
			// if cmd = write, check value
			if (next) {
			par = get_entry (next, &next);

			if (sscanf (par,"%d", &value) == 0) {
				//printf ("\nCommand error.\n");
				return;
			}
			else
			{
				txSettings.TXEN = value;
			}
					
		}
		// Response OK
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) uart_OK, 4)!= HAL_OK)
		{

		}
	}
	else if (strcmp(cmd, readPtr) == 0)
	{

		sprintf(Buf,"%d\n\r", txSettings.TXEN );
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) Buf, sizeof(Buf))!= HAL_OK)
		{

		}

	}
	
}

/*-----------------------------------------------------------------------------
 *   Enable And Disable Transmission
 *	"TXDMY w value"					           
	*	"TXDMY r value"					
 *----------------------------------------------------------------------------*/

static void cmd_TXDUMMY(char *par)
{
	char *cmd, *next;
	char *writePtr = "w";
	char *readPtr = "r";
	int value;

	//Check the Command
  cmd = get_entry (par, &next);
//		
	//If cmd is write
	if(strcmp(cmd, writePtr) == 0)	//it is write command
	{
			// if cmd = write, check value
			if (next) {
			par = get_entry (next, &next);

			if (sscanf (par,"%d", &value) == 0) {
				//printf ("\nCommand error.\n");
				return;
			}
			else
			{
				txSettings.TXDUMMY = value;
			}
					
		}
		// Response OK
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) uart_OK, 4)!= HAL_OK)
		{

		}
	}
	else if (strcmp(cmd, readPtr) == 0)
	{

		sprintf(Buf,"%d\n\r", txSettings.TXDUMMY );
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) Buf, sizeof(Buf))!= HAL_OK)
		{

		}

	}
	
}


/*-----------------------------------------------------------------------------
 *   Time between dummy packet and actual packet
 *	"TIMEP w value"					           
	*	"TIMEP r value"					
 *----------------------------------------------------------------------------*/

static void cmd_TimeP(char *par)
{
	char *cmd, *next;
	char *writePtr = "w";
	char *readPtr = "r";
	int value;

	//Check the Command
  cmd = get_entry (par, &next);
//		
	//If cmd is write
	if(strcmp(cmd, writePtr) == 0)	//it is write command
	{
			// if cmd = write, check value
			if (next) {
			par = get_entry (next, &next);

			if (sscanf (par,"%d", &value) == 0) {
				//printf ("\nCommand error.\n");
				return;
			}
			else
			{
				txSettings.TP = value;
			}
					
		}
		// Response OK
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) uart_OK, 4)!= HAL_OK)
		{

		}
	}
	else if (strcmp(cmd, readPtr) == 0)
	{

		sprintf(Buf,"%d\n\r", txSettings.TP );
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) Buf, sizeof(Buf))!= HAL_OK)
		{

		}

	}
	
}

/*-----------------------------------------------------------------------------
 *   Time between dummy packet and actual packet
 *	"TIMEUP w value"					           
	*	"TIMEUP r value"					
 *----------------------------------------------------------------------------*/

static void cmd_TimeMicroP(char *par)
{
	char *cmd, *next;
	char *writePtr = "w";
	char *readPtr = "r";
	int value;

	//Check the Command
  cmd = get_entry (par, &next);
//		
	//If cmd is write
	if(strcmp(cmd, writePtr) == 0)	//it is write command
	{
			// if cmd = write, check value
			if (next) {
			par = get_entry (next, &next);

			if (sscanf (par,"%d", &value) == 0) {
				//printf ("\nCommand error.\n");
				return;
			}
			else
			{
				txSettings.microTP = value;
			}
					
		}
		// Response OK
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) uart_OK, 4)!= HAL_OK)
		{

		}
	}
	else if (strcmp(cmd, readPtr) == 0)
	{

		sprintf(Buf,"%d\n\r", txSettings.microTP);
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) Buf, sizeof(Buf))!= HAL_OK)
		{

		}

	}
	
}

/*-----------------------------------------------------------------------------
 *  Time between each packet
 *	"TP2P w value"					            
	*	"TP2P r value"					
 *----------------------------------------------------------------------------*/

static void cmd_TimeP2P(char *par)
{
	char *cmd, *next;
	char *writePtr = "w";
	char *readPtr = "r";
	int value;

	//Check the Command
  cmd = get_entry (par, &next);
//		
	//If cmd is write
	if(strcmp(cmd, writePtr) == 0)	//it is write command
	{
			// if cmd = write, check value
			if (next) {
			par = get_entry (next, &next);

			if (sscanf (par,"%d", &value) == 0) {
				//printf ("\nCommand error.\n");
				return;
			}
			else
			{
				txSettings.TP2P = value;
			}
					
		}
		// Response OK
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) uart_OK, 4)!= HAL_OK)
		{

		}
	}
	else if (strcmp(cmd, readPtr) == 0)
	{

		sprintf(Buf,"%d\n\r", txSettings.TP2P );
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) Buf, sizeof(Buf))!= HAL_OK)
		{

		}

	}
	
}

/*-----------------------------------------------------------------------------
 *   Number of Packets to transmit
 *	"NUMP w value"					Write DC Offset Value for current sensors            
	*	"NUMP r value"					Read Current DC Offet Value
 *----------------------------------------------------------------------------*/

static void cmd_NumPackets(char *par)
{
	char *cmd, *next;
	char *writePtr = "w";
	char *readPtr = "r";
	int value;

	//Check the Command
  cmd = get_entry (par, &next);
//		
	//If cmd is write
	if(strcmp(cmd, writePtr) == 0)	//it is write command
	{
			// if cmd = write, check value
			if (next) {
			par = get_entry (next, &next);

			if (sscanf (par,"%d", &value) == 0) {
				//printf ("\nCommand error.\n");
				return;
			}
			else
			{
				txSettings.NPackets = value;
				
			}
					
		}
		// Response OK
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) uart_OK, 4)!= HAL_OK)
		{

		}
	}
	else if (strcmp(cmd, readPtr) == 0)
	{

		sprintf(Buf,"%d\n\r", txSettings.NPackets );
		while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) Buf, sizeof(Buf))!= HAL_OK)
		{

		}

	}
	
}

/*-----------------------------------------------------------------------------
 * Uart Send Message
 *----------------------------------------------------------------------------*/
void uart_sendmsg(char *msg)
{
	uint16_t buf_size=0;
	while(msg[buf_size]!=0)
	{
		buf_size++;
	}
	while(HAL_UART_Transmit_IT(&huart2, (uint8_t*) msg, buf_size)!= HAL_OK)
	{

	}
	osDelay(10);

}



/************************ (C) COPYRIGHT ROLLS ROYCE************************/
