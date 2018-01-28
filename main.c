#include "lpc17xx_can.h"
#include "lpc17xx_libcfg_default.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_clkpwr.h"
#include<time.h>

#include<stdio.h>

/** CAN variable definition **/
CAN_MSG_Type TXMsg, RXMsg;
uint32_t CANRxCount, CANTxCount = 0;
int i;
/************************** PRIVATE FUNCTIONS *************************/
void CAN_IRQHandler(void);
void CAN_InitMessage(void);
void task_CAN (void);
Bool Check_Message(CAN_MSG_Type* TX_Msg, CAN_MSG_Type* RX_Msg);

uint8_t CAN_RxRdy[2] = {0,0};
uint8_t CAN_TxRdy[2] = {0,0};



void CAN_IRQHandler()
{
	uint8_t IntStatus;
	/* Get CAN status */
	IntStatus = CAN_GetCTRLStatus(LPC_CAN2, CANCTRL_STS);
	/* check receive buffer status */
	if((IntStatus>>0)&0x01)
	{
		CAN_ReceiveMsg(LPC_CAN2,&RXMsg);
		/* Validate received and transmited message */
		if(Check_Message(&TXMsg, &RXMsg))
		{
			printf("rxd msg is %x\n ",*RXMsg.dataA);
		}
		else
		{
			printf("no msg rxd\n");
		}
		CAN_RxRdy[1] = 1;
	}
	IntStatus = CAN_GetCTRLStatus(LPC_CAN1, CANCTRL_INT_CAP);
	if( IntStatus & (1<<1) )
	{
		CAN_TxRdy[1] = 1;
	}
	IntStatus = CAN_GetCTRLStatus(LPC_CAN2, CANCTRL_INT_CAP);
		if( IntStatus & (1<<1) )
		{
			CAN_RxRdy[1] = 1;
		}
}

/*********************************************************************//**
 * @brief		Initialize transmit and receive message for Bypass operation
 * @param[in]	none
 * @return 		none
 **********************************************************************/
void CAN_InitMessage(void) {
	TXMsg.format = EXT_ID_FORMAT;
	TXMsg.id = 0x001234;
	TXMsg.len = 8;
	TXMsg.type = DATA_FRAME;
	TXMsg.dataA[0] = TXMsg.dataA[1] = TXMsg.dataA[2] = TXMsg.dataA[3] = 0x12;
	TXMsg.dataB[0] = TXMsg.dataB[1] = TXMsg.dataB[2] = TXMsg.dataB[3] = 0x34;

	RXMsg.format = EXT_ID_FORMAT;
	RXMsg.id = 0x001234;
	RXMsg.len = 0x00;
	RXMsg.type = 0x00;
	RXMsg.dataA[0] = RXMsg.dataA[1] = RXMsg.dataA[2] = RXMsg.dataA[3] = 0x00000000;
	RXMsg.dataB[0] = RXMsg.dataB[1] = RXMsg.dataB[2] = RXMsg.dataB[3] = 0x00000000;
}

/*********************************************************************//**
 * @brief		Compare two message
 * @param[in]	Tx_Msg transmit message
 * @param[in]	Rx_Msg receive message
 * @return 		Bool	should be:
 * 				- TRUE: if two message is the same
 * 				- FALSE: if two message is different
 **********************************************************************/
Bool Check_Message(CAN_MSG_Type* TX_Msg, CAN_MSG_Type* RX_Msg)
{

	uint8_t i;
	if((TXMsg.format != RXMsg.format)|(TXMsg.id != RXMsg.id)|(TXMsg.len != RXMsg.len)\
		|(TXMsg.type != RXMsg.type))
		return FALSE;
	for(i=0;i<4;i++)
	{
		if((TXMsg.dataA[i]!=RXMsg.dataA[i])|(TXMsg.dataB[i]!=RXMsg.dataB[i]))
			return FALSE;
	}
	return TRUE;
}

void task_CAN () {
	uint8_t IntStatus;
	/* Enable GPIO Clock */
		    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPIO, ENABLE);
		    // CAN1
		    // P0.0~1, function 0x01, Pin46 RxD1, Pin47 TxD1
		     LPC_PINCON->PINSEL0 &=~0x0F;        //clear P0.0~1
		     LPC_PINCON->PINSEL0 |= 0x05;        //set b0101

		    // CAN2:
		    // P0.4~5, function 0x01, Pin81 RxD2, Pin80 TxD2
		     LPC_PINCON->PINSEL0 &=~(0x0F<<8);    //clear P0.4~5
		     LPC_PINCON->PINSEL0 |= (0x0A<<8);    //set b1010

	/* Initialize CAN1 peripheral
		 * Note: Self-test mode doesn't require pin selection
		 */
		CAN_DeInit(LPC_CAN2);
		printf(" *before CAN_Init* \r\n");
		CAN_Init(LPC_CAN2, 500000);
		CAN_Init(LPC_CAN1,500000);
		NVIC_EnableIRQ(CAN_IRQn);
		CAN_InitMessage();


		/* Enable self-test mode */
		CAN_ModeConfig(LPC_CAN2, CAN_OPERATING_MODE, ENABLE);
		CAN_ModeConfig(LPC_CAN1, CAN_OPERATING_MODE, ENABLE);
		/* Enable CAN Interrupt */
		CAN_SetAFMode(LPC_CANAF,CAN_AccBP);
		/* Enable Interrupt */
		CAN_IRQCmd(LPC_CAN2, CANINT_RIE, ENABLE);
		CAN_IRQCmd(LPC_CAN2, CANINT_TIE1, ENABLE);
		CAN_IRQCmd(LPC_CAN1, CANINT_RIE, ENABLE);
		CAN_IRQCmd(LPC_CAN1, CANINT_TIE1, ENABLE);
		printf(" *after CAN_Init* \r\n");
		/** To test Bypass Mode: we send infinite messages to CAN2 and check
		 * receive process via COM1
		 */
		char a=CAN_SendMsg(LPC_CAN1, &TXMsg);
		if(a==SUCCESS)
		{
		printf("msg transmitted is %x\n",*TXMsg.dataA);
		}
        for(i=0;i<100;i++);
		IntStatus = CAN_GetCTRLStatus(LPC_CAN2, CANCTRL_STS);
		/* check receive buffer status */
		printf("%x\r\n",IntStatus);
		if((IntStatus>>0)&0x01)

		{
				//CAN_ReceiveMsg(LPC_CAN2,&RXMsg);
			char b=CAN_ReceiveMsg(LPC_CAN2,&RXMsg);
			if(b==SUCCESS){
				printf("rxd msg is %x\r\n",*RXMsg.dataA);}
			else{
			printf("msg not received\r\n");}

		}
		if(Check_Message(&TXMsg, &RXMsg))
				{
					printf("rxd msg is %x\r\n ",*RXMsg.dataA);
				}
				else
				{
					printf("no msg rxd\r\n");
				}



		}

int main(void)
{
	task_CAN();			    /*!< Start multitask	           */
	while(1)
	{
	printf("hi\r\n");
	}
}

