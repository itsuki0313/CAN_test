/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <stdio.h>

#define Tx_Data_MESSAGE_ID              (0x001u)
#define Tx_Data_MESSAGE_IDE             (0u)    /* Standard message */
#define Tx_Data_MESSAGE_IRQ             (0u)    /* No transmit IRQ */
#define Tx_Data_MESSAGE_RTR             (0u)    /* No RTR */

CY_ISR_PROTO(ISR_CAN_1);

CAN_1_DATA_BYTES_MSG dataTX;
CAN_1_TX_MSG Tx_Data;
uint8 Rx_Data;
char8 uartbuf[256] = {0};

int main()
{   
    /* BASIC CAN mailbox configuration */
    Tx_Data.dlc = CAN_1_TX_DLC_MAX_VALUE;
    Tx_Data.id  = Tx_Data_MESSAGE_ID;
    Tx_Data.ide = Tx_Data_MESSAGE_IDE;
    Tx_Data.irq = Tx_Data_MESSAGE_IRQ;
    Tx_Data.msg = &dataTX;
    Tx_Data.rtr = Tx_Data_MESSAGE_RTR;
    
    CAN_1_Start();
    UART_1_Start();
    
    /* Set CAN interrupt handler to local routine */
    CyIntSetVector(CAN_1_ISR_NUMBER, ISR_CAN_1);
    
    CyGlobalIntEnable;

    Rx_Data = 0;
    
    for(;;)
    {
        LED_Write(1);
        dataTX.byte[0u] = 100;
        CAN_1_SendMsg(&Tx_Data);
        CyDelay(50);
        
        Rx_Data = CAN_1_RX_DATA_BYTE1(CAN_1_RX_MAILBOX_0);
        //CyDelay(250);
        
        sprintf(uartbuf,"%d\n",Rx_Data);
        UART_1_PutString(uartbuf);
    }
}

CY_ISR(ISR_CAN_1)
{
    /* Clear Receive Message flag */
    CAN_1_INT_SR_REG.byte[1u] = CAN_1_RX_MESSAGE_MASK;
    
    /* Acknowledges receipt of new message */
    CAN_1_RX_ACK_MESSAGE(CAN_1_RX_MAILBOX_0);
}

/* [] END OF FILE */