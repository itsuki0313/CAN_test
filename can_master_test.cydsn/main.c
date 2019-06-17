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

CY_ISR_PROTO(ISR_CAN_1);

uint8 Tx_Data;
uint8 Rx_Data;
char8 uartbuf[256] = {0};

int main()
{   
    CAN_1_Start();
    UART_1_Start();
    
    /* Set CAN interrupt handler to local routine */
    CyIntSetVector(CAN_1_ISR_NUMBER, ISR_CAN_1);
    
    CyGlobalIntEnable;
    LED_Write(1);

    Rx_Data = 0;
    
    for(;;)
    {
        Tx_Data = 150;
        CAN_1_SendMsg0();
        CyDelay(50);
        
        Rx_Data = CAN_1_RX_DATA_BYTE1(CAN_1_RX_MAILBOX_0);
        
        sprintf(uartbuf,"%d %d\n",Rx_Data,Tx_Data);
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