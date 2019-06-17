/*******************************************************************************
* File Name: TeraTermINT.c
* Version 2.50
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "TeraTerm.h"
#include "cyapicallbacks.h"


/***************************************
* Custom Declarations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if (TeraTerm_RX_INTERRUPT_ENABLED && (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED))
    /*******************************************************************************
    * Function Name: TeraTerm_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_rxBuffer - RAM buffer pointer for save received data.
    *  TeraTerm_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  TeraTerm_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  TeraTerm_rxBufferOverflow - software overflow flag. Set to one
    *     when TeraTerm_rxBufferWrite index overtakes
    *     TeraTerm_rxBufferRead index.
    *  TeraTerm_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when TeraTerm_rxBufferWrite is equal to
    *    TeraTerm_rxBufferRead
    *  TeraTerm_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  TeraTerm_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(TeraTerm_RXISR)
    {
        uint8 readData;
        uint8 readStatus;
        uint8 increment_pointer = 0u;

    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef TeraTerm_RXISR_ENTRY_CALLBACK
        TeraTerm_RXISR_EntryCallback();
    #endif /* TeraTerm_RXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START TeraTerm_RXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        do
        {
            /* Read receiver status register */
            readStatus = TeraTerm_RXSTATUS_REG;
            /* Copy the same status to readData variable for backward compatibility support 
            *  of the user code in TeraTerm_RXISR_ERROR` section. 
            */
            readData = readStatus;

            if((readStatus & (TeraTerm_RX_STS_BREAK | 
                            TeraTerm_RX_STS_PAR_ERROR |
                            TeraTerm_RX_STS_STOP_ERROR | 
                            TeraTerm_RX_STS_OVERRUN)) != 0u)
            {
                /* ERROR handling. */
                TeraTerm_errorStatus |= readStatus & ( TeraTerm_RX_STS_BREAK | 
                                                            TeraTerm_RX_STS_PAR_ERROR | 
                                                            TeraTerm_RX_STS_STOP_ERROR | 
                                                            TeraTerm_RX_STS_OVERRUN);
                /* `#START TeraTerm_RXISR_ERROR` */

                /* `#END` */
                
            #ifdef TeraTerm_RXISR_ERROR_CALLBACK
                TeraTerm_RXISR_ERROR_Callback();
            #endif /* TeraTerm_RXISR_ERROR_CALLBACK */
            }
            
            if((readStatus & TeraTerm_RX_STS_FIFO_NOTEMPTY) != 0u)
            {
                /* Read data from the RX data register */
                readData = TeraTerm_RXDATA_REG;
            #if (TeraTerm_RXHW_ADDRESS_ENABLED)
                if(TeraTerm_rxAddressMode == (uint8)TeraTerm__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readStatus & TeraTerm_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readStatus & TeraTerm_RX_STS_ADDR_MATCH) != 0u)
                        {
                            TeraTerm_rxAddressDetected = 1u;
                        }
                        else
                        {
                            TeraTerm_rxAddressDetected = 0u;
                        }
                    }
                    if(TeraTerm_rxAddressDetected != 0u)
                    {   /* Store only addressed data */
                        TeraTerm_rxBuffer[TeraTerm_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* Without software addressing */
                {
                    TeraTerm_rxBuffer[TeraTerm_rxBufferWrite] = readData;
                    increment_pointer = 1u;
                }
            #else  /* Without addressing */
                TeraTerm_rxBuffer[TeraTerm_rxBufferWrite] = readData;
                increment_pointer = 1u;
            #endif /* (TeraTerm_RXHW_ADDRESS_ENABLED) */

                /* Do not increment buffer pointer when skip not addressed data */
                if(increment_pointer != 0u)
                {
                    if(TeraTerm_rxBufferLoopDetect != 0u)
                    {   /* Set Software Buffer status Overflow */
                        TeraTerm_rxBufferOverflow = 1u;
                    }
                    /* Set next pointer. */
                    TeraTerm_rxBufferWrite++;

                    /* Check pointer for a loop condition */
                    if(TeraTerm_rxBufferWrite >= TeraTerm_RX_BUFFER_SIZE)
                    {
                        TeraTerm_rxBufferWrite = 0u;
                    }

                    /* Detect pre-overload condition and set flag */
                    if(TeraTerm_rxBufferWrite == TeraTerm_rxBufferRead)
                    {
                        TeraTerm_rxBufferLoopDetect = 1u;
                        /* When Hardware Flow Control selected */
                        #if (TeraTerm_FLOW_CONTROL != 0u)
                            /* Disable RX interrupt mask, it is enabled when user read data from the buffer using APIs */
                            TeraTerm_RXSTATUS_MASK_REG  &= (uint8)~TeraTerm_RX_STS_FIFO_NOTEMPTY;
                            CyIntClearPending(TeraTerm_RX_VECT_NUM);
                            break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                        #endif /* (TeraTerm_FLOW_CONTROL != 0u) */
                    }
                }
            }
        }while((readStatus & TeraTerm_RX_STS_FIFO_NOTEMPTY) != 0u);

        /* User code required at end of ISR (Optional) */
        /* `#START TeraTerm_RXISR_END` */

        /* `#END` */

    #ifdef TeraTerm_RXISR_EXIT_CALLBACK
        TeraTerm_RXISR_ExitCallback();
    #endif /* TeraTerm_RXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
    }
    
#endif /* (TeraTerm_RX_INTERRUPT_ENABLED && (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)) */


#if (TeraTerm_TX_INTERRUPT_ENABLED && TeraTerm_TX_ENABLED)
    /*******************************************************************************
    * Function Name: TeraTerm_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_txBuffer - RAM buffer pointer for transmit data from.
    *  TeraTerm_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmitted byte.
    *  TeraTerm_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(TeraTerm_TXISR)
    {
    #if(CY_PSOC3)
        uint8 int_en;
    #endif /* (CY_PSOC3) */

    #ifdef TeraTerm_TXISR_ENTRY_CALLBACK
        TeraTerm_TXISR_EntryCallback();
    #endif /* TeraTerm_TXISR_ENTRY_CALLBACK */

        /* User code required at start of ISR */
        /* `#START TeraTerm_TXISR_START` */

        /* `#END` */

    #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
        int_en = EA;
        CyGlobalIntEnable;
    #endif /* (CY_PSOC3) */

        while((TeraTerm_txBufferRead != TeraTerm_txBufferWrite) &&
             ((TeraTerm_TXSTATUS_REG & TeraTerm_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer wrap around */
            if(TeraTerm_txBufferRead >= TeraTerm_TX_BUFFER_SIZE)
            {
                TeraTerm_txBufferRead = 0u;
            }

            TeraTerm_TXDATA_REG = TeraTerm_txBuffer[TeraTerm_txBufferRead];

            /* Set next pointer */
            TeraTerm_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START TeraTerm_TXISR_END` */

        /* `#END` */

    #ifdef TeraTerm_TXISR_EXIT_CALLBACK
        TeraTerm_TXISR_ExitCallback();
    #endif /* TeraTerm_TXISR_EXIT_CALLBACK */

    #if(CY_PSOC3)
        EA = int_en;
    #endif /* (CY_PSOC3) */
   }
#endif /* (TeraTerm_TX_INTERRUPT_ENABLED && TeraTerm_TX_ENABLED) */


/* [] END OF FILE */
