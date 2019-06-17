/*******************************************************************************
* File Name: TeraTerm.c
* Version 2.50
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "TeraTerm.h"
#if (TeraTerm_INTERNAL_CLOCK_USED)
    #include "TeraTerm_IntClock.h"
#endif /* End TeraTerm_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 TeraTerm_initVar = 0u;

#if (TeraTerm_TX_INTERRUPT_ENABLED && TeraTerm_TX_ENABLED)
    volatile uint8 TeraTerm_txBuffer[TeraTerm_TX_BUFFER_SIZE];
    volatile uint8 TeraTerm_txBufferRead = 0u;
    uint8 TeraTerm_txBufferWrite = 0u;
#endif /* (TeraTerm_TX_INTERRUPT_ENABLED && TeraTerm_TX_ENABLED) */

#if (TeraTerm_RX_INTERRUPT_ENABLED && (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED))
    uint8 TeraTerm_errorStatus = 0u;
    volatile uint8 TeraTerm_rxBuffer[TeraTerm_RX_BUFFER_SIZE];
    volatile uint8 TeraTerm_rxBufferRead  = 0u;
    volatile uint8 TeraTerm_rxBufferWrite = 0u;
    volatile uint8 TeraTerm_rxBufferLoopDetect = 0u;
    volatile uint8 TeraTerm_rxBufferOverflow   = 0u;
    #if (TeraTerm_RXHW_ADDRESS_ENABLED)
        volatile uint8 TeraTerm_rxAddressMode = TeraTerm_RX_ADDRESS_MODE;
        volatile uint8 TeraTerm_rxAddressDetected = 0u;
    #endif /* (TeraTerm_RXHW_ADDRESS_ENABLED) */
#endif /* (TeraTerm_RX_INTERRUPT_ENABLED && (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)) */


/*******************************************************************************
* Function Name: TeraTerm_Start
********************************************************************************
*
* Summary:
*  This is the preferred method to begin component operation.
*  TeraTerm_Start() sets the initVar variable, calls the
*  TeraTerm_Init() function, and then calls the
*  TeraTerm_Enable() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The TeraTerm_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time TeraTerm_Start() is called. This
*  allows for component initialization without re-initialization in all
*  subsequent calls to the TeraTerm_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void TeraTerm_Start(void) 
{
    /* If not initialized then initialize all required hardware and software */
    if(TeraTerm_initVar == 0u)
    {
        TeraTerm_Init();
        TeraTerm_initVar = 1u;
    }

    TeraTerm_Enable();
}


/*******************************************************************************
* Function Name: TeraTerm_Init
********************************************************************************
*
* Summary:
*  Initializes or restores the component according to the customizer Configure
*  dialog settings. It is not necessary to call TeraTerm_Init() because
*  the TeraTerm_Start() API calls this function and is the preferred
*  method to begin component operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void TeraTerm_Init(void) 
{
    #if(TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)

        #if (TeraTerm_RX_INTERRUPT_ENABLED)
            /* Set RX interrupt vector and priority */
            (void) CyIntSetVector(TeraTerm_RX_VECT_NUM, &TeraTerm_RXISR);
            CyIntSetPriority(TeraTerm_RX_VECT_NUM, TeraTerm_RX_PRIOR_NUM);
            TeraTerm_errorStatus = 0u;
        #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */

        #if (TeraTerm_RXHW_ADDRESS_ENABLED)
            TeraTerm_SetRxAddressMode(TeraTerm_RX_ADDRESS_MODE);
            TeraTerm_SetRxAddress1(TeraTerm_RX_HW_ADDRESS1);
            TeraTerm_SetRxAddress2(TeraTerm_RX_HW_ADDRESS2);
        #endif /* End TeraTerm_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        TeraTerm_RXBITCTR_PERIOD_REG = TeraTerm_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        TeraTerm_RXSTATUS_MASK_REG  = TeraTerm_INIT_RX_INTERRUPTS_MASK;
    #endif /* End TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED*/

    #if(TeraTerm_TX_ENABLED)
        #if (TeraTerm_TX_INTERRUPT_ENABLED)
            /* Set TX interrupt vector and priority */
            (void) CyIntSetVector(TeraTerm_TX_VECT_NUM, &TeraTerm_TXISR);
            CyIntSetPriority(TeraTerm_TX_VECT_NUM, TeraTerm_TX_PRIOR_NUM);
        #endif /* (TeraTerm_TX_INTERRUPT_ENABLED) */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if (TeraTerm_TXCLKGEN_DP)
            TeraTerm_TXBITCLKGEN_CTR_REG = TeraTerm_BIT_CENTER;
            TeraTerm_TXBITCLKTX_COMPLETE_REG = ((TeraTerm_NUMBER_OF_DATA_BITS +
                        TeraTerm_NUMBER_OF_START_BIT) * TeraTerm_OVER_SAMPLE_COUNT) - 1u;
        #else
            TeraTerm_TXBITCTR_PERIOD_REG = ((TeraTerm_NUMBER_OF_DATA_BITS +
                        TeraTerm_NUMBER_OF_START_BIT) * TeraTerm_OVER_SAMPLE_8) - 1u;
        #endif /* End TeraTerm_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if (TeraTerm_TX_INTERRUPT_ENABLED)
            TeraTerm_TXSTATUS_MASK_REG = TeraTerm_TX_STS_FIFO_EMPTY;
        #else
            TeraTerm_TXSTATUS_MASK_REG = TeraTerm_INIT_TX_INTERRUPTS_MASK;
        #endif /*End TeraTerm_TX_INTERRUPT_ENABLED*/

    #endif /* End TeraTerm_TX_ENABLED */

    #if(TeraTerm_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        TeraTerm_WriteControlRegister( \
            (TeraTerm_ReadControlRegister() & (uint8)~TeraTerm_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(TeraTerm_PARITY_TYPE << TeraTerm_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End TeraTerm_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: TeraTerm_Enable
********************************************************************************
*
* Summary:
*  Activates the hardware and begins component operation. It is not necessary
*  to call TeraTerm_Enable() because the TeraTerm_Start() API
*  calls this function, which is the preferred method to begin component
*  operation.

* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  TeraTerm_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void TeraTerm_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)
        /* RX Counter (Count7) Enable */
        TeraTerm_RXBITCTR_CONTROL_REG |= TeraTerm_CNTR_ENABLE;

        /* Enable the RX Interrupt */
        TeraTerm_RXSTATUS_ACTL_REG  |= TeraTerm_INT_ENABLE;

        #if (TeraTerm_RX_INTERRUPT_ENABLED)
            TeraTerm_EnableRxInt();

            #if (TeraTerm_RXHW_ADDRESS_ENABLED)
                TeraTerm_rxAddressDetected = 0u;
            #endif /* (TeraTerm_RXHW_ADDRESS_ENABLED) */
        #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */
    #endif /* (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED) */

    #if(TeraTerm_TX_ENABLED)
        /* TX Counter (DP/Count7) Enable */
        #if(!TeraTerm_TXCLKGEN_DP)
            TeraTerm_TXBITCTR_CONTROL_REG |= TeraTerm_CNTR_ENABLE;
        #endif /* End TeraTerm_TXCLKGEN_DP */

        /* Enable the TX Interrupt */
        TeraTerm_TXSTATUS_ACTL_REG |= TeraTerm_INT_ENABLE;
        #if (TeraTerm_TX_INTERRUPT_ENABLED)
            TeraTerm_ClearPendingTxInt(); /* Clear history of TX_NOT_EMPTY */
            TeraTerm_EnableTxInt();
        #endif /* (TeraTerm_TX_INTERRUPT_ENABLED) */
     #endif /* (TeraTerm_TX_INTERRUPT_ENABLED) */

    #if (TeraTerm_INTERNAL_CLOCK_USED)
        TeraTerm_IntClock_Start();  /* Enable the clock */
    #endif /* (TeraTerm_INTERNAL_CLOCK_USED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TeraTerm_Stop
********************************************************************************
*
* Summary:
*  Disables the UART operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void TeraTerm_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)
        TeraTerm_RXBITCTR_CONTROL_REG &= (uint8) ~TeraTerm_CNTR_ENABLE;
    #endif /* (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED) */

    #if (TeraTerm_TX_ENABLED)
        #if(!TeraTerm_TXCLKGEN_DP)
            TeraTerm_TXBITCTR_CONTROL_REG &= (uint8) ~TeraTerm_CNTR_ENABLE;
        #endif /* (!TeraTerm_TXCLKGEN_DP) */
    #endif /* (TeraTerm_TX_ENABLED) */

    #if (TeraTerm_INTERNAL_CLOCK_USED)
        TeraTerm_IntClock_Stop();   /* Disable the clock */
    #endif /* (TeraTerm_INTERNAL_CLOCK_USED) */

    /* Disable internal interrupt component */
    #if (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)
        TeraTerm_RXSTATUS_ACTL_REG  &= (uint8) ~TeraTerm_INT_ENABLE;

        #if (TeraTerm_RX_INTERRUPT_ENABLED)
            TeraTerm_DisableRxInt();
        #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */
    #endif /* (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED) */

    #if (TeraTerm_TX_ENABLED)
        TeraTerm_TXSTATUS_ACTL_REG &= (uint8) ~TeraTerm_INT_ENABLE;

        #if (TeraTerm_TX_INTERRUPT_ENABLED)
            TeraTerm_DisableTxInt();
        #endif /* (TeraTerm_TX_INTERRUPT_ENABLED) */
    #endif /* (TeraTerm_TX_ENABLED) */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: TeraTerm_ReadControlRegister
********************************************************************************
*
* Summary:
*  Returns the current value of the control register.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the control register.
*
*******************************************************************************/
uint8 TeraTerm_ReadControlRegister(void) 
{
    #if (TeraTerm_CONTROL_REG_REMOVED)
        return(0u);
    #else
        return(TeraTerm_CONTROL_REG);
    #endif /* (TeraTerm_CONTROL_REG_REMOVED) */
}


/*******************************************************************************
* Function Name: TeraTerm_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  TeraTerm_WriteControlRegister(uint8 control) 
{
    #if (TeraTerm_CONTROL_REG_REMOVED)
        if(0u != control)
        {
            /* Suppress compiler warning */
        }
    #else
       TeraTerm_CONTROL_REG = control;
    #endif /* (TeraTerm_CONTROL_REG_REMOVED) */
}


#if(TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)
    /*******************************************************************************
    * Function Name: TeraTerm_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the RX interrupt sources enabled.
    *
    * Parameters:
    *  IntSrc:  Bit field containing the RX interrupts to enable. Based on the 
    *  bit-field arrangement of the status register. This value must be a 
    *  combination of status register bit-masks shown below:
    *      TeraTerm_RX_STS_FIFO_NOTEMPTY    Interrupt on byte received.
    *      TeraTerm_RX_STS_PAR_ERROR        Interrupt on parity error.
    *      TeraTerm_RX_STS_STOP_ERROR       Interrupt on stop error.
    *      TeraTerm_RX_STS_BREAK            Interrupt on break.
    *      TeraTerm_RX_STS_OVERRUN          Interrupt on overrun error.
    *      TeraTerm_RX_STS_ADDR_MATCH       Interrupt on address match.
    *      TeraTerm_RX_STS_MRKSPC           Interrupt on address detect.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void TeraTerm_SetRxInterruptMode(uint8 intSrc) 
    {
        TeraTerm_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: TeraTerm_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns the next byte of received data. This function returns data without
    *  checking the status. You must check the status separately.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  TeraTerm_rxBuffer - RAM buffer pointer for save received data.
    *  TeraTerm_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  TeraTerm_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  TeraTerm_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 TeraTerm_ReadRxData(void) 
    {
        uint8 rxData;

    #if (TeraTerm_RX_INTERRUPT_ENABLED)

        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        TeraTerm_DisableRxInt();

        locRxBufferRead  = TeraTerm_rxBufferRead;
        locRxBufferWrite = TeraTerm_rxBufferWrite;

        if( (TeraTerm_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = TeraTerm_rxBuffer[locRxBufferRead];
            locRxBufferRead++;

            if(locRxBufferRead >= TeraTerm_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            TeraTerm_rxBufferRead = locRxBufferRead;

            if(TeraTerm_rxBufferLoopDetect != 0u)
            {
                TeraTerm_rxBufferLoopDetect = 0u;
                #if ((TeraTerm_RX_INTERRUPT_ENABLED) && (TeraTerm_FLOW_CONTROL != 0u))
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( TeraTerm_HD_ENABLED )
                        if((TeraTerm_CONTROL_REG & TeraTerm_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only in RX
                            *  configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            TeraTerm_RXSTATUS_MASK_REG  |= TeraTerm_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        TeraTerm_RXSTATUS_MASK_REG  |= TeraTerm_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end TeraTerm_HD_ENABLED */
                #endif /* ((TeraTerm_RX_INTERRUPT_ENABLED) && (TeraTerm_FLOW_CONTROL != 0u)) */
            }
        }
        else
        {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
            rxData = TeraTerm_RXDATA_REG;
        }

        TeraTerm_EnableRxInt();

    #else

        /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit */
        rxData = TeraTerm_RXDATA_REG;

    #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: TeraTerm_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Returns the current state of the receiver status register and the software
    *  buffer overflow status.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Side Effect:
    *  All status register bits are clear-on-read except
    *  TeraTerm_RX_STS_FIFO_NOTEMPTY.
    *  TeraTerm_RX_STS_FIFO_NOTEMPTY clears immediately after RX data
    *  register read.
    *
    * Global Variables:
    *  TeraTerm_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn't free space in
    *   TeraTerm_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   TeraTerm_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 TeraTerm_ReadRxStatus(void) 
    {
        uint8 status;

        status = TeraTerm_RXSTATUS_REG & TeraTerm_RX_HW_MASK;

    #if (TeraTerm_RX_INTERRUPT_ENABLED)
        if(TeraTerm_rxBufferOverflow != 0u)
        {
            status |= TeraTerm_RX_STS_SOFT_BUFF_OVER;
            TeraTerm_rxBufferOverflow = 0u;
        }
    #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */

        return(status);
    }


    /*******************************************************************************
    * Function Name: TeraTerm_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Returns the last received byte of data. TeraTerm_GetChar() is
    *  designed for ASCII characters and returns a uint8 where 1 to 255 are values
    *  for valid characters and 0 indicates an error occurred or no data is present.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  TeraTerm_rxBuffer - RAM buffer pointer for save received data.
    *  TeraTerm_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  TeraTerm_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  TeraTerm_rxBufferLoopDetect - cleared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 TeraTerm_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

    #if (TeraTerm_RX_INTERRUPT_ENABLED)
        uint8 locRxBufferRead;
        uint8 locRxBufferWrite;

        /* Protect variables that could change on interrupt */
        TeraTerm_DisableRxInt();

        locRxBufferRead  = TeraTerm_rxBufferRead;
        locRxBufferWrite = TeraTerm_rxBufferWrite;

        if( (TeraTerm_rxBufferLoopDetect != 0u) || (locRxBufferRead != locRxBufferWrite) )
        {
            rxData = TeraTerm_rxBuffer[locRxBufferRead];
            locRxBufferRead++;
            if(locRxBufferRead >= TeraTerm_RX_BUFFER_SIZE)
            {
                locRxBufferRead = 0u;
            }
            /* Update the real pointer */
            TeraTerm_rxBufferRead = locRxBufferRead;

            if(TeraTerm_rxBufferLoopDetect != 0u)
            {
                TeraTerm_rxBufferLoopDetect = 0u;
                #if( (TeraTerm_RX_INTERRUPT_ENABLED) && (TeraTerm_FLOW_CONTROL != 0u) )
                    /* When Hardware Flow Control selected - return RX mask */
                    #if( TeraTerm_HD_ENABLED )
                        if((TeraTerm_CONTROL_REG & TeraTerm_CTRL_HD_SEND) == 0u)
                        {   /* In Half duplex mode return RX mask only if
                            *  RX configuration set, otherwise
                            *  mask will be returned in LoadRxConfig() API.
                            */
                            TeraTerm_RXSTATUS_MASK_REG |= TeraTerm_RX_STS_FIFO_NOTEMPTY;
                        }
                    #else
                        TeraTerm_RXSTATUS_MASK_REG |= TeraTerm_RX_STS_FIFO_NOTEMPTY;
                    #endif /* end TeraTerm_HD_ENABLED */
                #endif /* TeraTerm_RX_INTERRUPT_ENABLED and Hardware flow control*/
            }

        }
        else
        {   rxStatus = TeraTerm_RXSTATUS_REG;
            if((rxStatus & TeraTerm_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO */
                rxData = TeraTerm_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (TeraTerm_RX_STS_BREAK | TeraTerm_RX_STS_PAR_ERROR |
                                TeraTerm_RX_STS_STOP_ERROR | TeraTerm_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        }

        TeraTerm_EnableRxInt();

    #else

        rxStatus =TeraTerm_RXSTATUS_REG;
        if((rxStatus & TeraTerm_RX_STS_FIFO_NOTEMPTY) != 0u)
        {
            /* Read received data from FIFO */
            rxData = TeraTerm_RXDATA_REG;

            /*Check status on error*/
            if((rxStatus & (TeraTerm_RX_STS_BREAK | TeraTerm_RX_STS_PAR_ERROR |
                            TeraTerm_RX_STS_STOP_ERROR | TeraTerm_RX_STS_OVERRUN)) != 0u)
            {
                rxData = 0u;
            }
        }
    #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: TeraTerm_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, returns received character and error
    *  condition.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains status and LSB contains UART RX data. If the MSB is nonzero,
    *  an error has occurred.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 TeraTerm_GetByte(void) 
    {
        
    #if (TeraTerm_RX_INTERRUPT_ENABLED)
        uint16 locErrorStatus;
        /* Protect variables that could change on interrupt */
        TeraTerm_DisableRxInt();
        locErrorStatus = (uint16)TeraTerm_errorStatus;
        TeraTerm_errorStatus = 0u;
        TeraTerm_EnableRxInt();
        return ( (uint16)(locErrorStatus << 8u) | TeraTerm_ReadRxData() );
    #else
        return ( ((uint16)TeraTerm_ReadRxStatus() << 8u) | TeraTerm_ReadRxData() );
    #endif /* TeraTerm_RX_INTERRUPT_ENABLED */
        
    }


    /*******************************************************************************
    * Function Name: TeraTerm_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of received bytes available in the RX buffer.
    *  * RX software buffer is disabled (RX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty RX FIFO or 1 for not empty RX FIFO.
    *  * RX software buffer is enabled: returns the number of bytes available in 
    *    the RX software buffer. Bytes available in the RX FIFO do not take to 
    *    account.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Number of bytes in the RX buffer. 
    *    Return value type depends on RX Buffer Size parameter.
    *
    * Global Variables:
    *  TeraTerm_rxBufferWrite - used to calculate left bytes.
    *  TeraTerm_rxBufferRead - used to calculate left bytes.
    *  TeraTerm_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 TeraTerm_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (TeraTerm_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt */
        TeraTerm_DisableRxInt();

        if(TeraTerm_rxBufferRead == TeraTerm_rxBufferWrite)
        {
            if(TeraTerm_rxBufferLoopDetect != 0u)
            {
                size = TeraTerm_RX_BUFFER_SIZE;
            }
            else
            {
                size = 0u;
            }
        }
        else if(TeraTerm_rxBufferRead < TeraTerm_rxBufferWrite)
        {
            size = (TeraTerm_rxBufferWrite - TeraTerm_rxBufferRead);
        }
        else
        {
            size = (TeraTerm_RX_BUFFER_SIZE - TeraTerm_rxBufferRead) + TeraTerm_rxBufferWrite;
        }

        TeraTerm_EnableRxInt();

    #else

        /* We can only know if there is data in the fifo. */
        size = ((TeraTerm_RXSTATUS_REG & TeraTerm_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

    #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */

        return(size);
    }


    /*******************************************************************************
    * Function Name: TeraTerm_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the receiver memory buffer and hardware RX FIFO of all received data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_rxBufferWrite - cleared to zero.
    *  TeraTerm_rxBufferRead - cleared to zero.
    *  TeraTerm_rxBufferLoopDetect - cleared to zero.
    *  TeraTerm_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *
    *******************************************************************************/
    void TeraTerm_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Clear the HW FIFO */
        enableInterrupts = CyEnterCriticalSection();
        TeraTerm_RXDATA_AUX_CTL_REG |= (uint8)  TeraTerm_RX_FIFO_CLR;
        TeraTerm_RXDATA_AUX_CTL_REG &= (uint8) ~TeraTerm_RX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (TeraTerm_RX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        TeraTerm_DisableRxInt();

        TeraTerm_rxBufferRead = 0u;
        TeraTerm_rxBufferWrite = 0u;
        TeraTerm_rxBufferLoopDetect = 0u;
        TeraTerm_rxBufferOverflow = 0u;

        TeraTerm_EnableRxInt();

    #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */

    }


    /*******************************************************************************
    * Function Name: TeraTerm_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Sets the software controlled Addressing mode used by the RX portion of the
    *  UART.
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  TeraTerm__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  TeraTerm__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  TeraTerm__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  TeraTerm__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  TeraTerm__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  TeraTerm_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void TeraTerm_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(TeraTerm_RXHW_ADDRESS_ENABLED)
            #if(TeraTerm_CONTROL_REG_REMOVED)
                if(0u != addressMode)
                {
                    /* Suppress compiler warning */
                }
            #else /* TeraTerm_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = TeraTerm_CONTROL_REG & (uint8)~TeraTerm_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << TeraTerm_CTRL_RXADDR_MODE0_SHIFT);
                TeraTerm_CONTROL_REG = tmpCtrl;

                #if(TeraTerm_RX_INTERRUPT_ENABLED && \
                   (TeraTerm_RXBUFFERSIZE > TeraTerm_FIFO_LENGTH) )
                    TeraTerm_rxAddressMode = addressMode;
                    TeraTerm_rxAddressDetected = 0u;
                #endif /* End TeraTerm_RXBUFFERSIZE > TeraTerm_FIFO_LENGTH*/
            #endif /* End TeraTerm_CONTROL_REG_REMOVED */
        #else /* TeraTerm_RXHW_ADDRESS_ENABLED */
            if(0u != addressMode)
            {
                /* Suppress compiler warning */
            }
        #endif /* End TeraTerm_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: TeraTerm_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Sets the first of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #1 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void TeraTerm_SetRxAddress1(uint8 address) 
    {
        TeraTerm_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: TeraTerm_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Sets the second of two hardware-detectable receiver addresses.
    *
    * Parameters:
    *  address: Address #2 for hardware address detection.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void TeraTerm_SetRxAddress2(uint8 address) 
    {
        TeraTerm_RXADDRESS2_REG = address;
    }

#endif  /* TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED*/


#if( (TeraTerm_TX_ENABLED) || (TeraTerm_HD_ENABLED) )
    /*******************************************************************************
    * Function Name: TeraTerm_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the TX interrupt sources to be enabled, but does not enable the
    *  interrupt.
    *
    * Parameters:
    *  intSrc: Bit field containing the TX interrupt sources to enable
    *   TeraTerm_TX_STS_COMPLETE        Interrupt on TX byte complete
    *   TeraTerm_TX_STS_FIFO_EMPTY      Interrupt when TX FIFO is empty
    *   TeraTerm_TX_STS_FIFO_FULL       Interrupt when TX FIFO is full
    *   TeraTerm_TX_STS_FIFO_NOT_FULL   Interrupt when TX FIFO is not full
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void TeraTerm_SetTxInterruptMode(uint8 intSrc) 
    {
        TeraTerm_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: TeraTerm_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Places a byte of data into the transmit buffer to be sent when the bus is
    *  available without checking the TX status register. You must check status
    *  separately.
    *
    * Parameters:
    *  txDataByte: data byte
    *
    * Return:
    * None.
    *
    * Global Variables:
    *  TeraTerm_txBuffer - RAM buffer pointer for save data for transmission
    *  TeraTerm_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  TeraTerm_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  TeraTerm_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void TeraTerm_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(TeraTerm_initVar != 0u)
        {
        #if (TeraTerm_TX_INTERRUPT_ENABLED)

            /* Protect variables that could change on interrupt. */
            TeraTerm_DisableTxInt();

            if( (TeraTerm_txBufferRead == TeraTerm_txBufferWrite) &&
                ((TeraTerm_TXSTATUS_REG & TeraTerm_TX_STS_FIFO_FULL) == 0u) )
            {
                /* Add directly to the FIFO. */
                TeraTerm_TXDATA_REG = txDataByte;
            }
            else
            {
                if(TeraTerm_txBufferWrite >= TeraTerm_TX_BUFFER_SIZE)
                {
                    TeraTerm_txBufferWrite = 0u;
                }

                TeraTerm_txBuffer[TeraTerm_txBufferWrite] = txDataByte;

                /* Add to the software buffer. */
                TeraTerm_txBufferWrite++;
            }

            TeraTerm_EnableTxInt();

        #else

            /* Add directly to the FIFO. */
            TeraTerm_TXDATA_REG = txDataByte;

        #endif /*(TeraTerm_TX_INTERRUPT_ENABLED) */
        }
    }


    /*******************************************************************************
    * Function Name: TeraTerm_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Reads the status register for the TX portion of the UART.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the TX status register, which is cleared on read.
    *  It is up to the user to handle all bits in this return value accordingly,
    *  even if the bit was not enabled as an interrupt source the event happened
    *  and must be handled accordingly.
    *
    *******************************************************************************/
    uint8 TeraTerm_ReadTxStatus(void) 
    {
        return(TeraTerm_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: TeraTerm_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Puts a byte of data into the transmit buffer to be sent when the bus is
    *  available. This is a blocking API that waits until the TX buffer has room to
    *  hold the data.
    *
    * Parameters:
    *  txDataByte: Byte containing the data to transmit
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_txBuffer - RAM buffer pointer for save data for transmission
    *  TeraTerm_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  TeraTerm_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  TeraTerm_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void TeraTerm_PutChar(uint8 txDataByte) 
    {
    #if (TeraTerm_TX_INTERRUPT_ENABLED)
        /* The temporary output pointer is used since it takes two instructions
        *  to increment with a wrap, and we can't risk doing that with the real
        *  pointer and getting an interrupt in between instructions.
        */
        uint8 locTxBufferWrite;
        uint8 locTxBufferRead;

        do
        { /* Block if software buffer is full, so we don't overwrite. */

        #if ((TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Disable TX interrupt to protect variables from modification */
            TeraTerm_DisableTxInt();
        #endif /* (TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3) */

            locTxBufferWrite = TeraTerm_txBufferWrite;
            locTxBufferRead  = TeraTerm_txBufferRead;

        #if ((TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3))
            /* Enable interrupt to continue transmission */
            TeraTerm_EnableTxInt();
        #endif /* (TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3) */
        }
        while( (locTxBufferWrite < locTxBufferRead) ? (locTxBufferWrite == (locTxBufferRead - 1u)) :
                                ((locTxBufferWrite - locTxBufferRead) ==
                                (uint8)(TeraTerm_TX_BUFFER_SIZE - 1u)) );

        if( (locTxBufferRead == locTxBufferWrite) &&
            ((TeraTerm_TXSTATUS_REG & TeraTerm_TX_STS_FIFO_FULL) == 0u) )
        {
            /* Add directly to the FIFO */
            TeraTerm_TXDATA_REG = txDataByte;
        }
        else
        {
            if(locTxBufferWrite >= TeraTerm_TX_BUFFER_SIZE)
            {
                locTxBufferWrite = 0u;
            }
            /* Add to the software buffer. */
            TeraTerm_txBuffer[locTxBufferWrite] = txDataByte;
            locTxBufferWrite++;

            /* Finally, update the real output pointer */
        #if ((TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3))
            TeraTerm_DisableTxInt();
        #endif /* (TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3) */

            TeraTerm_txBufferWrite = locTxBufferWrite;

        #if ((TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3))
            TeraTerm_EnableTxInt();
        #endif /* (TeraTerm_TX_BUFFER_SIZE > TeraTerm_MAX_BYTE_VALUE) && (CY_PSOC3) */

            if(0u != (TeraTerm_TXSTATUS_REG & TeraTerm_TX_STS_FIFO_EMPTY))
            {
                /* Trigger TX interrupt to send software buffer */
                TeraTerm_SetPendingTxInt();
            }
        }

    #else

        while((TeraTerm_TXSTATUS_REG & TeraTerm_TX_STS_FIFO_FULL) != 0u)
        {
            /* Wait for room in the FIFO */
        }

        /* Add directly to the FIFO */
        TeraTerm_TXDATA_REG = txDataByte;

    #endif /* TeraTerm_TX_INTERRUPT_ENABLED */
    }


    /*******************************************************************************
    * Function Name: TeraTerm_PutString
    ********************************************************************************
    *
    * Summary:
    *  Sends a NULL terminated string to the TX buffer for transmission.
    *
    * Parameters:
    *  string[]: Pointer to the null terminated string array residing in RAM or ROM
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void TeraTerm_PutString(const char8 string[]) 
    {
        uint16 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(TeraTerm_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent */
            while(string[bufIndex] != (char8) 0)
            {
                TeraTerm_PutChar((uint8)string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: TeraTerm_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Places N bytes of data from a memory array into the TX buffer for
    *  transmission.
    *
    * Parameters:
    *  string[]: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of bytes to be transmitted. The type depends on TX Buffer
    *             Size parameter.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  If there is not enough memory in the TX buffer for the entire string, this
    *  function blocks until the last character of the string is loaded into the
    *  TX buffer.
    *
    *******************************************************************************/
    void TeraTerm_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 bufIndex = 0u;

        /* If not Initialized then skip this function */
        if(TeraTerm_initVar != 0u)
        {
            while(bufIndex < byteCount)
            {
                TeraTerm_PutChar(string[bufIndex]);
                bufIndex++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: TeraTerm_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Writes a byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) to the transmit buffer.
    *
    * Parameters:
    *  txDataByte: Data byte to transmit before the carriage return and line feed.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void TeraTerm_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function */
        if(TeraTerm_initVar != 0u)
        {
            TeraTerm_PutChar(txDataByte);
            TeraTerm_PutChar(0x0Du);
            TeraTerm_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: TeraTerm_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Returns the number of bytes in the TX buffer which are waiting to be 
    *  transmitted.
    *  * TX software buffer is disabled (TX Buffer Size parameter is equal to 4): 
    *    returns 0 for empty TX FIFO, 1 for not full TX FIFO or 4 for full TX FIFO.
    *  * TX software buffer is enabled: returns the number of bytes in the TX 
    *    software buffer which are waiting to be transmitted. Bytes available in the
    *    TX FIFO do not count.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Number of bytes used in the TX buffer. Return value type depends on the TX 
    *  Buffer Size parameter.
    *
    * Global Variables:
    *  TeraTerm_txBufferWrite - used to calculate left space.
    *  TeraTerm_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 TeraTerm_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

    #if (TeraTerm_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        TeraTerm_DisableTxInt();

        if(TeraTerm_txBufferRead == TeraTerm_txBufferWrite)
        {
            size = 0u;
        }
        else if(TeraTerm_txBufferRead < TeraTerm_txBufferWrite)
        {
            size = (TeraTerm_txBufferWrite - TeraTerm_txBufferRead);
        }
        else
        {
            size = (TeraTerm_TX_BUFFER_SIZE - TeraTerm_txBufferRead) +
                    TeraTerm_txBufferWrite;
        }

        TeraTerm_EnableTxInt();

    #else

        size = TeraTerm_TXSTATUS_REG;

        /* Is the fifo is full. */
        if((size & TeraTerm_TX_STS_FIFO_FULL) != 0u)
        {
            size = TeraTerm_FIFO_LENGTH;
        }
        else if((size & TeraTerm_TX_STS_FIFO_EMPTY) != 0u)
        {
            size = 0u;
        }
        else
        {
            /* We only know there is data in the fifo. */
            size = 1u;
        }

    #endif /* (TeraTerm_TX_INTERRUPT_ENABLED) */

    return(size);
    }


    /*******************************************************************************
    * Function Name: TeraTerm_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears all data from the TX buffer and hardware TX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_txBufferWrite - cleared to zero.
    *  TeraTerm_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Data waiting in the transmit buffer is not sent; a byte that is currently
    *  transmitting finishes transmitting.
    *
    *******************************************************************************/
    void TeraTerm_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        enableInterrupts = CyEnterCriticalSection();
        /* Clear the HW FIFO */
        TeraTerm_TXDATA_AUX_CTL_REG |= (uint8)  TeraTerm_TX_FIFO_CLR;
        TeraTerm_TXDATA_AUX_CTL_REG &= (uint8) ~TeraTerm_TX_FIFO_CLR;
        CyExitCriticalSection(enableInterrupts);

    #if (TeraTerm_TX_INTERRUPT_ENABLED)

        /* Protect variables that could change on interrupt. */
        TeraTerm_DisableTxInt();

        TeraTerm_txBufferRead = 0u;
        TeraTerm_txBufferWrite = 0u;

        /* Enable Tx interrupt. */
        TeraTerm_EnableTxInt();

    #endif /* (TeraTerm_TX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: TeraTerm_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Transmits a break signal on the bus.
    *
    * Parameters:
    *  uint8 retMode:  Send Break return mode. See the following table for options.
    *   TeraTerm_SEND_BREAK - Initialize registers for break, send the Break
    *       signal and return immediately.
    *   TeraTerm_WAIT_FOR_COMPLETE_REINIT - Wait until break transmission is
    *       complete, reinitialize registers to normal transmission mode then return
    *   TeraTerm_REINIT - Reinitialize registers to normal transmission mode
    *       then return.
    *   TeraTerm_SEND_WAIT_REINIT - Performs both options: 
    *      TeraTerm_SEND_BREAK and TeraTerm_WAIT_FOR_COMPLETE_REINIT.
    *      This option is recommended for most cases.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  TeraTerm_initVar - checked to identify that the component has been
    *     initialized.
    *  txPeriod - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  There are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Function will block CPU until transmission
    *     complete.
    *  2) User may want to use blocking time if UART configured to the low speed
    *     operation
    *     Example for this case:
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to initialize and use the interrupt to
    *     complete break operation.
    *     Example for this case:
    *     Initialize TX interrupt with "TX - On TX Complete" parameter
    *     SendBreak(0);     - initialize Break signal transmission
    *         Add your code here to use CPU time
    *     When interrupt appear with TeraTerm_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *  The TeraTerm_SendBreak() function initializes registers to send a
    *  break signal.
    *  Break signal length depends on the break signal bits configuration.
    *  The register configuration should be reinitialized before normal 8-bit
    *  communication can continue.
    *
    *******************************************************************************/
    void TeraTerm_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(TeraTerm_initVar != 0u)
        {
            /* Set the Counter to 13-bits and transmit a 00 byte */
            /* When that is done then reset the counter value back */
            uint8 tmpStat;

        #if(TeraTerm_HD_ENABLED) /* Half Duplex mode*/

            if( (retMode == TeraTerm_SEND_BREAK) ||
                (retMode == TeraTerm_SEND_WAIT_REINIT ) )
            {
                /* CTRL_HD_SEND_BREAK - sends break bits in HD mode */
                TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() |
                                                      TeraTerm_CTRL_HD_SEND_BREAK);
                /* Send zeros */
                TeraTerm_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = TeraTerm_TXSTATUS_REG;
                }
                while((tmpStat & TeraTerm_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == TeraTerm_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == TeraTerm_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = TeraTerm_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & TeraTerm_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == TeraTerm_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == TeraTerm_REINIT) ||
                (retMode == TeraTerm_SEND_WAIT_REINIT) )
            {
                TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() &
                                              (uint8)~TeraTerm_CTRL_HD_SEND_BREAK);
            }

        #else /* TeraTerm_HD_ENABLED Full Duplex mode */

            static uint8 txPeriod;

            if( (retMode == TeraTerm_SEND_BREAK) ||
                (retMode == TeraTerm_SEND_WAIT_REINIT) )
            {
                /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode */
                #if( (TeraTerm_PARITY_TYPE != TeraTerm__B_UART__NONE_REVB) || \
                                    (TeraTerm_PARITY_TYPE_SW != 0u) )
                    TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() |
                                                          TeraTerm_CTRL_HD_SEND_BREAK);
                #endif /* End TeraTerm_PARITY_TYPE != TeraTerm__B_UART__NONE_REVB  */

                #if(TeraTerm_TXCLKGEN_DP)
                    txPeriod = TeraTerm_TXBITCLKTX_COMPLETE_REG;
                    TeraTerm_TXBITCLKTX_COMPLETE_REG = TeraTerm_TXBITCTR_BREAKBITS;
                #else
                    txPeriod = TeraTerm_TXBITCTR_PERIOD_REG;
                    TeraTerm_TXBITCTR_PERIOD_REG = TeraTerm_TXBITCTR_BREAKBITS8X;
                #endif /* End TeraTerm_TXCLKGEN_DP */

                /* Send zeros */
                TeraTerm_TXDATA_REG = 0u;

                do /* Wait until transmit starts */
                {
                    tmpStat = TeraTerm_TXSTATUS_REG;
                }
                while((tmpStat & TeraTerm_TX_STS_FIFO_EMPTY) != 0u);
            }

            if( (retMode == TeraTerm_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == TeraTerm_SEND_WAIT_REINIT) )
            {
                do /* Wait until transmit complete */
                {
                    tmpStat = TeraTerm_TXSTATUS_REG;
                }
                while(((uint8)~tmpStat & TeraTerm_TX_STS_COMPLETE) != 0u);
            }

            if( (retMode == TeraTerm_WAIT_FOR_COMPLETE_REINIT) ||
                (retMode == TeraTerm_REINIT) ||
                (retMode == TeraTerm_SEND_WAIT_REINIT) )
            {

            #if(TeraTerm_TXCLKGEN_DP)
                TeraTerm_TXBITCLKTX_COMPLETE_REG = txPeriod;
            #else
                TeraTerm_TXBITCTR_PERIOD_REG = txPeriod;
            #endif /* End TeraTerm_TXCLKGEN_DP */

            #if( (TeraTerm_PARITY_TYPE != TeraTerm__B_UART__NONE_REVB) || \
                 (TeraTerm_PARITY_TYPE_SW != 0u) )
                TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() &
                                                      (uint8) ~TeraTerm_CTRL_HD_SEND_BREAK);
            #endif /* End TeraTerm_PARITY_TYPE != NONE */
            }
        #endif    /* End TeraTerm_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: TeraTerm_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Configures the transmitter to signal the next bytes is address or data.
    *
    * Parameters:
    *  addressMode: 
    *       TeraTerm_SET_SPACE - Configure the transmitter to send the next
    *                                    byte as a data.
    *       TeraTerm_SET_MARK  - Configure the transmitter to send the next
    *                                    byte as an address.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  This function sets and clears TeraTerm_CTRL_MARK bit in the Control
    *  register.
    *
    *******************************************************************************/
    void TeraTerm_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable */
        if(addressMode != 0u)
        {
        #if( TeraTerm_CONTROL_REG_REMOVED == 0u )
            TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() |
                                                  TeraTerm_CTRL_MARK);
        #endif /* End TeraTerm_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
        #if( TeraTerm_CONTROL_REG_REMOVED == 0u )
            TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() &
                                                  (uint8) ~TeraTerm_CTRL_MARK);
        #endif /* End TeraTerm_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndTeraTerm_TX_ENABLED */

#if(TeraTerm_HD_ENABLED)


    /*******************************************************************************
    * Function Name: TeraTerm_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the receiver configuration in half duplex mode. After calling this
    *  function, the UART is ready to receive data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the transmitter
    *  configuration.
    *
    *******************************************************************************/
    void TeraTerm_LoadRxConfig(void) 
    {
        TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() &
                                                (uint8)~TeraTerm_CTRL_HD_SEND);
        TeraTerm_RXBITCTR_PERIOD_REG = TeraTerm_HD_RXBITCTR_INIT;

    #if (TeraTerm_RX_INTERRUPT_ENABLED)
        /* Enable RX interrupt after set RX configuration */
        TeraTerm_SetRxInterruptMode(TeraTerm_INIT_RX_INTERRUPTS_MASK);
    #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */
    }


    /*******************************************************************************
    * Function Name: TeraTerm_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Loads the transmitter configuration in half duplex mode. After calling this
    *  function, the UART is ready to transmit data.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Side Effects:
    *  Valid only in half duplex mode. You must make sure that the previous
    *  transaction is complete and it is safe to unload the receiver configuration.
    *
    *******************************************************************************/
    void TeraTerm_LoadTxConfig(void) 
    {
    #if (TeraTerm_RX_INTERRUPT_ENABLED)
        /* Disable RX interrupts before set TX configuration */
        TeraTerm_SetRxInterruptMode(0u);
    #endif /* (TeraTerm_RX_INTERRUPT_ENABLED) */

        TeraTerm_WriteControlRegister(TeraTerm_ReadControlRegister() | TeraTerm_CTRL_HD_SEND);
        TeraTerm_RXBITCTR_PERIOD_REG = TeraTerm_HD_TXBITCTR_INIT;
    }

#endif  /* TeraTerm_HD_ENABLED */


/* [] END OF FILE */
