/*******************************************************************************
* File Name: TeraTerm.h
* Version 2.50
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_TeraTerm_H)
#define CY_UART_TeraTerm_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */


/***************************************
* Conditional Compilation Parameters
***************************************/

#define TeraTerm_RX_ENABLED                     (0u)
#define TeraTerm_TX_ENABLED                     (1u)
#define TeraTerm_HD_ENABLED                     (0u)
#define TeraTerm_RX_INTERRUPT_ENABLED           (0u)
#define TeraTerm_TX_INTERRUPT_ENABLED           (0u)
#define TeraTerm_INTERNAL_CLOCK_USED            (1u)
#define TeraTerm_RXHW_ADDRESS_ENABLED           (0u)
#define TeraTerm_OVER_SAMPLE_COUNT              (8u)
#define TeraTerm_PARITY_TYPE                    (0u)
#define TeraTerm_PARITY_TYPE_SW                 (0u)
#define TeraTerm_BREAK_DETECT                   (0u)
#define TeraTerm_BREAK_BITS_TX                  (13u)
#define TeraTerm_BREAK_BITS_RX                  (13u)
#define TeraTerm_TXCLKGEN_DP                    (1u)
#define TeraTerm_USE23POLLING                   (1u)
#define TeraTerm_FLOW_CONTROL                   (0u)
#define TeraTerm_CLK_FREQ                       (0u)
#define TeraTerm_TX_BUFFER_SIZE                 (4u)
#define TeraTerm_RX_BUFFER_SIZE                 (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_50 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#if defined(TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG)
    #define TeraTerm_CONTROL_REG_REMOVED            (0u)
#else
    #define TeraTerm_CONTROL_REG_REMOVED            (1u)
#endif /* End TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Structure Definition
***************************************/

/* Sleep Mode API Support */
typedef struct TeraTerm_backupStruct_
{
    uint8 enableState;

    #if(TeraTerm_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End TeraTerm_CONTROL_REG_REMOVED */

} TeraTerm_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void TeraTerm_Start(void) ;
void TeraTerm_Stop(void) ;
uint8 TeraTerm_ReadControlRegister(void) ;
void TeraTerm_WriteControlRegister(uint8 control) ;

void TeraTerm_Init(void) ;
void TeraTerm_Enable(void) ;
void TeraTerm_SaveConfig(void) ;
void TeraTerm_RestoreConfig(void) ;
void TeraTerm_Sleep(void) ;
void TeraTerm_Wakeup(void) ;

/* Only if RX is enabled */
#if( (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) )

    #if (TeraTerm_RX_INTERRUPT_ENABLED)
        #define TeraTerm_EnableRxInt()  CyIntEnable (TeraTerm_RX_VECT_NUM)
        #define TeraTerm_DisableRxInt() CyIntDisable(TeraTerm_RX_VECT_NUM)
        CY_ISR_PROTO(TeraTerm_RXISR);
    #endif /* TeraTerm_RX_INTERRUPT_ENABLED */

    void TeraTerm_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void TeraTerm_SetRxAddress1(uint8 address) ;
    void TeraTerm_SetRxAddress2(uint8 address) ;

    void  TeraTerm_SetRxInterruptMode(uint8 intSrc) ;
    uint8 TeraTerm_ReadRxData(void) ;
    uint8 TeraTerm_ReadRxStatus(void) ;
    uint8 TeraTerm_GetChar(void) ;
    uint16 TeraTerm_GetByte(void) ;
    uint8 TeraTerm_GetRxBufferSize(void)
                                                            ;
    void TeraTerm_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define TeraTerm_GetRxInterruptSource   TeraTerm_ReadRxStatus

#endif /* End (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) */

/* Only if TX is enabled */
#if(TeraTerm_TX_ENABLED || TeraTerm_HD_ENABLED)

    #if(TeraTerm_TX_INTERRUPT_ENABLED)
        #define TeraTerm_EnableTxInt()  CyIntEnable (TeraTerm_TX_VECT_NUM)
        #define TeraTerm_DisableTxInt() CyIntDisable(TeraTerm_TX_VECT_NUM)
        #define TeraTerm_SetPendingTxInt() CyIntSetPending(TeraTerm_TX_VECT_NUM)
        #define TeraTerm_ClearPendingTxInt() CyIntClearPending(TeraTerm_TX_VECT_NUM)
        CY_ISR_PROTO(TeraTerm_TXISR);
    #endif /* TeraTerm_TX_INTERRUPT_ENABLED */

    void TeraTerm_SetTxInterruptMode(uint8 intSrc) ;
    void TeraTerm_WriteTxData(uint8 txDataByte) ;
    uint8 TeraTerm_ReadTxStatus(void) ;
    void TeraTerm_PutChar(uint8 txDataByte) ;
    void TeraTerm_PutString(const char8 string[]) ;
    void TeraTerm_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void TeraTerm_PutCRLF(uint8 txDataByte) ;
    void TeraTerm_ClearTxBuffer(void) ;
    void TeraTerm_SetTxAddressMode(uint8 addressMode) ;
    void TeraTerm_SendBreak(uint8 retMode) ;
    uint8 TeraTerm_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define TeraTerm_PutStringConst         TeraTerm_PutString
    #define TeraTerm_PutArrayConst          TeraTerm_PutArray
    #define TeraTerm_GetTxInterruptSource   TeraTerm_ReadTxStatus

#endif /* End TeraTerm_TX_ENABLED || TeraTerm_HD_ENABLED */

#if(TeraTerm_HD_ENABLED)
    void TeraTerm_LoadRxConfig(void) ;
    void TeraTerm_LoadTxConfig(void) ;
#endif /* End TeraTerm_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_TeraTerm) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    TeraTerm_CyBtldrCommStart(void) CYSMALL ;
    void    TeraTerm_CyBtldrCommStop(void) CYSMALL ;
    void    TeraTerm_CyBtldrCommReset(void) CYSMALL ;
    cystatus TeraTerm_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus TeraTerm_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_TeraTerm)
        #define CyBtldrCommStart    TeraTerm_CyBtldrCommStart
        #define CyBtldrCommStop     TeraTerm_CyBtldrCommStop
        #define CyBtldrCommReset    TeraTerm_CyBtldrCommReset
        #define CyBtldrCommWrite    TeraTerm_CyBtldrCommWrite
        #define CyBtldrCommRead     TeraTerm_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_TeraTerm) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define TeraTerm_BYTE2BYTE_TIME_OUT (25u)
    #define TeraTerm_PACKET_EOP         (0x17u) /* End of packet defined by bootloader */
    #define TeraTerm_WAIT_EOP_DELAY     (5u)    /* Additional 5ms to wait for End of packet */
    #define TeraTerm_BL_CHK_DELAY_MS    (1u)    /* Time Out quantity equal 1mS */

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define TeraTerm_SET_SPACE      (0x00u)
#define TeraTerm_SET_MARK       (0x01u)

/* Status Register definitions */
#if( (TeraTerm_TX_ENABLED) || (TeraTerm_HD_ENABLED) )
    #if(TeraTerm_TX_INTERRUPT_ENABLED)
        #define TeraTerm_TX_VECT_NUM            (uint8)TeraTerm_TXInternalInterrupt__INTC_NUMBER
        #define TeraTerm_TX_PRIOR_NUM           (uint8)TeraTerm_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* TeraTerm_TX_INTERRUPT_ENABLED */

    #define TeraTerm_TX_STS_COMPLETE_SHIFT          (0x00u)
    #define TeraTerm_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
    #define TeraTerm_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #if(TeraTerm_TX_ENABLED)
        #define TeraTerm_TX_STS_FIFO_FULL_SHIFT     (0x02u)
    #else /* (TeraTerm_HD_ENABLED) */
        #define TeraTerm_TX_STS_FIFO_FULL_SHIFT     (0x05u)  /* Needs MD=0 */
    #endif /* (TeraTerm_TX_ENABLED) */

    #define TeraTerm_TX_STS_COMPLETE            (uint8)(0x01u << TeraTerm_TX_STS_COMPLETE_SHIFT)
    #define TeraTerm_TX_STS_FIFO_EMPTY          (uint8)(0x01u << TeraTerm_TX_STS_FIFO_EMPTY_SHIFT)
    #define TeraTerm_TX_STS_FIFO_FULL           (uint8)(0x01u << TeraTerm_TX_STS_FIFO_FULL_SHIFT)
    #define TeraTerm_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << TeraTerm_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (TeraTerm_TX_ENABLED) || (TeraTerm_HD_ENABLED)*/

#if( (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) )
    #if(TeraTerm_RX_INTERRUPT_ENABLED)
        #define TeraTerm_RX_VECT_NUM            (uint8)TeraTerm_RXInternalInterrupt__INTC_NUMBER
        #define TeraTerm_RX_PRIOR_NUM           (uint8)TeraTerm_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* TeraTerm_RX_INTERRUPT_ENABLED */
    #define TeraTerm_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define TeraTerm_RX_STS_BREAK_SHIFT             (0x01u)
    #define TeraTerm_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define TeraTerm_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define TeraTerm_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define TeraTerm_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define TeraTerm_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define TeraTerm_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define TeraTerm_RX_STS_MRKSPC           (uint8)(0x01u << TeraTerm_RX_STS_MRKSPC_SHIFT)
    #define TeraTerm_RX_STS_BREAK            (uint8)(0x01u << TeraTerm_RX_STS_BREAK_SHIFT)
    #define TeraTerm_RX_STS_PAR_ERROR        (uint8)(0x01u << TeraTerm_RX_STS_PAR_ERROR_SHIFT)
    #define TeraTerm_RX_STS_STOP_ERROR       (uint8)(0x01u << TeraTerm_RX_STS_STOP_ERROR_SHIFT)
    #define TeraTerm_RX_STS_OVERRUN          (uint8)(0x01u << TeraTerm_RX_STS_OVERRUN_SHIFT)
    #define TeraTerm_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << TeraTerm_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define TeraTerm_RX_STS_ADDR_MATCH       (uint8)(0x01u << TeraTerm_RX_STS_ADDR_MATCH_SHIFT)
    #define TeraTerm_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << TeraTerm_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define TeraTerm_RX_HW_MASK                     (0x7Fu)
#endif /* End (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) */

/* Control Register definitions */
#define TeraTerm_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define TeraTerm_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define TeraTerm_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define TeraTerm_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define TeraTerm_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define TeraTerm_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define TeraTerm_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define TeraTerm_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define TeraTerm_CTRL_HD_SEND               (uint8)(0x01u << TeraTerm_CTRL_HD_SEND_SHIFT)
#define TeraTerm_CTRL_HD_SEND_BREAK         (uint8)(0x01u << TeraTerm_CTRL_HD_SEND_BREAK_SHIFT)
#define TeraTerm_CTRL_MARK                  (uint8)(0x01u << TeraTerm_CTRL_MARK_SHIFT)
#define TeraTerm_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << TeraTerm_CTRL_PARITY_TYPE0_SHIFT)
#define TeraTerm_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << TeraTerm_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define TeraTerm_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define TeraTerm_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define TeraTerm_SEND_BREAK                         (0x00u)
#define TeraTerm_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define TeraTerm_REINIT                             (0x02u)
#define TeraTerm_SEND_WAIT_REINIT                   (0x03u)

#define TeraTerm_OVER_SAMPLE_8                      (8u)
#define TeraTerm_OVER_SAMPLE_16                     (16u)

#define TeraTerm_BIT_CENTER                         (TeraTerm_OVER_SAMPLE_COUNT - 2u)

#define TeraTerm_FIFO_LENGTH                        (4u)
#define TeraTerm_NUMBER_OF_START_BIT                (1u)
#define TeraTerm_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation */
#define TeraTerm_TXBITCTR_BREAKBITS8X   ((TeraTerm_BREAK_BITS_TX * TeraTerm_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation */
#define TeraTerm_TXBITCTR_BREAKBITS ((TeraTerm_BREAK_BITS_TX * TeraTerm_OVER_SAMPLE_COUNT) - 1u)

#define TeraTerm_HALF_BIT_COUNT   \
                            (((TeraTerm_OVER_SAMPLE_COUNT / 2u) + (TeraTerm_USE23POLLING * 1u)) - 2u)
#if (TeraTerm_OVER_SAMPLE_COUNT == TeraTerm_OVER_SAMPLE_8)
    #define TeraTerm_HD_TXBITCTR_INIT   (((TeraTerm_BREAK_BITS_TX + \
                            TeraTerm_NUMBER_OF_START_BIT) * TeraTerm_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define TeraTerm_RXBITCTR_INIT  ((((TeraTerm_BREAK_BITS_RX + TeraTerm_NUMBER_OF_START_BIT) \
                            * TeraTerm_OVER_SAMPLE_COUNT) + TeraTerm_HALF_BIT_COUNT) - 1u)

#else /* TeraTerm_OVER_SAMPLE_COUNT == TeraTerm_OVER_SAMPLE_16 */
    #define TeraTerm_HD_TXBITCTR_INIT   ((8u * TeraTerm_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount = 16 */
    #define TeraTerm_RXBITCTR_INIT      (((7u * TeraTerm_OVER_SAMPLE_COUNT) - 1u) + \
                                                      TeraTerm_HALF_BIT_COUNT)
#endif /* End TeraTerm_OVER_SAMPLE_COUNT */

#define TeraTerm_HD_RXBITCTR_INIT                   TeraTerm_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 TeraTerm_initVar;
#if (TeraTerm_TX_INTERRUPT_ENABLED && TeraTerm_TX_ENABLED)
    extern volatile uint8 TeraTerm_txBuffer[TeraTerm_TX_BUFFER_SIZE];
    extern volatile uint8 TeraTerm_txBufferRead;
    extern uint8 TeraTerm_txBufferWrite;
#endif /* (TeraTerm_TX_INTERRUPT_ENABLED && TeraTerm_TX_ENABLED) */
#if (TeraTerm_RX_INTERRUPT_ENABLED && (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED))
    extern uint8 TeraTerm_errorStatus;
    extern volatile uint8 TeraTerm_rxBuffer[TeraTerm_RX_BUFFER_SIZE];
    extern volatile uint8 TeraTerm_rxBufferRead;
    extern volatile uint8 TeraTerm_rxBufferWrite;
    extern volatile uint8 TeraTerm_rxBufferLoopDetect;
    extern volatile uint8 TeraTerm_rxBufferOverflow;
    #if (TeraTerm_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 TeraTerm_rxAddressMode;
        extern volatile uint8 TeraTerm_rxAddressDetected;
    #endif /* (TeraTerm_RXHW_ADDRESS_ENABLED) */
#endif /* (TeraTerm_RX_INTERRUPT_ENABLED && (TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)) */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define TeraTerm__B_UART__AM_SW_BYTE_BYTE 1
#define TeraTerm__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define TeraTerm__B_UART__AM_HW_BYTE_BY_BYTE 3
#define TeraTerm__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define TeraTerm__B_UART__AM_NONE 0

#define TeraTerm__B_UART__NONE_REVB 0
#define TeraTerm__B_UART__EVEN_REVB 1
#define TeraTerm__B_UART__ODD_REVB 2
#define TeraTerm__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define TeraTerm_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define TeraTerm_NUMBER_OF_STOP_BITS    (1u)

#if (TeraTerm_RXHW_ADDRESS_ENABLED)
    #define TeraTerm_RX_ADDRESS_MODE    (0u)
    #define TeraTerm_RX_HW_ADDRESS1     (0u)
    #define TeraTerm_RX_HW_ADDRESS2     (0u)
#endif /* (TeraTerm_RXHW_ADDRESS_ENABLED) */

#define TeraTerm_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((0 << TeraTerm_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << TeraTerm_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << TeraTerm_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << TeraTerm_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << TeraTerm_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << TeraTerm_RX_STS_BREAK_SHIFT) \
                                        | (0 << TeraTerm_RX_STS_OVERRUN_SHIFT))

#define TeraTerm_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << TeraTerm_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << TeraTerm_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << TeraTerm_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << TeraTerm_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define TeraTerm_CONTROL_REG \
                            (* (reg8 *) TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define TeraTerm_CONTROL_PTR \
                            (  (reg8 *) TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(TeraTerm_TX_ENABLED)
    #define TeraTerm_TXDATA_REG          (* (reg8 *) TeraTerm_BUART_sTX_TxShifter_u0__F0_REG)
    #define TeraTerm_TXDATA_PTR          (  (reg8 *) TeraTerm_BUART_sTX_TxShifter_u0__F0_REG)
    #define TeraTerm_TXDATA_AUX_CTL_REG  (* (reg8 *) TeraTerm_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define TeraTerm_TXDATA_AUX_CTL_PTR  (  (reg8 *) TeraTerm_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define TeraTerm_TXSTATUS_REG        (* (reg8 *) TeraTerm_BUART_sTX_TxSts__STATUS_REG)
    #define TeraTerm_TXSTATUS_PTR        (  (reg8 *) TeraTerm_BUART_sTX_TxSts__STATUS_REG)
    #define TeraTerm_TXSTATUS_MASK_REG   (* (reg8 *) TeraTerm_BUART_sTX_TxSts__MASK_REG)
    #define TeraTerm_TXSTATUS_MASK_PTR   (  (reg8 *) TeraTerm_BUART_sTX_TxSts__MASK_REG)
    #define TeraTerm_TXSTATUS_ACTL_REG   (* (reg8 *) TeraTerm_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define TeraTerm_TXSTATUS_ACTL_PTR   (  (reg8 *) TeraTerm_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(TeraTerm_TXCLKGEN_DP)
        #define TeraTerm_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define TeraTerm_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define TeraTerm_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define TeraTerm_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define TeraTerm_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define TeraTerm_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define TeraTerm_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define TeraTerm_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define TeraTerm_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define TeraTerm_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) TeraTerm_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* TeraTerm_TXCLKGEN_DP */

#endif /* End TeraTerm_TX_ENABLED */

#if(TeraTerm_HD_ENABLED)

    #define TeraTerm_TXDATA_REG             (* (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__F1_REG )
    #define TeraTerm_TXDATA_PTR             (  (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__F1_REG )
    #define TeraTerm_TXDATA_AUX_CTL_REG     (* (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define TeraTerm_TXDATA_AUX_CTL_PTR     (  (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define TeraTerm_TXSTATUS_REG           (* (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_REG )
    #define TeraTerm_TXSTATUS_PTR           (  (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_REG )
    #define TeraTerm_TXSTATUS_MASK_REG      (* (reg8 *) TeraTerm_BUART_sRX_RxSts__MASK_REG )
    #define TeraTerm_TXSTATUS_MASK_PTR      (  (reg8 *) TeraTerm_BUART_sRX_RxSts__MASK_REG )
    #define TeraTerm_TXSTATUS_ACTL_REG      (* (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define TeraTerm_TXSTATUS_ACTL_PTR      (  (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End TeraTerm_HD_ENABLED */

#if( (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) )
    #define TeraTerm_RXDATA_REG             (* (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__F0_REG )
    #define TeraTerm_RXDATA_PTR             (  (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__F0_REG )
    #define TeraTerm_RXADDRESS1_REG         (* (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__D0_REG )
    #define TeraTerm_RXADDRESS1_PTR         (  (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__D0_REG )
    #define TeraTerm_RXADDRESS2_REG         (* (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__D1_REG )
    #define TeraTerm_RXADDRESS2_PTR         (  (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__D1_REG )
    #define TeraTerm_RXDATA_AUX_CTL_REG     (* (reg8 *) TeraTerm_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define TeraTerm_RXBITCTR_PERIOD_REG    (* (reg8 *) TeraTerm_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define TeraTerm_RXBITCTR_PERIOD_PTR    (  (reg8 *) TeraTerm_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define TeraTerm_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) TeraTerm_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define TeraTerm_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) TeraTerm_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define TeraTerm_RXBITCTR_COUNTER_REG   (* (reg8 *) TeraTerm_BUART_sRX_RxBitCounter__COUNT_REG )
    #define TeraTerm_RXBITCTR_COUNTER_PTR   (  (reg8 *) TeraTerm_BUART_sRX_RxBitCounter__COUNT_REG )

    #define TeraTerm_RXSTATUS_REG           (* (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_REG )
    #define TeraTerm_RXSTATUS_PTR           (  (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_REG )
    #define TeraTerm_RXSTATUS_MASK_REG      (* (reg8 *) TeraTerm_BUART_sRX_RxSts__MASK_REG )
    #define TeraTerm_RXSTATUS_MASK_PTR      (  (reg8 *) TeraTerm_BUART_sRX_RxSts__MASK_REG )
    #define TeraTerm_RXSTATUS_ACTL_REG      (* (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define TeraTerm_RXSTATUS_ACTL_PTR      (  (reg8 *) TeraTerm_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) */

#if(TeraTerm_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define TeraTerm_INTCLOCK_CLKEN_REG     (* (reg8 *) TeraTerm_IntClock__PM_ACT_CFG)
    #define TeraTerm_INTCLOCK_CLKEN_PTR     (  (reg8 *) TeraTerm_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define TeraTerm_INTCLOCK_CLKEN_MASK    TeraTerm_IntClock__PM_ACT_MSK
#endif /* End TeraTerm_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(TeraTerm_TX_ENABLED)
    #define TeraTerm_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End TeraTerm_TX_ENABLED */

#if(TeraTerm_HD_ENABLED)
    #define TeraTerm_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End TeraTerm_HD_ENABLED */

#if( (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) )
    #define TeraTerm_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) */


/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/

/* UART v2_40 obsolete definitions */
#define TeraTerm_WAIT_1_MS      TeraTerm_BL_CHK_DELAY_MS   

#define TeraTerm_TXBUFFERSIZE   TeraTerm_TX_BUFFER_SIZE
#define TeraTerm_RXBUFFERSIZE   TeraTerm_RX_BUFFER_SIZE

#if (TeraTerm_RXHW_ADDRESS_ENABLED)
    #define TeraTerm_RXADDRESSMODE  TeraTerm_RX_ADDRESS_MODE
    #define TeraTerm_RXHWADDRESS1   TeraTerm_RX_HW_ADDRESS1
    #define TeraTerm_RXHWADDRESS2   TeraTerm_RX_HW_ADDRESS2
    /* Backward compatible define */
    #define TeraTerm_RXAddressMode  TeraTerm_RXADDRESSMODE
#endif /* (TeraTerm_RXHW_ADDRESS_ENABLED) */

/* UART v2_30 obsolete definitions */
#define TeraTerm_initvar                    TeraTerm_initVar

#define TeraTerm_RX_Enabled                 TeraTerm_RX_ENABLED
#define TeraTerm_TX_Enabled                 TeraTerm_TX_ENABLED
#define TeraTerm_HD_Enabled                 TeraTerm_HD_ENABLED
#define TeraTerm_RX_IntInterruptEnabled     TeraTerm_RX_INTERRUPT_ENABLED
#define TeraTerm_TX_IntInterruptEnabled     TeraTerm_TX_INTERRUPT_ENABLED
#define TeraTerm_InternalClockUsed          TeraTerm_INTERNAL_CLOCK_USED
#define TeraTerm_RXHW_Address_Enabled       TeraTerm_RXHW_ADDRESS_ENABLED
#define TeraTerm_OverSampleCount            TeraTerm_OVER_SAMPLE_COUNT
#define TeraTerm_ParityType                 TeraTerm_PARITY_TYPE

#if( TeraTerm_TX_ENABLED && (TeraTerm_TXBUFFERSIZE > TeraTerm_FIFO_LENGTH))
    #define TeraTerm_TXBUFFER               TeraTerm_txBuffer
    #define TeraTerm_TXBUFFERREAD           TeraTerm_txBufferRead
    #define TeraTerm_TXBUFFERWRITE          TeraTerm_txBufferWrite
#endif /* End TeraTerm_TX_ENABLED */
#if( ( TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED ) && \
     (TeraTerm_RXBUFFERSIZE > TeraTerm_FIFO_LENGTH) )
    #define TeraTerm_RXBUFFER               TeraTerm_rxBuffer
    #define TeraTerm_RXBUFFERREAD           TeraTerm_rxBufferRead
    #define TeraTerm_RXBUFFERWRITE          TeraTerm_rxBufferWrite
    #define TeraTerm_RXBUFFERLOOPDETECT     TeraTerm_rxBufferLoopDetect
    #define TeraTerm_RXBUFFER_OVERFLOW      TeraTerm_rxBufferOverflow
#endif /* End TeraTerm_RX_ENABLED */

#ifdef TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define TeraTerm_CONTROL                TeraTerm_CONTROL_REG
#endif /* End TeraTerm_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(TeraTerm_TX_ENABLED)
    #define TeraTerm_TXDATA                 TeraTerm_TXDATA_REG
    #define TeraTerm_TXSTATUS               TeraTerm_TXSTATUS_REG
    #define TeraTerm_TXSTATUS_MASK          TeraTerm_TXSTATUS_MASK_REG
    #define TeraTerm_TXSTATUS_ACTL          TeraTerm_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(TeraTerm_TXCLKGEN_DP)
        #define TeraTerm_TXBITCLKGEN_CTR        TeraTerm_TXBITCLKGEN_CTR_REG
        #define TeraTerm_TXBITCLKTX_COMPLETE    TeraTerm_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define TeraTerm_TXBITCTR_PERIOD        TeraTerm_TXBITCTR_PERIOD_REG
        #define TeraTerm_TXBITCTR_CONTROL       TeraTerm_TXBITCTR_CONTROL_REG
        #define TeraTerm_TXBITCTR_COUNTER       TeraTerm_TXBITCTR_COUNTER_REG
    #endif /* TeraTerm_TXCLKGEN_DP */
#endif /* End TeraTerm_TX_ENABLED */

#if(TeraTerm_HD_ENABLED)
    #define TeraTerm_TXDATA                 TeraTerm_TXDATA_REG
    #define TeraTerm_TXSTATUS               TeraTerm_TXSTATUS_REG
    #define TeraTerm_TXSTATUS_MASK          TeraTerm_TXSTATUS_MASK_REG
    #define TeraTerm_TXSTATUS_ACTL          TeraTerm_TXSTATUS_ACTL_REG
#endif /* End TeraTerm_HD_ENABLED */

#if( (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) )
    #define TeraTerm_RXDATA                 TeraTerm_RXDATA_REG
    #define TeraTerm_RXADDRESS1             TeraTerm_RXADDRESS1_REG
    #define TeraTerm_RXADDRESS2             TeraTerm_RXADDRESS2_REG
    #define TeraTerm_RXBITCTR_PERIOD        TeraTerm_RXBITCTR_PERIOD_REG
    #define TeraTerm_RXBITCTR_CONTROL       TeraTerm_RXBITCTR_CONTROL_REG
    #define TeraTerm_RXBITCTR_COUNTER       TeraTerm_RXBITCTR_COUNTER_REG
    #define TeraTerm_RXSTATUS               TeraTerm_RXSTATUS_REG
    #define TeraTerm_RXSTATUS_MASK          TeraTerm_RXSTATUS_MASK_REG
    #define TeraTerm_RXSTATUS_ACTL          TeraTerm_RXSTATUS_ACTL_REG
#endif /* End  (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) */

#if(TeraTerm_INTERNAL_CLOCK_USED)
    #define TeraTerm_INTCLOCK_CLKEN         TeraTerm_INTCLOCK_CLKEN_REG
#endif /* End TeraTerm_INTERNAL_CLOCK_USED */

#define TeraTerm_WAIT_FOR_COMLETE_REINIT    TeraTerm_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_TeraTerm_H */


/* [] END OF FILE */
