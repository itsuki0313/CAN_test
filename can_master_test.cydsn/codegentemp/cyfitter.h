/*******************************************************************************
* File Name: cyfitter.h
* 
* PSoC Creator  4.2
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2018 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

#ifndef INCLUDED_CYFITTER_H
#define INCLUDED_CYFITTER_H
#include "cydevice.h"
#include "cydevice_trm.h"

/* SW */
#define SW__0__INTTYPE CYREG_PICU2_INTTYPE2
#define SW__0__MASK 0x04u
#define SW__0__PC CYREG_PRT2_PC2
#define SW__0__PORT 2u
#define SW__0__SHIFT 2u
#define SW__AG CYREG_PRT2_AG
#define SW__AMUX CYREG_PRT2_AMUX
#define SW__BIE CYREG_PRT2_BIE
#define SW__BIT_MASK CYREG_PRT2_BIT_MASK
#define SW__BYP CYREG_PRT2_BYP
#define SW__CTL CYREG_PRT2_CTL
#define SW__DM0 CYREG_PRT2_DM0
#define SW__DM1 CYREG_PRT2_DM1
#define SW__DM2 CYREG_PRT2_DM2
#define SW__DR CYREG_PRT2_DR
#define SW__INP_DIS CYREG_PRT2_INP_DIS
#define SW__INTTYPE_BASE CYDEV_PICU_INTTYPE_PICU2_BASE
#define SW__LCD_COM_SEG CYREG_PRT2_LCD_COM_SEG
#define SW__LCD_EN CYREG_PRT2_LCD_EN
#define SW__MASK 0x04u
#define SW__PORT 2u
#define SW__PRT CYREG_PRT2_PRT
#define SW__PRTDSI__CAPS_SEL CYREG_PRT2_CAPS_SEL
#define SW__PRTDSI__DBL_SYNC_IN CYREG_PRT2_DBL_SYNC_IN
#define SW__PRTDSI__OE_SEL0 CYREG_PRT2_OE_SEL0
#define SW__PRTDSI__OE_SEL1 CYREG_PRT2_OE_SEL1
#define SW__PRTDSI__OUT_SEL0 CYREG_PRT2_OUT_SEL0
#define SW__PRTDSI__OUT_SEL1 CYREG_PRT2_OUT_SEL1
#define SW__PRTDSI__SYNC_OUT CYREG_PRT2_SYNC_OUT
#define SW__PS CYREG_PRT2_PS
#define SW__SHIFT 2u
#define SW__SLW CYREG_PRT2_SLW

/* LED */
#define LED__0__INTTYPE CYREG_PICU2_INTTYPE1
#define LED__0__MASK 0x02u
#define LED__0__PC CYREG_PRT2_PC1
#define LED__0__PORT 2u
#define LED__0__SHIFT 1u
#define LED__AG CYREG_PRT2_AG
#define LED__AMUX CYREG_PRT2_AMUX
#define LED__BIE CYREG_PRT2_BIE
#define LED__BIT_MASK CYREG_PRT2_BIT_MASK
#define LED__BYP CYREG_PRT2_BYP
#define LED__CTL CYREG_PRT2_CTL
#define LED__DM0 CYREG_PRT2_DM0
#define LED__DM1 CYREG_PRT2_DM1
#define LED__DM2 CYREG_PRT2_DM2
#define LED__DR CYREG_PRT2_DR
#define LED__INP_DIS CYREG_PRT2_INP_DIS
#define LED__INTTYPE_BASE CYDEV_PICU_INTTYPE_PICU2_BASE
#define LED__LCD_COM_SEG CYREG_PRT2_LCD_COM_SEG
#define LED__LCD_EN CYREG_PRT2_LCD_EN
#define LED__MASK 0x02u
#define LED__PORT 2u
#define LED__PRT CYREG_PRT2_PRT
#define LED__PRTDSI__CAPS_SEL CYREG_PRT2_CAPS_SEL
#define LED__PRTDSI__DBL_SYNC_IN CYREG_PRT2_DBL_SYNC_IN
#define LED__PRTDSI__OE_SEL0 CYREG_PRT2_OE_SEL0
#define LED__PRTDSI__OE_SEL1 CYREG_PRT2_OE_SEL1
#define LED__PRTDSI__OUT_SEL0 CYREG_PRT2_OUT_SEL0
#define LED__PRTDSI__OUT_SEL1 CYREG_PRT2_OUT_SEL1
#define LED__PRTDSI__SYNC_OUT CYREG_PRT2_SYNC_OUT
#define LED__PS CYREG_PRT2_PS
#define LED__SHIFT 1u
#define LED__SLW CYREG_PRT2_SLW

/* RX_1 */
#define RX_1__0__INTTYPE CYREG_PICU12_INTTYPE3
#define RX_1__0__MASK 0x08u
#define RX_1__0__PC CYREG_PRT12_PC3
#define RX_1__0__PORT 12u
#define RX_1__0__SHIFT 3u
#define RX_1__AG CYREG_PRT12_AG
#define RX_1__BIE CYREG_PRT12_BIE
#define RX_1__BIT_MASK CYREG_PRT12_BIT_MASK
#define RX_1__BYP CYREG_PRT12_BYP
#define RX_1__DM0 CYREG_PRT12_DM0
#define RX_1__DM1 CYREG_PRT12_DM1
#define RX_1__DM2 CYREG_PRT12_DM2
#define RX_1__DR CYREG_PRT12_DR
#define RX_1__INP_DIS CYREG_PRT12_INP_DIS
#define RX_1__INTTYPE_BASE CYDEV_PICU_INTTYPE_PICU12_BASE
#define RX_1__MASK 0x08u
#define RX_1__PORT 12u
#define RX_1__PRT CYREG_PRT12_PRT
#define RX_1__PRTDSI__DBL_SYNC_IN CYREG_PRT12_DBL_SYNC_IN
#define RX_1__PRTDSI__OE_SEL0 CYREG_PRT12_OE_SEL0
#define RX_1__PRTDSI__OE_SEL1 CYREG_PRT12_OE_SEL1
#define RX_1__PRTDSI__OUT_SEL0 CYREG_PRT12_OUT_SEL0
#define RX_1__PRTDSI__OUT_SEL1 CYREG_PRT12_OUT_SEL1
#define RX_1__PRTDSI__SYNC_OUT CYREG_PRT12_SYNC_OUT
#define RX_1__PS CYREG_PRT12_PS
#define RX_1__SHIFT 3u
#define RX_1__SIO_CFG CYREG_PRT12_SIO_CFG
#define RX_1__SIO_DIFF CYREG_PRT12_SIO_DIFF
#define RX_1__SIO_HYST_EN CYREG_PRT12_SIO_HYST_EN
#define RX_1__SIO_REG_HIFREQ CYREG_PRT12_SIO_REG_HIFREQ
#define RX_1__SLW CYREG_PRT12_SLW

/* TX_1 */
#define TX_1__0__INTTYPE CYREG_PICU12_INTTYPE2
#define TX_1__0__MASK 0x04u
#define TX_1__0__PC CYREG_PRT12_PC2
#define TX_1__0__PORT 12u
#define TX_1__0__SHIFT 2u
#define TX_1__AG CYREG_PRT12_AG
#define TX_1__BIE CYREG_PRT12_BIE
#define TX_1__BIT_MASK CYREG_PRT12_BIT_MASK
#define TX_1__BYP CYREG_PRT12_BYP
#define TX_1__DM0 CYREG_PRT12_DM0
#define TX_1__DM1 CYREG_PRT12_DM1
#define TX_1__DM2 CYREG_PRT12_DM2
#define TX_1__DR CYREG_PRT12_DR
#define TX_1__INP_DIS CYREG_PRT12_INP_DIS
#define TX_1__INTTYPE_BASE CYDEV_PICU_INTTYPE_PICU12_BASE
#define TX_1__MASK 0x04u
#define TX_1__PORT 12u
#define TX_1__PRT CYREG_PRT12_PRT
#define TX_1__PRTDSI__DBL_SYNC_IN CYREG_PRT12_DBL_SYNC_IN
#define TX_1__PRTDSI__OE_SEL0 CYREG_PRT12_OE_SEL0
#define TX_1__PRTDSI__OE_SEL1 CYREG_PRT12_OE_SEL1
#define TX_1__PRTDSI__OUT_SEL0 CYREG_PRT12_OUT_SEL0
#define TX_1__PRTDSI__OUT_SEL1 CYREG_PRT12_OUT_SEL1
#define TX_1__PRTDSI__SYNC_OUT CYREG_PRT12_SYNC_OUT
#define TX_1__PS CYREG_PRT12_PS
#define TX_1__SHIFT 2u
#define TX_1__SIO_CFG CYREG_PRT12_SIO_CFG
#define TX_1__SIO_DIFF CYREG_PRT12_SIO_DIFF
#define TX_1__SIO_HYST_EN CYREG_PRT12_SIO_HYST_EN
#define TX_1__SIO_REG_HIFREQ CYREG_PRT12_SIO_REG_HIFREQ
#define TX_1__SLW CYREG_PRT12_SLW

/* Tx_2 */
#define Tx_2__0__INTTYPE CYREG_PICU12_INTTYPE7
#define Tx_2__0__MASK 0x80u
#define Tx_2__0__PC CYREG_PRT12_PC7
#define Tx_2__0__PORT 12u
#define Tx_2__0__SHIFT 7u
#define Tx_2__AG CYREG_PRT12_AG
#define Tx_2__BIE CYREG_PRT12_BIE
#define Tx_2__BIT_MASK CYREG_PRT12_BIT_MASK
#define Tx_2__BYP CYREG_PRT12_BYP
#define Tx_2__DM0 CYREG_PRT12_DM0
#define Tx_2__DM1 CYREG_PRT12_DM1
#define Tx_2__DM2 CYREG_PRT12_DM2
#define Tx_2__DR CYREG_PRT12_DR
#define Tx_2__INP_DIS CYREG_PRT12_INP_DIS
#define Tx_2__INTTYPE_BASE CYDEV_PICU_INTTYPE_PICU12_BASE
#define Tx_2__MASK 0x80u
#define Tx_2__PORT 12u
#define Tx_2__PRT CYREG_PRT12_PRT
#define Tx_2__PRTDSI__DBL_SYNC_IN CYREG_PRT12_DBL_SYNC_IN
#define Tx_2__PRTDSI__OE_SEL0 CYREG_PRT12_OE_SEL0
#define Tx_2__PRTDSI__OE_SEL1 CYREG_PRT12_OE_SEL1
#define Tx_2__PRTDSI__OUT_SEL0 CYREG_PRT12_OUT_SEL0
#define Tx_2__PRTDSI__OUT_SEL1 CYREG_PRT12_OUT_SEL1
#define Tx_2__PRTDSI__SYNC_OUT CYREG_PRT12_SYNC_OUT
#define Tx_2__PS CYREG_PRT12_PS
#define Tx_2__SHIFT 7u
#define Tx_2__SIO_CFG CYREG_PRT12_SIO_CFG
#define Tx_2__SIO_DIFF CYREG_PRT12_SIO_DIFF
#define Tx_2__SIO_HYST_EN CYREG_PRT12_SIO_HYST_EN
#define Tx_2__SIO_REG_HIFREQ CYREG_PRT12_SIO_REG_HIFREQ
#define Tx_2__SLW CYREG_PRT12_SLW

/* CAN_1 */
#define CAN_1_CanIP__CSR_BUF_SR CYREG_CAN0_CSR_BUF_SR
#define CAN_1_CanIP__CSR_CFG CYREG_CAN0_CSR_CFG
#define CAN_1_CanIP__CSR_CMD CYREG_CAN0_CSR_CMD
#define CAN_1_CanIP__CSR_ERR_SR CYREG_CAN0_CSR_ERR_SR
#define CAN_1_CanIP__CSR_INT_EN CYREG_CAN0_CSR_INT_EN
#define CAN_1_CanIP__CSR_INT_SR CYREG_CAN0_CSR_INT_SR
#define CAN_1_CanIP__PM_ACT_CFG CYREG_PM_ACT_CFG6
#define CAN_1_CanIP__PM_ACT_MSK 0x01u
#define CAN_1_CanIP__PM_STBY_CFG CYREG_PM_STBY_CFG6
#define CAN_1_CanIP__PM_STBY_MSK 0x01u
#define CAN_1_CanIP__RX0_ACR CYREG_CAN0_RX0_ACR
#define CAN_1_CanIP__RX0_ACRD CYREG_CAN0_RX0_ACRD
#define CAN_1_CanIP__RX0_AMR CYREG_CAN0_RX0_AMR
#define CAN_1_CanIP__RX0_AMRD CYREG_CAN0_RX0_AMRD
#define CAN_1_CanIP__RX0_CMD CYREG_CAN0_RX0_CMD
#define CAN_1_CanIP__RX0_DH CYREG_CAN0_RX0_DH
#define CAN_1_CanIP__RX0_DL CYREG_CAN0_RX0_DL
#define CAN_1_CanIP__RX0_ID CYREG_CAN0_RX0_ID
#define CAN_1_CanIP__RX1_ACR CYREG_CAN0_RX1_ACR
#define CAN_1_CanIP__RX1_ACRD CYREG_CAN0_RX1_ACRD
#define CAN_1_CanIP__RX1_AMR CYREG_CAN0_RX1_AMR
#define CAN_1_CanIP__RX1_AMRD CYREG_CAN0_RX1_AMRD
#define CAN_1_CanIP__RX1_CMD CYREG_CAN0_RX1_CMD
#define CAN_1_CanIP__RX1_DH CYREG_CAN0_RX1_DH
#define CAN_1_CanIP__RX1_DL CYREG_CAN0_RX1_DL
#define CAN_1_CanIP__RX1_ID CYREG_CAN0_RX1_ID
#define CAN_1_CanIP__RX10_ACR CYREG_CAN0_RX10_ACR
#define CAN_1_CanIP__RX10_ACRD CYREG_CAN0_RX10_ACRD
#define CAN_1_CanIP__RX10_AMR CYREG_CAN0_RX10_AMR
#define CAN_1_CanIP__RX10_AMRD CYREG_CAN0_RX10_AMRD
#define CAN_1_CanIP__RX10_CMD CYREG_CAN0_RX10_CMD
#define CAN_1_CanIP__RX10_DH CYREG_CAN0_RX10_DH
#define CAN_1_CanIP__RX10_DL CYREG_CAN0_RX10_DL
#define CAN_1_CanIP__RX10_ID CYREG_CAN0_RX10_ID
#define CAN_1_CanIP__RX11_ACR CYREG_CAN0_RX11_ACR
#define CAN_1_CanIP__RX11_ACRD CYREG_CAN0_RX11_ACRD
#define CAN_1_CanIP__RX11_AMR CYREG_CAN0_RX11_AMR
#define CAN_1_CanIP__RX11_AMRD CYREG_CAN0_RX11_AMRD
#define CAN_1_CanIP__RX11_CMD CYREG_CAN0_RX11_CMD
#define CAN_1_CanIP__RX11_DH CYREG_CAN0_RX11_DH
#define CAN_1_CanIP__RX11_DL CYREG_CAN0_RX11_DL
#define CAN_1_CanIP__RX11_ID CYREG_CAN0_RX11_ID
#define CAN_1_CanIP__RX12_ACR CYREG_CAN0_RX12_ACR
#define CAN_1_CanIP__RX12_ACRD CYREG_CAN0_RX12_ACRD
#define CAN_1_CanIP__RX12_AMR CYREG_CAN0_RX12_AMR
#define CAN_1_CanIP__RX12_AMRD CYREG_CAN0_RX12_AMRD
#define CAN_1_CanIP__RX12_CMD CYREG_CAN0_RX12_CMD
#define CAN_1_CanIP__RX12_DH CYREG_CAN0_RX12_DH
#define CAN_1_CanIP__RX12_DL CYREG_CAN0_RX12_DL
#define CAN_1_CanIP__RX12_ID CYREG_CAN0_RX12_ID
#define CAN_1_CanIP__RX13_ACR CYREG_CAN0_RX13_ACR
#define CAN_1_CanIP__RX13_ACRD CYREG_CAN0_RX13_ACRD
#define CAN_1_CanIP__RX13_AMR CYREG_CAN0_RX13_AMR
#define CAN_1_CanIP__RX13_AMRD CYREG_CAN0_RX13_AMRD
#define CAN_1_CanIP__RX13_CMD CYREG_CAN0_RX13_CMD
#define CAN_1_CanIP__RX13_DH CYREG_CAN0_RX13_DH
#define CAN_1_CanIP__RX13_DL CYREG_CAN0_RX13_DL
#define CAN_1_CanIP__RX13_ID CYREG_CAN0_RX13_ID
#define CAN_1_CanIP__RX14_ACR CYREG_CAN0_RX14_ACR
#define CAN_1_CanIP__RX14_ACRD CYREG_CAN0_RX14_ACRD
#define CAN_1_CanIP__RX14_AMR CYREG_CAN0_RX14_AMR
#define CAN_1_CanIP__RX14_AMRD CYREG_CAN0_RX14_AMRD
#define CAN_1_CanIP__RX14_CMD CYREG_CAN0_RX14_CMD
#define CAN_1_CanIP__RX14_DH CYREG_CAN0_RX14_DH
#define CAN_1_CanIP__RX14_DL CYREG_CAN0_RX14_DL
#define CAN_1_CanIP__RX14_ID CYREG_CAN0_RX14_ID
#define CAN_1_CanIP__RX15_ACR CYREG_CAN0_RX15_ACR
#define CAN_1_CanIP__RX15_ACRD CYREG_CAN0_RX15_ACRD
#define CAN_1_CanIP__RX15_AMR CYREG_CAN0_RX15_AMR
#define CAN_1_CanIP__RX15_AMRD CYREG_CAN0_RX15_AMRD
#define CAN_1_CanIP__RX15_CMD CYREG_CAN0_RX15_CMD
#define CAN_1_CanIP__RX15_DH CYREG_CAN0_RX15_DH
#define CAN_1_CanIP__RX15_DL CYREG_CAN0_RX15_DL
#define CAN_1_CanIP__RX15_ID CYREG_CAN0_RX15_ID
#define CAN_1_CanIP__RX2_ACR CYREG_CAN0_RX2_ACR
#define CAN_1_CanIP__RX2_ACRD CYREG_CAN0_RX2_ACRD
#define CAN_1_CanIP__RX2_AMR CYREG_CAN0_RX2_AMR
#define CAN_1_CanIP__RX2_AMRD CYREG_CAN0_RX2_AMRD
#define CAN_1_CanIP__RX2_CMD CYREG_CAN0_RX2_CMD
#define CAN_1_CanIP__RX2_DH CYREG_CAN0_RX2_DH
#define CAN_1_CanIP__RX2_DL CYREG_CAN0_RX2_DL
#define CAN_1_CanIP__RX2_ID CYREG_CAN0_RX2_ID
#define CAN_1_CanIP__RX3_ACR CYREG_CAN0_RX3_ACR
#define CAN_1_CanIP__RX3_ACRD CYREG_CAN0_RX3_ACRD
#define CAN_1_CanIP__RX3_AMR CYREG_CAN0_RX3_AMR
#define CAN_1_CanIP__RX3_AMRD CYREG_CAN0_RX3_AMRD
#define CAN_1_CanIP__RX3_CMD CYREG_CAN0_RX3_CMD
#define CAN_1_CanIP__RX3_DH CYREG_CAN0_RX3_DH
#define CAN_1_CanIP__RX3_DL CYREG_CAN0_RX3_DL
#define CAN_1_CanIP__RX3_ID CYREG_CAN0_RX3_ID
#define CAN_1_CanIP__RX4_ACR CYREG_CAN0_RX4_ACR
#define CAN_1_CanIP__RX4_ACRD CYREG_CAN0_RX4_ACRD
#define CAN_1_CanIP__RX4_AMR CYREG_CAN0_RX4_AMR
#define CAN_1_CanIP__RX4_AMRD CYREG_CAN0_RX4_AMRD
#define CAN_1_CanIP__RX4_CMD CYREG_CAN0_RX4_CMD
#define CAN_1_CanIP__RX4_DH CYREG_CAN0_RX4_DH
#define CAN_1_CanIP__RX4_DL CYREG_CAN0_RX4_DL
#define CAN_1_CanIP__RX4_ID CYREG_CAN0_RX4_ID
#define CAN_1_CanIP__RX5_ACR CYREG_CAN0_RX5_ACR
#define CAN_1_CanIP__RX5_ACRD CYREG_CAN0_RX5_ACRD
#define CAN_1_CanIP__RX5_AMR CYREG_CAN0_RX5_AMR
#define CAN_1_CanIP__RX5_AMRD CYREG_CAN0_RX5_AMRD
#define CAN_1_CanIP__RX5_CMD CYREG_CAN0_RX5_CMD
#define CAN_1_CanIP__RX5_DH CYREG_CAN0_RX5_DH
#define CAN_1_CanIP__RX5_DL CYREG_CAN0_RX5_DL
#define CAN_1_CanIP__RX5_ID CYREG_CAN0_RX5_ID
#define CAN_1_CanIP__RX6_ACR CYREG_CAN0_RX6_ACR
#define CAN_1_CanIP__RX6_ACRD CYREG_CAN0_RX6_ACRD
#define CAN_1_CanIP__RX6_AMR CYREG_CAN0_RX6_AMR
#define CAN_1_CanIP__RX6_AMRD CYREG_CAN0_RX6_AMRD
#define CAN_1_CanIP__RX6_CMD CYREG_CAN0_RX6_CMD
#define CAN_1_CanIP__RX6_DH CYREG_CAN0_RX6_DH
#define CAN_1_CanIP__RX6_DL CYREG_CAN0_RX6_DL
#define CAN_1_CanIP__RX6_ID CYREG_CAN0_RX6_ID
#define CAN_1_CanIP__RX7_ACR CYREG_CAN0_RX7_ACR
#define CAN_1_CanIP__RX7_ACRD CYREG_CAN0_RX7_ACRD
#define CAN_1_CanIP__RX7_AMR CYREG_CAN0_RX7_AMR
#define CAN_1_CanIP__RX7_AMRD CYREG_CAN0_RX7_AMRD
#define CAN_1_CanIP__RX7_CMD CYREG_CAN0_RX7_CMD
#define CAN_1_CanIP__RX7_DH CYREG_CAN0_RX7_DH
#define CAN_1_CanIP__RX7_DL CYREG_CAN0_RX7_DL
#define CAN_1_CanIP__RX7_ID CYREG_CAN0_RX7_ID
#define CAN_1_CanIP__RX8_ACR CYREG_CAN0_RX8_ACR
#define CAN_1_CanIP__RX8_ACRD CYREG_CAN0_RX8_ACRD
#define CAN_1_CanIP__RX8_AMR CYREG_CAN0_RX8_AMR
#define CAN_1_CanIP__RX8_AMRD CYREG_CAN0_RX8_AMRD
#define CAN_1_CanIP__RX8_CMD CYREG_CAN0_RX8_CMD
#define CAN_1_CanIP__RX8_DH CYREG_CAN0_RX8_DH
#define CAN_1_CanIP__RX8_DL CYREG_CAN0_RX8_DL
#define CAN_1_CanIP__RX8_ID CYREG_CAN0_RX8_ID
#define CAN_1_CanIP__RX9_ACR CYREG_CAN0_RX9_ACR
#define CAN_1_CanIP__RX9_ACRD CYREG_CAN0_RX9_ACRD
#define CAN_1_CanIP__RX9_AMR CYREG_CAN0_RX9_AMR
#define CAN_1_CanIP__RX9_AMRD CYREG_CAN0_RX9_AMRD
#define CAN_1_CanIP__RX9_CMD CYREG_CAN0_RX9_CMD
#define CAN_1_CanIP__RX9_DH CYREG_CAN0_RX9_DH
#define CAN_1_CanIP__RX9_DL CYREG_CAN0_RX9_DL
#define CAN_1_CanIP__RX9_ID CYREG_CAN0_RX9_ID
#define CAN_1_CanIP__TX0_CMD CYREG_CAN0_TX0_CMD
#define CAN_1_CanIP__TX0_DH CYREG_CAN0_TX0_DH
#define CAN_1_CanIP__TX0_DL CYREG_CAN0_TX0_DL
#define CAN_1_CanIP__TX0_ID CYREG_CAN0_TX0_ID
#define CAN_1_CanIP__TX1_CMD CYREG_CAN0_TX1_CMD
#define CAN_1_CanIP__TX1_DH CYREG_CAN0_TX1_DH
#define CAN_1_CanIP__TX1_DL CYREG_CAN0_TX1_DL
#define CAN_1_CanIP__TX1_ID CYREG_CAN0_TX1_ID
#define CAN_1_CanIP__TX2_CMD CYREG_CAN0_TX2_CMD
#define CAN_1_CanIP__TX2_DH CYREG_CAN0_TX2_DH
#define CAN_1_CanIP__TX2_DL CYREG_CAN0_TX2_DL
#define CAN_1_CanIP__TX2_ID CYREG_CAN0_TX2_ID
#define CAN_1_CanIP__TX3_CMD CYREG_CAN0_TX3_CMD
#define CAN_1_CanIP__TX3_DH CYREG_CAN0_TX3_DH
#define CAN_1_CanIP__TX3_DL CYREG_CAN0_TX3_DL
#define CAN_1_CanIP__TX3_ID CYREG_CAN0_TX3_ID
#define CAN_1_CanIP__TX4_CMD CYREG_CAN0_TX4_CMD
#define CAN_1_CanIP__TX4_DH CYREG_CAN0_TX4_DH
#define CAN_1_CanIP__TX4_DL CYREG_CAN0_TX4_DL
#define CAN_1_CanIP__TX4_ID CYREG_CAN0_TX4_ID
#define CAN_1_CanIP__TX5_CMD CYREG_CAN0_TX5_CMD
#define CAN_1_CanIP__TX5_DH CYREG_CAN0_TX5_DH
#define CAN_1_CanIP__TX5_DL CYREG_CAN0_TX5_DL
#define CAN_1_CanIP__TX5_ID CYREG_CAN0_TX5_ID
#define CAN_1_CanIP__TX6_CMD CYREG_CAN0_TX6_CMD
#define CAN_1_CanIP__TX6_DH CYREG_CAN0_TX6_DH
#define CAN_1_CanIP__TX6_DL CYREG_CAN0_TX6_DL
#define CAN_1_CanIP__TX6_ID CYREG_CAN0_TX6_ID
#define CAN_1_CanIP__TX7_CMD CYREG_CAN0_TX7_CMD
#define CAN_1_CanIP__TX7_DH CYREG_CAN0_TX7_DH
#define CAN_1_CanIP__TX7_DL CYREG_CAN0_TX7_DL
#define CAN_1_CanIP__TX7_ID CYREG_CAN0_TX7_ID
#define CAN_1_isr__INTC_CLR_EN_REG CYREG_NVIC_CLRENA0
#define CAN_1_isr__INTC_CLR_PD_REG CYREG_NVIC_CLRPEND0
#define CAN_1_isr__INTC_MASK 0x10000u
#define CAN_1_isr__INTC_NUMBER 16u
#define CAN_1_isr__INTC_PRIOR_NUM 7u
#define CAN_1_isr__INTC_PRIOR_REG CYREG_NVIC_PRI_16
#define CAN_1_isr__INTC_SET_EN_REG CYREG_NVIC_SETENA0
#define CAN_1_isr__INTC_SET_PD_REG CYREG_NVIC_SETPEND0

/* UART_1 */
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__A0_A1_REG CYREG_B1_UDB11_A0_A1
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__A0_REG CYREG_B1_UDB11_A0
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__A1_REG CYREG_B1_UDB11_A1
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__D0_D1_REG CYREG_B1_UDB11_D0_D1
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG CYREG_B1_UDB11_D0
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG CYREG_B1_UDB11_D1
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__DP_AUX_CTL_REG CYREG_B1_UDB11_ACTL
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__F0_F1_REG CYREG_B1_UDB11_F0_F1
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__F0_REG CYREG_B1_UDB11_F0
#define UART_1_BUART_sTX_sCLOCK_TxBitClkGen__F1_REG CYREG_B1_UDB11_F1
#define UART_1_BUART_sTX_TxShifter_u0__16BIT_A0_REG CYREG_B1_UDB10_11_A0
#define UART_1_BUART_sTX_TxShifter_u0__16BIT_A1_REG CYREG_B1_UDB10_11_A1
#define UART_1_BUART_sTX_TxShifter_u0__16BIT_D0_REG CYREG_B1_UDB10_11_D0
#define UART_1_BUART_sTX_TxShifter_u0__16BIT_D1_REG CYREG_B1_UDB10_11_D1
#define UART_1_BUART_sTX_TxShifter_u0__16BIT_DP_AUX_CTL_REG CYREG_B1_UDB10_11_ACTL
#define UART_1_BUART_sTX_TxShifter_u0__16BIT_F0_REG CYREG_B1_UDB10_11_F0
#define UART_1_BUART_sTX_TxShifter_u0__16BIT_F1_REG CYREG_B1_UDB10_11_F1
#define UART_1_BUART_sTX_TxShifter_u0__A0_A1_REG CYREG_B1_UDB10_A0_A1
#define UART_1_BUART_sTX_TxShifter_u0__A0_REG CYREG_B1_UDB10_A0
#define UART_1_BUART_sTX_TxShifter_u0__A1_REG CYREG_B1_UDB10_A1
#define UART_1_BUART_sTX_TxShifter_u0__D0_D1_REG CYREG_B1_UDB10_D0_D1
#define UART_1_BUART_sTX_TxShifter_u0__D0_REG CYREG_B1_UDB10_D0
#define UART_1_BUART_sTX_TxShifter_u0__D1_REG CYREG_B1_UDB10_D1
#define UART_1_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG CYREG_B1_UDB10_ACTL
#define UART_1_BUART_sTX_TxShifter_u0__F0_F1_REG CYREG_B1_UDB10_F0_F1
#define UART_1_BUART_sTX_TxShifter_u0__F0_REG CYREG_B1_UDB10_F0
#define UART_1_BUART_sTX_TxShifter_u0__F1_REG CYREG_B1_UDB10_F1
#define UART_1_BUART_sTX_TxSts__0__MASK 0x01u
#define UART_1_BUART_sTX_TxSts__0__POS 0
#define UART_1_BUART_sTX_TxSts__1__MASK 0x02u
#define UART_1_BUART_sTX_TxSts__1__POS 1
#define UART_1_BUART_sTX_TxSts__16BIT_STATUS_AUX_CTL_REG CYREG_B1_UDB10_11_ACTL
#define UART_1_BUART_sTX_TxSts__16BIT_STATUS_REG CYREG_B1_UDB10_11_ST
#define UART_1_BUART_sTX_TxSts__2__MASK 0x04u
#define UART_1_BUART_sTX_TxSts__2__POS 2
#define UART_1_BUART_sTX_TxSts__3__MASK 0x08u
#define UART_1_BUART_sTX_TxSts__3__POS 3
#define UART_1_BUART_sTX_TxSts__MASK 0x0Fu
#define UART_1_BUART_sTX_TxSts__MASK_REG CYREG_B1_UDB10_MSK
#define UART_1_BUART_sTX_TxSts__STATUS_AUX_CTL_REG CYREG_B1_UDB10_ACTL
#define UART_1_BUART_sTX_TxSts__STATUS_REG CYREG_B1_UDB10_ST
#define UART_1_IntClock__CFG0 CYREG_CLKDIST_DCFG0_CFG0
#define UART_1_IntClock__CFG1 CYREG_CLKDIST_DCFG0_CFG1
#define UART_1_IntClock__CFG2 CYREG_CLKDIST_DCFG0_CFG2
#define UART_1_IntClock__CFG2_SRC_SEL_MASK 0x07u
#define UART_1_IntClock__INDEX 0x00u
#define UART_1_IntClock__PM_ACT_CFG CYREG_PM_ACT_CFG2
#define UART_1_IntClock__PM_ACT_MSK 0x01u
#define UART_1_IntClock__PM_STBY_CFG CYREG_PM_STBY_CFG2
#define UART_1_IntClock__PM_STBY_MSK 0x01u

/* Miscellaneous */
#define BCLK__BUS_CLK__HZ 24000000U
#define BCLK__BUS_CLK__KHZ 24000U
#define BCLK__BUS_CLK__MHZ 24U
#define CY_PROJECT_NAME "can_master_test"
#define CY_VERSION "PSoC Creator  4.2"
#define CYDEV_CHIP_DIE_LEOPARD 1u
#define CYDEV_CHIP_DIE_PSOC4A 18u
#define CYDEV_CHIP_DIE_PSOC5LP 2u
#define CYDEV_CHIP_DIE_PSOC5TM 3u
#define CYDEV_CHIP_DIE_TMA4 4u
#define CYDEV_CHIP_DIE_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_FM0P 5u
#define CYDEV_CHIP_FAMILY_FM3 6u
#define CYDEV_CHIP_FAMILY_FM4 7u
#define CYDEV_CHIP_FAMILY_PSOC3 1u
#define CYDEV_CHIP_FAMILY_PSOC4 2u
#define CYDEV_CHIP_FAMILY_PSOC5 3u
#define CYDEV_CHIP_FAMILY_PSOC6 4u
#define CYDEV_CHIP_FAMILY_UNKNOWN 0u
#define CYDEV_CHIP_FAMILY_USED CYDEV_CHIP_FAMILY_PSOC5
#define CYDEV_CHIP_JTAG_ID 0x2E161069u
#define CYDEV_CHIP_MEMBER_3A 1u
#define CYDEV_CHIP_MEMBER_4A 18u
#define CYDEV_CHIP_MEMBER_4D 13u
#define CYDEV_CHIP_MEMBER_4E 6u
#define CYDEV_CHIP_MEMBER_4F 19u
#define CYDEV_CHIP_MEMBER_4G 4u
#define CYDEV_CHIP_MEMBER_4H 17u
#define CYDEV_CHIP_MEMBER_4I 23u
#define CYDEV_CHIP_MEMBER_4J 14u
#define CYDEV_CHIP_MEMBER_4K 15u
#define CYDEV_CHIP_MEMBER_4L 22u
#define CYDEV_CHIP_MEMBER_4M 21u
#define CYDEV_CHIP_MEMBER_4N 10u
#define CYDEV_CHIP_MEMBER_4O 7u
#define CYDEV_CHIP_MEMBER_4P 20u
#define CYDEV_CHIP_MEMBER_4Q 12u
#define CYDEV_CHIP_MEMBER_4R 8u
#define CYDEV_CHIP_MEMBER_4S 11u
#define CYDEV_CHIP_MEMBER_4T 9u
#define CYDEV_CHIP_MEMBER_4U 5u
#define CYDEV_CHIP_MEMBER_4V 16u
#define CYDEV_CHIP_MEMBER_5A 3u
#define CYDEV_CHIP_MEMBER_5B 2u
#define CYDEV_CHIP_MEMBER_6A 24u
#define CYDEV_CHIP_MEMBER_FM3 28u
#define CYDEV_CHIP_MEMBER_FM4 29u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1 25u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2 26u
#define CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3 27u
#define CYDEV_CHIP_MEMBER_UNKNOWN 0u
#define CYDEV_CHIP_MEMBER_USED CYDEV_CHIP_MEMBER_5B
#define CYDEV_CHIP_DIE_EXPECT CYDEV_CHIP_MEMBER_USED
#define CYDEV_CHIP_DIE_ACTUAL CYDEV_CHIP_DIE_EXPECT
#define CYDEV_CHIP_REV_LEOPARD_ES1 0u
#define CYDEV_CHIP_REV_LEOPARD_ES2 1u
#define CYDEV_CHIP_REV_LEOPARD_ES3 3u
#define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3u
#define CYDEV_CHIP_REV_PSOC4A_ES0 17u
#define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17u
#define CYDEV_CHIP_REV_PSOC5LP_ES0 0u
#define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0u
#define CYDEV_CHIP_REV_PSOC5TM_ES0 0u
#define CYDEV_CHIP_REV_PSOC5TM_ES1 1u
#define CYDEV_CHIP_REV_PSOC5TM_PRODUCTION 1u
#define CYDEV_CHIP_REV_TMA4_ES 17u
#define CYDEV_CHIP_REV_TMA4_ES2 33u
#define CYDEV_CHIP_REV_TMA4_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_3A_ES1 0u
#define CYDEV_CHIP_REVISION_3A_ES2 1u
#define CYDEV_CHIP_REVISION_3A_ES3 3u
#define CYDEV_CHIP_REVISION_3A_PRODUCTION 3u
#define CYDEV_CHIP_REVISION_4A_ES0 17u
#define CYDEV_CHIP_REVISION_4A_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_4D_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD 0u
#define CYDEV_CHIP_REVISION_4E_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA 0u
#define CYDEV_CHIP_REVISION_4F_PRODUCTION_256K 0u
#define CYDEV_CHIP_REVISION_4G_ES 17u
#define CYDEV_CHIP_REVISION_4G_ES2 33u
#define CYDEV_CHIP_REVISION_4G_PRODUCTION 17u
#define CYDEV_CHIP_REVISION_4H_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4I_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4J_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4K_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4L_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4M_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4N_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4O_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4P_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4Q_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4R_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4S_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4T_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4U_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_4V_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_5A_ES0 0u
#define CYDEV_CHIP_REVISION_5A_ES1 1u
#define CYDEV_CHIP_REVISION_5A_PRODUCTION 1u
#define CYDEV_CHIP_REVISION_5B_ES0 0u
#define CYDEV_CHIP_REVISION_5B_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_6A_ES 17u
#define CYDEV_CHIP_REVISION_6A_NO_UDB 33u
#define CYDEV_CHIP_REVISION_6A_PRODUCTION 33u
#define CYDEV_CHIP_REVISION_FM3_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_FM4_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION 0u
#define CYDEV_CHIP_REVISION_USED CYDEV_CHIP_REVISION_5B_PRODUCTION
#define CYDEV_CHIP_REV_EXPECT CYDEV_CHIP_REVISION_USED
#define CYDEV_CONFIG_FASTBOOT_ENABLED 1
#define CYDEV_CONFIG_UNUSED_IO_AllowButWarn 0
#define CYDEV_CONFIG_UNUSED_IO CYDEV_CONFIG_UNUSED_IO_AllowButWarn
#define CYDEV_CONFIG_UNUSED_IO_AllowWithInfo 1
#define CYDEV_CONFIG_UNUSED_IO_Disallowed 2
#define CYDEV_CONFIGURATION_COMPRESSED 1
#define CYDEV_CONFIGURATION_DMA 0
#define CYDEV_CONFIGURATION_ECC 1
#define CYDEV_CONFIGURATION_IMOENABLED CYDEV_CONFIG_FASTBOOT_ENABLED
#define CYDEV_CONFIGURATION_MODE_COMPRESSED 0
#define CYDEV_CONFIGURATION_MODE CYDEV_CONFIGURATION_MODE_COMPRESSED
#define CYDEV_CONFIGURATION_MODE_DMA 2
#define CYDEV_CONFIGURATION_MODE_UNCOMPRESSED 1
#define CYDEV_DEBUG_ENABLE_MASK 0x20u
#define CYDEV_DEBUG_ENABLE_REGISTER CYREG_MLOGIC_DEBUG
#define CYDEV_DEBUGGING_DPS_Disable 3
#define CYDEV_DEBUGGING_DPS_JTAG_4 1
#define CYDEV_DEBUGGING_DPS_JTAG_5 0
#define CYDEV_DEBUGGING_DPS_SWD 2
#define CYDEV_DEBUGGING_DPS_SWD_SWV 6
#define CYDEV_DEBUGGING_DPS CYDEV_DEBUGGING_DPS_SWD_SWV
#define CYDEV_DEBUGGING_ENABLE 1
#define CYDEV_DEBUGGING_XRES 0
#define CYDEV_DMA_CHANNELS_AVAILABLE 24u
#define CYDEV_ECC_ENABLE 0
#define CYDEV_HEAP_SIZE 0x80
#define CYDEV_INSTRUCT_CACHE_ENABLED 1
#define CYDEV_INTR_RISING 0x00000000u
#define CYDEV_IS_EXPORTING_CODE 0
#define CYDEV_IS_IMPORTING_CODE 0
#define CYDEV_PROJ_TYPE 0
#define CYDEV_PROJ_TYPE_BOOTLOADER 1
#define CYDEV_PROJ_TYPE_LAUNCHER 5
#define CYDEV_PROJ_TYPE_LOADABLE 2
#define CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER 4
#define CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER 3
#define CYDEV_PROJ_TYPE_STANDARD 0
#define CYDEV_PROTECTION_ENABLE 0
#define CYDEV_STACK_SIZE 0x0800
#define CYDEV_USE_BUNDLED_CMSIS 1
#define CYDEV_VARIABLE_VDDA 0
#define CYDEV_VDDA 5.0
#define CYDEV_VDDA_MV 5000
#define CYDEV_VDDD 5.0
#define CYDEV_VDDD_MV 5000
#define CYDEV_VDDIO0 5.0
#define CYDEV_VDDIO0_MV 5000
#define CYDEV_VDDIO1 5.0
#define CYDEV_VDDIO1_MV 5000
#define CYDEV_VDDIO2 5.0
#define CYDEV_VDDIO2_MV 5000
#define CYDEV_VDDIO3 5.0
#define CYDEV_VDDIO3_MV 5000
#define CYDEV_VIO0 5.0
#define CYDEV_VIO0_MV 5000
#define CYDEV_VIO1 5.0
#define CYDEV_VIO1_MV 5000
#define CYDEV_VIO2 5.0
#define CYDEV_VIO2_MV 5000
#define CYDEV_VIO3 5.0
#define CYDEV_VIO3_MV 5000
#define CYIPBLOCK_ARM_CM3_VERSION 0
#define CYIPBLOCK_P3_ANAIF_VERSION 0
#define CYIPBLOCK_P3_CAN_VERSION 0
#define CYIPBLOCK_P3_CAPSENSE_VERSION 0
#define CYIPBLOCK_P3_COMP_VERSION 0
#define CYIPBLOCK_P3_DECIMATOR_VERSION 0
#define CYIPBLOCK_P3_DFB_VERSION 0
#define CYIPBLOCK_P3_DMA_VERSION 0
#define CYIPBLOCK_P3_DRQ_VERSION 0
#define CYIPBLOCK_P3_DSM_VERSION 0
#define CYIPBLOCK_P3_EMIF_VERSION 0
#define CYIPBLOCK_P3_I2C_VERSION 0
#define CYIPBLOCK_P3_LCD_VERSION 0
#define CYIPBLOCK_P3_LPF_VERSION 0
#define CYIPBLOCK_P3_OPAMP_VERSION 0
#define CYIPBLOCK_P3_PM_VERSION 0
#define CYIPBLOCK_P3_SCCT_VERSION 0
#define CYIPBLOCK_P3_TIMER_VERSION 0
#define CYIPBLOCK_P3_USB_VERSION 0
#define CYIPBLOCK_P3_VIDAC_VERSION 0
#define CYIPBLOCK_P3_VREF_VERSION 0
#define CYIPBLOCK_S8_GPIO_VERSION 0
#define CYIPBLOCK_S8_IRQ_VERSION 0
#define CYIPBLOCK_S8_SAR_VERSION 0
#define CYIPBLOCK_S8_SIO_VERSION 0
#define CYIPBLOCK_S8_UDB_VERSION 0
#define DMA_CHANNELS_USED__MASK0 0x00000000u
#define CYDEV_BOOTLOADER_ENABLE 0

#endif /* INCLUDED_CYFITTER_H */
