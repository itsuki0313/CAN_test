/*******************************************************************************
* File Name: tx_en.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_tx_en_H) /* Pins tx_en_H */
#define CY_PINS_tx_en_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "tx_en_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 tx_en__PORT == 15 && ((tx_en__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    tx_en_Write(uint8 value);
void    tx_en_SetDriveMode(uint8 mode);
uint8   tx_en_ReadDataReg(void);
uint8   tx_en_Read(void);
void    tx_en_SetInterruptMode(uint16 position, uint16 mode);
uint8   tx_en_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the tx_en_SetDriveMode() function.
     *  @{
     */
        #define tx_en_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define tx_en_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define tx_en_DM_RES_UP          PIN_DM_RES_UP
        #define tx_en_DM_RES_DWN         PIN_DM_RES_DWN
        #define tx_en_DM_OD_LO           PIN_DM_OD_LO
        #define tx_en_DM_OD_HI           PIN_DM_OD_HI
        #define tx_en_DM_STRONG          PIN_DM_STRONG
        #define tx_en_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define tx_en_MASK               tx_en__MASK
#define tx_en_SHIFT              tx_en__SHIFT
#define tx_en_WIDTH              1u

/* Interrupt constants */
#if defined(tx_en__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in tx_en_SetInterruptMode() function.
     *  @{
     */
        #define tx_en_INTR_NONE      (uint16)(0x0000u)
        #define tx_en_INTR_RISING    (uint16)(0x0001u)
        #define tx_en_INTR_FALLING   (uint16)(0x0002u)
        #define tx_en_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define tx_en_INTR_MASK      (0x01u) 
#endif /* (tx_en__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define tx_en_PS                     (* (reg8 *) tx_en__PS)
/* Data Register */
#define tx_en_DR                     (* (reg8 *) tx_en__DR)
/* Port Number */
#define tx_en_PRT_NUM                (* (reg8 *) tx_en__PRT) 
/* Connect to Analog Globals */                                                  
#define tx_en_AG                     (* (reg8 *) tx_en__AG)                       
/* Analog MUX bux enable */
#define tx_en_AMUX                   (* (reg8 *) tx_en__AMUX) 
/* Bidirectional Enable */                                                        
#define tx_en_BIE                    (* (reg8 *) tx_en__BIE)
/* Bit-mask for Aliased Register Access */
#define tx_en_BIT_MASK               (* (reg8 *) tx_en__BIT_MASK)
/* Bypass Enable */
#define tx_en_BYP                    (* (reg8 *) tx_en__BYP)
/* Port wide control signals */                                                   
#define tx_en_CTL                    (* (reg8 *) tx_en__CTL)
/* Drive Modes */
#define tx_en_DM0                    (* (reg8 *) tx_en__DM0) 
#define tx_en_DM1                    (* (reg8 *) tx_en__DM1)
#define tx_en_DM2                    (* (reg8 *) tx_en__DM2) 
/* Input Buffer Disable Override */
#define tx_en_INP_DIS                (* (reg8 *) tx_en__INP_DIS)
/* LCD Common or Segment Drive */
#define tx_en_LCD_COM_SEG            (* (reg8 *) tx_en__LCD_COM_SEG)
/* Enable Segment LCD */
#define tx_en_LCD_EN                 (* (reg8 *) tx_en__LCD_EN)
/* Slew Rate Control */
#define tx_en_SLW                    (* (reg8 *) tx_en__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define tx_en_PRTDSI__CAPS_SEL       (* (reg8 *) tx_en__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define tx_en_PRTDSI__DBL_SYNC_IN    (* (reg8 *) tx_en__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define tx_en_PRTDSI__OE_SEL0        (* (reg8 *) tx_en__PRTDSI__OE_SEL0) 
#define tx_en_PRTDSI__OE_SEL1        (* (reg8 *) tx_en__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define tx_en_PRTDSI__OUT_SEL0       (* (reg8 *) tx_en__PRTDSI__OUT_SEL0) 
#define tx_en_PRTDSI__OUT_SEL1       (* (reg8 *) tx_en__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define tx_en_PRTDSI__SYNC_OUT       (* (reg8 *) tx_en__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(tx_en__SIO_CFG)
    #define tx_en_SIO_HYST_EN        (* (reg8 *) tx_en__SIO_HYST_EN)
    #define tx_en_SIO_REG_HIFREQ     (* (reg8 *) tx_en__SIO_REG_HIFREQ)
    #define tx_en_SIO_CFG            (* (reg8 *) tx_en__SIO_CFG)
    #define tx_en_SIO_DIFF           (* (reg8 *) tx_en__SIO_DIFF)
#endif /* (tx_en__SIO_CFG) */

/* Interrupt Registers */
#if defined(tx_en__INTSTAT)
    #define tx_en_INTSTAT            (* (reg8 *) tx_en__INTSTAT)
    #define tx_en_SNAP               (* (reg8 *) tx_en__SNAP)
    
	#define tx_en_0_INTTYPE_REG 		(* (reg8 *) tx_en__0__INTTYPE)
#endif /* (tx_en__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_tx_en_H */


/* [] END OF FILE */
