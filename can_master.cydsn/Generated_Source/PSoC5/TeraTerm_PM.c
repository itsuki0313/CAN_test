/*******************************************************************************
* File Name: TeraTerm_PM.c
* Version 2.50
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
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


/***************************************
* Local data allocation
***************************************/

static TeraTerm_BACKUP_STRUCT  TeraTerm_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: TeraTerm_SaveConfig
********************************************************************************
*
* Summary:
*  This function saves the component nonretention control register.
*  Does not save the FIFO which is a set of nonretention registers.
*  This function is called by the TeraTerm_Sleep() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  TeraTerm_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void TeraTerm_SaveConfig(void)
{
    #if(TeraTerm_CONTROL_REG_REMOVED == 0u)
        TeraTerm_backup.cr = TeraTerm_CONTROL_REG;
    #endif /* End TeraTerm_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: TeraTerm_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the nonretention control register except FIFO.
*  Does not restore the FIFO which is a set of nonretention registers.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  TeraTerm_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
* Notes:
*  If this function is called without calling TeraTerm_SaveConfig() 
*  first, the data loaded may be incorrect.
*
*******************************************************************************/
void TeraTerm_RestoreConfig(void)
{
    #if(TeraTerm_CONTROL_REG_REMOVED == 0u)
        TeraTerm_CONTROL_REG = TeraTerm_backup.cr;
    #endif /* End TeraTerm_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: TeraTerm_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. 
*  The TeraTerm_Sleep() API saves the current component state. Then it
*  calls the TeraTerm_Stop() function and calls 
*  TeraTerm_SaveConfig() to save the hardware configuration.
*  Call the TeraTerm_Sleep() function before calling the CyPmSleep() 
*  or the CyPmHibernate() function. 
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  TeraTerm_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void TeraTerm_Sleep(void)
{
    #if(TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED)
        if((TeraTerm_RXSTATUS_ACTL_REG  & TeraTerm_INT_ENABLE) != 0u)
        {
            TeraTerm_backup.enableState = 1u;
        }
        else
        {
            TeraTerm_backup.enableState = 0u;
        }
    #else
        if((TeraTerm_TXSTATUS_ACTL_REG  & TeraTerm_INT_ENABLE) !=0u)
        {
            TeraTerm_backup.enableState = 1u;
        }
        else
        {
            TeraTerm_backup.enableState = 0u;
        }
    #endif /* End TeraTerm_RX_ENABLED || TeraTerm_HD_ENABLED*/

    TeraTerm_Stop();
    TeraTerm_SaveConfig();
}


/*******************************************************************************
* Function Name: TeraTerm_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when 
*  TeraTerm_Sleep() was called. The TeraTerm_Wakeup() function
*  calls the TeraTerm_RestoreConfig() function to restore the 
*  configuration. If the component was enabled before the 
*  TeraTerm_Sleep() function was called, the TeraTerm_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  TeraTerm_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void TeraTerm_Wakeup(void)
{
    TeraTerm_RestoreConfig();
    #if( (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) )
        TeraTerm_ClearRxBuffer();
    #endif /* End (TeraTerm_RX_ENABLED) || (TeraTerm_HD_ENABLED) */
    #if(TeraTerm_TX_ENABLED || TeraTerm_HD_ENABLED)
        TeraTerm_ClearTxBuffer();
    #endif /* End TeraTerm_TX_ENABLED || TeraTerm_HD_ENABLED */

    if(TeraTerm_backup.enableState != 0u)
    {
        TeraTerm_Enable();
    }
}


/* [] END OF FILE */
