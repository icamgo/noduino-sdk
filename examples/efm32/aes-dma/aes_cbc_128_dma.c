/***************************************************************************//**
 * @file aes_cbc_128_dma.c
 * @brief AES CBC 128-bit DMA driven functions for EFM32
 * @author Silicon Labs
 * @version 1.11
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdbool.h>
#include "em_device.h"
#include "em_dma.h"
#include "em_cmu.h"
#include "aes_cbc_128_dma.h"
//#include "dmactrl.h"

/* The channel numbers can be changed, but the order must be the same */
#define DMA_CH_WRITEPREVDATA   0
#define DMA_CH_READDATA        1
#define DMA_CH_WRITEDATA       2

#define DMA_MAX_TRANSFERS   1024

/* DMA callback structures */
DMA_CB_TypeDef encryptCb;
DMA_CB_TypeDef decryptCb;

static uint16_t          numberOfBlocks;
static uint16_t          blockIndex;
static uint32_t*         outputDataG;
static uint32_t*         inputDataG;
static const uint32_t*   keyG;
static const uint32_t*   ivG;
static volatile bool     aesFinished;
static uint32_t          wordTransfers;

/**************************************************************************//**
 * @brief  Call-back called when DMA encryption read data transfer is done
 * All DMA channels are refreshed using this callback, until no more data is
 * left to encrypt. Up to DMA_MAX_TRANSFERS transfer are performed between
 * calls to this function. 
 *****************************************************************************/
static void dmaEncryptDataReadDone(unsigned int channel, bool primary, void *user)
{ 
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Update block index */
  blockIndex=blockIndex+wordTransfers/4;
  
  if (blockIndex < numberOfBlocks)
  {
    /* Calculate how many word transfers to set up the DMA for */
    wordTransfers = (numberOfBlocks - blockIndex)*4;
    if (wordTransfers > DMA_MAX_TRANSFERS)
      wordTransfers = DMA_MAX_TRANSFERS;
    
    /* Re-activate the data read DMA channel */
    DMA_ActivateBasic(DMA_CH_READDATA,
                      true,
                      false,
                      (void *)&outputDataG[blockIndex*4], /* Destination address */
                      (void *)&(AES->DATA),               /* Source address */
                      wordTransfers - 1);                 /* Number of transfers - 1 */ 
    
  /* Re-activate the data write DMA channel */
    DMA_ActivateBasic(DMA_CH_WRITEDATA,
                      true,
                      false,
                      (void *)&(AES->XORDATA),           /* Destination address */
                      (void *)&inputDataG[blockIndex*4], /* Source address */
                      wordTransfers - 1);                /* Number of transfers - 1 */
  }
  
  else
  { 
    /* Indicate last block */
    aesFinished = true;
  }
    
}

/**************************************************************************//**
 * @brief  Call-back called when DMA decryption read data transfer is done
 * All DMA channels are refreshed using this callback, until no more data is
 * left to decrypt. Up to DMA_MAX_TRANSFERS transfer are performed between
 * calls to this function. 
 *****************************************************************************/
static void dmaDecryptDataReadDone(unsigned int channel, bool primary, void *user)
{ 
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Update block index */  
  blockIndex=blockIndex+wordTransfers/4;
  
  if (blockIndex < numberOfBlocks)
  {
    /* Calculate how many word transfers to set up the DMA for */
    wordTransfers = (numberOfBlocks - blockIndex)*4;
    if (wordTransfers > DMA_MAX_TRANSFERS)
      wordTransfers = DMA_MAX_TRANSFERS;
    
    /* Re-activate the previous data write DMA channel */
    DMA_ActivateBasic(DMA_CH_WRITEPREVDATA,
                      true,
                      false,
                      (void *)&(AES->XORDATA),             /* Destination address */
                      (void *)&inputDataG[blockIndex*4-4], /* Source address */
                      wordTransfers - 1);                  /* Number of transfers - 1 */
    
    /* Re-activate the data read DMA channel */
    DMA_ActivateBasic(DMA_CH_READDATA,
                      true,
                      false,
                      (void *)&outputDataG[blockIndex*4], /* Destination address */
                      (void *)&(AES->DATA),               /* Source address */
                      wordTransfers - 1);                 /* Number of transfers - 1 */
    
    /* Re-activate the data write DMA channel */
    DMA_ActivateBasic(DMA_CH_WRITEDATA,
                      true,
                      false, 
                      (void *)&(AES->DATA),              /* Destination address */
                      (void *)&inputDataG[blockIndex*4], /* Source address */
                      wordTransfers - 1);                /* Number of transfers - 1 */
  }
  
  else
  { 
    /* Indicate last block */
    aesFinished = true;
  }
    
}



/**************************************************************************//**
 * @brief  Configure DMA for encryption
 * Two DMA channels are used:
 * - Read output data from AES->DATA
 * - Write new data to AES->XORDATA
 * Both DMA requests are set when an AES encryption finishes.
 *****************************************************************************/
static void setupEncryptDma(void)
{ 
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;
  
  /* Clearing AES DMA requests by writing AES->CTRL */
  AES->CTRL = AES->CTRL;  
  
  /* Enable DMA clock */
  CMU_ClockEnable(cmuClock_DMA, true);

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
    
  /* Setting up call-back function for read data */ 
  encryptCb.cbFunc  = dmaEncryptDataReadDone;
  encryptCb.userPtr = NULL;

  /* Setting up data read channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_AES_DATARD;  /* DMA request cleared by AES->DATA reads */
  chnlCfg.cb        = &encryptCb;
  DMA_CfgChannel(DMA_CH_READDATA, &chnlCfg);
  
  /* Setting up data read channel descriptor */
  descrCfg.dstInc  = dmaDataInc4;    /* Word destination address increase */
  descrCfg.srcInc  = dmaDataIncNone; /* No source address increase */
  descrCfg.size    = dmaDataSize4;   /* Word transfers */
  descrCfg.arbRate = dmaArbitrate4;  /* Arbitrate after 4 transfers */
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CH_READDATA, true, &descrCfg);
  
  /* Activate DMA channel */
  DMA_ActivateBasic(DMA_CH_READDATA,
                    true,
                    false,
                    (void *)&outputDataG[0], /* Destination address */
                    (void *)&(AES->DATA),    /* Source address */
                    wordTransfers - 1);      /* Number of transfers - 1 */
  
  /* Setting up data write channel */
  /* No callback used as channel is handled in the read channel callback */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_AES_XORDATAWR; /* DMA request cleared by AES->XORDATA writes */
  chnlCfg.cb        = NULL;
  DMA_CfgChannel(DMA_CH_WRITEDATA, &chnlCfg);
  
  /* Setting up data write channel descriptor */
  descrCfg.dstInc  = dmaDataIncNone;  /* No destination address increase */
  descrCfg.srcInc  = dmaDataInc4;     /* Word source address increase */
  descrCfg.size    = dmaDataSize4;    /* Word transfers */
  descrCfg.arbRate = dmaArbitrate4;   /* Arbitrate after 4 transfers */
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CH_WRITEDATA, true, &descrCfg);
  
  /* Activate DMA channel */
  DMA_ActivateBasic(DMA_CH_WRITEDATA,
                    true,
                    false,
                    (void *)&(AES->XORDATA), /* Destination address */
                    (void *)&inputDataG[0],  /* Source address */
                    wordTransfers - 1);      /* Number of transfers - 1 */
}

/**************************************************************************//**
 * @brief  Configure DMA for decryption
 * Three DMA channels are used:
 * - Read output data from AES->DATA
 * - Write new data to AES->DATA
 * - Write previous data to AES->XORDATA
 * All DMA requests are set when an AES decryption finishes.
 *****************************************************************************/
static void setupDecryptDma(void)
{ 
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;
  
  /* Clearing AES DMA requests by writing AES->CTRL */
  AES->CTRL = AES->CTRL;  
  
  /* Enable DMA clock */
  CMU_ClockEnable(cmuClock_DMA, true);

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
    
  /* Setting up call-back function for read data */  
  decryptCb.cbFunc  = dmaDecryptDataReadDone;
  decryptCb.userPtr = NULL;

  /* Setting up data read channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_AES_DATARD; /* DMA request cleared by AES->DATA reads */
  chnlCfg.cb        = &decryptCb;
  DMA_CfgChannel(DMA_CH_READDATA, &chnlCfg);
  
  /* Setting up data read channel descriptor */
  descrCfg.dstInc  = dmaDataInc4;    /* Word destination address increase */
  descrCfg.srcInc  = dmaDataIncNone; /* No source address increase */
  descrCfg.size    = dmaDataSize4;   /* Word transfers */
  descrCfg.arbRate = dmaArbitrate4;  /* Arbitrate after 4 transfers */
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CH_READDATA, true, &descrCfg);
  
  /* Activate DMA channel */
  DMA_ActivateBasic(DMA_CH_READDATA,
                    true,
                    false,
                    (void *)&outputDataG[0], /* Destination address */
                    (void *)&(AES->DATA),    /* Source address */
                    3);                      /* Only process first block */
  
  /* Setting up data write channel */
  /* No callback used as channel is handled in the read channel callback */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_AES_DATAWR; /* DMA request cleared by AES->DATA writes */
  chnlCfg.cb        = NULL;
  DMA_CfgChannel(DMA_CH_WRITEDATA, &chnlCfg);
  
  /* Setting up data write channel descriptor */
  descrCfg.dstInc  = dmaDataIncNone;   /* No destination address increase */
  descrCfg.srcInc  = dmaDataInc4;      /* Word source address increase */
  descrCfg.size    = dmaDataSize4;     /* Word transfers */
  descrCfg.arbRate = dmaArbitrate4;    /* Arbitrate after 4 transfers */
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CH_WRITEDATA, true, &descrCfg);
  
  /* Activate DMA channel */
  DMA_ActivateBasic(DMA_CH_WRITEDATA,
                    true,
                    false,
                    (void *)&(AES->DATA),    /* Destination address */
                    (void *)&inputDataG[0],  /* Source address */
                    3);                      /* Only process first block */
    
  /* Setting up previous data write channel */
  /* No callback used as channel is handled in the read channel callback */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_AES_XORDATAWR;  /* DMA request cleared by AES->XORDATA writes */
  chnlCfg.cb        = NULL;
  DMA_CfgChannel(DMA_CH_WRITEPREVDATA, &chnlCfg);
  
  /* Setting up previous data write channel descriptor */
  descrCfg.dstInc  = dmaDataIncNone;  /* No destination address increase */
  descrCfg.srcInc  = dmaDataInc4;     /* Word source address increase */
  descrCfg.size    = dmaDataSize4;    /* Word transfers */
  descrCfg.arbRate = dmaArbitrate4;   /* Arbitrate after 4 transfers */
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CH_WRITEPREVDATA, true, &descrCfg);
  
  /* Activate DMA channel */
  DMA_ActivateBasic(DMA_CH_WRITEPREVDATA,
                    true,
                    false,
                    (void *)&(AES->XORDATA),  /* Destination address */
                    (void *)&ivG[0],          /* Source address */
                    3);                       /* Only process first block */
  
}


/**************************************************************************//**
 * @brief  Decrypt with 128-bit key in CBC mode. Function returns after
 * initializing decryption. Use AesFinished() to check for 
 * completion. Output data only valid after completion. 
 *
 * @param[in] key
 *   This is the 128-bit decryption key. The decryption key may
 *   be generated from the encryption key with AES_DecryptKey128().
 *
 * @param[in] inputData
 *   Buffer holding data to decrypt.
 *
 * @param[out] outputData
 *   Buffer to put output data, must be of size blockNumber*16 bytes. This
 *   buffer can NOT be the same as inputData.
 *
 * @param[in] blockNumber
 *   Number of 128-bit blocks to decrypt.
 *
 * @param[in] iv
 *   128-bit initialization vector to use
 *****************************************************************************/
void AesCBC128DmaDecrypt(const uint8_t* key,
                         uint8_t*       inputData,
                         uint8_t*       outputData,
                         const uint32_t blockNumber,
                         const uint8_t* iv)
{
  int i;
  
  /* Copy to global variables */
  inputDataG     = (uint32_t*) inputData;
  outputDataG    = (uint32_t*) outputData;
  keyG           = (uint32_t*) key;
  ivG            = (uint32_t*) iv;
  numberOfBlocks = blockNumber;

  /* Reset block index */
  blockIndex = 0;
  
  /* Initialize finished flag */
  aesFinished = false;
  
  /* Set number of words to transfer in first DMA round */
  wordTransfers = 4;
    
  /* Setup DMA for decryption*/
  setupDecryptDma();
    
  /* Configure AES module */
  AES->CTRL = AES_CTRL_DECRYPT |     /* Set decryption */
              AES_CTRL_BYTEORDER |   /* Set byte order */
              AES_CTRL_DATASTART;    /* Start decryption on data write */  
    
  /* Write the KEY to AES module */
  /* Writing only the KEYLA 4 times will transfer the key to all 4
  * high key registers, used here, in 128 bit key mode, as buffers */   
  for (i = 0; i <= 3; i++)
  {
    AES->KEYLA = keyG[i];
  }  
  
  /* Start DMA by software trigger to write first data */
  DMA->CHSWREQ = 1 << DMA_CH_WRITEDATA;
}

/**************************************************************************//**
 * @brief  Encrypt with 128-bit key in CBC mode. Function returns after
 * initializing encryption. Use AesFinished() to check for 
 * completion. Output data only valid after completion. 
 *
 * @param[in] key
 *   This is the 128-bit encryption key. 
 *
 * @param[in] inputData
 *   Buffer holding data to encrypt.
 *
 * @param[out] outputData
 *   Buffer to put output data, must be of size blockNumber*16 bytes. This can
 *   be the same location as inputData
 *
 * @param[in] blockNumber
 *   Number of 128-bit blocks to encrypt.
 *
 * @param[in] iv
 *   128-bit initialization vector to use
 *****************************************************************************/
void AesCBC128DmaEncrypt(const uint8_t* key,
                         uint8_t*       inputData,
                         uint8_t*       outputData,
                         const uint32_t blockNumber,
                         const uint8_t* iv)
{
  int i;
  
  /* Copy to global variables */
  inputDataG     = (uint32_t*) inputData;
  outputDataG    = (uint32_t*) outputData;
  keyG           = (uint32_t*) key;
  ivG            = (uint32_t*) iv;
  numberOfBlocks = blockNumber;

  /* Reset block index */
  blockIndex = 0;
  
  /* Initialize finished flag */
  aesFinished = false;

  /* Set number of words to transfer in first DMA round */
  wordTransfers = numberOfBlocks*4;
  if (wordTransfers > DMA_MAX_TRANSFERS)
    wordTransfers = DMA_MAX_TRANSFERS;
    
  /* Setup DMA for encryption*/
  setupEncryptDma();
    
  AES->CTRL = AES_CTRL_BYTEORDER |    /* Set byte order */
              AES_CTRL_XORSTART;      /* Start encryption on data write */
    
  /* Write the KEY to AES module */
  /* Writing only the KEYLA 4 times will transfer the key to all 4
  * high key registers, used here, in 128 bit key mode, as buffers */   
  for (i = 0; i <= 3; i++)
  {
    AES->KEYLA = keyG[i];
  }

  /* Writing to the DATA register here does NOT trigger encryption */     
  for (i = 0; i <= 3; i++)
  {
    AES->DATA = ivG[i];
  }
  
  /* Start DMA by software trigger to write first data */
  DMA->CHSWREQ = 1 << DMA_CH_WRITEDATA;
}

/**************************************************************************//**
 * @brief  Function to check if AES process has finished
 * @return true if AES operation has finished
 *****************************************************************************/
bool AesFinished(void)
{
  return aesFinished;
}



