//===========================================================================
//
// NAME:    pd_stmem.h
//
// DESCRIPTION:
//
// The floating area of memory.  This memory will be used for preservation of statuses.
// This area of memory is accessible to commands DSPRegRD and DSPRegWR
//
// NOTES:
//
// AUTHOR:  d.k.
//
// DATE:    06-MAR-2003
//
// HISTORY:
//
//      Rev 0.1,    06-MAR-2003,d.k,   Initial version.
//---------------------------------------------------------------------------
//
//      Copyright (C) 2003 United Electronic Industries, Inc.
//      All rights reserved.
//      United Electronic Industries Confidential Information.
//
//===========================================================================

#define ADRAIBM_DEFDMASIZE         0x1D  // Default DMA burst size -1 (bursts>=32 usually not supported by BIOS)
#define ADRAIBM_DEFBURSTS          0x1E  // Default # of BM bursts per one DMA3 transfer
#define ADRAIBM_DEFBURSTSIZE       0x1F  // Default BM burst size (BL field)

#define ADRREALTIMEBMMODE          0x20  // Enable/Disable Real Time Bus master mode

#define ADRRTModeAIBM_TRANSFERSIZE 0x23  // Size of the transmitted data 

#define ADRRRTModeAIBM_SAMPLENUM   0x24  // # samples which is necessary for transferring from the FIFO (only for PDL-MF board)
