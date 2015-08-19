//=======================================================================
//
// NAME:    pxi.h
//
// SYNOPSIS:
//
//      Definitions for PXI releated functions
//
//
// DESCRIPTION:
//
//      This file contains definitions for the PXI-releated functions 
//
// AUTHOR: Frederic Villeneuve 
//         Alex Ivchenko
//
//---------------------------------------------------------------------------
//      Copyright (C) 2000,2004 United Electronic Industries, Inc.
//      All rights reserved.
//---------------------------------------------------------------------------
// For more informations on using and distributing this software, please see
// the accompanying "LICENSE" file.
//
//=======================================================================

#ifndef     pxiInput

#define     pxiInput                1      // decimal values!
#define     pxiOutput               10
#define     pxiInOut                100

#define     PXI_TRIG0               0
#define     PXI_TRIG1               1
#define     PXI_TRIG2               2
#define     PXI_TRIG3               3
#define     PXI_TRIG4               4
#define     PXI_TRIG5               5
#define     PXI_TRIG6               6
#define     PXI_TRIG7               7
#define     PXI_STAR                8

#define     MAX_PXI_LINE            9
#define     MAX_PXI_BOARD_LINE      5

typedef struct _PD_BoardPXILines
{
    int     BoardType;                      // MF/MFS  or  DIO 
    char*   LineNames[MAX_PXI_BOARD_LINE];  // Board line names
    int     LineTypes[MAX_PXI_BOARD_LINE];  // pxiInput, pxiOutput, pxiInOut
} PD_BOARD_PXI_LINES, *pPD_BOARD_PXI_LINES;

#define PD_BrdPDXI   0xAFFFF0               // PDXI I/O Register 
                                            // R = RESET State machine, 
                                            // W - set config

// Functions declaration

// Internal functions
PAdapter_Info InternalGetAdapterInfo (int board);
int InternalPXIConnectAllLines(int board);

// Exported functions
PD_BOARD_PXI_LINES* _PdPXIGetAdapterInfo (int board);
int _PdPXISaveSettings (int board);
int _PdPXIRestoreSettings (int board);
int _PdPXIConnect (int board);
int _PdPXIDisconnect (int board);
int _PdPXIConnectLine (int board, u32 dwBoardLine, u32 dwPXILine);
int _PdPXIDisconnectLine (int board, u32 dwBoardLine);
int _PdPXIGetLineState (int board, u32 dwBoardLine);

#endif
