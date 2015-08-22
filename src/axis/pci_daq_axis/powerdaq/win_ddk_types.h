//===========================================================================
//
// NAME:    win_ddk_types.h
//
// DESCRIPTION:
//
// Windows DDK types conversion
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


#ifndef __WIN_DDK_TYPES
#define __WIN_DDK_TYPES

#define ULONG uint32_t
#define DWORD uint32_t
#define USHORT uint16_t
#define WORD uint16_t
#define UCHAR uint8_t
#define BYTE UCHAR
#define PBYTE BYTE*
#define PDWORD DWORD*
#define PWORD WORD*
#define PULONG ULONG*
#define PUSHORT USHORT*
#define PUCHAR UCHAR*
#define HANDLE void*
#define PHANDLE HANDLE*
#define BOOLEAN int

#endif //__WIN_DDK_TYPES

