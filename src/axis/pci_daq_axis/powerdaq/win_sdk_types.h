/*==========================================================================*/
/*                                                                          */
/* NAME:    win_sdk_types.h                                                 */
/*                                                                          */
/* DESCRIPTION:                                                             */
/*                                                                          */
/* Windows SDK types conversion                                             */
/*                                                                          */
/* AUTHOR:  Alex Ivchenko                                                   */
/*                                                                          */
/* AUTHOR: Frederic Villeneuve                                              */
/*         Alex Ivchenko                                                    */
/*                                                                          */
/*--------------------------------------------------------------------------*/
/*      Copyright (C) 2000,2004 United Electronic Industries, Inc.          */
/*      All rights reserved.                                                */
/*--------------------------------------------------------------------------*/
/* For more informations on using and distributing this software, please see*/
/* the accompanying "LICENSE" file.                                         */
/*==========================================================================*/

#ifndef __WIN_SDK_TYPES
#define __WIN_SDK_TYPES

typedef signed char i8;
typedef unsigned char u8;
typedef signed short i16;
typedef unsigned short u16;
typedef signed int i32;
typedef unsigned int u32;
typedef u8 BYTE;
typedef u16 WORD;
typedef u32 DWORD;
typedef DWORD BOOL;
typedef DWORD* PDWORD;
typedef WORD* PWORD;
typedef BYTE* PBYTE;
typedef DWORD ULONG;
typedef DWORD* PULONG;
typedef DWORD HANDLE;
typedef DWORD handle;
typedef u8 UCHAR;
typedef u16 USHORT;
typedef i8* LPSTR;


#endif /*__WIN_SDK_TYPES */

