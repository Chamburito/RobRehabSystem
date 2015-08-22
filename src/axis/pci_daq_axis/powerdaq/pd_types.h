/*===========================================================================*/
/*                                                                           */
/* NAME:    pd_types.h                                                       */
/*                                                                           */
/* DESCRIPTION:                                                              */
/*                                                                           */
/* PowerDAQ basic types                                                      */
/*                                                                           */
/* AUTHOR: Frederic Villeneuve                                               */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*      Copyright (C) 2000,2004 United Electronic Industries, Inc.           */
/*      All rights reserved.                                                 */
/*---------------------------------------------------------------------------*/
/* For more informations on using and distributing this software, please see */
/* the accompanying "LICENSE" file.                                          */
/*===========================================================================*/


#ifndef __PD_TYPES__
#define __PD_TYPES__

#ifndef u8
   #define u8 unsigned char
#endif
#ifndef u16
   #define u16 unsigned short
#endif
#ifndef u32
   #define u32 unsigned int  
#endif
#ifndef u64
   #define u64 unsigned long long  
#endif
#ifndef uint8_t
   #define uint8_t unsigned char
#endif
#ifndef uint16_t
   #define uint16_t unsigned short
#endif
#ifndef uint32_t
   #define uint32_t unsigned int
#endif
#ifndef uint64_t
   #define uint64_t unsigned long long
#endif
#ifndef i8
   #define i8 char
#endif
#ifndef i16
   #define i16 short
#endif
#ifndef i32
   #define i32 int  
#endif
#ifndef i64
   #define i64 long long 
#endif
#ifndef int8_t
   #define int8_t char
#endif
#ifndef int16_t
   #define int16_t short
#endif
#ifndef int32_t
   #define int32_t int
#endif
#ifndef int64_t
   #define int64_t long long
#endif
#ifndef ptr_t
   #define ptr_t u32
#endif
#ifndef dma_addr_t
   #if defined(__x86_64__) || defined(CONFIG_HIGHMEM64G) || defined(CONFIG_ARCH_DMA_ADDR_T_64BIT)
      #define dma_addr_t u64
   #elif defined(__i386__)
      #define dma_addr_t u32
   #endif
#endif
#ifndef ULONG
   #define ULONG unsigned int
#endif
#ifndef DWORD
   #define DWORD unsigned int 
#endif
#ifndef USHORT
   #define USHORT unsigned short
#endif
#ifndef WORD
   #define WORD unsigned short
#endif
#ifndef UCHAR
   #define UCHAR unsigned char
#endif
#ifndef BYTE
   #define BYTE char
#endif
#ifndef PBYTE
   #define PBYTE BYTE*
#endif
#ifndef LPSTR
   #define LPSTR char*
#endif
#ifndef PDWORD
   #define PDWORD DWORD*
#endif
#ifndef PWORD
   #define PWORD WORD*
#endif
#ifndef PULONG
   #define PULONG ULONG*
#endif
#ifndef PUSHORT
   #define PUSHORT USHORT*
#endif
#ifndef PUCHAR
   #define PUCHAR UCHAR*
#endif
#ifndef HANDLE
   #define HANDLE unsigned int
#endif
#ifndef PHANDLE
   #define PHANDLE HANDLE*
#endif
#ifndef BOOLEAN
   #define BOOLEAN unsigned int
#endif
#ifndef BOOL
   #define BOOL int
#endif
#ifndef TRUE
   #define TRUE 1
#endif
#ifndef true
   #define true 1
#endif
#ifndef FALSE
   #define FALSE 0
#endif
#ifndef false
   #define false 0
#endif
#ifndef INVALID_HANDLE_VALUE
   #define INVALID_HANDLE_VALUE 0xFFFFFFFF
#endif
#ifndef INFINITE
   #define INFINITE 0x7FFFFFFF
#endif



#endif /*__PD_TYPES___*/
