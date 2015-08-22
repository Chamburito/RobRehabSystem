//===========================================================================
//
// NAME:    pd_dsp_es.h
//
// DESCRIPTION: PowerDAQ DLL Low-Level Driver 
//		Interface functions. File handle
//
// NOTES:   This is ESSI port register definitions
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
//===========================================================================
//
// General Information
//
// Using of ESSI port require proper initialization procedure.
// Generally at least two control registers, CRA and CRB need to be defined.
// Also generally port C(ESSI0)/D(ESSI1) control/direction/GPIO registers 
// should be set before outputting any data.
// See Motorola DSP56301 User Manual for the details about ESSI programming
// Manual is available for download from http:\\www.mot.com
//
// PD2-DIO ESSI Support
//
// General Information
//
// PowerDAQ DIO board has two Enhanced Synchronous Serial Interface (ESSI).
// The ESSI provides a full-duplex serial port for communicating with a variety of
// serial devices. The ESSI comprises independent transmitter and receiver sections 
// and a common ESSI clock generator. Three transmit shift registers enable it to 
// transmit from three different pins simultaneously. 
//
// Please refer to the Motorola DSP56301 User Manual for the details
//
//
//
//---------------------------------------------------------------------------
//
//       DSP EQUATES for I/O Port Programming
//
//------------------------------------------------------------------------
#define M_PCRC   0xFFFFBF		  // Port C Control Register 
#define M_PRRC	 0xFFFFBE		  // Port C Direction Register 			
#define M_PDRC	 0xFFFFBD		  // Port C GPIO Data Register 	
#define M_PCRC0  0x2F				   
#define M_PRRC0	 0x2F			   			
#define M_PDRC0	 0x2F			   	

#define M_PCRD   0xFFFFAF         // Port D Control Register 
#define M_PRRD	 0xFFFFAE		  // Port D Direction Register 			
#define M_PDRD	 0xFFFFAD		  // Port D GPIO Data Register 	
#define M_PCRD0  0x2F				   
#define M_PRRD0	 0x2F			   			
#define M_PDRD0	 0x2F			   	



//------------------------------------------------------------------------
//
//       EQUATES for Enhanced Synchronous Serial Interface (ESSI)
//
//------------------------------------------------------------------------

//       Register Addresses Of ESSI0 
#define M_TX00   0xFFFFBC         // ESSI0 Transmit Data Register 0
#define M_TX01   0xFFFFBB         // ESSIO Transmit Data Register 1
#define M_TX02   0xFFFFBA         // ESSIO Transmit Data Register 2
#define M_TSR0   0xFFFFB9         // ESSI0 Time Slot Register
#define M_RX0    0xFFFFB8         // ESSI0 Receive Data Register
#define M_SSISR0 0xFFFFB7         // ESSI0 Status Register
#define M_CRB0   0xFFFFB6         // ESSI0 Control Register B
#define M_CRA0   0xFFFFB5         // ESSI0 Control Register A
#define M_TSMA0  0xFFFFB4         // ESSI0 Transmit Slot Mask Register A
#define M_TSMB0  0xFFFFB3         // ESSI0 Transmit Slot Mask Register B
#define M_RSMA0  0xFFFFB2         // ESSI0 Receive Slot Mask Register A
#define M_RSMB0  0xFFFFB1         // ESSI0 Receive Slot Mask Register B

//       Register Addresses Of ESSI1                                        
#define M_TX10   0xFFFFAC         // ESSI1 Transmit Data Register 0
#define M_TX11   0xFFFFAB         // ESSI1 Transmit Data Register 1
#define M_TX12   0xFFFFAA         // ESSI1 Transmit Data Register 2
#define M_TSR1   0xFFFFA9         // ESSI1 Time Slot Register
#define M_RX1    0xFFFFA8         // ESSI1 Receive Data Register
#define M_SSISR1 0xFFFFA7         // ESSI1 Status Register
#define M_CRB1   0xFFFFA6         // ESSI1 Control Register B
#define M_CRA1   0xFFFFA5         // ESSI1 Control Register A
#define M_TSMA1  0xFFFFA4         // ESSI1 Transmit Slot Mask Register A
#define M_TSMB1  0xFFFFA3         // ESSI1 Transmit Slot Mask Register B
#define M_RSMA1  0xFFFFA2         // ESSI1 Receive Slot Mask Register A
#define M_RSMB1  0xFFFFA1         // ESSI1 Receive Slot Mask Register B

//       ESSI Control Register A Bit Flags
#define M_PM     0xFF             // Prescale Modulus Select Mask (PM0-PM7)              
#define M_PM0    0
#define M_PSR    11               // Prescaler Range       
#define M_DC     0x1F000          // Frame Rate Divider Control Mask (DC0-DC7)
#define M_DC0    12 
#define M_ALC    18               // Alignment Control (ALC)
#define M_WL     0x380000         // Word Length Control Mask (WL0-WL7)
#define M_WL0    19
#define M_SSC1   22               // Select SC1 as TR #0 drive enable (SSC1)

//       ESSI Control Register B Bit Flags                                 
#define M_OF     0x3              // Serial Output Flag Mask
#define M_OF0    0                // Serial Output Flag 0                     
#define M_OF1    1                // Serial Output Flag 1                     
#define M_SCD    0x1C             // Serial Control Direction Mask            
#define M_SCD0   2                // Serial Control 0 Direction                
#define M_SCD1   3                // Serial Control 1 Direction               
#define M_SCD2   4                // Serial Control 2 Direction               
#define M_SCKD   5                // Clock Source Direction
#define M_SHFD   6                // Shift Direction                          
#define M_FSL    0x180            // Frame Sync Length Mask (FSL0-FSL1)
#define M_FSL0   7                // Frame Sync Length 0
#define M_FSL1   8                // Frame Sync Length 1
#define M_FSR    9                // Frame Sync Relative Timing
#define M_FSP    10               // Frame Sync Polarity
#define M_CKP    11               // Clock Polarity                           
#define M_SYN    12               // Sync/Async Control                       
#define M_MOD    13               // ESSI Mode Select
#define M_SSTE   0x1C000          // ESSI Transmit enable Mask                  
#define M_SSTE2  14               // ESSI Transmit #2 Enable                   
#define M_SSTE1  15               // ESSI Transmit #1 Enable                    
#define M_SSTE0  16               // ESSI Transmit #0 Enable                    
#define M_SSRE   17               // ESSI Receive Enable                       
#define M_SSTIE  18               // ESSI Transmit Interrupt Enable            
#define M_SSRIE  19               // ESSI Receive Interrupt Enable              
#define M_STLIE  20               // ESSI Transmit Last Slot Interrupt Enable 
#define M_SRLIE  21               // ESSI Receive Last Slot Interrupt Enable 
#define M_STEIE  22               // ESSI Transmit Error Interrupt Enable 
#define M_SREIE  23               // ESSI Receive Error Interrupt Enable              

//       ESSI Status Register Bit Flags                                       
#define M_IF     0x3              // Serial Input Flag Mask           
#define M_IF0    0                // Serial Input Flag 0                      
#define M_IF1    1                // Serial Input Flag 1                      
#define M_TFS    2                // Transmit Frame Sync Flag                 
#define M_RFS    3                // Receive Frame Sync Flag                  
#define M_TUE    4                // Transmitter Underrun Error FLag          
#define M_ROE    5                // Receiver Overrun Error Flag              
#define M_TDE    6                // Transmit Data Register Empty             
#define M_RDF    7                // Receive Data Register Full

//       ESSI Transmit Slot Mask Register A
#define M_SSTSA  0xFFFF           // ESSI Transmit Slot Bits Mask A (TS0-TS15)

//       ESSI Transmit Slot Mask Register B
#define M_SSTSB  0xFFFF           // ESSI Transmit Slot Bits Mask B (TS16-TS31)

//       ESSI Receive Slot Mask Register A
#define M_SSRSA  0xFFFF           // ESSI Receive Slot Bits Mask A (RS0-RS15)
 
//       ESSI Receive Slot Mask Register B
#define M_SSRSB  0xFFFF           // ESSI Receive Slot Bits Mask B (RS16-RS31)


// 	 Word Length Control Mask.  CRA[21:19]
#define M_WL_MASK0     0x00000000        
#define M_WL_MASK1     0x00080000
#define M_WL_MASK2     0x00100000
#define M_WL_MASK3     0x00180000


// ESSI Direct access commands:
int _PdEssiReset(int hAdapter);
int _PdEssiRegWrite(int hAdapter, DWORD dwReg, DWORD dwValue);
int _PdEssiRegRead(int hAdapter, DWORD dwReg, DWORD *pdwValue);
int _PdEssi0PinsEnable(int hAdapter);
int _PdEssi1PinsEnable(int hAdapter);
