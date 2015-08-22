//===========================================================================
//
// NAME:    pd_boards.h
//
// DESCRIPTION:
//
//          PowerDAQ boards IDs
//
//
// AUTHOR: Frederic Villeneuve 
//         Alex Ivchenko and Sebastian Kuzminski
//
//---------------------------------------------------------------------------
//      Copyright (C) 2000,2004 United Electronic Industries, Inc.
//      All rights reserved.
//---------------------------------------------------------------------------
// For more informations on using and distributing this software, please see
// the accompanying "LICENSE" file.
//
typedef struct _pd_modelstr 
{
    char *modelname;
    unsigned int modelid;
} pd_modelstr;

const pd_modelstr pd_models[] = 

{
// PowerDAQ series ---------------------------
//
// 12-bit multifunction medium speed (330 kHz)
{ "PD-MF-16-330/12L", 0x101 },
{ "PD-MF-16-330/12H", 0x102 },
{ "PD-MF-64-330/12L", 0x103 },
{ "PD-MF-64-330/12H", 0x104 },

// 12-bit multifunction high speed (1 MHz)
{ "PD-MF-16-1M/12L", 0x105 },
{ "PD-MF-16-1M/12H", 0x106 },
{ "PD-MF-64-1M/12L", 0x107 },
{ "PD-MF-64-1M/12H", 0x108 },

// 16-bit multifunction (250 kHz)
{ "PD-MF-16-250/16L", 0x109 },
{ "PD-MF-16-250/16H", 0x10A },
{ "PD-MF-64-250/16L", 0x10B },
{ "PD-MF-64-250/16H", 0x10C },

// 16-bit multifunction (50 kHz)
{ "PD-MF-16-50/16L", 0x10D },
{ "PD-MF-16-50/16H", 0x10E },
                          
// 12-bit, 6-ch 1M SSH   
{ "PD-MFS-6-1M/12", 0x10F  },
{ "PD_Nothing    ", 0x110  },

// PowerDAQ II series -------------------------
//
// 14-bit multifunction low speed (400 kHz)
{ "PD2-MF-16-400/14L", 0x111 },
{ "PD2-MF-16-400/14H", 0x112 },
{ "PD2-MF-64-400/14L", 0x113 },
{ "PD2-MF-64-400/14H", 0x114 },

// 14-bit multifunction medium speed (800 kHz)
{ "PD2-MF-16-800/14L", 0x115 },
{ "PD2-MF-16-800/14H", 0x116 },
{ "PD2-MF-64-800/14L", 0x117 },
{ "PD2-MF-64-800/14H", 0x118 },

// 12-bit multifunction high speed (1.25 MHz)
{ "PD2-MF-16-1M/12L", 0x119 },
{ "PD2-MF-16-1M/12H", 0x11A },
{ "PD2-MF-64-1M/12L", 0x11B },
{ "PD2-MF-64-1M/12H", 0x11C },

// 16-bit multifunction (50 kHz)
{ "PD2-MF-16-50/16L", 0x11D },
{ "PD2-MF-16-50/16H", 0x11E },
{ "PD2-MF-64-50/16L", 0x11F },
{ "PD2-MF-64-50/16H", 0x120 },

// 16-bit multifunction (333 kHz)
{ "PD2-MF-16-333/16L", 0x121 },
{ "PD2-MF-16-333/16H", 0x122 },
{ "PD2-MF-64-333/16L", 0x123 },
{ "PD2-MF-64-333/16H", 0x124 },

// 14-bit multifunction high speed (2.2 MHz)
{ "PD2-MF-16-2M/14L", 0x125 },
{ "PD2-MF-16-2M/14H", 0x126 },
{ "PD2-MF-64-2M/14L", 0x127 },
{ "PD2-MF-64-2M/14H", 0x128 },

{ "PD2-MF-16-500/16L", 0x129 },
{ "PD2-MF-16-500/16H", 0x12A },
{ "PD2-MF-64-500/16L", 0x12B },
{ "PD2-MF-64-500/16H", 0x12C },

// PowerDAQ II SSH section ---------------------
// 14-bit multifunction SSH low speed (500 kHz)
{ "PD2-MFS-4-500/14",   0x12D },
{ "PD2-MFS-8-500/14",   0x12E },
{ "PD2-MFS-4-500/14DG", 0x12F },
{ "PD2-MFS-4-500/14H",  0x130 },
{ "PD2-MFS-8-500/14DG", 0x131 },
{ "PD2-MFS-8-500/14H",  0x132 },

// 14-bit multifunction SSH medium speed (800 kHz)
{ "PD2-MFS-4-800/14",   0x133 },
{ "PD2-MFS-8-800/14",   0x134 },
{ "PD2-MFS-4-800/14DG", 0x135 },
{ "PD2-MFS-4-800/14H",  0x136 },
{ "PD2-MFS-8-800/14DG", 0x137 },
{ "PD2-MFS-8-800/14H",  0x138 },

// 12-bit multifunction SSH high speed (1.25 MHz)
{ "PD2-MFS-4-1M/12",   0x139 },
{ "PD2-MFS-8-1M/12",   0x13A },
{ "PD2-MFS-4-1M/12DG", 0x13B },
{ "PD2-MFS-4-1M/12H",  0x13C },
{ "PD2-MFS-8-1M/12DG", 0x13D },
{ "PD2-MFS-8-1M/12H",  0x13E },

// 14-bit multifunction SSH high speed (2.2 MHz)
{ "PD2-MFS-4-2M/14",   0x13F },
{ "PD2-MFS-8-2M/14",   0x140 },
{ "PD2-MFS-4-2M/14DG", 0x141 },
{ "PD2-MFS-4-2M/14H",  0x142 },
{ "PD2-MFS-8-2M/14DG", 0x143 },
{ "PD2-MFS-8-2M/14H",  0x144 },

// 16-bit multifunction SSH high speed (333 kHz)
{ "PD2-MFS-4-300/16",  0x145 },
{ "PD2-MFS-8-300/16",  0x146 },
{ "PD2-MFS-4-300/16DG",0x147 },
{ "PD2-MFS-4-300/16H", 0x148 },
{ "PD2-MFS-8-300/16DG",0x149 },
{ "PD2-MFS-8-300/16H", 0x14A },

// DIO series
{ "PD2-DIO-64",  0x14B },
{ "PD2-DIO-128", 0x14C },
{ "PD2-DIO-256", 0x14D },

// AO series          
{ "PD2-AO-8/16",  0x14E },
{ "PD2-AO-16/16", 0x14F },
{ "PD2-AO-32/16", 0x150 },
{ "PD2-AO-96/16", 0x151 },

// reserved
{ "PD2-AO-32/16HC", 0x152 },
{ "PD2-AO-32/16HV", 0x153 },
{ "PD2-AO-96/16HS", 0x154 },
{ "PD2-AO-8/16HSG", 0x155 },
{ "PD2-AO-R5", 0x156 },
{ "PD2-AO-R6", 0x157 },
{ "PD2-AO-R7", 0x158 },

// extended DIO series
{ "PD2-DIO-64CE", 0x159 },
{ "PD2-DIO-128CE", 0x15A },
{ "PD2-DIO-64ST", 0x15B },
{ "PD2-DIO-128ST", 0x15C },
{ "PD2-DIO-64HS", 0x15D },
{ "PD2-DIO-128HS", 0x15E },
{ "PD2-DIO-64CT", 0x15F },
{ "PD2-DIO-128CT", 0x160 },

{ "PD2-DIO-64TS", 0x161 },
{ "PD2-DIO-128TS", 0x162 },
{ "PD2-DIO-128I", 0x163 },
{ "PD2-DIO-R3", 0x164 },
{ "PD2-DIO-R4", 0x165 },
{ "PD2-DIO-R5", 0x166 },
{ "PD2-DIO-R6", 0x167 },
{ "PD2-DIO-R7", 0x168 },

// 16-bit multifunction SSH high speed (500 kHz)
{ "PD2-MFS-4-500/16", 0x169 },
{ "PD2-MFS-8-500/16", 0x16A },
{ "PD2-MFS-4-500/16DG", 0x16B },
{ "PD2-MFS-8-500/16H", 0x16C },
{ "PD2-MFS-4-500/16DG", 0x16D },
{ "PD2-MFS-8-500/16H", 0x16E },

{ "PD2-MF-16-150/16L", 0x16F },
{ "PD2-MF-16-150/16H", 0x170 },
{ "PD2-MF-64-150/16L", 0x171 },
{ "PD2-MF-64-150/16H", 0x172 },

// 12-bit multifunction (3 MHz)
{ "PD2-MF-16-3M/12H", 0x173 },
{ "PD2-MF-64-3M/12H", 0x174 },
{ "PD2-MF-16-3M/12L", 0x175 },
{ "PD2-MF-64-3M/12L", 0x176 },

// 12-bit multifunction (3 MHz)
{ "PD2-MF-16-4M/16H", 0x177 },
{ "PD2-MF-64-4M/16H", 0x178 },

// reserved for MF/MFS boards
{ "PD2-MF-R6", 0x179 },
{ "PD2-MF-R7", 0x17A },
{ "PD2-MF-R8", 0x17B },
{ "PD2-MF-R9", 0x17C },
{ "PD2-MF-RA", 0x17D },
{ "PD2-MF-RB", 0x17E },
{ "PD2-MF-RC", 0x17F },
{ "PD2-MF-RD", 0x180 },
{ "PD2-MF-RE", 0x181 },
{ "PD2-MF-RF", 0x182 },

// reserved for LAB board
{ "PDL-MF-16-50/16", 0x183 },
{ "PDL-MF-16-333/16", 0x184 },
{ "PDL-MF-16-160/16", 0x185 },
{ "PDL-MF-16-50/16TSG", 0x186 },

{ "PDL-DIO-64", 0x187 },
{ "PDL-DIO-64ST", 0x188 },
{ "PDL-DIO-64CT", 0x189 },
{ "PDL-DIO-64TS", 0x18A },

{ "PDL-AO-64", 0x18B },
{ "PDL-AO-64-1", 0x18C },
{ "PDL-AO-64-2", 0x18D },
{ "PDL-AO-64-3", 0x18E }

};

const pd_modelstr pdxi_models[] = 
{

// 14-bit multifunction low speed (400 kHz)
{ "PDXI-MF-16-400/14L", 0x211},
{ "PDXI-MF-16-400/14H", 0x212},
{ "PDXI-MF-64-400/14L", 0x213},
{ "PDXI-MF-64-400/14H", 0x214},

// 14-bit multifunction medium speed (800 kHz)
{ "PDXI-MF-16-800/14L", 0x215},
{ "PDXI-MF-16-800/14H", 0x216},
{ "PDXI-MF-64-800/14L", 0x217},
{ "PDXI-MF-64-800/14H", 0x218},

// 12-bit multifunction high speed (1.25 MHz)
{ "PDXI-MF-16-1M/12L", 0x219},
{ "PDXI-MF-16-1M/12H", 0x21A},
{ "PDXI-MF-64-1M/12L", 0x21B},
{ "PDXI-MF-64-1M/12H", 0x21C},

// 16-bit multifunction (50 kHz)
{ "PDXI-MF-16-50/16L", 0x21D},
{ "PDXI-MF-16-50/16H", 0x21E},
{ "PDXI-MF-64-50/16L", 0x21F},
{ "PDXI-MF-64-50/16H", 0x220},

// 16-bit multifunction (333 kHz)
{ "PDXI-MF-16-333/16L", 0x221},
{ "PDXI-MF-16-333/16H", 0x222},
{ "PDXI-MF-64-333/16L", 0x223},
{ "PDXI-MF-64-333/16H", 0x224},

// 14-bit multifunction high speed (2.2 MHz)
{ "PDXI-MF-16-2M/14L", 0x225},
{ "PDXI-MF-16-2M/14H", 0x226},
{ "PDXI-MF-64-2M/14L", 0x227},
{ "PDXI-MF-64-2M/14H", 0x228},

// 16-bit multifunction high speed (500 kHz)
{ "PDXI-MF-16-500/16L", 0x229},
{ "PDXI-MF-16-500/16H", 0x22A},
{ "PDXI-MF-64-500/16L", 0x22B},
{ "PDXI-MF-64-500/16H", 0x22C},

// PowerDAQ II SSH section ---------------------
// 14-bit multifunction SSH low speed (500 kHz)
{ "PDXI-MFS-4-500/14", 0x22D},
{ "PDXI-MFS-8-500/14", 0x22E},
{ "PDXI-MFS-4-500/14DG", 0x22F},
{ "PDXI-MFS-4-500/14H", 0x230},
{ "PDXI-MFS-8-500/14DG", 0x231},
{ "PDXI-MFS-8-500/14H", 0x232},

// 14-bit multifunction SSH medium speed (800 kHz)
{ "PDXI-MFS-4-800/14", 0x233},
{ "PDXI-MFS-8-800/14", 0x234},
{ "PDXI-MFS-4-800/14DG", 0x235},
{ "PDXI-MFS-4-800/14H", 0x236},
{ "PDXI-MFS-8-800/14DG", 0x237},
{ "PDXI-MFS-8-800/14H", 0x238},

// 12-bit multifunction SSH high speed (1.25 MHz)
{ "PDXI-MFS-4-1M/12", 0x239},
{ "PDXI-MFS-8-1M/12", 0x23A},
{ "PDXI-MFS-4-1M/12DG", 0x23B},
{ "PDXI-MFS-4-1M/12H", 0x23C},
{ "PDXI-MFS-8-1M/12DG", 0x23D},
{ "PDXI-MFS-8-1M/12H", 0x23E},

// 14-bit multifunction SSH high speed (2.2 MHz)
{ "PDXI-MFS-4-2M/14", 0x23F},
{ "PDXI-MFS-8-2M/14", 0x240},
{ "PDXI-MFS-4-2M/14DG", 0x241},
{ "PDXI-MFS-4-2M/14H", 0x242},
{ "PDXI-MFS-8-2M/14DG", 0x243},
{ "PDXI-MFS-8-2M/14H", 0x244},

// 16-bit multifunction SSH high speed (333 kHz)
{ "PDXI-MFS-4-300/16", 0x245},
{ "PDXI-MFS-8-300/16", 0x246},
{ "PDXI-MFS-4-300/16DG", 0x247},
{ "PDXI-MFS-4-300/16H", 0x248},
{ "PDXI-MFS-8-300/16DG", 0x249},
{ "PDXI-MFS-8-300/16H", 0x24A},

// DIO series
{ "PDXI-DIO-64", 0x24B},
{ "PDXI-DIO-128" , 0x24C},
{ "PDXI-DIO-256", 0x24D},

// AO series
{ "PDXI-AO-8/16", 0x24E},
{ "PDXI-AO-16/16", 0x24F},
{ "PDXI-AO-32/16", 0x250},

{ "PDXI-AO-R0", 0x251},
{ "PDXI-AO-R1", 0x252},
{ "PDXI-AO-R2", 0x253},
{ "PDXI-AO-R3", 0x254},
{ "PDXI-AO-R4", 0x255},
{ "PDXI-AO-R5", 0x256},
{ "PDXI-AO-R6", 0x257},
{ "PDXI-AO-R7", 0x258},

// extended DIO series
{ "PDXI-DIO-64CE", 0x259},
{ "PDXI-DIO-128CE", 0x25A},

{ "PDXI-DIO-64ST", 0x25B},
{ "PDXI-DIO-128ST", 0x25C},

{ "PDXI-DIO-64ER", 0x25D},
{ "PDXI-DIO-128ER", 0x25E},

{ "PDXI-DIO-64CT", 0x25F},
{ "PDXI-DIO-128CT", 0x260},

{ "PDXI-DIO-64TS", 0x261},
{ "PDXI-DIO-128TS", 0x262},
{ "PDXI-DIO-128I", 0x263},
{ "PDXI-DIO-R3", 0x264},
{ "PDXI-DIO-R4", 0x265},
{ "PDXI-DIO-R5", 0x266},
{ "PDXI-DIO-R6", 0x267},
{ "PDXI-DIO-R7", 0x268},

// 16-bit multifunction SSH high speed (500 kHz)
{ "PDXI-MFS-4-500/16", 0x269},
{ "PDXI-MFS-8-500/16", 0x26A},
{ "PDXI-MFS-4-500/16DG", 0x26B},
{ "PDXI-MFS-4-500/16H", 0x26C},
{ "PDXI-MFS-8-500/16DG", 0x26D},
{ "PDXI-MFS-8-500/16H", 0x26E},

// 16-bit multifunction (200 kHz)
{ "PDXI-MF-16-200/16L", 0x26F},
{ "PDXI-MF-16-200/16H", 0x270},
{ "PDXI-MF-64-200/16L", 0x271},
{ "PDXI-MF-64-200/16H", 0x272},

// 12-bit multifunction (3 MHz)
{ "PDXI-MF-16-3M/12H", 0x273 },
{ "PDXI-MF-64-3M/12H", 0x274 },
{ "PDXI-MF-16-3M/12L", 0x275 },
{ "PDXI-MF-64-3M/12L", 0x276 },

// 12-bit multifunction (3 MHz)
{ "PDXI-MF-16-4M/16H", 0x277 },
{ "PDXI-MF-64-4M/16H", 0x278 },

// reserved for MF/MFS boards
{ "PDXI-MF-R6", 0x279},
{ "PDXI-MF-R7", 0x27A},
{ "PDXI-MF-R8", 0x27B},
{ "PDXI-MF-R9", 0x27C},
{ "PDXI-MF-RA", 0x27D},
{ "PDXI-MF-RB", 0x27E},
{ "PDXI-MF-RC", 0x27F},
{ "PDXI-MF-RD", 0x280},
{ "PDXI-MF-RE", 0x281},
{ "PDXI-MF-RF", 0x282},

// reserved for LAB board
{ "PDXL-MF-16-50/16", 0x283},
{ "PDXL-MF-16-100/16", 0x284},
{ "PDXL-MF-16-160/16", 0x285},
{ "PDXL-MF-16-3", 0x286},

{ "PDXL-DIO-64", 0x287},
{ "PDXL-DIO-64ST", 0x288},
{ "PDXL-DIO-64CT", 0x289},
{ "PDXL-DIO-64TS", 0x28A},

{ "PDXL-AO-64", 0x28B},
{ "PDXL-AO-64-1", 0x28C},
{ "PDXL-AO-64-2", 0x28D},
{ "PDXL-AO-64-3", 0x28E}


};

