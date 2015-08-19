// PrintH.h - definition file
//
// Copyright (C) 2000 UEI, Inc. by Alex Ivchenko

#define HercAddrBase 0xb0000    // Hercules text mode buffer
#define HERCULES_LINES    25    // default lines
#define HERCULES_LINES43  43    // 8x8 char lines
#define HERCULES_COLUMNS  80    // default columns

#define HGCMON_NODISPLAY   0    // attribute field
#define HGCMON_UNDERLINE   1    // normal + underline
#define HGCMON_NORMAL      2    // normal
#define HGCMON_INTENSE    10    // for high intense symbols

// Viewport - where to output text
// Default viewport is a full screen
typedef struct _VIEWPORT
{
    int x1, y1;     // viewport start coords
    int x2, y2;     // viewport end coords
    int xc,yc;      // viewport current cursor position
    int tab;        // tabulation
    int vlines,     // do not fill - number of lines in viewport
        vcolumns,   // do not fill - number of columns in viewport
        linebytes,  // do not fill - number of bytes in the line
        offset;     // do not fill - first line offset in the buffer
    u8  attrib;     // character attribute
                
} VIEWPORT, *PVIEWPORT;

// imported from kernel
extern int vsprintf(char *buf, const char *, va_list);

// function prototypes to work with viewports on Hercules screen
// default viewport is full screen in 25x80 mode

void hercmon_scroll(int lines);             // lines = 0 to clear all

void hercmon_printf(const char *fmt,...);   // just like printf

void hercmon_vp_set_cursor(int x, int y);   // set cursor position

void hercmon_vp_set(int x1, int y1, int x2, int y2); // set up viewport size

void hercmon_vp_set_attrib(int attribute);  // sets up char attribute

void hercmon_vp_setptr(PVIEWPORT pVP);      // fill and pass viewport to use


