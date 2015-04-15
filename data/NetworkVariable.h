/**************************************************************************/
/* LabWindows/CVI User Interface Resource (UIR) Include File              */
/*                                                                        */
/* WARNING: Do not add to, delete from, or otherwise modify the contents  */
/*          of this include file.                                         */
/**************************************************************************/

#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

     /* Panels and Controls: */

#define  PANEL                            1
#define  PANEL_GRAPH_EMG                  2       /* control type: graph, callback function: (none) */
#define  PANEL_GRAPH_POSITION             3       /* control type: graph, callback function: (none) */
#define  PANEL_QUITBUTTON                 4       /* control type: command, callback function: QuitCallback */
#define  PANEL_TOGGLEBUTTON_2             5       /* control type: textButton, callback function: (none) */
#define  PANEL_TOGGLEBUTTON               6       /* control type: textButton, callback function: (none) */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK QuitCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
