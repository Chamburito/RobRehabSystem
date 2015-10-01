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

#define  PANEL                            1       /* callback function: QuitCallback */
#define  PANEL_GRAPH_2                    2       /* control type: graph, callback function: (none) */
#define  PANEL_GRAPH_1                    3       /* control type: graph, callback function: (none) */
#define  PANEL_OFFSET_TOGGLE              4       /* control type: textButton, callback function: ChangeStateCallback */
#define  PANEL_MOTOR_TOGGLE               5       /* control type: textButton, callback function: ChangeStateCallback */
#define  PANEL_AXIS_SELECTOR              6       /* control type: ring, callback function: (none) */
#define  PANEL_STIFFNESS                  7       /* control type: scale, callback function: ChangeValueCallback */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK ChangeStateCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ChangeValueCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK QuitCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
