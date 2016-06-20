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
#define  PANEL_SAMPLE_TOGGLE              4       /* control type: textButton, callback function: ChangeStateCallback */
#define  PANEL_CAL_TOGGLE                 5       /* control type: textButton, callback function: ChangeStateCallback */
#define  PANEL_OFFSET_TOGGLE              6       /* control type: textButton, callback function: ChangeStateCallback */
#define  PANEL_MOTOR_TOGGLE               7       /* control type: textButton, callback function: ChangeStateCallback */
#define  PANEL_STIFFNESS_SLIDER           8       /* control type: scale, callback function: ChangeValueCallback */
#define  PANEL_CONNECT_BUTTON             9       /* control type: command, callback function: ConnectCallback */
#define  PANEL_AXIS_STRING                10      /* control type: string, callback function: (none) */
#define  PANEL_JOINT_STRING               11      /* control type: string, callback function: (none) */
#define  PANEL_MEASURE_SLIDER             12      /* control type: scale, callback function: (none) */
#define  PANEL_SETPOINT_SLIDER            13      /* control type: scale, callback function: (none) */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK ChangeStateCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ChangeValueCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ConnectCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK QuitCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
