/*!
   \file nixnet.h
   \brief NI-XNET C API Implementation
*/
/*
   Copyright (c) 2008 National Instruments Corporation.
   All rights reserved.
*/

#ifndef ___nixnet_h___
#define ___nixnet_h___

/***********************************************************************
                     C O M P I L E R   D E F I N E S
***********************************************************************/

   /* Needed for C++ to C (DLL) calls  */
#ifdef __cplusplus
   extern "C" {
#endif

   /* Used for function prototypes */
#ifdef _NX_NOT_API_LIBRARY_
   #define  _NXFUNC
#else
   #define  _NXFUNC   __declspec(dllexport) __cdecl
#endif

#ifdef _CVI_
   #pragma  EnableLibraryRuntimeChecking
#endif /* _CVI_ */

/***********************************************************************
                              S T A T U S
***********************************************************************/

#define NX_STATUS_QUALIFIER                  (nxStatus_t)(0x3FF63000)
#define NX_STATUS_WARNING                    (nxStatus_t)(0x00000000)
#define NX_STATUS_ERROR                      (nxStatus_t)(0x80000000)

#define NX_WARNING_BASE                      (NX_STATUS_QUALIFIER | NX_STATUS_WARNING)
#define NX_ERROR_BASE                        (NX_STATUS_QUALIFIER | NX_STATUS_ERROR)

#define nxSuccess                            0

   // For a description of each error/warning (including solutions), use nxStatusToString

//! An internal fault occurred in the NI-XNET driver. Please contact National
//! Instruments and provide the information from the file %APPDATA%\National
//! Instruments\NI-XNET\niXntErr.log’. Please note that this location may be
//! hidden on your computer.
#define nxErrInternalError                   (NX_ERROR_BASE | 0x001)

//! Board self test failed(code 2). Solution: try reinstalling the driver or
//! switching the slot(s) of the board(s). If the error persists,contact
//! National Instruments.
#define nxErrSelfTestError1                  (NX_ERROR_BASE | 0x002)

//! Board self test failed(code 3). Solution: try reinstalling the driver or
//! switching the slot(s) of the board(s). If the error persists,contact
//! National Instruments.
#define nxErrSelfTestError2                  (NX_ERROR_BASE | 0x003)

//! Board self test failed(code 4). Solution: try reinstalling the driver or
//! switching the slot(s) of the board(s). If the error persists,contact
//! National Instruments.
#define nxErrSelfTestError3                  (NX_ERROR_BASE | 0x004)

//! Board self test failed(code 5). Solution: try reinstalling the driver or
//! switching the slot(s) of the board(s). If the error persists,contact
//! National Instruments.
#define nxErrSelfTestError4                  (NX_ERROR_BASE | 0x005)

//! Board self test failed(code 6). Solution: try reinstalling the driver or
//! switching the slot(s) of the board(s). If the error persists,contact
//! National Instruments.
#define nxErrSelfTestError5                  (NX_ERROR_BASE | 0x006)

//! Computer went to hibernation mode and the board lost power. Solution:
//! prevent the computer from going to hibernation mode in the control panel.
#define nxErrPowerSuspended                  (NX_ERROR_BASE | 0x007)

//! A write queue overflowed. Solution: wait until queue space becomes available
//! and retry.
#define nxErrOutputQueueOverflow             (NX_ERROR_BASE | 0x008)

//! The board's firmware did not answer a command. Solution: Stop your
//! application and execute a self test. Try deactivating/reactivating the
//! driver in the Device Manager. If the problem persists, contact National
//! Instruments.
#define nxErrFirmwareNoResponse              (NX_ERROR_BASE | 0x009)

//! The operation timed out. Solution: specify a timeout long enough to complete
//! the operation, or change the operation in a way that it can get completed in
//! less time (e.g. read less data).
#define nxErrEventTimeout                    (NX_ERROR_BASE | 0x00A)

//! A read queue overflowed. Solution: reduce your data rate or call Read more
//! frequently.
#define nxErrInputQueueOverflow              (NX_ERROR_BASE | 0x00B)

//! The Read buffer is too small to hold a single frame. Solution: provide a
//! buffer large enough.
#define nxErrInputQueueReadSize              (NX_ERROR_BASE | 0x00C)

//! You tried to open the same frame twice. This is not permitted. Solution:
//! open each frame only once.
#define nxErrDuplicateFrameObject            (NX_ERROR_BASE | 0x00D)

//! You tried to open the same stream object twice. This is not permitted.
//! Solution: open each stream object only once.
#define nxErrDuplicateStreamObject           (NX_ERROR_BASE | 0x00E)

//! Self test is not possible since the board is in use by an application.
//! Solution: stop all NI-XNET applications before executing a self test.
#define nxErrSelfTestNotPossible             (NX_ERROR_BASE | 0x00F)

//! Allocation of memory failed. You do not have sufficient memory in the
//! LabVIEW target. Solution: add more RAM or try to use fewer resources in
//! your applications (arrays, XNET sessions, etc).
#define nxErrMemoryFull                      (NX_ERROR_BASE | 0x010)

//! The maximum number of sessions has exceeded. Solution: use less sessions.
#define nxErrMaxSessions                     (NX_ERROR_BASE | 0x011)

//! The maximum number of frame has been exceeded. Solution: Use less frames in
//! your sessions.
#define nxErrMaxFrames                       (NX_ERROR_BASE | 0x012)

//! The maximum number of devices has been detected. Solution: use less devices.
#define nxErrMaxDevices                      (NX_ERROR_BASE | 0x013)

//! A driver support file is missing. Solution: try reinstalling the driver. If
//! the error persists, contact National Instruments.
#define nxErrMissingFile                     (NX_ERROR_BASE | 0x014)

//! This indicates that a NULL pointer or an empty string was passed to a
//! function. The user should verify that the parameters passed in make sense
//! for the given function.
#define nxErrParameterNullOrEmpty            (NX_ERROR_BASE | 0x015)

//! An invalid reference has been passed to a NI-XNET session function.
//! Solution: Only pass reference retrieved from Create Session, or from an IO
//! name of a session in LabVIEW project.
#define nxErrInvalidSessionHandle            (NX_ERROR_BASE | 0x020)

//! An invalid handle has been passed to a NI-XNET system function. Solution:
//! Only pass a valid system handle.
#define nxErrInvalidSystemHandle             (NX_ERROR_BASE | 0x021)

//! A device handle was expected for a NI-XNET session function. Solution: Only
//! pass a device handle.
#define nxErrDeviceHandleExpected            (NX_ERROR_BASE | 0x022)

//! An interface handle was expected for a NI-XNET session function. Solution:
//! Only pass an interface handle.
#define nxErrIntfHandleExpected              (NX_ERROR_BASE | 0x023)

//! You have configured a property that conflicts with the current mode of the
//! session. For example, you have created a CAN output session with a frame
//! configured with a Timing Type = Cyclic and a Transmit Time of 0.
#define nxErrPropertyModeConflicting         (NX_ERROR_BASE | 0x024)

//! XNET Create Timing Source VI is not supported on Windows. This VI is
//! supported on LabVIEW Real-Time targets only.
#define nxErrTimingSourceNotSupported        (NX_ERROR_BASE | 0x025)

//! You tried to create more than one LabVIEW timing source for a single interface.
//! Only one timing source per interface is supported. The timing source remains
//! until the top-level VI is idle (no longer running).
//! Solution: Call the XNET Create Timing Source VI only once per interface.
//! You can use the timing source with multiple timed structures (e.g. timed loops).
#define nxErrMultipleTimingSource            (NX_ERROR_BASE | 0x026)

//! The transceiver value set is invalid (for this board, e.g. LS on a HS board)
//! or you are trying to perform an operation that requires a different
//! transceiver (e.g., trying to change the state of a disconnected
//! transceiver). Solution: set a valid value.
#define nxErrInvalidTransceiver              (NX_ERROR_BASE | 0x071)

//! The baud rate value set is invalid. Solution: set a valid value.
#define nxErrInvalidBaudRate                 (NX_ERROR_BASE | 0x072)

//! No baud rate value has been set. Solution: set a valid value.
#define nxErrBaudRateNotConfigured           (NX_ERROR_BASE | 0x073)

//! The bit timing values set are invalid. Solution: set valid values.
#define nxErrInvalidBitTimings               (NX_ERROR_BASE | 0x074)

//! The baud rate set does not match the transceiver's allowed range. Solution:
//! change either the baud rate or the transceiver.
#define nxErrBaudRateXcvrMismatch            (NX_ERROR_BASE | 0x075)

//! The configured timing source is not known for this interface. Solution: Make
//! sure that that you pass in a valid value to nxConnectTerminals or
//! nxDisconnectTerminals.
#define nxErrUnknownTimingSource             (NX_ERROR_BASE | 0x076)

//! The configured synchronization source is inappropriate for the hardware. For
//! example, setting a source to nxTerm_FrontPanel0 on XNET hardware that
//! doesn't have front-panel trigger inputs or selecting nxTerm_PXI_Clk10 for a
//! non-PXI board Solution: Pick an appropriate timing source for the hardware.
#define nxErrUnknownSynchronizationSource    (NX_ERROR_BASE | 0x077)

//! The source that you connected to the Master Timebase destination is missing.
//! When the start trigger is received, the interface verifies that a signal is
//! present on the configured source. This check has determined that this signal
//! is missing. Solution: Verify that your cables are configured correctly and
//! that your timebase source is generating an appropriate waveform.
#define nxErrMissingTimebaseSource              (NX_ERROR_BASE | 0x078)

//! The source that you connected to the Master Timebase destination is not
//! generating an appropriate signal. When the start trigger is received, the
//! interface verifies that a signal of a known frequency is present on the
//! configured source. This check has determined that this source is generating
//! a signal, but that the signal is not one of the supported frequencies for
//! this hardware. Solution: Verify that your source is generating a signal at a
//! supported frequency.
#define nxErrUnknownTimebaseFrequency           (NX_ERROR_BASE | 0x079)

//! You are trying to disconnect a synchronization terminal that is not
//! currently connected. Solution: Only disconnect synchronization terminals
//! that have previously been connected.
#define nxErrUnconnectedSynchronizationSource   (NX_ERROR_BASE | 0x07A)

//! You are trying to connect a synchronization terminal that is already in use.
//! For example, you are trying to connect a trigger line to the Master Timebase
//! when a different trigger line is already connected to the Master Timebase.
//! Solution: Only connect to synchronization terminals that are not currently
//! in use.
#define nxErrConnectedSynchronizationTerminal   (NX_ERROR_BASE | 0x07B)

//! You are trying to connect an XNET terminal as a source terminal, but the
//! desired XNET terminal is not valid as a source terminal. Solution: Only
//! connect valid source terminals to the source terminal in XNET Connect
//! Terminals.
#define nxErrInvalidSynchronizationSource       (NX_ERROR_BASE | 0x07C)

//! You are trying to connect an XNET terminal as a destination terminal, but
//! the desired XNET terminal is not valid as a destination terminal. Solution:
//! Only connect valid destination terminals to the destination terminal in XNET
//! Connect Terminals.
#define nxErrInvalidSynchronizationDestination  (NX_ERROR_BASE | 0x07D)

//! You are trying to connect two XNET terminals that are incompatible.
//! Solution: Only connect a source and destination terminals that are
//! compatible with each other.
#define nxErrInvalidSynchronizationCombination  (NX_ERROR_BASE | 0x07E)

//! The source that you connected to the Master Timebase destination has
//! disappeared. When the start trigger is received, the interface verifies that
//! a signal is present on the configured source. This check has determined that
//! this signal was present, but while the interface was running, the signal
//! disappeared, so all timebase configuration has reverted to using the onboard
//! (unsynchronized) oscillator. Solution: Verify that your cables are
//! configured correctly and that your timebase source is generating an
//! appropriate waveform the entire time your application is running.
#define nxErrTimebaseDisappeared                (NX_ERROR_BASE | 0x07F)

//! You called Read (State : FlexRay : Cycle Macrotick), and the FlexRay
//! Macrotick is not connected as the master timebase of the interface.
//! Solution: Call Connect Terminals to connect source of FlexRay
//! Macrotick to destination of Master Timebase.
#define nxErrMacrotickDisconnected           (NX_ERROR_BASE | 0x080)

//! The database specified could not be opened. Solution: Check that the alias
//! and/or the file exist and that it is a valid database.
#define nxErrCannotOpenDatabaseFile          (NX_ERROR_BASE | 0x081)

//! The cluster was not found in the database. Solution: Make sure you only
//! initialize a cluster in a session that is defined in the database.
#define nxErrClusterNotFound                 (NX_ERROR_BASE | 0x082)

//! The frame was not found in the database. Solution: Make sure you only
//! initialize frames in a session that are defined in the database.
#define nxErrFrameNotFound                   (NX_ERROR_BASE | 0x083)

//! The signal was not found in the database. Solution: Make sure you only
//! initialize signals in a session that are defined in the database.
#define nxErrSignalNotFound                  (NX_ERROR_BASE | 0x084)

//! A necessary property for a cluster was not found in the database. Solution:
//! Make sure you only initialize a cluster in a session that is completely
//! defined in the database.
#define nxErrUnconfiguredCluster             (NX_ERROR_BASE | 0x085)

//! A necessary property for a frame was not found in the database. Solution:
//! Make sure you only initialize frames in a session that are completely
//! defined in the database.
#define nxErrUnconfiguredFrame               (NX_ERROR_BASE | 0x086)

//! A necessary property for a signal was not found in the database. Solution:
//! Make sure you only initialize signals in a session that are completely
//! defined in the database.
#define nxErrUnconfiguredSignal              (NX_ERROR_BASE | 0x087)

//! Multiple clusters have been specified in one session, either directly (Frame
//! Stream), or through the signals or frames specified. Solution: Make sure
//! that in one session, you open only one cluster resp. frames or signals that
//! belong to the same cluster.
#define nxErrMultipleClusters                (NX_ERROR_BASE | 0x088)

//! You specified a database of ':subordinate:' for a session mode other than
//! nxMode_FrameInStream. Solution: either open a StreamInput session, or use
//! a real or in-memory database.
#define nxErrSubordinateNotAllowed           (NX_ERROR_BASE | 0x089)

//! The interface name given does not specify a valid and existing interface.
//! Solution: use a valid and existing interface. These can be obtained using
//! the XNET Interface IO name, or using the interface properties of the XNET
//! System property node.
#define nxErrInvalidInterface                (NX_ERROR_BASE | 0x08A)

//! The operation is invalid for this interface (e.g. you tried to open a set of
//! FlexRay frames on a CAN interface, or tried to request a CAN property from a
//! FlexRay interface). Solution: run this operation on a suitable interface.
#define nxErrInvalidProtocol                 (NX_ERROR_BASE | 0x08B)

//! You tried to set the AutoStart property to FALSE for an Input session. This
//! is not allowed. Solution: don't set the AutoStart property (TRUE is default).
#define nxErrInputSessionMustAutoStart       (NX_ERROR_BASE | 0x08C)

//! The property ID you specified is not valid (or not valid for the current
//! session mode).
#define nxErrInvalidPropertyId               (NX_ERROR_BASE | 0x08D)

//! The contents of the property is bigger than the size specified. Use the
//! nxGetPropertySize function to determine the size of the buffer needed.
#define nxErrInvalidPropertySize             (NX_ERROR_BASE | 0x08E)

//! The function you called is not defined for the session mode (e.g. you called
//! a frame I/O command on a signal session).
#define nxErrIncorrectMode                   (NX_ERROR_BASE | 0x08F)

//! The buffer specified is too small to hold all the data for the read/write
//! function. Solution: determine the size of the buffer from the session and
//! read/write parameters and specify a big enough buffer.
#define nxErrBufferTooSmall                  (NX_ERROR_BASE | 0x090)

//! For Signal Write sessions, the multiplexer signals used in the session must
//! be specified explicitly in the signal list.
#define nxErrMustSpecifyMultiplexers         (NX_ERROR_BASE | 0x091)

//! You used an XNET Session IO name, and that session was not found in your
//! LabVIEW project. Solution: Within LabVIEW project, right-click the target
//! (RT or My Computer), and select New > NI-XNET Session. Add the VI that uses
//! the session under the target. If you are using the session with a built
//! application (.EXE), ensure that you copy the built configuration file
//! nixnetSession.txt such that it resides in the same folder as the executable.
#define nxErrSessionNotFound                 (NX_ERROR_BASE | 0x092)

//! You used the same XNET session name in multiple top-level VIs, which is not
//! supported. Solution: Use each session in only one top-level VI (application)
//! at a time.
#define nxErrMultipleUseOfSession            (NX_ERROR_BASE | 0x093)

//! To execute this command properly, the session may span only one frame.
//! Solution: break your session up into multiple, each of which contains only
//! one frame.
#define nxErrOnlyOneFrame                    (NX_ERROR_BASE | 0x094)

//! You used the same alias for different database files which is not allowed.
//! Solution: Use each alias only for a single database file.
#define nxErrDuplicateAlias                  (NX_ERROR_BASE | 0x095)

//! You try to deploy a database file while another deployment is in progress.
//! Solution: wait until the other deployment has finished and try again.
#define nxErrDeploymentInProgress            (NX_ERROR_BASE | 0x096)

//! A signal or frame session has been opened, but it doesn’t contain signals or
//! frames. Solution: specify at least one signal or frame.
#define nxErrNoFramesOrSignals               (NX_ERROR_BASE | 0x097)

//! An invalid value has been specified for the ‘mode’ parameter. Solution:
//! specify a valid value.
#define nxErrInvalidMode                     (NX_ERROR_BASE | 0x098)

//! A session was created by references, but no database references have been
//! specified. Solution: specify at least one appropriate database reference
//! (i.e. signal or frame or cluster ref depending on the session mode).
#define nxErrNeedReference                   (NX_ERROR_BASE | 0x099)

//! The interface has already been opened with different cluster settings than
//! the ones specified for this session. Solution: make sure that the cluster
//! settings agree for the interface, or use a different interface.
#define nxErrDifferentClusterOpen            (NX_ERROR_BASE | 0x09A)

//! The cycle repetition of a frame in the database for the FlexRay protocol is
//! invalid. Solution: Make sure that the cycle repetition is a power of 2
//! between 1 and 64.
#define nxErrFlexRayInvalidCycleRep          (NX_ERROR_BASE | 0x09B)

//! You called XNET Clear for the session, then tried to perform another
//! operation. Solution: Defer clear (session close) until you are done using
//! it. This error can also occur if you branch a wire after creating the
//! session. Solution: Do not branch a session to multiple flows in the diagram.
#define nxErrSessionCleared                  (NX_ERROR_BASE | 0x09C)

//! You wired a mode to the XNET Create Session VI that does not match the class
//! of items wired in. This includes wiring: 1) XNET Signal items for a Frame
//! I/O mode 2) XNET Frame items for a Signal I/O mode 3) XNET Cluster item for
//! a mode other than Frame Input Stream or Frame Output Stream
#define nxErrWrongModeForCreateSelection     (NX_ERROR_BASE | 0x09D)

//! You tried to create a new session while the interface is already running.
//! Solution: Create all sessions before starting any of them.
#define nxErrInterfaceRunning                (NX_ERROR_BASE | 0x09E)

//! You wrote a frame whose payload length is larger than the maximum payload
//! allowed by the database (e.g. wrote 10 bytes for CAN frame, max 8 bytes). Solution:
//! Never write more payload bytes than the Payload Length Maximum property of
//! the session.
#define nxErrFrameWriteTooLarge              (NX_ERROR_BASE | 0x09F)

//! You called a Read function with a nonzero timeout, and you used a negative
//! numberToRead. Negative value for numberToRead requests all available data
//! from the Read, which is ambiguous when used with a timeout. Solutions: 1)
//! Pass timeout of and numberToRead of -1, to request all available data. 2)
//! Pass timeout > 0, and numberToRead > 0, to wait for a specific number of
//! data elements.
#define nxErrTimeoutWithoutNumToRead         (NX_ERROR_BASE | 0x0A0)

//! Timestamps are not (yet) supported for Write Signal XY. Solution: Do not
//! provide a timestamp array for Write Signal XY.
#define nxErrTimestampsNotSupported          (NX_ERROR_BASE | 0x0A1)

// \REVIEW: Rename to WaitCondition
//! The condition parameter passed to nxWait is not known. Solution: Pass a
//! valid parameter.
#define nxErrUnknownCondition                (NX_ERROR_BASE | 0x0A2)

//! You attempted an IO operation, but the session is not yet started (and the
//! AutoStart property is set to FALSE). Solution: call nxStart before you use
//! this IO operation.
#define nxErrSessionNotStarted               (NX_ERROR_BASE | 0x0A3)

//! The maximum number of Wait operations per object has been exceeded.
//! Solution: Use less Wait operations on this object.
#define nxErrMaxWaitsExceeded                (NX_ERROR_BASE | 0x0A4)

//! You used an invalid name for an XNET Device. Solution: Get valid XNET Device
//! names from the XNET System property node (only).
#define nxErrInvalidDevice                   (NX_ERROR_BASE | 0x0A5)

//! A terminal name passed to nx(Dis)ConnectTerminals is unknown. Solution: only
//! pass valid names.
#define nxErrInvalidTerminalName             (NX_ERROR_BASE | 0x0A6)

//! You tried to blink the port LEDs but these are currently busy. Solution:
//! stop all applications running on that port; do not access it from MAX or the
//! LV Project Provider.
#define nxErrPortLEDsBusy                    (NX_ERROR_BASE | 0x0A7)

//! You tried to set a keyslot id that is not listed as valid in the database.
//! Solution: only pass slotIDs of frames that have the startup or sync property
//! set in the database.
#define nxErrInvalidKeyslot                  (NX_ERROR_BASE | 0x0A8)

//! You tried to set a queue size that is bigger than the maximum allowed.
//! Solution: Specify an in-range queue size.
#define nxErrMaxQueueSizeExceeded            (NX_ERROR_BASE | 0x0A9)

//! You wrote a frame whose payload length is different than the payload length
//! configured by the database. Solution: Never write a different payload length
//! for a frame that is different than the configured payload length.
#define nxErrFrameSizeMismatch               (NX_ERROR_BASE | 0x0AA)

//! Allocation of memory failed for the data returned from LabVIEW XNET Read.
//! Solutions: 1) Wire a smaller "number to read" to XNET Read
//! (default -1 uses queue size).
//! 2) For Signal Input Waveform, use a smaller resample rate.
//! 3) Set smaller value for session's queue size property
//! (default is large to avoid loss of data).
#define nxErrMemoryFullReadData              (NX_ERROR_BASE | 0x0B0)

//! Allocation of memory failed in the firmware
//! Solutions:
//! 1) Create less firmware objects
//! 2) Set smaller value for output session's queue size property
//! (default is large to avoid loss of data).
#define nxErrMemoryFullFirmware              (NX_ERROR_BASE | 0x0B1)

//! A property value was out of range or incorrect. Solution: specify a correct
//! value.
#define nxErrInvalidPropertyValue            (NX_ERROR_BASE | 0x0C0)

//! Integration of the interface into the FlexRay cluster failed. Solution:
//! check the cluster and/or interface parameters and verify that there are
//! startup frames defined.
#define nxErrFlexRayIntegrationFailed        (NX_ERROR_BASE | 0x0C1)

//! RPC communication with the RT target failed. Solution: check if NI-XNET has
//! been installed on the RT target and check if the NI-XNET RPC server has been
//! started.
#define nxErrRPCCommunication                (NX_ERROR_BASE | 0x100)

//! FTP communication with the RT target failed. Solution: check if the RT
//! target has been powered on, the RT target has been connected to the network
//! and if the IP address settings are correct.
#define nxErrFTPCommunication                (NX_ERROR_BASE | 0x101)

//! FTP file access on the RT target failed. Solution: You may have executed a
//! VI that opened the database, but did not close. If that is the case, you
//! should change the VI to call Database Close, then reboot the RT controller
//! to continue.
#define nxErrFTPFileAccess                   (NX_ERROR_BASE | 0x102)

//! The database file you want to use is already assigned to another alias.
//! Solution: Each database file can only be assigned to a single alias. Use the
//! alias that is already assigned to the database instead.
#define nxErrDatabaseAlreadyInUse            (NX_ERROR_BASE | 0x103)

//! An internal file used by NI-XNET could not be accessed. Solution: Make sure
//! that the internal NI-XNET files are not write protected and that the
//! directories for these files exist.
#define nxErrInternalFileAccess              (NX_ERROR_BASE | 0x104)

//! The file cannot be deployed because another file deployment is already
//! active. Solution: wait until the other file deployment has finished and try
//! again.
#define nxErrFTPTxActive							(NX_ERROR_BASE | 0x105)

//! The nixnet.dll or one of its components could not be loaded. Solution: try
//! reinstalling NI-XNET. If the error persists,contact National Instruments.
#define nxErrDllLoad                         (NX_ERROR_BASE | 0x117)

//! You attempted to perform an action on an object that is started that
//! requires the object to be not started. Solution: Stop the object before
//! performing this action.
#define nxErrObjectStarted                   (NX_ERROR_BASE | 0x11E)

//! You have passed a default payload to the firmware where the number of bytes
//! in the payload is larger than the number of bytes that this frame can
//! transmit. Solution: Decrease the number of bytes in your default payload.
#define  nxErrDefaultPayloadNumBytes         (NX_ERROR_BASE | 0x11F)

//! You attempted to set an arbitration id with an invalid value. For example, a
//! CAN standard arbitration ID supports only 11 bits. If you attempt to set a
//! standard arbitration ID that uses more than 11 bits, this error is returned.
//! Solution: Use a valid arbitration id.
#define nxErrInvalidArbitrationId            (NX_ERROR_BASE | 0x123)

//! Too many open files. The database API allows up to 7 files to be opened
//! simultaneously. Solution: Open less files.
#define  nxErrTooManyOpenFiles               (NX_ERROR_BASE | 0x130)

//! Bad reference has been passed to a database function, e.g. a session
//! reference, or frame reference to retrieve properties from a signal object
#define  nxErrDatabaseBadReference           (NX_ERROR_BASE | 0x131)

//! Creating a FIBEX file failed. Solution: Verify access rights to the
//! destination directory or check if overwritten file has read only permission.
#define  nxErrCreateDatabaseFile             (NX_ERROR_BASE | 0x132)

//! A cluster with the same name already exists in the database. Solution: Use
//! another name for this cluster.
#define  nxErrDuplicateClusterName           (NX_ERROR_BASE | 0x133)

//! A frame with the same name already exists in the cluster. Solution: Use
//! another name for this frame.
#define  nxErrDuplicateFrameName             (NX_ERROR_BASE | 0x134)

//! A signal with the same name already exists in the frame. Solution: Use
//! another name for this signal.
#define  nxErrDuplicateSignalName            (NX_ERROR_BASE | 0x135)

//! An ECU with the same name already exists in the cluster. Solution: Use
//! another name for this ECU.
#define  nxErrDuplicateECUName               (NX_ERROR_BASE | 0x136)

//! A subframe with the same name already exists in the frame. Solution: Use
//! another name for this subframe.
#define  nxErrDuplicateSubframeName          (NX_ERROR_BASE | 0x137)

//! The operation is improper to the define protocol, e.g. you cannot assign
//! FlexRay channels to a CAN frame.
#define  nxErrImproperProtocol               (NX_ERROR_BASE | 0x138)

//! Wrong parent relationship for a child that you are creating with XNET
//! Database Create.
#define  nxErrObjectRelation                 (NX_ERROR_BASE | 0x139)

//! Retrieved required property is not defined on the specified object
#define  nxErrUnconfiguredRequiredProperty   (NX_ERROR_BASE | 0x13B)

//! The feature is not supported under LabVIEW RT, e.g. nxdbSaveDatabase
#define  nxErrNotSupportedOnRT               (NX_ERROR_BASE | 0x13C)

//! The object name contains unsupported characters. The name must contain just
//! alphanumeric characters and the underscore, but cannot begin with a digit.
//! The maximum size is 128.
#define  nxErrNameSyntax                     (NX_ERROR_BASE | 0x13D)

//! Unsupported database format. The extension of a database must be .xml, .dbc
//! or .ncd.
#define  nxErrFileExtension                  (NX_ERROR_BASE | 0x13E)

//! Database object not found, e.g. an object with given name doesn't exist.
#define  nxErrDatabaseObjectNotFound         (NX_ERROR_BASE | 0x13F)

//! Cache file cannot be removed or replaced on the disc, e.g. it is
//! write-protected.
#define  nxErrRemoveDatabaseCacheFile        (NX_ERROR_BASE | 0x140)

//! You are trying to write a read-only property, e.g. the mux value on a signal
//! is a read only property (can be changed on the subframe).
#define  nxErrReadOnlyProperty               (NX_ERROR_BASE | 0x141)

//! You are trying to change a signal to be a mux signal, but a mux is already
//! defined in this frame
#define  nxErrFrameMuxExists                 (NX_ERROR_BASE | 0x142)

//! You are trying to define in-cycle-repetition slots before defining the first
//! slot. Define the first slot (frame ID) before defining in-cycle-repetition
//! slots.
#define  nxErrUndefinedFirstSlot             (NX_ERROR_BASE | 0x144)

//! You are trying to define in-cycle-repetition channels before defining the
//! first channels. Define the Channel Assignment on a frame before defining
//! in-cycle-repetition channels.
#define  nxErrUndefinedFirstChannels         (NX_ERROR_BASE | 0x145)

//! You must define the protocol before setting this property, e.g. the frame ID
//! has a different meaning in a CAN or FlexRay cluster.
#define  nxErrUndefinedProtocol              (NX_ERROR_BASE | 0x146)

//! The cache file on RT system has been created with an older XNET version.
//! This version is not longer supported. Solution: Re-deploy your database to
//! the RT system.
#define  nxErrOldDatabaseCacheFile           (NX_ERROR_BASE | 0x147)

//! nxPropFrm_ConfigStatus: A signal within the frame exceeds the frame
//! boundaries nxPropSig_ConfigStatus: Signal exceeds frame boundaries
#define  nxErrDbConfigSigOutOfFrame          (NX_ERROR_BASE | 0x148)

//! nxPropFrm_ConfigStatus: Any signal within the frame overlaps another signal
//! nxPropSig_ConfigStatus: Signal overlaps another signal in this frame
#define  nxErrDbConfigSigOverlapped          (NX_ERROR_BASE | 0x149)

//! nxPropFrm_ConfigStatus: Any integer signal within the frame is defined with
//! more than 52 bit. Not supported nxPropSig_ConfigStatus: Signal is an integer
//! signal with more than 52 bits. Not supported
#define  nxErrDbConfigSig52BitInteger        (NX_ERROR_BASE | 0x14A)

//! nxPropFrm_ConfigStatus: Frame is defined with wrong number of bytes Allowed
//! values: - CAN: 0-8, - Flexray: 0-254 and even number nxPropSig_ConfigStatus:
//! Not used
#define  nxErrDbConfigFrameNumBytes          (NX_ERROR_BASE | 0x14B)

//! You are trying to add transmitted frames to an ECU, with at least two of
//! them having Startup or Sync property on. Only one Sync or Startup frame is
//! allowed to be sent by an ECU
#define  nxErrMultSyncStartup                (NX_ERROR_BASE | 0x14C)

//! You are trying to add TX/RX frames to an ECU which are defined in a
//! different cluster than the ECU.
#define  nxErrInvalidCluster                 (NX_ERROR_BASE | 0x14D)

//! Database name parameter is incorrect. Solution: Use a valid name for the
//! database, e.g. ":memory:" for in-memory database.
#define  nxErrDatabaseName                   (NX_ERROR_BASE | 0x14E)

//! Database object is locked because it is used in a session. Solution:
//! Configure the database before using it in a session.
#define  nxErrDatabaseObjectLocked           (NX_ERROR_BASE | 0x14F)

//! Alias name passed to a function is not defined. Solution: Define the alias
//! before calling the function.
#define nxErrAliasNotFound                   (NX_ERROR_BASE | 0x150)

//! FIBEX file cannot be saved because frames are assigned to FlexRay channels
//! not defined in the cluster. Solution: Verify that all frames in the FlexRay
//! cluster are assigned to an existing cluster channel.
#define nxErrClusterFrameChannelRelation     (NX_ERROR_BASE | 0x151)

//! Frame Config_Status: This FlexRay frame transmitted in a dynamic segment
//! uses both channels A and B. This is not allowed. Solution: Use either
//! channel A or B.
#define nxErrDynFlexRayFrameChanAandB        (NX_ERROR_BASE | 0x152)

//! Database is locked because it is being modified by an another instance of
//! the same application. Solution: Close the database in the other application
//! instance.
#define  nxErrDatabaseLockedInUse            (NX_ERROR_BASE | 0x153)

//! A frame name is ambiguous, e.g. a frame with the same name exists in another
//! cluster. Solution: Specify the cluster name for the frame using the required
//! syntax.
#define  nxErrAmbiguousFrameName             (NX_ERROR_BASE | 0x154)

//! A signal name is ambiguous, e.g. a signal with the same name exists in
//! another frame. Solution: Use <frame>.<signal> syntax for the signal.
#define  nxErrAmbiguousSignalName            (NX_ERROR_BASE | 0x155)

//! An ECU name is ambiguous, e.g. an ECU with the same name exists in another
//! cluster. Solution: Specify the cluster name for the ECU using the required
//! syntax.
#define  nxErrAmbiguousECUName               (NX_ERROR_BASE | 0x156)

//! A subframe name is ambiguous, e.g. a subframe with the same name exists in
//! another cluster. Solution: Specify the cluster name for the subframe using
//! the required syntax.
#define  nxErrAmbiguousSubframeName          (NX_ERROR_BASE | 0x157)

//! You cannot mix open of NI-XNET database objects as both manual and
//! automatic. You open manually by calling the Database Open VI. You open
//! automatically when you 1) wire the IO name directly to a property node or
//! VI, 2) branch a wire to multiple data flows on the diagram, 3) use the IO
//! name with a VI or property node after closing it with the Database Close VI.
//! Solution: Change your diagram to use the manual technique in all locations
//! (always call Open and Close VIs), or to use the automatic technique in all
//! locations (never call Open or Close VIs).
#define nxErrMixAutoManualOpen               (NX_ERROR_BASE | 0x15E)

//! Due to problems in LabVIEW versions 8.5 through 8.6.1, automatic open of
//! NI-XNET database objects is not suppported. You open automatically when you
//! 1) wire the IO name directly to a property node or VI, 2) branch a wire to
//! multiple data flows on the diagram, 3) use the IO name with a VI or property
//! node after closing it with the Database Close VI. Solution: Change your
//! diagram to call the Database Open VI prior to any use (VI or property node)
//! in a data flow (including a new wire branch). Change your diagram to call
//! the Database Close VI when you are finished using the database in your
//! application.
#define nxErrAutoOpenNotSupported            (NX_ERROR_BASE | 0x15F)

//! You called a Write function with the number of array elements (frames or
//! signals) different than the number of elements configured in the session
//! (such as the "list" parameter of the Create Session function). Solution:
//! Write the same number of elements as configured in the session.
#define nxErrWrongNumSignalsWritten          (NX_ERROR_BASE | 0x160)

//! You used XNET session from multiple LabVIEW projects (or multiple
//! executables), which NI-XNET does not support. Solution: Run XNET sessions in
//! only one LabVIEW project at a time.
#define nxErrMultipleLvProject               (NX_ERROR_BASE | 0x161)

//! When an XNET session is used at runtime, all sessions in the same scope are
//! created on the interface. The same scope is defined as all sessions within
//! the same LabVIEW project which use the same cluster and interface (same
//! physical cable configuration). If you attempt to use a session in the same
//! scope after running the VI, this error occurs. The most likely cause is that
//! you added a new session, and tried to use that new session in a running VI.
//! Solution: Configure all session in LabVIEW project, then run the VI(s) that
//! use those sessions.
#define nxErrSessionConflictLvProject        (NX_ERROR_BASE | 0x162)

//! You used an empty name for an XNET database object (database, cluster, ECU,
//! frame, or signal). Empty name is not supported. Solution: Refer to NI-XNET
//! help for IO names to review the required syntax for the name, and change
//! your code to use that syntax.
#define nxErrDbObjectNameEmpty               (NX_ERROR_BASE | 0x163)

//! You used a name for an XNET database object (such as frame or signal) that
//! did not include a valid cluster selection. Solution: Refer to the NI-XNET
//! help for the IO name that you are using, and use the syntax specified for
//! that class, which includes the cluster selection.
#define nxErrMissingAliasInDbObjectName      (NX_ERROR_BASE | 0x164)

//! Unsupported FIBEX file version. Solution: Use only FIBEX versions that are
//! supported by this version of NI-XNET. Please see the NI-XNET documentation
//! for information on which FIBEX versions are currently supported.
#define nxErrFibexImportVersion              (NX_ERROR_BASE | 0x165)

//! You used an empty name for the XNET Session. Empty name is not supported.
//! Solution: Use a valid XNET session name from your LabVIEW project.
#define nxErrEmptySessionName                (NX_ERROR_BASE | 0x166)

//! There is not enough message RAM on the FlexRay hardware to configure the
//! data partition for the object(s). Solution: Please refer to the manual for
//! limitations on the number of objects that can be created at any given time
//! based on the payload length.
#define nxErrNotEnoughMessageRAMForObject     (NX_ERROR_BASE | 0x167)

//! The keyslot ID has been configured and a startup session has been created.
//! Either the keyslot ID needs to be configured OR the startup session needs to
//! be created. Both cannot exist at the same time. Solution: Choose a single
//! method to configure startup sessions in your application.
#define nxErrKeySlotIDConfig                 (NX_ERROR_BASE | 0x168)

//! An unsupported session was created. For example, stream output is not
//! supported on FlexRay hardware. Solution: Only use supported sessions in your
//! application.
#define nxErrUnsupportedSession         (NX_ERROR_BASE | 0x169)


//! An XNET session was created after starting the Interface. Only the Stream
//! Input session in the subordinate mode can be created after the Interface has
//! started. Solution: Create sessions prior to starting the XNET Interface in
//! your application.
#define nxErrObjectCreatedAfterStart         (NX_ERROR_BASE | 0x170)

//! The Single Slot property was enabled on the XNET FlexRay Interface after the
//! interface had started. Solution: Enable the Single Slot property prior to
//! starting the XNET FlexRay Interface.
#define nxErrSingleSlotEnabledAfterStart      (NX_ERROR_BASE | 0x171)

//! The macroticks offset specified for XNET Create Timing Source is
//! unsupported.Example: Specifying a macroticks offset greater than
//! MacroPerCycle will result in this error. Solution: Specify a macrotick
//! offset within the supported range for the cluster.
#define nxErrUnsupportedNumMacroticks         (NX_ERROR_BASE | 0x172)

//! You used invalid syntax in the name of a database object (signal,
//! frame, or ECU). For example, you may have specified a frame's name
//! as <cluster>.<frame>, which is allowed in NI-XNET for C/C++, but
//! not NI-XNET for LabVIEW. Solution: Use the string syntax specified
//! in the help topic for the XNET I/O name class you are using.
#define nxErrBadSyntaxInDatabaseObjectName    (NX_ERROR_BASE | 0x173)

   // Warning Section

//! There is some warning from importing the FIBEX file. For details, refer to
//! the import log file nixnetfx-log.txt under %ALLUSERSPROFILE%\Application
//! Data\National Instruments\NI-XNET
#define nxWarnDatabaseImport                 (NX_WARNING_BASE | 0x085)

//! The FIBEX file has been imported, but it was not stored by the XNET Editor
//! or using the XNET API. Saving the FIBEX file back with the XNET API or the
//! XNET Editor may loose information from the original file.
#define nxWarnDatabaseImportFIBEXNoXNETFile  (NX_WARNING_BASE | 0x086)

//! The FIBEX file was not written by XNET tools or the XNET API. Additionally,
//! there is another warning. For details, refer to the import log file
//! nixnetfx-log.txt under %ALLUSERSPROFILE%\Application Data\National
//! Instruments\NI-XNET
#define nxWarnDatabaseImportFIBEXNoXNETFilePlusWarning \
                                             (NX_WARNING_BASE | 0x087)

//! nxdbCloseDatabase returns a warning instead of an error, when invalid
//! reference is passed to the function.
#define  nxWarnDatabaseBadReference          (NX_WARNING_BASE | 0x131)

/***********************************************************************
                          P R O P E R T Y   I D S
***********************************************************************/

   // Class IDs used for encoding of property IDs (nxProp*)
   // Also class parameter of function nxdbCreateObject, nxdbDeleteObject, and nxdbFindObject
#define nxClass_Database                  (u32)0x00000000   //Database
#define nxClass_Cluster                   (u32)0x00010000   //Cluster
#define nxClass_Frame                     (u32)0x00020000   //Frame
#define nxClass_Signal                    (u32)0x00030000   //Signal
#define nxClass_Subframe                  (u32)0x00040000   //Subframe
#define nxClass_ECU                       (u32)0x00050000   //ECU
#define nxClass_Session                   (u32)0x00100000   //Session
#define nxClass_System                    (u32)0x00110000   //System
#define nxClass_Device                    (u32)0x00120000   //Device
#define nxClass_Interface                 (u32)0x00130000   //Interface

#define nxClass_Mask                      (u32)0x00FF0000   //mask for object class

   // Datatype IDs used in encoding of property IDs (nxProp*)
#define nxPrptype_u32                     (u32)0x00000000
#define nxPrptype_f64                     (u32)0x01000000
#define nxPrptype_bool                    (u32)0x02000000   // use u8 as datatype (semantic only)
#define nxPrptype_string                  (u32)0x03000000
#define nxPrptype_1Dstring                (u32)0x04000000   // comma-separated list
#define nxPrptype_ref                     (u32)0x05000000   // u32 reference (handle)
#define nxPrptype_1Dref                   (u32)0x06000000   // array of u32 reference
#define nxPrptype_time                    (u32)0x07000000   // nxTimestamp_t
#define nxPrptype_1Du32                   (u32)0x08000000   // array of u32 values
#define nxPrptype_u64                     (u32)0x09000000
#define nxPrptype_1Du8                    (u32)0x0A000000   // array of u8 values

#define nxPrptype_Mask                    (u32)0xFF000000   // mask for proptype
    /* PropertyId parameter of nxGetProperty, nxGetPropertySize, nxSetProperty functions. */

      // Session:Auto Start?
#define nxPropSession_AutoStart                 ((u32)0x00000001 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:ClusterName
#define nxPropSession_ClusterName               ((u32)0x0000000A | nxClass_Session | nxPrptype_string)   //--r
      // Session:Database
#define nxPropSession_DatabaseName              ((u32)0x00000002 | nxClass_Session | nxPrptype_string)   //--r
      // Session:List of Signals / List of Frames
#define nxPropSession_List                      ((u32)0x00000003 | nxClass_Session | nxPrptype_1Dstring) //--r
      // Session:Mode
#define nxPropSession_Mode                      ((u32)0x00000004 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Number in List
#define nxPropSession_NumInList                 ((u32)0x00000005 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Number of Values Pending
#define nxPropSession_NumPend                   ((u32)0x00000006 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Number of Values Unused
#define nxPropSession_NumUnused                 ((u32)0x0000000B | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Payload Length Maximum
#define nxPropSession_PayldLenMax               ((u32)0x00000009 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Protocol
#define nxPropSession_Protocol                  ((u32)0x00000008 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Queue Size
#define nxPropSession_QueueSize                 ((u32)0x0000000C | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Resample Rate
#define nxPropSession_ResampRate                ((u32)0x00000007 | nxClass_Session | nxPrptype_f64)      //--rw
      // Session:Interface:Echo Transmit?
#define nxPropSession_IntfEchoTx                ((u32)0x00000010 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:Log Cluster Errors?
#define nxPropSession_IntfLogClstErr            ((u32)0x00000011 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:Log Wakeup?
#define nxPropSession_IntfLogWakeup             ((u32)0x00000012 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:Name
#define nxPropSession_IntfName                  ((u32)0x00000013 | nxClass_Session | nxPrptype_string)   //--r
      // Session:Interface:Reference
#define nxPropSession_IntfRef                   ((u32)0x00000015 | nxClass_Session | nxPrptype_string)   //--r
      // Session:Interface:Baud Rate
#define nxPropSession_IntfBaudRate              ((u32)0x00000016 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:CAN:Listen Only?
#define nxPropSession_IntfCANLstnOnly           ((u32)0x00000022 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:CAN:Single Shot Transmit?
#define nxPropSession_IntfCANSingShot           ((u32)0x00000024 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:CAN:Termination
#define nxPropSession_IntfCANTerm               ((u32)0x00000025 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:CAN:Transceiver State
#define nxPropSession_IntfCANTcvrState          ((u32)0x00000028 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:CAN:Transceiver Type
#define nxPropSession_IntfCANTcvrType           ((u32)0x00000029 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Accepted Startup Range
#define nxPropSession_IntfFlexRayAccStartRng    ((u32)0x00000030 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Allow Halt Due To Clock?
#define nxPropSession_IntfFlexRayAlwHltClk      ((u32)0x00000031 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:FlexRay:Allow Passive to Active
#define nxPropSession_IntfFlexRayAlwPassAct     ((u32)0x00000032 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Cluster Drift Damping
#define nxPropSession_IntfFlexRayClstDriftDmp   ((u32)0x00000033 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Coldstart?
#define nxPropSession_IntfFlexRayColdstart      ((u32)0x00000034 | nxClass_Session | nxPrptype_bool)     //--r
      // Session:Interface:FlexRay:Decoding Correction
#define nxPropSession_IntfFlexRayDecCorr        ((u32)0x00000035 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Delay Compensation Ch A
#define nxPropSession_IntfFlexRayDelayCompA     ((u32)0x00000036 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Delay Compensation Ch B
#define nxPropSession_IntfFlexRayDelayCompB     ((u32)0x00000037 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Key Slot Identifier
#define nxPropSession_IntfFlexRayKeySlotID      ((u32)0x00000038 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Latest Tx
#define nxPropSession_IntfFlexRayLatestTx       ((u32)0x00000041 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Interface:FlexRay:Listen Timeout
#define nxPropSession_IntfFlexRayListTimo       ((u32)0x00000042 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Macro Initial Offset Ch A
#define nxPropSession_IntfFlexRayMacInitOffA    ((u32)0x00000043 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Macro Initial Offset Ch B
#define nxPropSession_IntfFlexRayMacInitOffB    ((u32)0x00000044 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Micro Initial Offset Ch A
#define nxPropSession_IntfFlexRayMicInitOffA    ((u32)0x00000045 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Micro Initial Offset Ch B
#define nxPropSession_IntfFlexRayMicInitOffB    ((u32)0x00000046 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Max Drift
#define nxPropSession_IntfFlexRayMaxDrift       ((u32)0x00000047 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Microtick
#define nxPropSession_IntfFlexRayMicrotick      ((u32)0x00000048 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Interface:FlexRay:Null Frames To Input Stream?
#define nxPropSession_IntfFlexRayNullToInStrm   ((u32)0x00000049 | nxClass_Session | nxPrptype_bool)      //--r
      // Session:Interface:FlexRay:Offset Correction
#define nxPropSession_IntfFlexRayOffCorr        ((u32)0x00000058 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Interface:FlexRay:Offset Correction Out
#define nxPropSession_IntfFlexRayOffCorrOut     ((u32)0x00000050 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Rate Correction
#define nxPropSession_IntfFlexRayRateCorr       ((u32)0x00000059 | nxClass_Session | nxPrptype_u32)      //--r
      // Session:Interface:FlexRay:Rate Correction Out
#define nxPropSession_IntfFlexRayRateCorrOut    ((u32)0x00000052 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Samples Per Microtick
#define nxPropSession_IntfFlexRaySampPerMicro   ((u32)0x00000053 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Single Slot Enabled?
#define nxPropSession_IntfFlexRaySingSlotEn     ((u32)0x00000054 | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:FlexRay:Statistics Enabled?
#define nxPropSession_IntfFlexRayStatisticsEn   ((u32)0x0000005A | nxClass_Session | nxPrptype_bool)     //--rw
      // Session:Interface:FlexRay:Sync on Ch A Even Cycle
#define nxPropSession_IntfFlexRaySyncChAEven    ((u32)0x0000005B | nxClass_Session | nxPrptype_1Du32)    //--r
      // Session:Interface:FlexRay:Sync on Ch A Odd Cycle
#define nxPropSession_IntfFlexRaySyncChAOdd     ((u32)0x0000005C | nxClass_Session | nxPrptype_1Du32)    //--r
      // Session:Interface:FlexRay:Sync on Ch B Even Cycle
#define nxPropSession_IntfFlexRaySyncChBEven    ((u32)0x0000005D | nxClass_Session | nxPrptype_1Du32)    //--r
      // Session:Interface:FlexRay:Sync on Ch B Odd Cycle
#define nxPropSession_IntfFlexRaySyncChBOdd     ((u32)0x0000005E | nxClass_Session | nxPrptype_1Du32)    //--r
      // Session:Interface:FlexRay:Sync Frame Status
#define nxPropSession_IntfFlexRaySyncStatus     ((u32)0x0000005F | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Termination
#define nxPropSession_IntfFlexRayTerm           ((u32)0x00000057 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Wakeup Channel
#define nxPropSession_IntfFlexRayWakeupCh       ((u32)0x00000055 | nxClass_Session | nxPrptype_u32)      //--rw
      // Session:Interface:FlexRay:Wakeup Pattern
#define nxPropSession_IntfFlexRayWakeupPtrn     ((u32)0x00000056 | nxClass_Session | nxPrptype_u32)      //--rw
      // System:Devices
#define nxPropSys_DevRefs                       ((u32)0x00000002 | nxClass_System | nxPrptype_1Dref)     //--r
      // System:Interfaces (All)
#define nxPropSys_IntfRefs                      ((u32)0x00000003 | nxClass_System | nxPrptype_1Dref)     //--r
      // System:Interfaces (CAN)
#define nxPropSys_IntfRefsCAN                   ((u32)0x00000004 | nxClass_System | nxPrptype_1Dref)     //--r
      // System:Interfaces (FlexRay)
#define nxPropSys_IntfRefsFlexRay               ((u32)0x00000005 | nxClass_System | nxPrptype_1Dref)     //--r
      // System:Version:Build
#define nxPropSys_VerBuild                      ((u32)0x00000006 | nxClass_System | nxPrptype_u32)       //--r
      // System:Version:Major
#define nxPropSys_VerMajor                      ((u32)0x00000008 | nxClass_System | nxPrptype_u32)       //--r
      // System:Version:Minor
#define nxPropSys_VerMinor                      ((u32)0x00000009 | nxClass_System | nxPrptype_u32)       //--r
      // System:Version:Phase
#define nxPropSys_VerPhase                      ((u32)0x0000000A | nxClass_System | nxPrptype_u32)       //--r
      // System:Version:Update
#define nxPropSys_VerUpdate                     ((u32)0x0000000B | nxClass_System | nxPrptype_u32)       //--r
      // Device:Form Factor
#define nxPropDev_FormFac                       ((u32)0x00000001 | nxClass_Device | nxPrptype_u32)       //--r
      // Device:Interfaces
#define nxPropDev_IntfRefs                      ((u32)0x00000002 | nxClass_Device | nxPrptype_1Dref)     //--r
      // Device:Name
#define nxPropDev_Name                          ((u32)0x00000003 | nxClass_Device | nxPrptype_string)    //--r
      // Device:Number of Ports
#define nxPropDev_NumPorts                      ((u32)0x00000004 | nxClass_Device | nxPrptype_u32)       //--r
      // Device:Product Number
#define nxPropDev_ProductNum                    ((u32)0x00000008 | nxClass_Device | nxPrptype_u32)       //--r
      // Device:Serial Number
#define nxPropDev_SerNum                        ((u32)0x00000005 | nxClass_Device | nxPrptype_u32)       //--r
      // Interface:Device
#define nxPropIntf_DevRef                       ((u32)0x00000001 | nxClass_Interface | nxPrptype_ref)    //--r
      // Interface:Name
#define nxPropIntf_Name                         ((u32)0x00000002 | nxClass_Interface | nxPrptype_string) //--r
      // Interface:Number
#define nxPropIntf_Num                          ((u32)0x00000003 | nxClass_Interface | nxPrptype_u32)    //--r
      // Interface:Port Number
#define nxPropIntf_PortNum                      ((u32)0x00000004 | nxClass_Interface | nxPrptype_u32)    //--r
      // Interface:Protocol
#define nxPropIntf_Protocol                     ((u32)0x00000005 | nxClass_Interface | nxPrptype_u32)    //--r
      // Interface:CAN:Termination Capability
#define nxPropIntf_CANTermCap                   ((u32)0x00000008 | nxClass_Interface | nxPrptype_u32)    //--r
      // Interface:CAN:Transceiver Capability
#define nxPropIntf_CANTcvrCap                   ((u32)0x00000007 | nxClass_Interface | nxPrptype_u32)    //--r
      // Database:Name
#define nxPropDatabase_Name                     ((u32)0x00000001 | nxClass_Database | nxPrptype_string)  //--r
      // Database:Clusters
#define nxPropDatabase_ClstRefs                 ((u32)0x00000002 | nxClass_Database | nxPrptype_1Dref)   //--r
      // Database:Show wrongly defined frames at nxdbOpenDatabase time (default: false)
#define nxPropDatabase_ShowInvalidFromOpen      ((u32)0x00000003 | nxClass_Database | nxPrptype_bool)    //--rw
      // Cluster:Baud Rate
#define nxPropClst_BaudRate                     ((u32)0x00000001 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:Comment
#define nxPropClst_Comment                      ((u32)0x00000008 | nxClass_Cluster | nxPrptype_string)   //--rw
      // Cluster:Configuration Status
#define nxPropClst_ConfigStatus                 ((u32)0x00000009 | nxClass_Cluster | nxPrptype_u32)      //--r
      // Cluster:Database
#define nxPropClst_DatabaseRef                  ((u32)0x00000002 | nxClass_Cluster | nxPrptype_ref)      //--r
      // Cluster:ECUs
#define nxPropClst_ECURefs                      ((u32)0x00000003 | nxClass_Cluster | nxPrptype_1Dref)    //--r
      // Cluster:Frames
#define nxPropClst_FrmRefs                      ((u32)0x00000004 | nxClass_Cluster | nxPrptype_1Dref)    //--r
      // Cluster:Name
#define nxPropClst_Name                         ((u32)0x00000005 | nxClass_Cluster | nxPrptype_string)   //--rw
      // Cluster:Protocol
#define nxPropClst_Protocol                     ((u32)0x00000006 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:Signals
#define nxPropClst_SigRefs                      ((u32)0x00000007 | nxClass_Cluster | nxPrptype_1Dref)    //--r
      // Cluster:FlexRay:Action Point Offset
#define nxPropClst_FlexRayActPtOff              ((u32)0x00000020 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:CAS Rx Low Max
#define nxPropClst_FlexRayCASRxLMax             ((u32)0x00000021 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Channels
#define nxPropClst_FlexRayChannels              ((u32)0x00000022 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Cluster Drift Damping
#define nxPropClst_FlexRayClstDriftDmp          ((u32)0x00000023 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Cold Start Attempts
#define nxPropClst_FlexRayColdStAts             ((u32)0x00000024 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Cycle
#define nxPropClst_FlexRayCycle                 ((u32)0x00000025 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Dynamic Segment Start
#define nxPropClst_FlexRayDynSegStart           ((u32)0x00000026 | nxClass_Cluster | nxPrptype_u32)      //--r
      // Cluster:FlexRay:Dynamic Slot Idle Phase
#define nxPropClst_FlexRayDynSlotIdlPh          ((u32)0x00000027 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Latest Usable Dynamic Slot
#define nxPropClst_FlexRayLatestUsableDyn       ((u32)0x0000002A | nxClass_Cluster | nxPrptype_u32)      //--r
      // Cluster:FlexRay:Latest Guaranteed Dynamic Slot
#define nxPropClst_FlexRayLatestGuarDyn         ((u32)0x0000002B | nxClass_Cluster | nxPrptype_u32)      //--r
      // Cluster:FlexRay:Listen Noise
#define nxPropClst_FlexRayLisNoise              ((u32)0x00000028 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Macro Per Cycle
#define nxPropClst_FlexRayMacroPerCycle         ((u32)0x00000029 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Macrotick
#define nxPropClst_FlexRayMacrotick             ((u32)0x00000030 | nxClass_Cluster | nxPrptype_f64)      //--r
      // Cluster:FlexRay:Max Without Clock Correction Fatal
#define nxPropClst_FlexRayMaxWoClkCorFat        ((u32)0x00000031 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Max Without Clock Correction Passive
#define nxPropClst_FlexRayMaxWoClkCorPas        ((u32)0x00000032 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Minislot Action Point Offset
#define nxPropClst_FlexRayMinislotActPt         ((u32)0x00000033 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Minislot
#define nxPropClst_FlexRayMinislot              ((u32)0x00000034 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Network Management Vector Length
#define nxPropClst_FlexRayNMVecLen              ((u32)0x00000035 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:NIT
#define nxPropClst_FlexRayNIT                   ((u32)0x00000036 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:NIT Start
#define nxPropClst_FlexRayNITStart              ((u32)0x00000037 | nxClass_Cluster | nxPrptype_u32)      //--r
      // Cluster:FlexRay:Number Of Minislots
#define nxPropClst_FlexRayNumMinislt            ((u32)0x00000038 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Number Of Static Slots
#define nxPropClst_FlexRayNumStatSlt            ((u32)0x00000039 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Offset Correction Start
#define nxPropClst_FlexRayOffCorSt              ((u32)0x00000040 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Payload Length Dynamic Maximum
#define nxPropClst_FlexRayPayldLenDynMax        ((u32)0x00000041 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Payload Length Maximum
#define nxPropClst_FlexRayPayldLenMax           ((u32)0x00000042 | nxClass_Cluster | nxPrptype_u32)      //--r
      // Cluster:FlexRay:Payload Length Static
#define nxPropClst_FlexRayPayldLenSt            ((u32)0x00000043 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Static Slot
#define nxPropClst_FlexRayStatSlot              ((u32)0x00000045 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Symbol Window
#define nxPropClst_FlexRaySymWin                ((u32)0x00000046 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Symbol Window Start
#define nxPropClst_FlexRaySymWinStart           ((u32)0x00000047 | nxClass_Cluster | nxPrptype_u32)      //--r
      // Cluster:FlexRay:Sync Node Max
#define nxPropClst_FlexRaySyncNodeMax           ((u32)0x00000048 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:TSS Transmitter
#define nxPropClst_FlexRayTSSTx                 ((u32)0x00000049 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Wakeup Symbol Rx Idle
#define nxPropClst_FlexRayWakeSymRxIdl          ((u32)0x00000050 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Wakeup Symbol Rx Low
#define nxPropClst_FlexRayWakeSymRxLow          ((u32)0x00000051 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Wakeup Symbol Rx Window
#define nxPropClst_FlexRayWakeSymRxWin          ((u32)0x00000052 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Wakeup Symbol Tx Idle
#define nxPropClst_FlexRayWakeSymTxIdl          ((u32)0x00000053 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Cluster:FlexRay:Wakeup Symbol Tx Low
#define nxPropClst_FlexRayWakeSymTxLow          ((u32)0x00000054 | nxClass_Cluster | nxPrptype_u32)      //--rw
      // Frame:Cluster
#define nxPropFrm_ClusterRef                    ((u32)0x00000001 | nxClass_Frame | nxPrptype_ref)        //--r
      // Frame:Comment
#define nxPropFrm_Comment                       ((u32)0x00000002 | nxClass_Frame | nxPrptype_string)     //--rw
      // Frame:Configuration Status
#define nxPropFrm_ConfigStatus                  ((u32)0x00000009 | nxClass_Frame | nxPrptype_u32)        //--r
      // Frame:Default Payload
#define nxPropFrm_DefaultPayload                ((u32)0x00000005 | nxClass_Frame | nxPrptype_1Du8)       //--rw
      // Frame:Identifier (Slot)
#define nxPropFrm_ID                            ((u32)0x00000003 | nxClass_Frame | nxPrptype_u32)        //--rw
      // Frame:Name
#define nxPropFrm_Name                          ((u32)0x00000004 | nxClass_Frame | nxPrptype_string)     //--rw
      // Frame:Payload Length (in bytes)
#define nxPropFrm_PayloadLen                    ((u32)0x00000007 | nxClass_Frame | nxPrptype_u32)        //--rw
      // Frame:Signals
#define nxPropFrm_SigRefs                       ((u32)0x00000008 | nxClass_Frame | nxPrptype_1Dref)      //--r
      // Frame:CAN:Extended Identifier?
#define nxPropFrm_CANExtID                      ((u32)0x00000010 | nxClass_Frame | nxPrptype_bool)       //--rw
      // Frame:CAN:Timing Type
#define nxPropFrm_CANTimingType                 ((u32)0x00000011 | nxClass_Frame | nxPrptype_u32)        //--rw
      // Frame:CAN:Transmit Time
#define nxPropFrm_CANTxTime                     ((u32)0x00000012 | nxClass_Frame | nxPrptype_f64)        //--rw
      // Frame:FlexRay:Base Cycle
#define nxPropFrm_FlexRayBaseCycle              ((u32)0x00000020 | nxClass_Frame | nxPrptype_u32)        //--rw
      // Frame:FlexRay:Channel Assignment
#define nxPropFrm_FlexRayChAssign               ((u32)0x00000021 | nxClass_Frame | nxPrptype_u32)        //--rw
      // Frame:FlexRay:Cycle Repetition
#define nxPropFrm_FlexRayCycleRep               ((u32)0x00000022 | nxClass_Frame | nxPrptype_u32)        //--rw
      // Frame:FlexRay:Payload Preamble?
#define nxPropFrm_FlexRayPreamble               ((u32)0x00000023 | nxClass_Frame | nxPrptype_bool)       //--rw
      // Frame:FlexRay:Startup?
#define nxPropFrm_FlexRayStartup                ((u32)0x00000024 | nxClass_Frame | nxPrptype_bool)       //--rw
      // Frame:FlexRay:Sync?
#define nxPropFrm_FlexRaySync                   ((u32)0x00000025 | nxClass_Frame | nxPrptype_bool)       //--rw
      // Frame:FlexRay:Timing Type
#define nxPropFrm_FlexRayTimingType             ((u32)0x00000026 | nxClass_Frame | nxPrptype_u32)        //--rw
      // Frame:FlexRay:In Cycle Repetitions:Enabled?
#define nxPropFrm_FlexRayInCycRepEnabled        ((u32)0x00000030 | nxClass_Frame | nxPrptype_bool)       //--r
      // Frame:FlexRay:In Cycle Repetitions:Identifiers
#define nxPropFrm_FlexRayInCycRepIDs            ((u32)0x00000031 | nxClass_Frame | nxPrptype_1Du32)      //--rw
      // Frame:FlexRay:In Cycle Repetitions:Channel Assignments
#define nxPropFrm_FlexRayInCycRepChAssigns      ((u32)0x00000032 | nxClass_Frame | nxPrptype_1Du32)      //--rw
      // Frame:Mux:Is Data Multiplexed?
#define nxPropFrm_MuxIsMuxed                    ((u32)0x00000040 | nxClass_Frame | nxPrptype_bool)       //--r
      // Frame:Mux:Data Multiplexer Signal
#define nxPropFrm_MuxDataMuxSigRef              ((u32)0x00000041 | nxClass_Frame | nxPrptype_ref)        //--r
      // Frame:Mux:Static Signals
#define nxPropFrm_MuxStaticSigRefs              ((u32)0x00000042 | nxClass_Frame | nxPrptype_1Dref)      //--r
      // Frame:Mux:Subframes
#define nxPropFrm_MuxSubframeRefs               ((u32)0x00000043 | nxClass_Frame | nxPrptype_1Dref)      //--r
      // Signal:Byte Order
#define nxPropSig_ByteOrdr                      ((u32)0x00000001 | nxClass_Signal | nxPrptype_u32)       //--rw
      // Signal:Comment
#define nxPropSig_Comment                       ((u32)0x00000002 | nxClass_Signal | nxPrptype_string)    //--rw
      // Signal:Configuration Status
#define nxPropSig_ConfigStatus                  ((u32)0x00000009 | nxClass_Signal | nxPrptype_u32)       //--r
      // Signal:Data Type
#define nxPropSig_DataType                      ((u32)0x00000003 | nxClass_Signal | nxPrptype_u32)       //--rw
      // Signal:Default Value
#define nxPropSig_Default                       ((u32)0x00000004 | nxClass_Signal | nxPrptype_f64)       //--rw
      // Signal:Frame
#define nxPropSig_FrameRef                      ((u32)0x00000005 | nxClass_Signal | nxPrptype_ref)       //--r
      // Signal:Maximum Value
#define nxPropSig_Max                           ((u32)0x00000006 | nxClass_Signal | nxPrptype_f64)       //--rw
      // Signal:Minimum Value
#define nxPropSig_Min                           ((u32)0x00000007 | nxClass_Signal | nxPrptype_f64)       //--rw
      // Signal:Name (Short)
#define nxPropSig_Name                          ((u32)0x00000008 | nxClass_Signal | nxPrptype_string)    //--rw
      // Signal:Name (Unique to Cluster)
#define nxPropSig_NameUniqueToCluster           ((u32)0x00000010 | nxClass_Signal | nxPrptype_string)    //--r
      // Signal:Number of Bits
#define nxPropSig_NumBits                       ((u32)0x00000012 | nxClass_Signal | nxPrptype_u32)       //--rw
      // Signal:Scaling Factor
#define nxPropSig_ScaleFac                      ((u32)0x00000013 | nxClass_Signal | nxPrptype_f64)       //--rw
      // Signal:Scaling Offset
#define nxPropSig_ScaleOff                      ((u32)0x00000014 | nxClass_Signal | nxPrptype_f64)       //--rw
      // Signal:Start Bit
#define nxPropSig_StartBit                      ((u32)0x00000015 | nxClass_Signal | nxPrptype_u32)       //--rw
      // Signal:Unit
#define nxPropSig_Unit                          ((u32)0x00000016 | nxClass_Signal | nxPrptype_string)    //--rw
      // Signal:Mux:Data Multiplexer?
#define nxPropSig_MuxIsDataMux                  ((u32)0x00000030 | nxClass_Signal | nxPrptype_bool)      //--rw
      // Signal:Mux:Dynamic?
#define nxPropSig_MuxIsDynamic                  ((u32)0x00000031 | nxClass_Signal | nxPrptype_bool)      //--r
      // Signal:Mux:Multiplexer Value
#define nxPropSig_MuxValue                      ((u32)0x00000032 | nxClass_Signal | nxPrptype_u32)       //--r
      // Signal:Mux:Subframe
#define nxPropSig_MuxSubfrmRef                  ((u32)0x00000033 | nxClass_Signal | nxPrptype_ref)       //--r
      // Subframe:Configuration Status
#define nxPropSubfrm_ConfigStatus               ((u32)0x00000009 | nxClass_Subframe | nxPrptype_u32)     //--r
      // Subframe:Dynamic Signals
#define nxPropSubfrm_DynSigRefs                 ((u32)0x00000001 | nxClass_Subframe | nxPrptype_1Dref)   //--r
      // Subframe:"Frame"
#define nxPropSubfrm_FrmRef                     ((u32)0x00000002 | nxClass_Subframe | nxPrptype_ref)     //--r
      // Subframe:Multiplexer Value
#define nxPropSubfrm_MuxValue                   ((u32)0x00000003 | nxClass_Subframe | nxPrptype_u32)     //--rw
      // Subframe:Name
#define nxPropSubfrm_Name                       ((u32)0x00000004 | nxClass_Subframe | nxPrptype_string)  //--rw
      // ECU:Cluster
#define nxPropECU_ClstRef                       ((u32)0x00000001 | nxClass_ECU | nxPrptype_ref)          //--r
      // ECU:Comment
#define nxPropECU_Comment                       ((u32)0x00000005 | nxClass_ECU | nxPrptype_string)       //--rw
      // ECU:Configuration Status
#define nxPropECU_ConfigStatus                  ((u32)0x00000009 | nxClass_ECU | nxPrptype_u32)          //--r
      // ECU:Name
#define nxPropECU_Name                          ((u32)0x00000002 | nxClass_ECU | nxPrptype_string)       //--rw
      // ECU:Frames Received
#define nxPropECU_RxFrmRefs                     ((u32)0x00000003 | nxClass_ECU | nxPrptype_1Dref)        //--rw
      // ECU:Frames Transmitted
#define nxPropECU_TxFrmRefs                     ((u32)0x00000004 | nxClass_ECU | nxPrptype_1Dref)        //--rw
      // ECU:FlexRay:Coldstart?
#define nxPropECU_FlexRayIsColdstart            ((u32)0x00000010 | nxClass_ECU | nxPrptype_bool)         //--r
      // ECU:FlexRay:Startup Frame
#define nxPropECU_FlexRayStartupFrameRef        ((u32)0x00000011 | nxClass_ECU | nxPrptype_ref)          //--r

/***********************************************************************
   C O N S T A N T S   F O R   F U N C T I O N   P A R A M E T E R S
***********************************************************************/

   // Parameter Mode of function nxCreateSession
#define nxMode_SignalInSinglePoint           0  // SignalInSinglePoint
#define nxMode_SignalInWaveform              1  // SignalInWaveform
#define nxMode_SignalInXY                    2  // SignalInXY
#define nxMode_SignalOutSinglePoint          3  // SignalOutSinglePoint
#define nxMode_SignalOutWaveform             4  // SignalOutWaveform
#define nxMode_SignalOutXY                   5  // SignalOutXY
#define nxMode_FrameInStream                 6  // FrameInStream
#define nxMode_FrameInQueued                 7  // FrameInQueued
#define nxMode_FrameInSinglePoint            8  // FrameInSinglePoint
#define nxMode_FrameOutStream                9  // FrameOutStream
#define nxMode_FrameOutQueued                10 // FrameOutQueued
#define nxMode_FrameOutSinglePoint           11 // FrameOutSinglePoint

   // Parameter Scope of functions nxStart, nxStop
#define nxStartStop_Normal                   0  // StartStop_Normal
#define nxStartStop_SessionOnly              1  // StartStop_SessionOnly
#define nxStartStop_InterfaceOnly            2  // StartStop_InterfaceOnly

   // Parameter Modifier of nxBlink
#define nxBlink_Disable                      0  // Blink_Disable
#define nxBlink_Enable                       1  // Blink_Enable

   // Terminal names for nxConnectTerminals and nxDisconnectTerminals (source or destination)
#define nxTerm_PXI_Trig0                     "PXI_Trig0"             // PXI_Trig0 same as RTSI0
#define nxTerm_PXI_Trig1                     "PXI_Trig1"
#define nxTerm_PXI_Trig2                     "PXI_Trig2"
#define nxTerm_PXI_Trig3                     "PXI_Trig3"
#define nxTerm_PXI_Trig4                     "PXI_Trig4"
#define nxTerm_PXI_Trig5                     "PXI_Trig5"
#define nxTerm_PXI_Trig6                     "PXI_Trig6"
#define nxTerm_PXI_Trig7                     "PXI_Trig7"
#define nxTerm_FrontPanel0                   "FrontPanel0"
#define nxTerm_FrontPanel1                   "FrontPanel1"
#define nxTerm_PXI_Star                      "PXI_Star"
#define nxTerm_PXI_Clk10                     "PXI_Clk10"
#define nxTerm_10MHzTimebase                 "10MHzTimebase"
#define nxTerm_1MHzTimebase                  "1MHzTimebase"
#define nxTerm_MasterTimebase                "MasterTimebase"
#define nxTerm_CommTrigger                   "CommTrigger"
#define nxTerm_StartTrigger                  "StartTrigger"
#define nxTerm_FlexRayStartCycle             "FlexRayStartCycle"
#define nxTerm_FlexRayMacrotick              "FlexRayMacrotick"

   /* StateID for nxReadState
   These constants use an encoding similar to property ID (nxProp_ prefix).
   */
      // Current time of the interface (using nxTimestamp_t)
#define nxState_TimeCurrent                  ((u32)0x00000001 | nxClass_Interface | nxPrptype_time)   // TimeCurrent
      // Time when communication began on the interface (protocol operational / integrated)
#define nxState_TimeCommunicating            ((u32)0x00000002 | nxClass_Interface | nxPrptype_time)   // TimeCommunicating
      // Start time of the interface, when the attempt to communicate began (startup protocol)
#define nxState_TimeStart                    ((u32)0x00000003 | nxClass_Interface | nxPrptype_time)   // TimeStart
      // CAN communication: Use macros with prefix nxCANComm_Get_ to get fields of the u32
#define nxState_CANComm                      ((u32)0x00000010 | nxClass_Interface | nxPrptype_u32)    // CANComm
      // FlexRay communication: Use macros with prefix nxFlexRayComm_Get_ to get fields of the u32
#define nxState_FlexRayComm                  ((u32)0x00000020 | nxClass_Interface | nxPrptype_u32)    // FlexRayComm
      // FlexRay statistics: Use typedef nxFlexRayStats_t to read these statistics using a struct of multiple u32
#define nxState_FlexRayStats                 ((u32)0x00000021 | nxClass_Interface | nxPrptype_1Du32)  // FlexRayStats

   // Macros to get fields of u32 returned by nxReadState of nxState_CANComm
      // Get CAN communication state; uses constants with prefix nxCANCommState_
#define nxCANComm_Get_CommState(StateValue)  ((u8)( (u32)StateValue         & 0x0000000F))
      // Get CAN transceiver error (!NERR); 1 = error, 0 = no error
#define nxCANComm_Get_TcvrErr(StateValue)    ((u8)( ((u32)StateValue >> 4)  & 0x00000001))
      // Get indication of CAN controller/transceiver sleep; 1 = asleep, 0 = awake
#define nxCANComm_Get_Sleep(StateValue)      ((u8)( ((u32)StateValue >> 5)  & 0x00000001))
      // Get last bus error that incremented counters; uses constants with prefix nxCANLastErr_
#define nxCANComm_Get_LastErr(StateValue)    ((u8)( ((u32)StateValue >> 8)  & 0x0000000F))
      // Get Transmit Error Counter as defined by the CAN protocol specification
#define nxCANComm_Get_TxErrCount(StateValue) ((u8)( ((u32)StateValue >> 16) & 0x000000FF))
      // Get Receive Error Counter as defined by the CAN protocol specification
#define nxCANComm_Get_RxErrCount(StateValue) ((u8)( ((u32)StateValue >> 24) & 0x000000FF))

   // Communication state from nxCANComm_Get_State (nxCANComm_Get_CommState)
#define nxCANCommState_ErrorActive           0
#define nxCANCommState_ErrorPassive          1
#define nxCANCommState_BusOff                2
#define nxCANCommState_Init                  3

   // Last bus error from nxCANComm_Get_State (nxCANComm_Get_LastErr)
#define nxCANLastErr_None                    0
#define nxCANLastErr_Stuff                   1
#define nxCANLastErr_Form                    2
#define nxCANLastErr_Ack                     3
#define nxCANLastErr_Bit1                    4
#define nxCANLastErr_Bit0                    5
#define nxCANLastErr_CRC                     6

   // Macros to get fields of u32 returned by nxReadState of nxState_FlexRayComm
      /* Get FlexRay Protocol Operation Control (POC) state,
      which uses constants with prefix nxFlexRayPOCState_ */
#define nxFlexRayComm_Get_POCState(StateValue)        ((u8)((u32)StateValue & 0x0000000F))
      /* From FlexRay spec 9.3.1.3.4: "the number of consecutive even/odd cycle pairs
      (vClockCorrectionFailed) that have passed without clock synchronization having performed an offset or a rate
      correction due to lack of synchronization frames (as maintained by the POC process)."
      This value is used for comparison to the cluster thresholds MaxWithoutClockCorrectFatal and
      MaxWithoutClockCorrectionPassive (XNET properties nxPropClst_FlexRayMaxWoClkCorFat
      and nxPropClst_FlexRayMaxWoClkCorPas). */
#define nxFlexRayComm_Get_ClockCorrFailed(StateValue) ((u8)( ((u32)StateValue >> 4) & 0x0000000F))
      /* From FlexRay spec 9.3.1.3.1: "the number of consecutive even/odd cycle pairs (vAllowPassiveToActive)
      that have passed with valid rate and offset correction terms, but the node still in POC:normal passive
      state due to a host configured delay to POC:normal active state (as maintained by the POC process).
      This value is used for comparison to the interface threshold AllowPassiveToActive
      (XNET property nxPropSession_IntfFlexRayAlwPassAct). */
#define nxFlexRayComm_Get_PassiveToActiveCount(StateValue)  \
                                                      ((u8)( ((u32)StateValue >> 8) & 0x0000001F))

   // POC state (Protocol Operation Control state) from nxFlexRayPOC_Get_State
#define nxFlexRayPOCState_DefaultConfig      0
#define nxFlexRayPOCState_Ready              1
#define nxFlexRayPOCState_NormalActive       2
#define nxFlexRayPOCState_NormalPassive      3
#define nxFlexRayPOCState_Halt               4
#define nxFlexRayPOCState_Monitor            5
#define nxFlexRayPOCState_Config             15

   // Condition of nxWait
#define nxCondition_TransmitComplete          0x8001  // TransmitComplete
#define nxCondition_IntfCommunicating         0x8002  // IntfCommunicating
#define nxCondition_IntfRemoteWakeup          0x8003  // IntfRemoteWakeup

      // Constants for use with Timeout parameter of read and write functions
#define nxTimeout_None                       (0)
#define nxTimeout_Infinite                   (-1)

/***********************************************************************
   C O N S T A N T S   F O R   H A R D W A R E   P R O P E R T I E S
***********************************************************************/

// System/Device/Interface properties (hardware info)

   // Property ID nxPropSys_VerPhase
#define nxPhase_Development                  0
#define nxPhase_Alpha                        1
#define nxPhase_Beta                         2
#define nxPhase_Release                      3

   // Property ID nxPropDev_FormFac
#define nxDevForm_PXI                        0
#define nxDevForm_PCI                        1

   // Property ID nxPropIntf_CANTermCap
#define nxCANTermCap_No                      0
#define nxCANTermCap_Yes                     1

   // Property ID nxPropIntf_CANTcvrCap
#define nxCANTcvrCap_HS                      0
#define nxCANTcvrCap_LS                      1
#define nxCANTcvrCap_XS                      3

   // Property ID nxPropIntf_Protocol and nxPropClst_Protocol
#define nxProtocol_CAN                       0
#define nxProtocol_FlexRay                   1

/***********************************************************************
   C O N S T A N T S   F O R   S E S S I O N   P R O P E R T I E S
***********************************************************************/

// Session properties (including runtime interface properties)

   // Macro to set nxPropSession_IntfBaudRate for an advanced CAN baud rate (bit timings)
   // If you pass a basic baud rate like 125000 or 500000, NI-XNET calculates bit timings for you
#define nxAdvCANBaudRate_Set(TimeQuantum, TimeSeg0, TimeSeg1, SyncJumpWidth) ( \
            (((u32)TimeQuantum) & 0x0000FFFF) | \
            (((u32)TimeSeg0 << 16) & 0x000F0000) | \
            (((u32)TimeSeg1 << 20) & 0x00700000) | \
            (((u32)SyncJumpWidth << 24) & 0x03000000) | \
            ((u32)0x80000000) )

   // Macros to get fields of nxPropSession_IntfBaudRate for an advanced CAN baud rate
#define nxAdvCANBaudRate_Get_TimeQuantum(AdvBdRt)     ((u16)( ((u32)AdvBdRt) & 0x0000FFFF))
#define nxAdvCANBaudRate_Get_TimeSeg0(AdvBdRt)        ((u8)( ((u32)AdvBdRt >> 16) & 0x0000000F))
#define nxAdvCANBaudRate_Get_TimeSeg1(AdvBdRt)        ((u8)( ((u32)AdvBdRt >> 20) & 0x00000007))
#define nxAdvCANBaudRate_Get_SyncJumpWidth(AdvBdRt)   ((u8)( ((u32)AdvBdRt >> 24) & 0x00000003))
#define nxAdvCANBaudRate_Get_NumSamples(AdvBdRt)      ((u8)( ((u32)AdvBdRt >> 26) & 0x00000001))

   // Property ID nxPropSession_IntfCANTerm
#define nxCANTerm_Off                        0
#define nxCANTerm_On                         1

    // Property ID nxPropSession_IntfCANTcvrState
#define nxCANTcvrState_Normal                0
#define nxCANTcvrState_Sleep                 1
#define nxCANTcvrState_SWWakeup              2
#define nxCANTcvrState_SWHighSpeed           3

   // Property ID nxPropSession_IntfCANTcvrType
#define nxCANTcvrType_HS                     0
#define nxCANTcvrType_LS                     1
#define nxCANTcvrType_SW                     2

   // Property ID nxPropSession_IntfFlexRaySampPerMicro
#define nxFlexRaySampPerMicro_1              0
#define nxFlexRaySampPerMicro_2              1
#define nxFlexRaySampPerMicro_4              2

   // Property ID nxPropSession_IntfFlexRayTerm
#define nxFlexRayTerm_Off                    0
#define nxFlexRayTerm_On                     1

/***********************************************************************
   C O N S T A N T S   F O R   D A T A B A S E   P R O P E R T I E S
***********************************************************************/

// Database properties (Database/Cluster/ECU/Frame/Subframe/Signal)

   // Property ID nxPropClst_FlexRayChannels and nxPropFrm_FlexRayChAssign
#define nxFrmFlexRayChAssign_A               1
#define nxFrmFlexRayChAssign_B               2
#define nxFrmFlexRayChAssign_AandB           3

   // Property ID nxPropClst_FlexRaySampClkPer
#define nxClstFlexRaySampClkPer_p0125us      0
#define nxClstFlexRaySampClkPer_p025us       1
#define nxClstFlexRaySampClkPer_p05us        2

   // Property ID nxPropClst_Protocol uses prefix nxProtocol_

   // Property ID nxPropFrm_FlexRayTimingType
#define nxFrmFlexRayTiming_Cyclic            0
#define nxFrmFlexRayTiming_Event             1

   // Property ID nxPropFrm_CANTimingType
#define nxFrmCANTiming_CyclicData            0
#define nxFrmCANTiming_EventData             1
#define nxFrmCANTiming_CyclicRemote          2
#define nxFrmCANTiming_EventRemote           3

   // Property ID nxPropSig_ByteOrdr
#define nxSigByteOrdr_LittleEndian           0  // Intel
#define nxSigByteOrdr_BigEndian              1  // Motorola

   // Property ID nxPropSig_DataType
#define nxSigDataType_Signed                 0
#define nxSigDataType_Unsigned               1
#define nxSigDataType_IEEEFloat              2

/***********************************************************************
                            D A T A   T Y P E S
***********************************************************************/

   /* The ANSI C99 standard defines simple numeric types of a specific size,
   such as int32_t for a signed 32-bit integer.
   Many C/C++ compilers are not ANSI C99 by default, such as Microsoft Visual C/C++.
   Therefore, NI-XNET does not require use of ANSI C99.
   Since NI-XNET does not attempt to override ANSI C99 types (as defined in stdint.h),
   it uses legacy National Instruments numeric types such as i32. If desired, you can use
   ANSI C99 numeric types instead of the analogous NI-XNET numeric type
   (i.e. int32_t instead of i32). */

#ifndef _NIDAQ_Header_      // Traditional NI-DAQ header defines numeric types same as below.
#ifndef nNISS100_kCPP       // Same for NI SS layer.

#ifndef _NI_i8_DEFINED_
#define _NI_i8_DEFINED_
typedef signed char        i8;
#endif
#ifndef _NI_i16_DEFINED_
#define _NI_i16_DEFINED_
typedef signed short       i16;
#endif
#ifndef _NI_i32_DEFINED_
#define _NI_i32_DEFINED_
typedef signed long        i32;
#endif
#ifndef _NI_i64_DEFINED_
#define _NI_i64_DEFINED_
#if defined(_MSC_VER)
   typedef __int64     i64;
#elif defined(__GNUC__)
   typedef long long   i64;
#endif
#endif
#ifndef _NI_u8_DEFINED_
#define _NI_u8_DEFINED_
typedef unsigned char      u8;
#endif
#ifndef _NI_u16_DEFINED_
#define _NI_u16_DEFINED_
typedef unsigned short     u16;
#endif
#ifndef _NI_u32_DEFINED_
#define _NI_u32_DEFINED_
typedef unsigned long      u32;
#endif
#ifndef _NI_u64_DEFINED_
#define _NI_u64_DEFINED_
#if (defined(_MSC_VER) || defined(_CVI_) || defined(__BORLANDC__))
   typedef unsigned __int64     u64;
#elif defined(__GNUC__)
   typedef unsigned long long   u64;
#endif
#endif
#ifndef _NI_f32_DEFINED_
#define _NI_f32_DEFINED_
typedef float              f32;
#endif
#ifndef _NI_f64_DEFINED_
#define _NI_f64_DEFINED_
typedef double             f64;
#endif
typedef void*              nxVoidPtr;
typedef u32*               nxU32Ptr;
#endif // nNISS100_kCPP
#endif // _NIDAQ_Header_

   // Session Reference (handle).
typedef u32 nxSessionRef_t;

   // Database Reference (handle).
typedef u32 nxDatabaseRef_t;

typedef i32 nxStatus_t;       // Return value

   // Absolute timestamp.
typedef u64 nxTimestamp_t;

typedef struct _nxFlexRayStats_t {
      u32 NumSyntaxErrorChA;
      u32 NumSyntaxErrorChB;
      u32 NumContentErrorChA;
      u32 NumContentErrorChB;
      u32 NumSlotBoundaryViolationChA;
      u32 NumSlotBoundaryViolationChB;
   } nxFlexRayStats_t;

/***********************************************************************
                                F R A M E
***********************************************************************/

#define nxFrameType_CAN_Data                 0x00
#define nxFrameType_CAN_Remote               0x01
#define nxFrameType_FlexRay_Data             0x20
#define nxFrameType_FlexRay_Null             0x21

   /* For Data frames, your application may not be concerned with specifics for
   CAN or FlexRay. For example, you can use fields of the frame to determine
   the contents of Payload, and write general-purpose code to map signal
   values in/out of the Payload data bytes.
   This macro can be used with the frame's Type to determine if the frame is a
   data frame. The macro is used in boolean conditionals. */
#define nxFrameType_IsData(frametype) \
            ((u8)(frametype) & 0x1F) == 0)

#define nxFrameId_CAN_IsExtended             0x20000000

   // Macros to get fields of frame Identifier for FlexRay input
#define nxFrameId_FlexRay_Get_Slot(FrameId)        (u16)( ((u32)FrameId) & 0x0000FFFF)

   /* When Type is nxFrameType_FlexRay_Data,
   the following bitmasks are used with the Flags field.
   */
#define nxFrameFlags_FlexRay_Startup         0x01     // Startup frame
#define nxFrameFlags_FlexRay_Sync            0x02     // Sync frame
#define nxFrameFlags_FlexRay_Preamble        0x04     // Preamble bit
#define nxFrameFlags_FlexRay_ChA             0x10     // Transfer on Channel A
#define nxFrameFlags_FlexRay_ChB             0x20     // Transfer on Channel B

#define nxFrameFlags_TransmitEcho            0x80

#define  nxInternal_PadPayload(paylod) \
            ( (u16)(paylod) ? (( (u16)(paylod) + 7) & 0x01F8) : 8)

#define  nxFrameFixed_t(payld) \
            struct { \
               nxTimestamp_t       Timestamp; \
               u32                 Identifier; \
               u8                  Type; \
               u8                  Flags; \
               u8                  Info; \
               u8                  PayloadLength; \
               u8                  Payload[ nxInternal_PadPayload(payld) ]; \
            }

typedef nxFrameFixed_t(8) nxFrameCAN_t;
typedef nxFrameFixed_t(1) nxFrameVar_t;

#define nxSizeofFrameHeader                  (16)

#define nxFrameSize(payload) \
            (  nxSizeofFrameHeader + nxInternal_PadPayload(payload) )

   /* Use this macro to iterate through variable-length frames.
   You call this macro as a function, as if it used the following prototype:
      nxFrameVar_t * nxFrameIterate(nxFrameVar_t * frameptr);
   The input parameter must be initialized to point to the header of a valid frame.
   The macro returns a pointer to the header of the next frame in the buffer.
   In other words, the macro will iterate from one variable-length frame to
   the next variable-length frame.
   */
#define nxFrameIterate(frameptr) \
            (nxFrameVar_t *) ( (u8 *)(frameptr) + nxFrameSize((frameptr)->PayloadLength) )

/***********************************************************************
       F U N C T I O N   P R O T O T Y P E S  :  S E S S I O N
***********************************************************************/

nxStatus_t _NXFUNC nxCreateSession (
                           const char * DatabaseName,
                           const char * ClusterName,
                           const char * List,
                           const char * Interface,
                           u32 Mode,
                           nxSessionRef_t * SessionRef);

nxStatus_t _NXFUNC nxCreateSessionByRef (
                           u32 NumberOfDatabaseRef,
                           nxDatabaseRef_t * ArrayOfDatabaseRef,
                           const char * Interface,
                           u32 Mode,
                           nxSessionRef_t * SessionRef);

nxStatus_t _NXFUNC nxGetProperty (
                           nxSessionRef_t SessionRef,
                           u32 PropertyID,
                           u32 PropertySize,
                           void * PropertyValue);

nxStatus_t _NXFUNC nxGetPropertySize (
                           nxSessionRef_t SessionRef,
                           u32 PropertyID,
                           u32 * PropertySize);

nxStatus_t _NXFUNC nxSetProperty (
                           nxSessionRef_t SessionRef,
                           u32 PropertyID,
                           u32 PropertySize,
                           void * PropertyValue);

nxStatus_t _NXFUNC nxReadFrame (
                           nxSessionRef_t SessionRef,
                           void * Buffer,
                           u32 SizeOfBuffer,
                           f64 Timeout,
                           u32 * NumberOfBytesReturned);

nxStatus_t _NXFUNC nxReadSignalSinglePoint (
                           nxSessionRef_t SessionRef,
                           f64 * ValueBuffer,
                           u32 SizeOfValueBuffer,
                           nxTimestamp_t * TimestampBuffer,
                           u32 SizeOfTimestampBuffer);

nxStatus_t _NXFUNC nxReadSignalWaveform (
                           nxSessionRef_t SessionRef,
                           f64 Timeout,
                           nxTimestamp_t * StartTime,
                           f64 * DeltaTime,
                           f64 * ValueBuffer,
                           u32 SizeOfValueBuffer,
                           u32 * NumberOfValuesReturned);

nxStatus_t _NXFUNC nxReadSignalXY (
                           nxSessionRef_t SessionRef,
                           nxTimestamp_t * TimeLimit,
                           f64 * ValueBuffer,
                           u32 SizeOfValueBuffer,
                           nxTimestamp_t * TimestampBuffer,
                           u32 SizeOfTimestampBuffer,
                           u32 * NumPairsBuffer,
                           u32 SizeOfNumPairsBuffer);

nxStatus_t _NXFUNC nxReadState (
                           nxSessionRef_t SessionRef,
                           u32 StateID,
                           u32 StateSize,
                           void * StateValue,
                           nxStatus_t * Fault);

nxStatus_t _NXFUNC nxWriteFrame (
                           nxSessionRef_t SessionRef,
                           void * Buffer,
                           u32 NumberOfBytesForFrames,
                           f64 Timeout);

nxStatus_t _NXFUNC nxWriteSignalSinglePoint (
                           nxSessionRef_t SessionRef,
                           f64 * ValueBuffer,
                           u32 SizeOfValueBuffer);

nxStatus_t _NXFUNC nxWriteSignalWaveform (
                           nxSessionRef_t SessionRef,
                           f64 Timeout,
                           f64 * ValueBuffer,
                           u32 SizeOfValueBuffer);

nxStatus_t _NXFUNC nxWriteSignalXY (
                           nxSessionRef_t SessionRef,
                           f64 Timeout,
                           f64 * ValueBuffer,
                           u32 SizeOfValueBuffer,
                           nxTimestamp_t * TimestampBuffer,
                           u32 SizeOfTimestampBuffer,
                           u32 * NumPairsBuffer,
                           u32 SizeOfNumPairsBuffer);


nxStatus_t _NXFUNC nxBlink (
                           nxSessionRef_t InterfaceRef,
                           u32 Modifier);

nxStatus_t _NXFUNC nxClear (
                           nxSessionRef_t SessionRef);

nxStatus_t _NXFUNC nxConnectTerminals (
                           nxSessionRef_t SessionRef,
                           const char * source,
                           const char * destination);


nxStatus_t _NXFUNC nxDisconnectTerminals (
                           nxSessionRef_t SessionRef,
                           const char * source,
                           const char * destination);

nxStatus_t _NXFUNC nxFlush (
                           nxSessionRef_t SessionRef);

nxStatus_t _NXFUNC nxStart (
                           nxSessionRef_t SessionRef,
                           u32 Scope);

nxStatus_t _NXFUNC nxStop (
                           nxSessionRef_t SessionRef,
                           u32 Scope);

void _NXFUNC nxStatusToString (
                           nxStatus_t Status,
                           u32 SizeofString,
                           char * StatusDescription);

nxStatus_t _NXFUNC nxSystemOpen (
                           nxSessionRef_t * SystemRef);

nxStatus_t _NXFUNC nxSystemClose (
                           nxSessionRef_t SystemRef);

nxStatus_t _NXFUNC nxWait (
                           nxSessionRef_t SessionRef,
                           u32 Condition,
                           u32 ParamIn,
                           f64 Timeout,
                           u32 * ParamOut);


/***********************************************************************
       F U N C T I O N   P R O T O T Y P E S  :  D A T A B A S E
***********************************************************************/

nxStatus_t _NXFUNC nxdbOpenDatabase (
                           const char * DatabaseName,
                           nxDatabaseRef_t * DatabaseRef);

nxStatus_t _NXFUNC nxdbCloseDatabase (
                           nxDatabaseRef_t DatabaseRef,
                           u32 CloseAllRefs);

nxStatus_t _NXFUNC nxdbCreateObject (
                           nxDatabaseRef_t ParentObjectRef,
                           u32 ObjectClass,
                           const char * ObjectName,
                           nxDatabaseRef_t * DbObjectRef);

nxStatus_t _NXFUNC nxdbFindObject (
                           nxDatabaseRef_t ParentObjectRef,
                           u32 ObjectClass,
                           const char * ObjectName,
                           nxDatabaseRef_t * DbObjectRef);

nxStatus_t _NXFUNC nxdbDeleteObject (
                           nxDatabaseRef_t DbObjectRef);

nxStatus_t _NXFUNC nxdbSaveDatabase (
                           nxDatabaseRef_t DatabaseRef,
                           const char * DbFilepath);

nxStatus_t _NXFUNC nxdbGetProperty (
                           nxDatabaseRef_t DbObjectRef,
                           u32 PropertyID,
                           u32 PropertySize,
                           void * PropertyValue);

nxStatus_t _NXFUNC nxdbGetPropertySize (
                           nxDatabaseRef_t DbObjectRef,
                           u32 PropertyID,
                           u32 * PropertySize);

nxStatus_t _NXFUNC nxdbSetProperty (
                           nxDatabaseRef_t DbObjectRef,
                           u32 PropertyID,
                           u32 PropertySize,
                           void * PropertyValue);

nxStatus_t _NXFUNC nxdbAddAlias (
                           const char * DatabaseAlias,
                           const char * DatabaseFilepath,
                           u32          DefaultBaudRate);

nxStatus_t _NXFUNC nxdbRemoveAlias (
                           const char * DatabaseAlias);

nxStatus_t _NXFUNC nxdbDeploy (
                           const char * IPAddress,
                           const char * DatabaseAlias,
                           u32 WaitForComplete,
                           u32 * PercentComplete);

nxStatus_t _NXFUNC nxdbUndeploy (
                           const char * IPAddress,
                           const char * DatabaseAlias);

nxStatus_t _NXFUNC nxdbGetDatabaseList (
                           const char * IPAddress,
                           u32 SizeofAliasBuffer,
                           char * AliasBuffer,
                           u32 SizeofFilepathBuffer,
                           char * FilepathBuffer,
                           u32 * NumberOfDatabases);

nxStatus_t _NXFUNC nxdbGetDatabaseListSizes (
                           const char * IPAddress,
                           u32 * SizeofAliasBuffer,
                           u32 * SizeofFilepathBuffer);

#ifdef __cplusplus
   /* See top of header file.  */
   }
#endif // __cplusplus

#endif // ___nixnet_h___
