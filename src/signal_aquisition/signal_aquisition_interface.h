#ifndef SIGNAL_AQUISITION_INTERFACE_H
#define SIGNAL_AQUISITION_INTERFACE_H

typedef struct _SignalAquisitionOperations
{
  int (*InitTask)( const char* );
  void (*EndTask)( int );
  double* (*Read)( int, unsigned int, size_t* );
  bool (*AquireChannel)( int, unsigned int );
  void (*ReleaseChannel)( int, unsigned int );
  size_t (*GetMaxSamplesNumber)( int );
}
SignalAquisitionOperations;

typedef SignalAquisitionOperations* SignalAquisitionInterface;

#endif // SIGNAL_AQUISITION_INTERFACE_H 
