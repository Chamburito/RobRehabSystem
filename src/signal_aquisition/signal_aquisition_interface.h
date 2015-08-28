#ifndef SIGNAL_AQUISITION_INTERFACE_H
#define SIGNAL_AQUISITION_INTERFACE_H

typedef struct _SignalAquisitionOperations
{
  int (*InitTask)( const char*, size_t );
  void (*EndTask)( int );
  double* (*Read)( int, unsigned int );
  size_t (*GetChannelsNumber)( int );
}
SignalAquisitionOperations;

typedef SignalAquisitionOperations* SignalAquisitionInterface;

#endif // SIGNAL_AQUISITION_INTERFACE_H 
