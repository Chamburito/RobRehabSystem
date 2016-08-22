/* 
 *  pci4eHelper.c
 *
 */

#include "pci4e_helper.h"

#include "pci4e_device.h"

/******** Global Variables **********************/
DEVICE_DATA m_PCI4EDevice[MAX_DEVICES];
long m_dwVendorID =  0x1892;
long  m_dwDeviceID = 0x5747;
int   m_iCardCount = 0;

// Helper Function
long Power(long base, int exp)
{
	long retVal = 1;
	int i = 0;
	for(i = 1; i <= exp; i++)
	{
		retVal *= base;
	}

	return  retVal;
}

/****************************************************
|	Exported Functions... 
****************************************************/

int PCI4E_CardCount()
{
	return m_iCardCount;
}

/******** PCI-4E Functions **********************/
int PCI4E_GetSlotNumber(short iDeviceNo, short *iSlotNo)
{
	int retVal = 0;

	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.
	else
		*iSlotNo = (short)m_PCI4EDevice[iDeviceNo].iSlotNo;
	
	return retVal;
}

int PCI4E_GotoU2CommandMode(short iDeviceNo, short iEncoder)
{
	int retVal = 0;
	long lCmdBits = 0;
	
	// Select the UART port number that will receive the 10ms signal.
	retVal = PCI4E_SetPortNumber(iDeviceNo, iEncoder);

	// Check return code...
	if(retVal == 0)
	{
		// Read the command register so that it may be preserved...
		retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lCmdBits);

		// Check return code...
		if(retVal == 0)
		{
			// Write 0 to bit 7 of the command register.
			retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, lCmdBits & 0xFFFFFF7F);

			// Check return code...
			if(retVal == 0)
			{
				// Write 1 to bit 7 of the command register.
				retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, lCmdBits | 0x80);
			}
		}
	}

	return retVal;
}

int PCI4E_Initialize(short *piDeviceCount )
{
    // Returns 0 if success otherwise non-zero.
    // Returns -1 if there are no devices detected.

    int i = 0;
    m_iCardCount = 0;   // intialize card count
			// TODO: check if already intialized

    // printf("PCI4E_Initialize() enter...\n");

    /* Attempt to open all available pci4e devices... */
    for(i=0; i<MAX_DEVICES; i++) {
        sprintf(m_PCI4EDevice[i].devname, "/dev/pci4e%x",i);
    // printf("Initializing device %s...\n", m_PCI4EDevice[i].devname);
	m_PCI4EDevice[i].fd = open(m_PCI4EDevice[i].devname, O_RDWR);
        if (m_PCI4EDevice[i].fd < 0) {
            // printf("Device not found rc = %d.\n", m_PCI4EDevice[i].fd);
        } else {
            // printf("Found device %s\n", m_PCI4EDevice[i].devname);
            m_iCardCount++;
	    // TODO: get slotNo and version...
        }
     }

    *piDeviceCount = m_iCardCount;

    if (m_iCardCount == 0)
        return NO_CARDS_DETECTED;    // No cards detected.
    else
        return 0;
}

int PCI4E_ReadRegister(short iDeviceNo, short iRegister, long *plVal)
{
    // Returns 0 if success otherwise non-zero.

    // Make sure device number is valid...
    if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
	return INVALID_DEVICE_NUMBER;	// Invalid device number.

    // Make sure device is open...
    if (!m_PCI4EDevice[iDeviceNo].fd || m_PCI4EDevice[iDeviceNo].fd == -1)
        return DEVICE_NOT_OPEN;

    // Make sure register number is valid...
    if (iRegister < 0 || iRegister > 39)
	return INVALID_REGISTER_NUMBER;
	
    struct pci4e_io_struct ios;

    ios.region = '0'; // Access memory starting at BAR0.
    ios.offset = iRegister * 4;
    ios.len = 4;    // always read 4 bytes   
	    
    if (ioctl(m_PCI4EDevice[iDeviceNo].fd, PCI4E_IOCREAD, &ios)) {
	fprintf(stderr, "pci4eHelper: ioctl(%s): %s\n",
		m_PCI4EDevice[iDeviceNo].devname, strerror(errno));
        return FATAL_ERROR;	
    } else {
	//printf("read  %i bytes from '%c':0x%08lx -- 0x%08lx\n",
	//	ios.len, ios.region, ios.offset, ios.value);
    }	    

    *plVal = ios.value;
    return 0;
}

void PCI4E_Shutdown()
{
	int i = 0;

	// Close all open connections...
	for ( i=0; i < MAX_DEVICES; i++ )
	{
		if( m_PCI4EDevice[i].fd )
			close(m_PCI4EDevice[i].fd);
	}
}

int PCI4E_GetCount(short iDeviceNo, short iEncoder, long *lVal)
{
	int retVal = 0;
	short regAddr = 0;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if (retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			// Write to register 1. 
			regAddr = (iEncoder * 8) + OUTPUT_LATCH_REGISTER;
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, 0 );

			// Read from register 1.
			if ( retVal == 0 )
			{
				//regAddr = (iEncoder * 8) + OUTPUT_LATCH_REGISTER;
				retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, lVal);
			}
		}
	}
	return retVal;
}

int PCI4E_GetPresetValue(short iDeviceNo, short iEncoder, long *lVal)
{
	int retVal = 0;
	short regAddr = 0;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if (retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			// Write to register 0. 
			regAddr = (iEncoder * 8) + PRESET_REGISTER;
			retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, lVal );
		}
	}
	return retVal;
}

int PCI4E_SetPresetValue(short iDeviceNo, short iEncoder, long lVal)
{
	int retVal = 0;
	short regAddr = 0;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if (retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			// Write to register 0. 
			regAddr = (iEncoder * 8) + PRESET_REGISTER;
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lVal );
		}
	}
	return retVal;
}

int PCI4E_GetMultiplier(short iDeviceNo, short iEncoder, short *iMode)
{
	long retVal = 0;
	short regAddr = 0;
	long lCtrlState = 0;

	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			// Get the current control state...
			regAddr = (iEncoder * 8) + CONTROL_REGISTER;
			retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, &lCtrlState );
			if (retVal == 0)
			{	
				// Bits 14 and 15 contain the quad mode.
				*iMode = (lCtrlState >> 14) & 0x3; 
			}
		}
	}
	return retVal;
}

int PCI4E_SetMultiplier(short iDeviceNo, short iEncoder, short iMode)
{
	int retVal = 0;
		
	long lCtrlState = 0;
	short regAddr = 0;
	int maxEncoder = 0;
	short i = 0;

	if( iEncoder > -1 && iEncoder < 4)
	{
		maxEncoder = iEncoder + 1;
		i = iEncoder;
	}
	else if( iEncoder == 4 )
		maxEncoder = 4;
	else
		retVal = INVALID_ENCODER_NUMBER;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if (retVal == 0)
	{
		for( ; i < maxEncoder; i++ )
		{	
			// Make sure the mode is valid...
			if ( iMode < 0 || iMode > 3 )
				retVal = INVALID_QUADRATURE_MODE;	// Invalid Quadrature mode
			{
				// Get the current control state...
				regAddr = (i * 8) + CONTROL_REGISTER;
				retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, &lCtrlState );
				if (retVal == 0)
				{
					lCtrlState = lCtrlState & 0xFF3FFF;

					switch( iMode )
					{
						
						case 0:
							lCtrlState = lCtrlState | 0x0000;
							break;
						case 1:
							lCtrlState = lCtrlState | 0x4000;
							break;
						case 2:
							lCtrlState = lCtrlState | 0x8000;
							break;
						case 3:
							lCtrlState = lCtrlState | 0xC000;
							break;
					}
				
					// Write to register 0. 
					retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlState );
				}
			}
		}
	}
	
	return retVal;
}

int PCI4E_SetCount(short iDeviceNo, short iEncoder, long lVal)
{
	int retVal = 0;
	short regAddr = 0;
	long presetVal = 0;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if (retVal == 0)
	{
		// Make sure encoder is valid...
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			retVal = PCI4E_GetPresetValue(iDeviceNo, iEncoder, &presetVal);
			if( retVal == 0 )
			{
				// Write new position value to preset register...
				regAddr = (iEncoder * 8) + PRESET_REGISTER;
				retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lVal);
				if ( retVal == 0)
				{
					// Write value to transfer preset register...
					regAddr = (iEncoder * 8) + TRANSFER_PRESET_REGISTER;
					retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, 0);
					if( retVal == 0 )
					{
						// Write old preset value back to preset register...
						regAddr = (iEncoder * 8) + PRESET_REGISTER;
						retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, presetVal);
					}
				}
			}
		}
	}	
	return retVal;
}

int PCI4E_GetMatch(short iDeviceNo, short iEncoder, long *lVal)
{
	// Match Register: used as reference to generate interrupt when counter matches value.
	int retVal = 0;
	short regAddr = iEncoder * 8 + MATCH_REGISTER;
	
	// Test for valid device number...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			// Get the preset register value...
			retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, lVal );
		}
	}
	return retVal;
}

int PCI4E_SetMatch(short iDeviceNo, short iEncoder, long lVal)
{
	// Match Register: used as reference to generate interrupt when counter matches value.
	int retVal = 0;
	short regAddr = iEncoder * 8 + MATCH_REGISTER;
	
	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{	// Write the preset register value...
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lVal );
		}
	}
	return retVal;

}

int PCI4E_GetControlMode(short iDeviceNo, short iEncoder, long *plVal)
{
	// Control Register: holds bits that control operation of channel.
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Test for valid device number...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			// Get the control register value...
			retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, plVal );
		}
	}
	return retVal;
}

int PCI4E_SetControlMode(short iDeviceNo, short iEncoder, long lVal)
{
	// Control Register: holds bits that control operation of channel.
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{	// Write the control register value...
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lVal );
			if(lVal == 0)
				retVal = CONTROL_MODE_IS_ZERO;
		}
	}
	return retVal;
}

int PCI4E_GetStatus(short iDeviceNo, short iEncoder, long *plVal)
{
	// Status Register: reports status of channel.
	int retVal = 0;
	short regAddr = iEncoder * 8 + STATUS_REGISTER;
	
	// Test for valid device number...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{
			// Get the preset register value...
			retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, plVal );
		}
	}
	return retVal;
}

int PCI4E_GetEnableAccumulator(short iDeviceNo, short iEncoder, int *bVal)
{
	// Enabled: Master enable for accummulator (must be set to count).
	long lCtrlMode = 0;
	int retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x40000) > 0; // Check if bit 18 is set...
	return retVal;
}

int PCI4E_SetEnableAccumulator(short iDeviceNo, short iEncoder, int bVal)
{
	// Enabled: Master enable for accummulator (must be set to count).
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x40000) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFBFFFF;	// Disabled bit 18.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x40000;	// Enable bit 18.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}
	return retVal;
}

int PCI4E_GetForward(short iDeviceNo, short iEncoder, int *bVal)
{
	// Forward: swap a and b quadrature Foward = True, Reverse = False.
	long lCtrlMode = 0;
	int retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x80000) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 19 is set...
	return retVal;
}

int PCI4E_SetForward(short iDeviceNo, short iEncoder, int bVal)
{
	// Forward: swap a and b quadrature Foward = True, Reverse = False.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x80000) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xF7FFFF;	// Disabled bit 19.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x80000;	// Enable bit 19.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}
	return retVal;

}

int PCI4E_GetEnableIndex(short iDeviceNo, short iEncoder, int *bVal)
{
	// EnableIndex: when set, index will either reset or preset accummulator.
	long lCtrlMode = 0;
	int retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x100000) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 20 is set...
	return retVal;
}

int PCI4E_SetEnableIndex(short iDeviceNo, short iEncoder, int bVal)
{
	// EnableIndex: when set, index will either reset or preset accummulator.
	// InvertIndex: set for active low index.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x100000) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xEFFFFF;	// Disabled bit 20.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x100000;	// Enable bit 20.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}
	return retVal;
}

int PCI4E_GetInvertIndex(short iDeviceNo, short iEncoder, int *bVal)
{
	// InvertIndex: set for active low index.
	long lCtrlMode = 0;
	int retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x200000) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 21 is set...
	return retVal;
}

int PCI4E_SetInvertIndex(short iDeviceNo, short iEncoder, int bVal)
{
	// InvertIndex: set for active low index.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( bVal == 0 && (lCtrlMode && 0x200000) )	// new value is False and old is True
	{
		lCtrlMode = lCtrlMode & 0xDFFFFF;	// Disabled bit 21.
		retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
	}
	else						// new value is True and old is False
	{
		lCtrlMode = lCtrlMode | 0x200000;	// Enable bit 21.
		retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
	}

	return retVal;
}

int PCI4E_GetPresetOnIndex(short iDeviceNo, short iEncoder, int *bVal)
{
	// PresetOnIndex: causes preset when set and index is enabled.
	long lCtrlMode = 0;
	int retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x400000) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 22 is set...
	return retVal;
}

int PCI4E_SetPresetOnIndex(short iDeviceNo, short iEncoder, int bVal)
{
	// PresetOnIndex: causes preset when set and index is enabled.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);

	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x400000) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xBFFFFF;	// Disabled bit 22.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x400000;	// Enable bit 22.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetCaptureEnabled(short iDeviceNo, short iEncoder, int *bVal)
{
	// CaptureEnabled: allow trigger_in to cause transfer from accummulator to output.
	long lCtrlMode = 0;
	int retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x800000) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 23 is set...
	return retVal;
}

int PCI4E_SetCaptureEnabled(short iDeviceNo, short iEncoder, int bVal)
{
	// CaptureEnabled: allow trigger_in to cause transfer from accummulator to output.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x800000) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0x7FFFFF;	// Disabled bit 23.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x800000;	// Enable bit 23.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetCounterMode(short iDeviceNo, short iEncoder, short *iMode)
{
	// CounterMode: Governs counter behavior and limits: 
	//	0 = acc. acts like a 24 bit counter 
	//	1 = acc. uses preset register in range-limit mode 
	//	2 = acc. uses preset register in non-recycle mode 
	//	3 = acc. uses preset register in modulo-N mode.
	long lCtrlMode = 0;
	int retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*iMode = (lCtrlMode & 0x30000) >> 16; 
	return retVal;
}

int PCI4E_SetCounterMode(short iDeviceNo, short iEncoder, short iMode)
{
	// CounterMode: Governs counter behavior and limits: 
	//	0 = acc. acts like a 24 bit counter 
	//	1 = acc. uses preset register in range-limit mode 
	//	2 = acc. uses preset register in non-recycle mode 
	//	3 = acc. uses preset register in modulo-N mode.
	long lCtrlMode = 0;
	int retVal = 0;

	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	if( iMode < 0 || iMode > 3 )
		retVal = INVALID_COUNTER_MODE;
	else
	{
		// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);

		lCtrlMode = lCtrlMode & 0xFCFFFF;				// Disabled bit 16 and 17.

		switch ( iMode )
		{
			case 0:
				// Do nothing...				// Both bits 16 and 17 should be disabled.
				break;
			case 1:
				lCtrlMode = lCtrlMode | 0x10000;		// Enable bit 16.
				break;
			case 2:
				lCtrlMode = lCtrlMode | 0x20000;		// Enable bit 17.
				break;
			case 3:
				lCtrlMode = lCtrlMode | 0x30000;		// Enable bits 16 and 17.
				break;
			default :
				retVal = INVALID_COUNTER_MODE;
				break;
		}

		if( retVal == 0 )		
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
	}
	
	return retVal;
}

int PCI4E_GetInterruptOnZero(short iDeviceNo, short iEncoder, int *bVal)
{
	// 	InterruptOnZero: Interrupt will be set when Count is zero.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal =  (lCtrlMode & 0x1) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 0 is set...
	return retVal;
}

int PCI4E_SetInterruptOnZero(short iDeviceNo, short iEncoder, int bVal)
{
	// 	InterruptOnZero: Interrupt will be set when Count is zero.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);

	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x1) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFFFE;	// Disabled bit 0.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x1;		// Enable bit 0
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetInterruptOnMatch(short iDeviceNo, short iEncoder, int *bVal)
{
	// InterruptOnMatch: Interrupt will be set when Count = match.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x2) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 1 is set...
	return retVal;
}

int PCI4E_SetInterruptOnMatch(short iDeviceNo, short iEncoder, int bVal)
{
	// InterruptOnMatch: Interrupt will be set when Count = match.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x2) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFFFD;	// Disabled bit 1.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x2;		// Enable bit 1
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetInterruptOnRollover(short iDeviceNo, short iEncoder, int *bVal)
{
	// InterruptOnRollover: Interrupt will be set when rolling over N-1 to 0 in modulo-N mode.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x4) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 2 is set...
	return retVal;
}

int PCI4E_SetInterruptOnRollover(short iDeviceNo, short iEncoder, int bVal)
{
	// InterruptOnRollover: Interrupt will be set when rolling over N-1 to 0 in modulo-N mode.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x4) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFFFB;	// Disabled bit 2.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x4;		// Enable bit 2
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetInterruptOnRollunder(short iDeviceNo, short iEncoder, int *bVal)
{
	// InterruptOnRollunder: Interrupt will be set when rolling under 0 to N-1 in modulo-N mode.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal =  (lCtrlMode & 0x8) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 3 is set...
	return retVal;
}

int PCI4E_SetInterruptOnRollunder(short iDeviceNo, short iEncoder, int bVal)
{
	// InterruptOnRollunder: Interrupt will be set when rolling under 0 to N-1 in modulo-N mode.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x8) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFFF7;	// Disabled bit 3.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x8;		// Enable bit 3
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetInterruptOnIndex(short iDeviceNo, short iEncoder, int *bVal)
{
	// InterruptOnIndex: Interrupt will be set when edge of index occurs.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x10) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 4 is set...
	return retVal;
}

int PCI4E_SetInterruptOnIndex(short iDeviceNo, short iEncoder, int bVal)
{
	// InterruptOnIndex: Interrupt will be set when edge of index occurs.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x10) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFFEF;	// Disabled bit 4.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x10;		// Enable bit 4.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}
	
	return retVal;
}

int PCI4E_GetInterruptOnIncrease(short iDeviceNo, short iEncoder, int *bVal)
{
	// InterruptOnIncrease: Interrupt will be set when Count increases.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x20) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 5 is set...
	return retVal;
}

int PCI4E_SetInterruptOnIncrease(short iDeviceNo, short iEncoder, int bVal)
{
	// InterruptOnIncrease: Interrupt will be set when Count increases.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x20) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFFDF;	// Disabled bit 5.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x20;		// Enable bit 5.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetInterruptOnDecrease(short iDeviceNo, short iEncoder, int *bVal)
{
	// InterruptOnDecrease: Interrupt will be set when Count decreases.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x40) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 6 is set...
	return retVal;
}

int PCI4E_SetInterruptOnDecrease(short iDeviceNo, short iEncoder, int bVal)
{
	// InterruptOnDecrease: Interrupt will be set when Count decreases.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x40) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFFBF;	// Disabled bit 6.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x40;		// Enable bit 6.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetTriggerOnZero(short iDeviceNo, short iEncoder, int *bVal)
{
	// TriggerOnZero: Trigger will be set when Count = 0.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x80) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 7 is set...
	return retVal;
}

int PCI4E_SetTriggerOnZero(short iDeviceNo, short iEncoder, int bVal)
{
	// TriggerOnZero: Trigger will be set when Count = 0.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x80) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFF7F;	// Disabled bit 7.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x80;		// Enable bit 7.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetTriggerOnMatch(short iDeviceNo, short iEncoder, int *bVal)
{
	// TriggerOnMatch: Trigger will be set when Count = match.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x100) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 8 is set...
	return retVal;
}

int PCI4E_SetTriggerOnMatch(short iDeviceNo, short iEncoder, int bVal)
{
	// TriggerOnMatch: Trigger will be set when Count = match.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x100) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFEFF;		// Disabled bit 8.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else							// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x100;			// Enable bit 8.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetTriggerOnRollover(short iDeviceNo, short iEncoder, int *bVal)
{
	// TriggerOnRollover: Trigger will be set when rolling over N-1 to 0 in modulo-N mode.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x200) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 9 is set...
	return retVal;
}

int PCI4E_SetTriggerOnRollover(short iDeviceNo, short iEncoder, int bVal)
{
	// TriggerOnRollover: Trigger will be set when rolling over N-1 to 0 in modulo-N mode.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x200) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFDFF;		// Disabled bit 9.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else							// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x200;			// Enable bit 9.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}
	return retVal;
}

int PCI4E_GetTriggerOnRollunder(short iDeviceNo, short iEncoder, int *bVal)
{
	// TriggerOnRollunder: Trigger will be set when rolling under 0 to N-1 in modulo-N mode.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x400) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 10 is set...
	return retVal;
}

int PCI4E_SetTriggerOnRollunder(short iDeviceNo, short iEncoder, int bVal)
{
	// TriggerOnRollunder: Trigger will be set when rolling under 0 to N-1 in modulo-N mode.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;
	
	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);

	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x800) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFFBFF;		// Disabled bit 10.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else							// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x400;			// Enable bit 10.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_GetTriggerOnIndex(short iDeviceNo, short iEncoder, int *bVal)
{
	// TriggerOnIndex:  Trigger will be set when edge of index occurs.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x800) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 11 is set...
	return retVal;
}

int PCI4E_SetTriggerOnIndex(short iDeviceNo, short iEncoder, int bVal)
{
	// TriggerOnIndex:  Trigger will be set when edge of index occurs.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x800) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFF7FF;		// Disabled bit 11.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else							// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x800;			// Enable bit 11.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}
	return retVal;
}

int PCI4E_GetTriggerOnIncrease(short iDeviceNo, short iEncoder, int *bVal)
{
	// TriggerOnIncrease: Trigger will be set when Count increases.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x1000) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 12 is set...
	return retVal;
}

int PCI4E_SetTriggerOnIncrease(short iDeviceNo, short iEncoder, int bVal)
{
	// TriggerOnIncrease: Trigger will be set when Count increases.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
		retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 )
	{
		if( bVal == 0 && (lCtrlMode && 0x1000) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFEFFF;	// Disabled bit 12.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x1000;		// Enable bit 12.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}
	return retVal;
}

int PCI4E_GetTimeStamp(short iDeviceNo, long *plVal)
{
	int retVal = 0;
	long lCmdVal = 0;
//	long lTm = 0;
	// To get the TimeStamp counter to latch to the TimeStamp Output Latch
    // a transition bit 5 of the CMD_REGISTER from 0 -> 1 must occur.
	
	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lCmdVal);

	retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0xFFFFFFDF & lCmdVal));

	if(retVal == 0)
	{
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0x20 | lCmdVal));
	}	

	if(retVal == 0)
	{
		retVal = PCI4E_ReadRegister(iDeviceNo, TIMESTAMP_OUTPUT_LATCH_REGISTER, plVal);
	}
//	lTm = *plVal;
	return retVal;
}

int PCI4E_GetTriggerOnDecrease(short iDeviceNo, short iEncoder, int *bVal)
{
	// TriggerOnDecrease: Trigger will be set when Count decreases.
	long lCtrlMode = 0;
	int  retVal = 0;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	*bVal = (lCtrlMode & 0x2000) > 0; // ? VARIANT_TRUE : VARIANT_FALSE; // Check if bit 13 is set...
	return retVal;
}

int PCI4E_SetTriggerOnDecrease(short iDeviceNo, short iEncoder, int bVal)
{
	// TriggerOnDecrease: Trigger will be set when Count decreases.
	long lCtrlMode = 0;
	int retVal = 0;
	short regAddr = iEncoder * 8 + CONTROL_REGISTER;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlMode);
	
	if( retVal == 0 ) 
	{
		if( bVal == 0 && (lCtrlMode && 0x2000) )	// new value is False and old is True
		{
			lCtrlMode = lCtrlMode & 0xFFDFFF;	// Disabled bit 13.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
		else						// new value is True and old is False
		{
			lCtrlMode = lCtrlMode | 0x2000;		// Enable bit 13.
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlMode);
		}
	}

	return retVal;
}

int PCI4E_ClearCapturedStatus(short iDeviceNo, short iEncoder)
{
	// Clear Interrupts: writes to the Status Register of then channel.
	int retVal = 0;
	short regAddr = iEncoder * 8 + STATUS_REGISTER;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{	// Write to the status register value...
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, 0xFFFFFFFF );
		}
	}
	return retVal;
}

int PCI4E_GetABSPosition(short iDeviceNo, short iEncoder, long *plVal)
{
	// Get the current control mode...
	// Enables the index and disable InvertIndex and disable PresetOnIndex...
	// Get value from register and ensure bit for specified encoder is low.
	// If not low then 
	//		Set it low by sending cmd 0 to register 7...	
	//		Wait a minimum of 65 milliseconds.
	// Set it hi by sending cmd 1 to register 7...	
	// Wait a minimum of 65 milliseconds.
	// Set it low by sending cmd 0 to register 7...	
	// Reset the control mode if change was made control mode.
	// Get and return then current count..
	int retVal = 0;
	long lCmdBits = 0;
	long lCtrlModeOld = 0;
	long lMask = Power(2, iEncoder);
	long lCtrlModeNew = 0;
	short regAddr = iEncoder * 8 + CMD_REGISTER;

	// Get the current control mode...
	retVal = PCI4E_GetControlMode(iDeviceNo, iEncoder, &lCtrlModeOld);
	
	if(retVal == 0)
	{
		lCtrlModeNew = lCtrlModeOld | 0x100000;	// Enable bit 20. EnableIndex
		lCtrlModeNew = lCtrlModeNew & 0xEFFFFF;	// Disabled bit 20. InvertIndex
		lCtrlModeNew = lCtrlModeNew & 0xBFFFFF;	// Disabled bit 22.	PresetOnIndex

		// Get the cmd register value...
		retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lCmdBits);

		// Check if cmd bit is hi for the specified encoder.
		if(lCmdBits & lMask)
		{
			// Set cmd bit low for specified encoder.
			retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdBits ^ lMask));
			PCI4E_WaitTimer(CMD_CHANGE_SLEEP_TIME);
		}

		lCmdBits = (lCmdBits | lMask);	// Set cmd bit hi...
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, lCmdBits);

		if(retVal == 0)
		{
			PCI4E_WaitTimer(CMD_CHANGE_SLEEP_TIME);
			retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdBits ^ lMask));
		}

		if(retVal == 0 && lCtrlModeOld != lCtrlModeNew)	// reset control mode to orginal value...
		{
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, lCtrlModeOld);	
		}
				
		if(retVal == 0)
		{
			PCI4E_WaitTimer(CMD_CHANGE_SLEEP_TIME + 10);
			retVal = PCI4E_GetCount(iDeviceNo, iEncoder, plVal);
		}

	}
	return retVal;
}

int PCI4E_PresetCount(short iDeviceNo, short iEncoder)
{
	// Preset Count: set the current Count to resolution value.
	int retVal = 0;
	short regAddr = iEncoder * 8 + TRANSFER_PRESET_REGISTER;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{	// Write the preset register value...
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, 0 );
		}
	}
	return retVal;
}

int PCI4E_ResetCount(short iDeviceNo, short iEncoder)
{
	// Reset Count: sets the current Count to 0.
	int retVal = 0;
	short regAddr = iEncoder * 8 + RESET_CHANNEL_REGISTER;

	// Make sure device is valid and open...
	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.

	if(retVal == 0)
	{
		// Make sure encoder is valid.
		if ( iEncoder < 0 || iEncoder > 3)
			retVal = INVALID_ENCODER_NUMBER;	// Invalid Encoder number
		else
		{	// Write the preset register value...
			retVal = PCI4E_WriteRegister(iDeviceNo, regAddr, 0 );
		}
	}
	return retVal;
}

long PCI4E_GetDeviceID()
{
	return (long)m_dwDeviceID;
}

void PCI4E_SetDeviceID(long deviceID)
{
	m_dwDeviceID = deviceID;
}
	
long PCI4E_GetVendorID()
{
	return (long)m_dwVendorID;
}
	
void PCI4E_SetVendorID(long vendorID)
{
	m_dwVendorID = vendorID;
}

int PCI4E_GetVersion(short iDeviceNo, short *iVersion)
{
	int retVal = 0;

	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.
	else
		*iVersion = (short)m_PCI4EDevice[iDeviceNo].version;
	
	return retVal;
}

int PCI4E_GetROM_ID(short iDeviceNo, short *iROM_ID)
{
	int retVal = 0;
	long lRegVal = 0;

	if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
		retVal = INVALID_DEVICE_NUMBER;	// Invalid device number.
	else
	{
		PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lRegVal);

		*iROM_ID = (short)((lRegVal & 0xFF000000) >> 24); 
	}
	
	return retVal;
}

int PCI4E_ReadOutputLatch(short iDeviceNo, short iEncoder, long *plVal)
{
	int retVal = 0;
	short regAddr = iEncoder * 8 + OUTPUT_LATCH_REGISTER;
	PCI4E_ReadRegister(iDeviceNo, regAddr, plVal);
	return retVal;
}

int PCI4E_ReadTimeStamp(short iDeviceNo, long *plVal)
{
	// This function does not read the time stamp output latch
	// but, instead reads the free running timestamp register.
	return PCI4E_ReadRegister(iDeviceNo, TIMESTAMP_REGISTER, plVal);
}

int PCI4E_ResetTimeStamp(short iDeviceNo)
{
	int retVal = 0;
	long lCmdVal = 0;

	// To reset the TimeStamp counter stop the counter by setting bit 6 of the CMD_REGISTER 
	// to 1 and then restart the counter by setting bit 6 back to 0.
	// Note: while bit 6 is 1 the TimeStamp counter value is zero.
	// When the TimeStamp is started, we also set bit 5 to 1.

	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lCmdVal);

	retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0xFFFFFFDF & lCmdVal) | 0x40);
	if(retVal == 0)
	{
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0xFFFFFFBF & lCmdVal) | 0x20);
	}	

	return retVal;
}

int PCI4E_ReadTimeAndCount(short iDeviceNo, short iEncoder, long *plValue,  long *plTimeStamp)
{
	int retVal = 0;
	short regAddr =0;
	long lTm = 0;
	
	retVal = PCI4E_ReadTimeStamp(iDeviceNo, plTimeStamp);		
	lTm = *plTimeStamp;

	if(retVal == 0)
	{
		regAddr = iEncoder * 8 + OUTPUT_LATCH_REGISTER;
		retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, plValue);
	}
	//LeaveCriticalSection(&m_cs);
	return retVal;
}

int PCI4E_ReadTimeAndCounts(short iDeviceNo, long *plValues,  long *plTimeStamp)
{
	// lValues is an array of 4 longs...
	int retVal = 0;
	int i = 0;
	short regAddr =0;
	long lTm = 0;
	
	retVal = PCI4E_ReadTimeStamp(iDeviceNo, plTimeStamp);		
	lTm = *plTimeStamp;

	if(retVal == 0)
	{
		for(i=0; i < 4; i++)
		{
			regAddr = i * 8 + OUTPUT_LATCH_REGISTER;
			retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, &plValues[i]);
			if(retVal != 0)
				break;
		}
	}
	//LeaveCriticalSection(&m_cs);
	return retVal;
}

int PCI4E_CaptureTimeAndCount(short iDeviceNo,  short iEncoder, long *plValue,  long *plTimeStamp)
{
	// lValues is an array of 4 longs...
	int retVal = 0;
	short regAddr = 0;
	long lCmdVal = 0;

	// To get the TimeStamp counter to latch to the TimeStamp Output Latch and trigger a capture
    	// transition bit 4 of the CMD_REGISTER from 0 -> 1.	

	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lCmdVal);

	retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0xFFFFFFEF & lCmdVal));
	if(retVal == 0)
	{
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0x10 | lCmdVal));
	}		

	retVal = PCI4E_ReadTimeStamp(iDeviceNo, plTimeStamp);

	if(retVal == 0)
	{
		retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, plValue);
	}

	//LeaveCriticalSection(&m_cs);
	return retVal;
}

int PCI4E_CaptureTimeAndCounts(short iDeviceNo, long *plValues,  long *plTimeStamp)
{
	// lValues is an array of 4 longs...
	int retVal = 0;
	int i = 0;
	short regAddr = 0;
	long lCmdVal = 0;

	// To get the TimeStamp counter to latch to the TimeStamp Output Latch and trigger a capture
    	// transition bit 4 of the CMD_REGISTER from 0 -> 1.	

	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lCmdVal);

	retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0xFFFFFFEF & lCmdVal));
	if(retVal == 0)
	{
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (0x10 | lCmdVal));
	}		

	retVal = PCI4E_ReadTimeStamp(iDeviceNo, plTimeStamp);

	if(retVal == 0)
	{
		for(i=0; i < 4; i++)
		{
			regAddr = i * 8 + OUTPUT_LATCH_REGISTER;
			retVal = PCI4E_ReadRegister(iDeviceNo, regAddr, &plValues[i]);
			if(retVal != 0)
				break;
		}
	}

	//LeaveCriticalSection(&m_cs);
	return retVal;
}

// ***** Added to support PC I/O Bus Board 32-bit Control...			*****
int PCI4E_PCIOReadCycle(short iDeviceNo, short iBoardSelect, short iAddress, long *plData)
{
	int retVal = 0;
	long lVal = 0;
	long lOrigCmdVal = 0;
	long lCmdVal = 0;
	long lRemoteBoardAddr = 0;

	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lOrigCmdVal);
	
	// Read the first 5 bits from the address and write to bits 16-20 of Register 31...
	lVal = iAddress & (unsigned short)0x1F;	
	lVal <<= 16;
	
	// Step 1: Set internal register of remote board...
	retVal = PCI4E_WriteRegister(iDeviceNo, OUTPUT_REGISTER, lVal);

	switch(iBoardSelect)
	{	
		case 0:
			lRemoteBoardAddr = 0x1C00;		// set bits 10, 11, and 12 hi.
			break;
		case 1:
			lRemoteBoardAddr = 0x1A00;		// set bits 9, 11, and 12 hi
			break;
		case 2:
			lRemoteBoardAddr = 0x1600;		// set bits 9, 10, and 12 hi.
			break;
		case 3:						// bits 5 and 6 are hi.
			lRemoteBoardAddr = 0xE00;		// set bits 9, 10, and 11 hi
			break;
	}
	
	lCmdVal = 0xFFFF00FF & lOrigCmdVal;		// initialize lNewCmdVal by setting bits 8 - 15 low...
	lCmdVal = lCmdVal | lRemoteBoardAddr;	// set lRemoteBoardAddr bits hi...
	
	// Step 2: Setup input...
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | SETUP_INPUT));

	//PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);
	PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);

	// Step 3: Transfer/Read input...
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | READ_INPUT));

	//PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);
	PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);
	
	// Step 4: Read the data...
	if(retVal == 0)	
		retVal = PCI4E_ReadRegister(iDeviceNo, INPUT_REGISTER, plData);
	
	//PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);
	PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);

	// Step 5: Set inactive by setting bits 8 - 15 of lCmdVal hi...
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, lOrigCmdVal | SET_INACTIVE);
	
	return retVal;
}

int PCI4E_PCIOResetCycle(short iDeviceNo)
{
	int retVal = 0;
	long lCmdVal = 0;
	
	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lCmdVal);
	
	// toggle reset bit for all remote boards...
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | RESET));
	
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | SET_INACTIVE));

	return retVal;
}

int PCI4E_PCIOWriteCycle(short iDeviceNo, short iBoardSelect, short iAddress, long lDataOut)
{
	int retVal = 0;
	long lVal = 0;
	long lOrigCmdVal = 0;
	long lCmdVal = 0;
	long lRemoteBoardAddr = 0;

	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lOrigCmdVal);
	
	// step 1...
	// Read the first 5 bits from the address and write to bits 16-20 of Register 31...
	lVal = iAddress & (unsigned short)0x1F;	
	lVal <<= 16;
	
	// Read all 16 bits of the dataOut parameter and write to 0-15 of Register 31...
	lVal = lVal | lDataOut;

	retVal = PCI4E_WriteRegister(iDeviceNo, OUTPUT_REGISTER, lVal);
	
	switch(iBoardSelect)
	{	
		case 0:
			lRemoteBoardAddr = 0x1C00;		// set bits 10, 11, and 12 hi.
			break;
		case 1:
			lRemoteBoardAddr = 0x1A00;		// set bits 9, 11, and 12 hi
			break;
		case 2:
			lRemoteBoardAddr = 0x1600;		// set bits 9, 10, and 12 hi.
			break;
		case 3:						// bits 5 and 6 are hi.
			lRemoteBoardAddr = 0xE00;		// set bits 9, 10, and 11 hi
			break;
	}
	
	lCmdVal = 0xFFFF00FF & lOrigCmdVal;	// initialize lCmdVal by setting bits 8 - 15 low...
	lCmdVal = lCmdVal | lRemoteBoardAddr;	// set lRemoteBoardAddr bits hi...
	
	// step 2...
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | SETUP_OUTPUT));
	
	PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);

	// step 3...
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | WRITE_OUTPUT));
	
	PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);

	// step 4...
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | HOLD_OUTPUT));
	
	PCI4E_WaitTimer(IO_WRITE_CYCLE_SLEEP_TIME);

	// step 5: set bits 8 - 15 of lCmdVal hi......
	if(retVal == 0)
		retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, lOrigCmdVal | SET_INACTIVE);

	return retVal;
}

int PCI4E_PCIOSignal(short iDeviceNo, short iBoardSelect, short iState)
{
	int retVal = 0;
	long lOrigCmdVal = 0;
	long lCmdVal = 0;
	long lRemoteBoardAddr = 0;

	// Read command register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, CMD_REGISTER, &lOrigCmdVal);

	switch(iBoardSelect)
	{	
		case 0:
			lRemoteBoardAddr = 0x1C00;		// set bits 10, 11, and 12 hi.
			break;
		case 1:
			lRemoteBoardAddr = 0x1A00;		// set bits 9, 11, and 12 hi
			break;
		case 2:
			lRemoteBoardAddr = 0x1600;		// set bits 9, 10, and 12 hi.
			break;
		case 3:						// bits 5 and 6 are hi.
			lRemoteBoardAddr = 0xE00;		// set bits 9, 10, and 11 hi
			break;
		default:
			lRemoteBoardAddr = 0x1E00;		// set bits 9, 10, 11, and 12 hi
			break;
	}
	
	lCmdVal = 0xFFFF00FF & lOrigCmdVal;		// initialize lCmdVal by setting bits 8 - 15 low...
	lCmdVal = lCmdVal | lRemoteBoardAddr;	// set lRemoteBoardAddr bits hi...
	
	retVal = PCI4E_WriteRegister(iDeviceNo, CMD_REGISTER, (lCmdVal | iState));
	return retVal;
}

void PCI4E_WaitTimer(short iTimeOut)
{
	struct timeb theTime;
	unsigned short int expire_tm = 0;
	
	ftime(&theTime);
	expire_tm = theTime.millitm + iTimeOut;
	
	do 
	{
		// nothing...
		ftime(&theTime);
	}
	while (theTime.millitm < expire_tm);
}

// ***** Added to support UART Communication Control...	*****
int PCI4E_ClearInBuffer(short iDeviceNo)
{
	int retVal = 0;
	long lOrigVal = 0;

	// Read the RX Control Register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, RX_CONTROL_REGISTER, &lOrigVal);
	if(retVal == 0)
	{
		// Update bit 1 of the RX Control Register, currently register 32...
		// Write 1 to clear input buffer.
		retVal = PCI4E_WriteRegister(iDeviceNo,
									 RX_CONTROL_REGISTER,
									 RX_INIT | lOrigVal);
		if(retVal==0)
		{	// Write 0 to make it ready to receive data.
			retVal = PCI4E_WriteRegister(iDeviceNo,
										 RX_CONTROL_REGISTER,
										 NOT_RX_INIT & lOrigVal);
		}
	}
	return retVal;
}

int PCI4E_GetInBufferCount(short iDeviceNo, short *piCount)
{
	int retVal = 0;
	long lVal = 0;
	short iRomID = 0;

	retVal = PCI4E_GetROM_ID(iDeviceNo, &iRomID);

	if(retVal==0)
		if(iRomID < UART_SUPPORTED)
			retVal = FEATURE_NOT_SUPPORTED;

	if(retVal == 0)
	{
		// Read bits 10-21 of the RX Status Register to get the input buffer count.
		retVal = PCI4E_ReadRegister(iDeviceNo, RX_STATUS_REGISTER, &lVal);
		
		*piCount = (short)(lVal >> 10) & 0xFFF;
	}

	return retVal;
}

int PCI4E_GetPortNumber(short iDeviceNo, short *piPortNumber)
{
	int retVal = 0;
	long lVal = 0;

	// Read bits 12-13 of the TX Control Register to get the Port Number.
	retVal = PCI4E_ReadRegister(iDeviceNo, PORT_SELECT_REGISTER, &lVal);
	
	*piPortNumber = (short)(PORT_MASK & lVal);

	return retVal;
}

int PCI4E_ReceiveData(short iDeviceNo, short *piSize, unsigned char *pucData)
{
	int retVal = 0;
	long lVal = 0;
	long lOrigVal = 0;
	short iBuffCnt = 0;
	short iBytesRead = 0;
	long lStatus = 0;
	unsigned char ucData = 0;
	short iRomID = 0;

	retVal = PCI4E_GetROM_ID(iDeviceNo, &iRomID);

	if(retVal==0)
		if(iRomID < UART_SUPPORTED)
			retVal = FEATURE_NOT_SUPPORTED;

	if(retVal==0)
	{
		// Read the RX Control Register so that it can be preserved.
		retVal = PCI4E_ReadRegister(iDeviceNo, RX_CONTROL_REGISTER, &lOrigVal);

		// Check if overrun or framing error exists...
		PCI4E_ReadRegister(iDeviceNo, TX_STATUS_REGISTER, &lStatus);

		retVal = (lStatus & OVERRUN_ERROR) ? OVERRUN_ERROR_DETECTED : 0; 
		
		if(retVal==0)
		{
			retVal = (lStatus & FRAMING_ERROR) ? FRAMING_ERROR_DETECTED : 0; 
		}
		else
		{
			ResetUart(iDeviceNo);
		}

		if(retVal==0)
		{
			// Check if buffer is full...
			PCI4E_ReadRegister(iDeviceNo, RX_STATUS_REGISTER, &lStatus);
			retVal = (lStatus & RX_FULL) ? RX_BUFFER_FULL : 0; 
		}
		else
		{
			ResetUart(iDeviceNo);
		}

		if(retVal == 0)
		{
			// Check if input buffer has data available.
			retVal = PCI4E_GetInBufferCount(iDeviceNo, &iBuffCnt);
		}
		else
		{
			ResetUart(iDeviceNo);
		}

		while(retVal==0 && iBuffCnt >  0 && iBytesRead < *piSize)
		{
			// Write 0 then 1 to bit 1 of the RX Control Register to transfer data from the
			// input buffer to the the rxdata bits 0-7 of the RX Status Register.
			retVal = PCI4E_WriteRegister(iDeviceNo, 
						RX_CONTROL_REGISTER, 
						lOrigVal & NOT_RX_READ);

			if(retVal==0)
			{
				retVal = PCI4E_WriteRegister(iDeviceNo, 
							 RX_CONTROL_REGISTER, 
							 lOrigVal | RX_READ);

				if(retVal==0)
				{
					// Read bits 0-7 of RX Status Register...
					retVal = PCI4E_ReadRegister(iDeviceNo, RX_STATUS_REGISTER, &lVal);

					if(retVal==0)
					{
						ucData = (lVal & RX_DATA_MASK);
						pucData[iBytesRead] = ucData;
					}

					iBytesRead++;
					retVal = PCI4E_GetInBufferCount(iDeviceNo, &iBuffCnt);
				}
			}
		}	
	}
	*piSize = iBytesRead;
	return retVal;
}

int PCI4E_SetPortNumber(short iDeviceNo, short iPortNumber)
{
	int retVal = 0;
	long lOrigVal = 0;
	
	// Read the PORT_SELECT_REGISTER so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, PORT_SELECT_REGISTER, &lOrigVal);
	
	if(retVal==0)
	{
		// Write to bits 0-3 of the PORT_SELECT_REGISTER to set the Port Number.
		retVal = PCI4E_WriteRegister(iDeviceNo, PORT_SELECT_REGISTER, (lOrigVal & NOT_PORT_MASK) | iPortNumber);
	}

	return retVal;
}

int PCI4E_TransmitData(short iDeviceNo, short iSize, unsigned char *piData)
{
	int retVal = 0;
	long lOrigVal = 0;
	long lOrigTXDataVal= 0;
	short iTxCnt = 0;
	long lVal = 0;
	short iRomID = 0;

	retVal = PCI4E_GetROM_ID(iDeviceNo, &iRomID);

	if(retVal==0)
		if(iRomID < UART_SUPPORTED)
			retVal = FEATURE_NOT_SUPPORTED;

	// Read TX Control Register so that it can be preserved.
	if(retVal == 0)
		retVal = PCI4E_ReadRegister(iDeviceNo, TX_CONTROL_REGISTER, &lOrigVal);

	// Read TX Data Register so that it can be preserved.
	if(retVal == 0)
		retVal = PCI4E_ReadRegister(iDeviceNo, TX_DATA_REGISTER, &lOrigTXDataVal);

	while(retVal==0 && iTxCnt < iSize)
	{
		while(!IsTxBufferEmpty(iDeviceNo)){}	// DO NOTHING.  WAIT UNTIL BUFFER IS EMPTY.

		// Write data to TX buffer... and set bit 8 to 0.
		lVal = (lOrigTXDataVal & CLEAR_DATA) | piData[iTxCnt];
		retVal = PCI4E_WriteRegister(iDeviceNo, TX_DATA_REGISTER, lVal);


		// Write 0 to bit 0 of the TX Control Register to transfer data from
		if(retVal==0)
			retVal = PCI4E_WriteRegister(iDeviceNo, 
						TX_CONTROL_REGISTER, 
						lOrigVal & NOT_TX_WRITE);

		// Write 1 to bit 0 of the TX Control Register to transfer data from
		if(retVal==0)
		{
			retVal = PCI4E_WriteRegister(iDeviceNo, 
						 TX_CONTROL_REGISTER, 
						 lOrigVal | TX_WRITE);
			iTxCnt++;
		}
	}

	return retVal;	
}

int IsTxBufferEmpty(short iDeviceNo)
{
	long lStatus = 0;
	// Get the status of the TX buffer.  Wait until it's empty to send data.
	PCI4E_ReadRegister(iDeviceNo, TX_STATUS_REGISTER, &lStatus);
	return lStatus &= TX_EMPTY;
}

void ResetUart(short iDeviceNo)
{
	long lOrigVal = 0;
	int retVal = 0;
	// Read the RX Control Register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, TX_CONTROL_REGISTER, &lOrigVal);
	// Set bits 2 and 1 low.
	retVal = PCI4E_WriteRegister(iDeviceNo, TX_CONTROL_REGISTER, lOrigVal & CLEAR_ERROR_0);
	// Set bits 2 and 1 hi.
	retVal = PCI4E_WriteRegister(iDeviceNo, TX_CONTROL_REGISTER, lOrigVal | CLEAR_ERROR_1);
	// Flush the input buffer...
	retVal = PCI4E_ClearInBuffer(iDeviceNo);
}

// Added to support Channel Buffering...
int PCI4E_ClearChannelBuffer(short iDeviceNo)
{
	int retVal = 0;
	long lOrigVal = 0;

	// Read the RX Control Register so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, FIFO2_STATUS_CONTROL_REGISTER, &lOrigVal);
	if(retVal == 0)
	{
		// Update bit 1 of the FIFO2 Status Control Register, currently register 38...
		// Write 1 to clear the channel buffer.
		retVal = PCI4E_WriteRegister(iDeviceNo,
					 FIFO2_STATUS_CONTROL_REGISTER,
					 RX_INIT | lOrigVal);
		if(retVal==0)
		{	// Write 0 to make the channel buffer ready to receive data.
			retVal = PCI4E_WriteRegister(iDeviceNo,
						 FIFO2_STATUS_CONTROL_REGISTER,
						 NOT_RX_INIT & lOrigVal);
		}
	}
	return retVal;
}

int PCI4E_DisableChannelBuffer(short iDeviceNo)
{
	int retVal = 0;
	long lOrigVal = 0;
	
	// Read the PORT_SELECT_REGISTER so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, PORT_SELECT_REGISTER, &lOrigVal);
	
	if(retVal==0)
	{
		// Write 0 to bit 8 of the PORT_SELECT_REGISTER to disable channel buffering.
		retVal = PCI4E_WriteRegister(iDeviceNo, PORT_SELECT_REGISTER, (lOrigVal & DISABLE_BUFFERING));
	}

	return retVal;
}

int PCI4E_EnableChannelBuffer(short iDeviceNo)
{
	int retVal = 0;
	long lOrigVal = 0;
	
	// Read the PORT_SELECT_REGISTER so that it can be preserved.
	retVal = PCI4E_ReadRegister(iDeviceNo, PORT_SELECT_REGISTER, &lOrigVal);
	
	if(retVal==0)
	{
		// Write 1 to bit 8 of the PORT_SELECT_REGISTER to enable channel buffering.
		retVal = PCI4E_WriteRegister(iDeviceNo, PORT_SELECT_REGISTER, (lOrigVal | ENABLE_BUFFERING));
	}

	return retVal;
}

int PCI4E_GetChannelBufferCount(short iDeviceNo, short *piCount)
{
	int retVal = 0;
	long lVal = 0;
	short iRomID = 0;
	short iCnt = 0;
	retVal = PCI4E_GetROM_ID(iDeviceNo, &iRomID);

	if(retVal==0)
		if(iRomID < CHANNEL_BUFFER_SUPPORTED)
			retVal = FEATURE_NOT_SUPPORTED;

	if( piCount == NULL )
		retVal = INVALID_PARAMETER;

	if(retVal == 0)
	{
		// Read bits 10-19 of the RX Status Register to get the input buffer count.
		retVal = PCI4E_ReadRegister(iDeviceNo, FIFO2_STATUS_CONTROL_REGISTER, &lVal);
		iCnt = (short)((lVal & 0x1FFC00) >> 10);
		*piCount = iCnt;
		
		if(retVal == 0)
			retVal = (lVal & RX_FULL) ? RX_BUFFER_FULL : 0; 
	}

	return retVal;
}

int PCI4E_ReadChannelBuffer(short iDeviceNo, short *piSize, ChannelBufferRecord *pCBR)
{
	// piRecord is a pointer to an array of ChannelBufferRecord
	int retVal = 0;
	int i = 0;
	long lVal = 0;
	long lOrigVal = 0;
	short iBuffCnt = 0;
	short iRecordsRead = 0;
	short iRomID = 0;

	retVal = PCI4E_GetROM_ID(iDeviceNo, &iRomID);

	if(retVal==0)
		if(iRomID < CHANNEL_BUFFER_SUPPORTED)
			retVal = FEATURE_NOT_SUPPORTED;

	if(retVal==0)
	{
		// Read the Channel Buffer Status/Control Register so that it can be preserved.
		retVal = PCI4E_ReadRegister(iDeviceNo, FIFO2_STATUS_CONTROL_REGISTER, &lOrigVal);

		if(retVal == 0)
		{
			// Check if input buffer has data available.
			retVal = PCI4E_GetChannelBufferCount(iDeviceNo, &iBuffCnt);
		}
		else
		{
			ResetUart(iDeviceNo);
		}

		while(iBuffCnt >  0 && iRecordsRead < *piSize)
		{
			// Write 0 then 1 to bit 1 of the FIFO2 Status Control Register to transfer data from the
			// channel buffer to the ReadData Refister.  
			
			// Get the timestamp...
			PCI4E_WriteRegister(iDeviceNo, 
					FIFO2_STATUS_CONTROL_REGISTER, 
					lOrigVal & NOT_RX_READ);

			PCI4E_WriteRegister(iDeviceNo,
					FIFO2_STATUS_CONTROL_REGISTER, 
					lOrigVal | RX_READ);

			// Read FIFO2_READ_DATA_REGISTER...
			// Timestamp data contained in bits 0-31.
			PCI4E_ReadRegister(iDeviceNo, FIFO2_READ_DATA_REGISTER, &lVal);
			pCBR[iRecordsRead].Time = lVal;

			// Get the count and status for each channel...
			for(i=0; i<4; i++)
			{
				// Write 0 then 1 to bit 1 of the FIFO2 Status Control Register to transfer data from the
				// channel buffer to the ReadData Refister.  
									
				PCI4E_WriteRegister(iDeviceNo, 
						FIFO2_STATUS_CONTROL_REGISTER, 
						lOrigVal & NOT_RX_READ);

				PCI4E_WriteRegister(iDeviceNo,
						FIFO2_STATUS_CONTROL_REGISTER, 
						lOrigVal | RX_READ);

				// Count data contained in bits 0-23 and status contained in bits 24-31.
				PCI4E_ReadRegister(iDeviceNo, FIFO2_READ_DATA_REGISTER, &lVal);
				pCBR[iRecordsRead].ChannelData[i] = lVal & COUNT_MASK;
				pCBR[iRecordsRead].ChannelStatus[i] = (unsigned char)((lVal >> 24) & STATUS_MASK);
			}
			
			PCI4E_GetInBufferCount(iDeviceNo, &iBuffCnt);
			iRecordsRead++;
		}	
	}
	*piSize = iRecordsRead;
	return retVal;
}
int PCI4E_WriteRegister(short iDeviceNo, short iRegister, long lVal)
{
    // Returns 0 if success otherwise non-zero.

    // Make sure device number is valid...
    if ( iDeviceNo < -1 || iDeviceNo > MAX_DEVICES )
	return INVALID_DEVICE_NUMBER;	// Invalid device number.

    // Make sure device is open...
    if (!m_PCI4EDevice[iDeviceNo].fd  || m_PCI4EDevice[iDeviceNo].fd == -1)
        return DEVICE_NOT_OPEN;

    // Make sure register number is valid...
    if (iRegister < 0 || iRegister > 39)
	return INVALID_REGISTER_NUMBER;
	
    struct pci4e_io_struct ios;

    ios.region = '0'; // Access memory starting at BAR0.
    ios.offset = iRegister * 4;
    ios.len = 4;    // always read 4 bytes   
    ios.value = lVal;
	    
    if (ioctl(m_PCI4EDevice[iDeviceNo].fd, PCI4E_IOCWRITE, &ios)) {
	fprintf(stderr, "pci4eHelper: ioctl(%s): %s\n",
		m_PCI4EDevice[iDeviceNo].devname, strerror(errno));
        return FATAL_ERROR;	
    } else {
	//printf("read  %i bytes from '%c':0x%08lx -- 0x%08lx\n",
	//	ios.len, ios.region, ios.offset, ios.value);
    }	    

    return 0;
}

