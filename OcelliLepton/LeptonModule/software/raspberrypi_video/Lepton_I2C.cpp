#include "Lepton_I2C.h"

#include "leptonSDKEmb32PUB/LEPTON_SDK.h"
#include "leptonSDKEmb32PUB/LEPTON_SYS.h"
#include "leptonSDKEmb32PUB/LEPTON_OEM.h"
#include "leptonSDKEmb32PUB/LEPTON_Types.h"

#include <iostream>

bool _connected;

LEP_CAMERA_PORT_DESC_T _port;



int lepton_connect() {
	LEP_OpenPort(1, LEP_CCI_TWI, 400, &_port);
	_connected = true;
	return 0;
}

void lepton_perform_ffc() {
	if(!_connected) {
		lepton_connect();
	}
	LEP_RunSysFFCNormalization(&_port);
}

//presumably more commands could go here if desired

void lepton_reboot() {
	if(!_connected) {
		lepton_connect();
	}
	LEP_RunOemReboot(&_port);
}

// Commands written by Jordan Diaz
void set_factory_ffc()
{
	if (!_connected){
		lepton_connect();
	}
	LEP_SYS_FFC_SHUTTER_MODE_OBJ_T factory_ffc_settings = 
	{
		LEP_SYS_FFC_SHUTTER_MODE_AUTO,
		LEP_SYS_SHUTTER_LOCKOUT_INACTIVE,
		LEP_SYS_ENABLE,
		LEP_SYS_DISABLE,
		0,
		180000,
		0,
		150,
		52
	};
	
	LEP_SetSysFfcShutterModeObj(&_port, factory_ffc_settings);
}


void reset_to_i2c_defaults()
{
	// AGC MODE disabled
	// AGC ROI (0, 0, 79, 59) or (0, 0, 159, 159)
	// AGC Dampening Factor 64
	// AGC Clip Limit High 19200
	
	
	
	
	
}


void lepton_enable_agc()
{
	
}


