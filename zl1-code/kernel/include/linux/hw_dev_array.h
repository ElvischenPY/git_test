#ifndef _ZEUSIS_DEV_ARRAY_H_
#define _ZEUSIS_DEV_ARRAY_H_
#include "hw_dev_dec.h"

typedef struct {
    int devices_id;
    char* devices_name;
}hw_dec_struct;

static hw_dec_struct hw_dec_device_array[] =
{
	{ DEV_PERIPHIAL_TOUCH_PANEL,"touch_panel" },
	{ DEV_PERIPHIAL_COMPASS,"compass" },
	{ DEV_PERIPHIAL_GG_SENSOR,"gravity_gyroscope_sensor" },
	{ DEV_PERIPHIAL_PL_SENSOR, "proximity_light_sensor"},
	{ DEV_PERIPHIAL_HALL_SENSOR, "hall_sensor"},
	{ DEV_PERIPHIAL_CAMERA_MAIN,"camera_main" },
	{ DEV_PERIPHIAL_CAMERA_SLAVE,"camera_slave" },
	{ DEV_PERIPHIAL_FFLASH,"fflash"},
	{ DEV_PERIPHIAL_NFC,"nfc" },
	{ DEV_PERIPHIAL_RF,"rf" },
	{ DEV_PERIPHIAL_PA_RECEIVER,"pa_receiver" },
	{ DEV_PERIPHIAL_PA_SPEAKER, "pa_speaker" },
	{ DEV_PERIPHIAL_CHARGER_SMB,"charge_smb1351" },
	{ DEV_PERIPHIAL_CHARGER_PMI,"charge_pmi8994" },
	{ DEV_PERIPHIAL_BATTERY,"battery" },
	{ DEV_PERIPHIAL_IRDA,"irda" },
	{ DEV_PERIPHIAL_USB_SWITCH,"usb_switch"},
	{ DEV_PERIPHIAL_VIBRATOR,"vibrator"},
	{ DEV_PERIPHIAL_FINGERPRINT, "fingerprint"},
	{ DEV_CONNECTIVITY_WIFI,"wifi" },
	{ DEV_CONNECTIVITY_BT,"bt" },
	{ DEV_CONNECTIVITY_FM,"fm" },
	{ DEV_CONNECTIVITY_GPS,"gps" },
	{ DEV_PERIPHIAL_MAX,"NULL" },
};

#endif
