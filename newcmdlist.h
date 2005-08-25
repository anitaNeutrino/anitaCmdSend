/* newcmdlist.h */

#ifndef _CMDLIST_H
#define _CMDLIST_H

#define KEYBD_CMD_MAX	128	/* maximum ascii key */
#define SHUTDOWN_HALT	129	/* shuts computer down, must be powered down and then up to restart*/
#define REBOOT          130	/* reboots flight computer */
#define KILL_PROGS	131	/* kills daemons */
#define RESPAWN_PROGS  	132	/* respawns daemons*/
#define START_PROGS     133     /* starts daemons*/
#define MOUNT           134     /* mounts devices? */

#define TURN_GPS_ON	150	/* turns on GPS and magnetometer via relay*/
#define TURN_GPS_OFF	151	/* turns off GPS and magnetometer via relay*/
#define TURN_RFCM_ON	152	/*turns on RFCM and magnetometer via relay*/
#define TURN_RFCM_OFF	153	/* turns off RFCM and magnetometer via relay */
#define TURN_CALPULSER_ON	154	/*turns on CalPulser and magnetometer via relay */
#define TURN_CALPULSER_OFF	155	/*turns off CalPulser and magnetometer via relay */
#define TURN_ND_ON	156	/*turns the Noise Diode on via relay*/
#define TURN_ND_OFF     157     /*turns the Noise Diode off via relay*/	
#define TURN_ALL_ON	158	/*turns on ND, CalPulser, GPS and RFCM via relays*/
#define TURN_ALL_OFF	159	/*turns oFF ND, CalPulser, GPS and RFCM via relays*/

#define SET_CALPULSER   171	/* sets the CalPulser Address (?) */
#define SET_SS_GAIN     172	/* sets the Sun Sensor gain */

#define SET_ADU5_PAT_PERIOD     180	/* sets the ADU5 pat period*/
#define SET_ADU5_SAT_PERIOD   	181	/* sets the ADU5 pat period*/
#define SET_G12_PPS_PERIOD	182	/* sets the G12 PPS period */
#define SET_G12_OFFSET  183	/* sets the G12 PPS offset*/
#define SET_ADU5_CAL_12 184     /* sets the ADU5 Calibration to 12*/
#define SET_ADU5_CAL_13 185     /* sets the ADU5 Calibration to 13*/
#define SET_ADU5_CAL_14 186     /* sets the ADU5 Calibration to 14*/

#define SET_HSK_PERIOD  190     /* sets the period for Housekeeping */
#define SET_HSK_CAL_PERIOD      191     /* sets the Housekeeping Cal period */

#define CLEAN_DIRS	200	/* cleans all? directories */

#define SEND_CONFIG	210	/* sends down current all? config files  */
#define DEFAULT_CONFIG	211	/* returns to default configuration*/

#define SURF_ADU5_TRIG  230	/* sets the SURF to ADU5 triger */
#define SURF_G12_TRIG   231     /* sets the SURF to G12 trigger */
#define SURF_RF_TRIG    232     /* sets the SURF to trigger on signal? */
#define SURF_SOFT_TRIG  233     /* sets the SURF to software trigger? */
#define SURF_SOFT_TRIG_PERIOD   234     /* sets the period for software drected trigger */

#endif /* _CMDLIST_H */



