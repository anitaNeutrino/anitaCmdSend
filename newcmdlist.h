/* newcmdlist.h */

#ifndef _CMDLIST_H
#define _CMDLIST_H

#define KEYBD_CMD_MAX	128	/* maximum ascii key */


#define TAIL_MESSAGES  1
#define TAIL_ANITA  2
#define CMD_START_NEW_RUN  3
//#define CMD_MAKE_NEW_RUN_DIRS  4 -- Should never be called
#define LOG_REQUEST_COMMAND  10

#define CMD_REALLY_KILL_PROGS  127
#define CMD_SIPD_REBOOT  128
#define CMD_SHUTDOWN_HALT  129
#define CMD_REBOOT  130
#define CMD_KILL_PROGS  131
#define CMD_RESPAWN_PROGS 132
#define CMD_START_PROGS  133
#define CMD_DISABLE_DISK 135
#define CMD_MOUNT_NEXT_USB  136
#define CMD_MOUNT_NEXT_SATA  137 //Switch for blade v mini
#define CMD_EVENT_DISKTYPE  138
#define CMD_HK_DISKTYPE  139

#define ARCHIVE_STORAGE_TYPE  140
#define ARCHIVE_PRI_DISK  141
#define ARCHIVE_PRI_ENC_TYPE142
#define ARCHIVE_DECIMATE_PRI  144 //For each disk
#define SET_DECIMATION  145 
#define TELEM_TYPE  146
#define TELEM_PRI_ENC_TYPE  147
#define SET_SPECIAL_PRI  148
#define SET_SPECIAL_DECIMATE  149
 
#define TURN_GPS_ON  150
#define TURN_GPS_OFF  151
#define TURN_RFCM_ON  152
#define TURN_RFCM_OFF  153
#define TURN_CALPULSER_ON  154
#define TURN_CALPULSER_OFF  155
#define TURN_NADIR_ON  156
#define TURN_NADIR_OFF  157
#define TURN_ALL_ON  158
#define TURN_ALL_OFF  159
 
#define SET_CALPULSER_SWITCH  171
#define SET_CALPULSER_ATTEN  172
#define CP_ATTEN_LOOP_PERIOD  173
#define CP_SWITCH_LOOP_PERIOD  174
#define CP_PULSER_OFF_PERIOD  175
#define CP_CALIB_WRITE_PERIOD  176
  
#define SET_ADU5_PAT_PERIOD  180
#define SET_ADU5_SAT_PERIOD  181
#define SET_G12_PPS_PERIOD  182
#define SET_G12_PPS_OFFSET  183
#define ADU5_CAL_12  184
#define ADU5_CAL_13  185
#define ADU5_CAL_14  186
#define SET_ADU5_VTG_PERIOD  187
#define SET_G12_POS_PERIOD  188
#define GPSD_EXTRA_COMMAND  189

#define SET_HK_PERIOD  190
#define SET_HK_CAL_PERIOD  191
#define SET_HK_TELEM_EVERY  192

#define SIPD_CONTROL_COMMAND  195
#define LOSD_CONTROL_COMMAND 196

  

#define CLEAN_DIRS  200
#define CLEAR_RAMDISK  201
   
#define SEND_CONFIG  210
#define DEFAULT_CONFIG 211
#define SWITCH_CONFIG 212
#define LAST_CONFIG213
#define SAVE_CONFIG214

#define RAMDISK_KILL_ACQD  220
#define RAMDISK_DUMP_DATA  221
#define MONITORD_ACQD_WAIT  222
#define MONITORD_PERIOD  223
#define USB_CHANGE_THRESH  224
#define SATA_CHANGE_THRESH  225
#define MAX_QUEUE_LENGTH  226 
#define INODES_KILL_ACQD  227 
#define INODES_DUMP_DATA  228

#define ACQD_ADU5_TRIG_FLAG  230
#define ACQD_G12_TRIG_FLAG  231
#define ACQD_SOFT_TRIG_FLAG  232
#define ACQD_SOFT_TRIG_PERIOD  233
#define ACQD_PEDESTAL_RUN  236
#define THRESHOLD_SCAN 237
#define ACQD_EXTRA_COMMAND  240 
#define ACQD_REPROGRAM_TURF  241
#define SURFHK_PERIOD  242
#define SURFHK_TELEM_EVERY  243
#define TURFHK_TELEM_EVERY 244
#define NUM_PED_EVENTS  245
#define THRESH_SCAN_STEP_SIZE  246
#define THRESH_SCAN_REPEAT  247
#define ACQD_RATE_COMMAND  250 //Guess at 10 for now  
#define EVENTD_MATCH_GPS  251
#define GPS_PHI_MASK_COMMAND  252
#define PRIORITIZERD_COMMAND 253  
#define PLAYBACKD_COMMAND 254  


#endif /* _CMDLIST_H */



