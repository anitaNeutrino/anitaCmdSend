#ifndef CMDUTILDEF_H
#define CMDUTILDEF_H

typedef enum {
    ID_FIRST =100,
    ID_ACQD = 100,
    ID_ARCHIVED,
    ID_CALIBD,
    ID_CMDD,
    ID_EVENTD,
    ID_GPSD,
    ID_HKD,
    ID_LOSD,
    ID_PRIORITIZERD,
    ID_SIPD,
    ID_MONITORD,
    ID_PLAYBACKD,
    ID_LOGWATCHD,
    ID_NTUD,
    ID_OPENPORTD,
    ID_NOT_AN_ID
} ProgramId_t;


//Program Id Masks
#define ACQD_ID_MASK 0x001
#define ARCHIVED_ID_MASK 0x002
#define CALIBD_ID_MASK 0x004
#define CMDD_ID_MASK 0x008
#define EVENTD_ID_MASK 0x010
#define GPSD_ID_MASK 0x020
#define HKD_ID_MASK 0x040
#define LOSD_ID_MASK 0x080
#define PRIORITIZERD_ID_MASK 0x100
#define SIPD_ID_MASK 0x200
#define MONITORD_ID_MASK 0x400
#define PLAYBACKD_ID_MASK 0x800
#define LOGWATCHD_ID_MASK 0x1000
#define NTUD_ID_MASK 0x2000
#define OPENPORTD_ID_MASK 0x4000
#define ALL_ID_MASK 0xffff



typedef enum {
  ACQD_RATE_ENABLE_CHAN_SERVO=1,
  ACQD_RATE_SET_PID_GOALS=2,
  ACQD_RATE_SET_ANT_TRIG_MASK=3,
  ACQD_RATE_SET_PHI_MASK=4,
  ACQD_RATE_SET_SURF_BAND_TRIG_MASK=5,
  ACQD_RATE_SET_CHAN_PID_GOAL_SCALE=6,
  ACQD_RATE_SET_RATE_SERVO=7,
  ACQD_RATE_ENABLE_DYNAMIC_PHI_MASK=8,
  ACQD_RATE_ENABLE_DYNAMIC_ANT_MASK=9,
  ACQD_RATE_SET_DYNAMIC_PHI_MASK_OVER=10,  //Set over rate and over window
  ACQD_RATE_SET_DYNAMIC_PHI_MASK_UNDER=11,  //Set under rate and under window
  ACQD_RATE_SET_DYNAMIC_ANT_MASK_OVER=12, //Set over rate and over window
  ACQD_RATE_SET_DYNAMIC_ANT_MASK_UNDER=13, //Set under rate and under window
  ACQD_RATE_SET_GLOBAL_THRESHOLD=14,
  ACQD_RATE_SET_GPS_PHI_MASK=15,
  ACQD_SET_NADIR_PID_GOALS=16,
  ACQD_SET_PID_PGAIN=17,
  ACQD_SET_PID_IGAIN=18,
  ACQD_SET_PID_DGAIN=19,
  ACQD_SET_PID_IMAX=20,
  ACQD_SET_PID_IMIN=21,
  ACQD_SET_PID_AVERAGE=22,
  ACQD_RATE_SET_PHI_MASK_HPOL=23
} AcqdRateCommandCode_t;

typedef enum {
  ACQD_DISABLE_SURF = 127,
  ACQD_SET_TURF_RATE_AVERAGE = 128,
  ACQD_SET_PHOTO_SHUTTER_MASK = 140,  
  ACQD_SET_PPS_SOURCE = 141,  
  ACQD_SET_REF_CLOCK_SOURCE = 142  
} AcqdExtraCommand_t;
  
  
  
  

typedef enum {
    PRI_HORN_THRESH=1,
    PRI_HORN_DESC_WIDTH=2,
    PRI_HORN_SECTOR_WIDTH=3,
    PRI_CONE_THRESH=4,
    PRI_CONE_DESC_WIDTH=5,
    PRI_HOLDOFF=6,
    PRI_DELAY=7,
    PRI_HORN_GUARD_OFFSET=8,
    PRI_HORN_GUARD_WIDTH=9,
    PRI_HORN_GUARD_THRESH=10,
    PRI_CONE_GUARD_OFFSET=11,
    PRI_CONE_GUARD_WIDTH=12,
    PRI_CONE_GUARD_THRESH=13,
    PRI_FFT_PEAK_MAX_A=14,
    PRI_FFT_PEAK_MAX_B=15,
    PRI_RMS_MAX=16,
    PRI_RMS_EVENTS=17,
    PRI_WINDOW_CUT=18,
    PRI_BEGIN_WINDOW=19,
    PRI_END_WINDOW=20,
    PRI_METHOD_MASK=21,
    PRI_FFT_MAX_CHANNELS=22,
    PRI_FFT_PEAK_WINDOW_L=23,
    PRI_FFT_PEAK_WINDOW_R=24,
    PRI_NU_CUT=25
} PrioritizerdCommandCode_t;
  
typedef enum {
  GPS_PHI_MASK_ENABLE=1,
  GPS_PHI_MASK_UPDATE_PERIOD=2,
  GPS_PHI_MASK_SET_SOURCE_LATITUDE=3,
  GPS_PHI_MASK_SET_SOURCE_LONGITUDE=4,
  GPS_PHI_MASK_SET_SOURCE_ALTITUDE=5,
  GPS_PHI_MASK_SET_SOURCE_HORIZON=6,
  GPS_PHI_MASK_SET_SOURCE_WIDTH=7
} GpsPhiMaskCommandCode_t;
    

typedef enum {
    PLAY_GET_EVENT=1,    
    PLAY_START_PRI=2,
    PLAY_STOP_PRI=3,
    PLAY_USE_DISK=4,
    PLAY_START_EVENT=5,
    PLAY_START_PLAY=6,
    PLAY_STOP_PLAY=7,
    PLAY_SLEEP_PERIOD=8
} PlaybackCommandCode_t;

typedef enum {
  SIPD_SEND_WAVE = 127,
  SIPD_THROTTLE_RATE = 128,
  SIPD_PRIORITY_BANDWIDTH = 129,
  SIPD_HEADERS_PER_EVENT = 130,
  SIPD_HK_TELEM_ORDER = 131,
  SIPD_HK_TELEM_MAX_PACKETS = 132
} SipdCommandCode_t;

typedef enum {
  LOSD_SEND_DATA = 1,
  LOSD_PRIORITY_BANDWIDTH = 2,
  LOSD_HK_BANDWIDTH = 3
} LosdCommandCode_t;
  
typedef enum {
    LOG_FIRST_LOG=1,
  LOG_REQUEST_MESSAGES = 1,
  LOG_REQUEST_ANITA = 2,
  LOG_REQUEST_SECURITY = 3,
  LOG_REQUEST_NTU = 4,
  LOG_REQUEST_BOOT = 5, 
  LOG_REQUEST_PROC_CPUINFO = 6,
  LOG_REQUEST_PROC_DEVICES = 7,
  LOG_REQUEST_PROC_DISKSTATS = 8,
  LOG_REQUEST_PROC_FILESYSTEMS = 9,
  LOG_REQUEST_PROC_INTERRUPTS = 10,
  LOG_REQUEST_PROC_IOMEM = 11,
  LOG_REQUEST_PROC_IOPORTS = 12,
  LOG_REQUEST_PROC_LOADAVG =13,
  LOG_REQUEST_PROC_MEMINFO =14,
  LOG_REQUEST_PROC_MISC = 15,
  LOG_REQUEST_PROC_MODULES = 16,
  LOG_REQUEST_PROC_MOUNTS = 17,
  LOG_REQUEST_PROC_MTRR = 18,
  LOG_REQUEST_PROC_PARTITIONS = 19,
  LOG_REQUEST_PROC_SCHEDDEBUG = 20,
  LOG_REQUEST_PROC_SCHEDSTAT = 21,
  LOG_REQUEST_PROC_STAT = 22,
  LOG_REQUEST_PROC_SWAPS = 23,
  LOG_REQUEST_PROC_TIMERLIST = 24,
  LOG_REQUEST_PROC_TIMERSTATS= 25,
  LOG_REQUEST_PROC_UPTIME = 26,
  LOG_REQUEST_PROC_VERSION = 27,
  LOG_REQUEST_PROC_VMCORE = 28,
  LOG_REQUEST_PROC_VMSTAT = 29,
    LOG_REQUEST_PROC_ZONEINFO = 30,
    LOG_NOT_A_LOG
} LogRequestCommand_t;

typedef enum {
    JOURNALCTL_OPT_COMM=0,
    JOURNALCTL_OPT_PRIORITY=1,
    JOURNALCTL_OPT_SYSLOG_FACILITY=2,
    JOURNALCTL_NO_OPT=3
} JournalctlOptionCommand_t;

typedef enum {
  GPS_SET_GGA_PERIOD = 130,
  GPS_SET_PAT_TELEM_EVERY = 131,
  GPS_SET_VTG_TELEM_EVERY = 132,
  GPS_SET_SAT_TELEM_EVERY = 133,
  GPS_SET_GGA_TELEM_EVERY = 134,
  GPS_SET_POS_TELEM_EVERY = 135,
  GPS_SET_INI_RESET_FLAG = 136,
  GPS_SET_ELEVATION_MASK = 137,
  GPS_SET_CYC_PHASE_ERROR = 138,
  GPS_SET_MXB_BASELINE_ERROR = 139,
  GPS_SET_MXM_PHASE_ERROR = 140
} GpsExtraCommand_t;

//Disk Bit Masks
#define HELIUM1_DISK_MASK 0x1
#define HELIUM2_DISK_MASK 0x2
#define USB_DISK_MASK 0x4
#define NTU_DISK_MASK 0x8
#define PMC_DISK_MASK 0x10

#define DISK_TYPES 5

char *journalOptionName(JournalctlOptionCommand_t opt);
char *logRequestName(LogRequestCommand_t req);


#endif
