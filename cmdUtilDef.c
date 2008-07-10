
#include "cmdUtilDef.h"


char *logRequestName(LogRequestCommand_t req) 
{
    switch(req) {	
	case LOG_REQUEST_MESSAGES: return "/var/log/messages";
	case LOG_REQUEST_ANITA: return "/var/log/anita.log";
	case LOG_REQUEST_SECURITY: return "/var/log/security";
	case LOG_REQUEST_NEOBRICK: return "/var/log/neobrick";
	case LOG_REQUEST_BOOT: return "/var/log/boot.log";	    
	case LOG_REQUEST_PROC_CPUINFO: return "/proc/cpuinfo";
	case LOG_REQUEST_PROC_DEVICES: return "/proc/devices";
	case LOG_REQUEST_PROC_DISKSTATS: return "/proc/diskstats";
	case LOG_REQUEST_PROC_FILESYSTEMS: return "/proc/filesystems";
	case LOG_REQUEST_PROC_INTERRUPTS: return "/proc/interrupts";
	case LOG_REQUEST_PROC_IOMEM: return "/proc/iomem";
	case LOG_REQUEST_PROC_IOPORTS: return "/proc/ioports";
	case LOG_REQUEST_PROC_LOADAVG: return "/proc/loadavg";
	case LOG_REQUEST_PROC_MEMINFO: return "/proc/meminfo";
	case LOG_REQUEST_PROC_MISC: return "/proc/misc";
	case LOG_REQUEST_PROC_MODULES: return "/proc/modules";
	case LOG_REQUEST_PROC_MOUNTS: return "/proc/mounts";
	case LOG_REQUEST_PROC_MTRR: return "/proc/mtrr";
	case LOG_REQUEST_PROC_PARTITIONS: return "/proc/partitions";
	case LOG_REQUEST_PROC_SCHEDDEBUG: return "/proc/sched_debug";
	case LOG_REQUEST_PROC_SCHEDSTAT: return "/proc/sched_stat";
	case LOG_REQUEST_PROC_STAT: return "/proc/stat";
	case LOG_REQUEST_PROC_SWAPS: return "/proc/swaps";
	case LOG_REQUEST_PROC_TIMERLIST: return "/proc/timer_list";
	case LOG_REQUEST_PROC_TIMERSTATS: return "/proc/timer_stats";
	case LOG_REQUEST_PROC_UPTIME: return "/proc/uptime";
	case LOG_REQUEST_PROC_VERSION: return "/proc/version";
	case LOG_REQUEST_PROC_VMCORE: return "/proc/vmcore";
	case LOG_REQUEST_PROC_VMSTAT: return "/proc/vmstat";
	case LOG_REQUEST_PROC_ZONEINFO: return "/proc/zoneinfo";
	default: return "Unknown";
    }
    return "Unknown";
}
