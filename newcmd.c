/* cmd - interface to NSBF for sending LDB commands
 *
 * USAGE: cmd [-d] [-l link] [-r route] [-g logfile]
 *
 * Marty Olevitch, June '01, editted KJP 7/05
 * 	12/27/06 Check for another 'cmd' running at startup. (MAO)
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <string.h>	/* strlen, strerror */
#include <stdlib.h>	/* malloc */
#include <curses.h>
#include <math.h>
#include <time.h>

#include <signal.h>
#include <ctype.h>

#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
/* select() */
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "newcmdfunc.h"	// generated from newcmdlist.h
#include "cmdUtilDef.h"

WINDOW *Wuser;
WINDOW *Wmenu;

void catchsig(int sig);
static void quit_confirm(void);
int screen_confirm(char *fmt, ...);
void screen_dialog(char *response, int nbytes, char *fmt, ...);
void screen_mesg(char *fmt, ...);
void screen_printf(char *fmt, ...);
void screen_beep(void);
void write_cmd_json_file(char *fmt, ...);
void get_next_cmd_count();

int diskBitMasks[DISK_TYPES]={HELIUM1_DISK_MASK,HELIUM2_DISK_MASK,USB_DISK_MASK,NTU_DISK_MASK};

#define GETKEY wgetch(Wuser)

#define TIMEOUT	't'	/* set_timeout */
#define NEWCMD	'n'	/* new_cmd */
#define LINKSEL 'l'	/* select_link */
#define RTSEL	'r'	/* select_route */
#define SHOWCMD	's'	/* show_cmds */
#define EXPERT  'x'     /*show expert commands*/
#define QUIT	'Q'

static void set_timeout(void);
static void new_cmd(void);
//static void cur_cmd(void);
static void show_cmds(void);
static void expert(void);

#define NMENULINES 2
char *progmenu[NMENULINES-1];
#define LINELEN	80
#define CTL(x)	(x & 0x0f)	/* control(x) */

static void initdisp(void);
static void menu(char **m, int nlines);
static void clear_screen(void);
static void generic_exit_routine(void);

char *menuformat[] = {
"%c=set_timeout %c=quit  %c=link %c=route %c=show_cmds",
};

#define IBUFSIZE 32
unsigned char ibuf[IBUFSIZE];


#define CMDLEN		25	/* max length of command buffer */
#define LINK_LOS	0
#define LINK_TDRSS	1
#define LINK_HF		2
#define LINK_LOS	0
#define PORT		"/dev/ttyUSB0"
//#define PORT		"/dev/ttyXR1"
//#define PORT		"/dev/ttyS2"
#define PROMPT		": "
#define ROUTE_COMM1	0x09
#define ROUTE_COMM2	0x0C
//#define PRINT_EXTRA_STUFF

static void accum_cmd(int key);
static void select_link(void);
static void select_route(void);
void sendcmd(int fd, unsigned char *s, int len);
int serial_end(void);
int serial_init(void);
void wait_for_ack(void);

unsigned int curCmdNumber;
unsigned char Curcmd[32];
int Curcmdlen = 0;
unsigned char Curlink = LINK_TDRSS;
unsigned char Curroute = ROUTE_COMM1;
int Fd;
struct termios Origopts;
int Timeout = 10L;

static short numLines=0;

static short Dir_det =0;
static unsigned int Prog_det =0;
static char Which_RFCM =0;
static unsigned int Config_det =0;
static short switchConfig =0;
static short Priorit_det =0;
static short CalPulserSwitch =0;
static short disableHelium1=0;
static short disableUsbInt=0;
static short disableUsbExt=0;
static short calPulserAtten =0;
static short cpAttenLoopPeriod =0;
static short cpSwitchLoopPeriod =0;
static short cpOffLoopPeriod =0;
static short calibWritePeriod=0;
static int ADU5PatPer =0;
static int ADU5SatPer =0;
static int ADU5VtgPer =0;
static int G12PPSPer =0;
static int G12PosPer =0;
static float G12Offset =0;
static unsigned short HskPer =0;
static unsigned short HskCalPer =0;
static short HskTelemEvery=0;
static short sendWave=1;
static unsigned short sipThrottle=680;
static short losSendData=0;
static int SoftTrigPer =0;
static short TrigADU5 =0;
static short TrigG12 =0;
static short TrigSoft =0;
static short enableChanServo=0;
static short pidGoal=0;
static short pedestalRun=0;
static short thresholdRun=0;

static unsigned short surfTrigBandSurf=0;
static unsigned short surfTrigBandVal1=0;
static unsigned short surfTrigBandVal2=0;
static short globalThreshold=0;
static short reprogramTurf=0;
static short surfhkPeriod=0;
static short surfhkTelemEvery=0;
static short turfhkTelemEvery=0;
static short numPedEvents=0;
static short threshScanStepSize=0;
static short threshScanPointsPerStep=0;
static unsigned short diskChoice;
static unsigned short diskBitMask;
static unsigned short eventBitMask;
static unsigned short hkBitMask;
static unsigned short storageType;
static unsigned short telemType;
static unsigned short encType;
static unsigned short altUsb;
static short whichSurf=0;
static short whichDac=0;
static float scaleFactor=1;
static short acqdWait=180;
static short maxQueue=300;
static short inodesKill=10000;
static short inodesDump=2000;



static int Direct = 0;		/* nonzero for direct connection to flight */

#define LOGSTRSIZE 2048
static char Logstr[LOGSTRSIZE];
static char Jsonstr[LOGSTRSIZE];
static char Logfilename[LOGSTRSIZE];
static FILE *Logfp;

#define PIDFILENAME "/tmp/cmd.pid"
#define CMD_COUNT_FILE "/home/anita/cmdSend/cmdCount.txt"
#define JSON_LOG_DIR "/home/anita/cmdSend/jsonLog"
int check_pidfile();
int make_pidfile();

void clr_cmd_log(void);
void set_cmd_log(char *fmt, ...);
void cmd_log(void);
int log_init(char *filename, char *msg);
void log_out(char *fmt, ...);
void log_close(void);

int
main(int argc, char *argv[])
{
    int key = 0;
    char logname[LOGSTRSIZE] = "cmdlog";

    // for getopt
    int c;
    extern char *optarg;

    while ((c = getopt(argc, argv, "dl:r:g:")) != EOF) {
	switch (c) {
	    case 'd':
		// Using direct connect to flight system.
		Direct = 1;
		break;
	    case 'l':
		Curlink = atoi(optarg);
		break;
	    case 'r':
		if (optarg[0] == '9') {
		    Curroute = ROUTE_COMM1;
		} else if (optarg[0] == 'C' || optarg[0] == 'c') {
		    Curroute = ROUTE_COMM2;
		}
		break;
	    case 'g':
	      snprintf(logname, LOGSTRSIZE, "%s", optarg);
		break;
	    default:
	    case '?':
		fprintf(stderr, "USAGE: cmd [-l link] [-r route] [-g logfile]\n");
		exit(1);
		break;
	}
    }

    {
	int pid = check_pidfile();
	if (pid) {
	    fprintf(stderr, "There may be another cmd program running, PID %d\n", pid);
	    fprintf(stderr, "\n");
	    fprintf(stderr, "Check using this command:\n");
	    fprintf(stderr, "   ps ax | grep '\\<cmd\\>' | grep -v grep\n");
	    fprintf(stderr, "It should print something like this if it is in use:\n");
	    fprintf(stderr, "  %5d pts/2    S+     0:00 cmdSend/cmd\n", pid);
	    fprintf(stderr, "\n");
	    fprintf(stderr, "If you see that another 'cmd' is in use, you\n");
	    fprintf(stderr, "may want to try to contact whoever is using it.\n");
	    fprintf(stderr, "\n");
	    fprintf(stderr, "If you don't know who to contact, you may want to\n");
	    fprintf(stderr, "kill the running program.  In our case, that would be\n");
	    fprintf(stderr, "    kill %d\n", pid);
	    fprintf(stderr, "and try starting 'cmd' again.\n");
	    fprintf(stderr, "\n");
	    fprintf(stderr, "If there was no 'cmd' program running or if\n");
	    fprintf(stderr, "you can't kill it, then just remove the %s file:\n",
	    	PIDFILENAME);
	    fprintf(stderr, "    rm %s\n", PIDFILENAME);
	    fprintf(stderr, "and try starting 'cmd' again.\n");
	    exit(1);
	}

	if (make_pidfile()) {
	    char junk[1];
	    fprintf(stderr,
	    	"Can't make PID file '%s' (%d). Press <ret> to continue.\n",
		PIDFILENAME, pid);

	    fgets(junk, 1, stdin);
	}
    }

    signal(SIGINT, catchsig);
    signal(SIGTERM, catchsig);

    if (serial_init()) {
	exit(1);
    }
    sprintf(Curcmd, "Default comd");
    Curcmdlen = strlen(Curcmd);
    initdisp();

    if (log_init(logname, "program starting.")) {
	screen_printf(
	    "WARNING! Can't open log file '%s' (%s). NO COMMAND LOGGING!\n",
	    "cmdlog", strerror(errno));
    }
    log_out("link is %d, route is %x.", Curlink, Curroute);

    while(1) {
	wprintw(Wuser, PROMPT);
	wrefresh(Wuser);
	key = GETKEY;
	if (isdigit(key)) {
	    /* get the numerical value of the cmd to send to flight */
	    accum_cmd(key);
	    continue;
	}
	wprintw(Wuser, "%c\n", key);
	wrefresh(Wuser);
	switch(key) {
	case TIMEOUT:
	    set_timeout();
	    break;
	case NEWCMD:
	    new_cmd();
	    break;
//	case CURCMD:
//	    cur_cmd();
//	    break;
	case LINKSEL:
	    select_link();
	    break;
	case RTSEL:
	    select_route();
	    break;
	case SHOWCMD:
	    show_cmds();
	    break;
	case EXPERT:
	     expert();
	     break;
	case CTL('L'):
	    clear_screen();
	    break;
	case CTL('C'):
	case QUIT:
	    quit_confirm();
	    break;
	default:
	    if(key != '\r' && key != '\n')
		wprintw(Wuser, "%c? What is that?\n", key);
		wrefresh(Wuser);
	    break;
	}
    }
    return 0;
}

static int Endmenu;	/* last line of menu */
static int Startuser;	/* first line of user area */

static void
initdisp()
{
    int i;

    (void)initscr();
    (void)cbreak();
    noecho();
    Endmenu = NMENULINES - 1;
    Wmenu = newwin(NMENULINES, 80, 0, 0);
    if (Wmenu == NULL) {
	fprintf(stderr,"Bad newwin [menu]\n");
	exit(1);
    }

    Startuser = Endmenu + 1;
    Wuser = newwin(LINES-NMENULINES, 80, Startuser, 0);
    if (Wuser == NULL) {
	fprintf(stderr,"Bad newwin [user]\n");
	exit(1);
    }
    scrollok(Wuser, 1);
    for(i=0; i<NMENULINES-1; i++) {
	if((progmenu[i] = malloc(strlen(menuformat[i] + 1))) == NULL) {
	    wprintw(Wuser, "Not enough memory\n");
	    exit(1);
	}
    }
    sprintf(progmenu[0], menuformat[0] , TIMEOUT, QUIT,
	LINKSEL, RTSEL, SHOWCMD, EXPERT);

    menu(progmenu, NMENULINES);
}

static void
menu(char **m, int nlines)
{
    int i;
    wclear(Wmenu);
    for (i=0; i<nlines-1; i++) {
	wmove(Wmenu, i, 0);
	wprintw(Wmenu, "%s", progmenu[i]);
    }
    wmove(Wmenu, Endmenu, 0);
    whline(Wmenu, ACS_HLINE, 80);
    wclear(Wuser);
    wnoutrefresh(Wmenu);
    wnoutrefresh(Wuser);
    doupdate();
}

static void
set_timeout()
{
    char resp[32];
    screen_dialog(resp, 31, "New timeout value [%ld] ", Timeout);
    if (resp[0] == '\0') {
	screen_printf("value unchanged\n");
    } else {
	Timeout = atol(resp);
	screen_printf("new timeout is %ld seconds\n", Timeout);
	log_out("timeout set to %ld seconds", Timeout);
    }
}

static void
select_link(void)
{
    char resp[32];
    screen_dialog(resp, 31,
    	"Choose link 0=LOS, 1=TDRSS (COMM1), 2=Iridium (COMM2) [%d] ", Curlink);
    if (resp[0] == '\0') {
	screen_printf("value unchanged\n");
    } else {
	if (resp[0] < '0' || resp[0] > '2') {
	    screen_printf("Sorry, link must be 0, 1, or 2, not %c\n", resp[0]);
	    return;
	}
	Curlink = resp[0] - '0';
	screen_printf("New link is %d\n", Curlink);
	log_out("link set to '%d'", Curlink);
    }
}

static void
select_route(void)
{
    char resp[32];
    screen_dialog(resp, 31, "Choose route 9=COMM1, C=COMM2 [%x] ", Curroute);
    if (resp[0] == '\0') {
	screen_printf("value unchanged\n");
    } else {
	if (resp[0] == '9') {
	    Curroute = ROUTE_COMM1;
	} else if (resp[0] == 'C' || resp[0] == 'c') {
	    Curroute = ROUTE_COMM2;
	} else {
	    screen_printf("Sorry, route must be 9 or C, not %c\n", resp[0]);
	    return;
	}
	screen_printf("New route is %x\n", Curroute);
	log_out("route set to '%x'", Curroute);
    }
}

static void
new_cmd(void)
{
    unsigned char cmd[10];
    int i;
    char msg[1024];
    char *mp = msg;
    int n = 0;
    char resp[32];
    int val;

    screen_printf("Enter the value of each command byte.\n");
    screen_printf("Enter '.' (period) to finish.\n");
    memset(msg, '\0', 1024);
    while (1) {
	screen_dialog(resp, 32, "Command byte %d ", n);
	if (resp[0] == '.') {
	    break;
	}
	val = atoi(resp);
	if (n == 0 && (val < 128 || val > 255)) {
	    screen_printf("Sorry, val must be between 129 and 255, not %d\n",
		val);
	    screen_printf("Try again.\n");
	} else {
	    cmd[n] = (unsigned char)(val & 0x000000ff);
	    sprintf(mp, "%d ", cmd[n]);
	    mp += strlen(mp);
	    n++;
	    if (n >= 10) {
		break;
	    }
	}
    }

    set_cmd_log("new_cmd: %s", msg);
    for (i=0; i < n; i++) {
	Curcmd[i*2] = i;
	Curcmd[(i*2)+1] = cmd[i];
    }
    Curcmdlen = n*2;


    //    char cmdString[180];
    //    sprintf(cmdString,"%d",Curcmd[0]);
    //    for(i=1;i<Curcmdlen;i++)
    //      sprintf(cmdString,"%s %d",Curcmd[i]);
    //    screen_printf("%s",cmdString);

    sendcmd(Fd, Curcmd, Curcmdlen);
}

#ifdef NOTDEF
static void
new_cmd(void)
{
    char resp[32];
    screen_dialog(resp, 31, "New command string? ");
    strncpy(Curcmd, resp, CMDLEN-5);
    sendcmd(Fd, Curcmd, strlen(Curcmd));
}
#endif

//static void
//cur_cmd()
//{
//     screen_printf("This is not a used ANITA function.\n");
//     int i;
//    screen_printf("Sending: ");
//    for (i=0; i<Curcmdlen; i++) {
//	if (i % 2) {
//	    screen_printf("%d ", Curcmd[i]);
//	}
//    }
//    screen_printf("\n");
//    sendcmd(Fd, Curcmd, Curcmdlen);
//}



static void
clear_screen(void)
{
    menu(progmenu, NMENULINES);
}

static void
quit_confirm(void)
{
    if (screen_confirm("Really quit")) {
	screen_printf("Bye bye...");
	endwin();
	generic_exit_routine();
    } else {
	screen_printf("\nNot quitting\n");
    }
}

static void
generic_exit_routine()
{
    unlink(PIDFILENAME);
    log_close();
    if (serial_end()) {
	exit(1);
    }
    exit(0);
}

void
catchsig(int sig)
{
    signal(sig, catchsig);
    if (sig == SIGINT) {
	quit_confirm();
	wprintw(Wuser, PROMPT);
	wrefresh(Wuser);
    } else if (sig == SIGTERM) {
	screen_printf("I've been killed...");
	endwin();
	generic_exit_routine();
    }
}

int
screen_confirm(char *fmt, ...)
{
    char s[512];
    va_list ap;
    int key;
    if (fmt != NULL) {
	va_start(ap, fmt);
	vsprintf(s, fmt, ap);
	va_end(ap);
    	wprintw(Wuser, "%s [yn] ", s);
	wrefresh(Wuser);
	/* nodelay(Wuser, FALSE); */
	key = wgetch(Wuser);
	/* nodelay(Wuser, TRUE); */
	if (key == 'y' || key == 'Y' || key == '\n') {
	    return 1;
	} else {
	    return 0;
	}
    }
    return 0;
}

void
screen_dialog(char *response, int nbytes, char *fmt, ...)
{
    char prompt[512];
    va_list ap;
    if (fmt != NULL) {
	va_start(ap, fmt);
	vsprintf(prompt, fmt, ap);
	va_end(ap);
    } else {
	strcpy(prompt, "Well? ");
    }
    wprintw(Wuser, "%s ", prompt);
    wrefresh(Wuser);
    echo();
    /* nodelay(Wuser, FALSE); */
    wgetnstr(Wuser, response, nbytes);
    noecho();
    /* nodelay(Wuser, TRUE); */
}

void
screen_mesg(char *fmt, ...)
{
    char s[512];
    va_list ap;
    if (fmt != NULL) {
	va_start(ap, fmt);
	vsprintf(s, fmt, ap);
	va_end(ap);
	wprintw(Wuser, "%s ", s);
    }
    wprintw(Wuser, "[press any key to continue] ");
    wrefresh(Wuser);
    /* nodelay(Wuser, FALSE); */
    (void)wgetch(Wuser);
    /* nodelay(Wuser, TRUE); */
}

void
screen_printf(char *fmt, ...)
{
    char s[1024];
    va_list ap;
    if (fmt != NULL) {
	va_start(ap, fmt);
	vsprintf(s, fmt, ap);
	va_end(ap);
	wprintw(Wuser, "%s", s);
	wrefresh(Wuser);
    }
}

void
screen_beep(void)
{
    beep();
}


int toggleCRTCTS(char devName[])
/*! This is needed on the whiteheat ports after a reboot, after a power cycle it comes up in the correct state.
*/
{
    int fd,retVal;
    struct termios options;

/* open a serial port */
    fd = open(devName, O_RDWR | O_NOCTTY |O_NONBLOCK );
    if(fd<0)
    {
        fprintf(stderr,"open %s: %s\n",devName,strerror(errno));
        return -1;
    }

/* port settings */
    retVal=tcgetattr(fd, &options);            /* get current port settings */
    if(retVal<0) {
        fprintf(stderr,"tcgetattr on fd %d: %s\n",fd,strerror(errno));
        return -1;
    }

    options.c_iflag=1;
    options.c_oflag=0;
    options.c_cflag=3261;
    options.c_lflag=0;
    strncpy((char*)options.c_cc,"",2);
    options.c_ispeed=13;
    options.c_ospeed=13;

    cfsetispeed(&options, B2400);    /* set input speed */
    cfsetospeed(&options, B2400);    /* set output speed */

    options.c_cflag &= ~PARENB;         /* clear the parity bit  */
    options.c_cflag &= ~CSTOPB;         /* clear the stop bit  */
    options.c_cflag &= ~CSIZE;          /* clear the character size  */
    options.c_cflag |= CS8;  /* set charater size to 8 bits  */
    options.c_cflag |= CRTSCTS;        /* Toggle on */
    options.c_lflag &= (ICANON | ECHO | ECHOE | ISIG); /* raw input mode  */
    options.c_cflag |= (CLOCAL | CREAD);

    options.c_oflag &= ~(OPOST | ONLCR );
/*     options.c_oflag |= ( ONLCR | OPOST); */

    tcsetattr(fd, TCSANOW, &options);   /* activate the settings  */
    if(retVal<0) {
        fprintf(stderr,"tcsetattr on fd %d: %s\n",fd,strerror(errno));
        return -1;
    }
    close(fd);

    return 0;
}



int
serial_init(void)
{
  //  toggleCRTCTS(PORT);
    struct termios newopts;
    Fd = open(PORT, O_RDWR | O_NDELAY);
    if (Fd == -1) {
	fprintf(stderr,"can't open '%s' (%s)\n",
	    PORT, strerror(errno));
	return -1;
    } else {
	//fcntl(Fd, F_SETFL, 0);	/* blocking reads */
	fcntl(Fd, F_SETFL, FNDELAY);	/* non-blocking reads */
    }

    tcgetattr(Fd, &Origopts);
    tcgetattr(Fd, &newopts);

    if (Direct) {
	// direct connection to flight system.
	cfsetispeed(&newopts, B1200);
	cfsetospeed(&newopts, B1200);
    } else {
	// connection via sip
	cfsetispeed(&newopts, B2400);
	cfsetospeed(&newopts, B2400);
    }

    newopts.c_cflag |= (CLOCAL | CREAD);   /* enable receiver, local mode */

    /* set 8N1 - 8 bits, no parity, 1 stop bit */
    newopts.c_cflag &= ~PARENB;
    newopts.c_cflag &= ~CSTOPB;
    newopts.c_cflag &= ~CSIZE;
    newopts.c_cflag |= CS8;
    newopts.c_cflag &= ~CRTSCTS;

    newopts.c_iflag &= ~(INLCR | ICRNL);
    newopts.c_iflag &= ~(IXON | IXOFF | IXANY);	/* no XON/XOFF */

    newopts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	/* raw input */
    newopts.c_oflag &= ~OPOST;	/* raw output */

    tcflush(Fd, TCIFLUSH);

    tcsetattr(Fd, TCSANOW, &newopts);



    // play with RTS & DTR
    int iFlags;

    // turn on RTS
    iFlags = TIOCM_RTS;
    ioctl(Fd, TIOCMBIS, &iFlags);

    // turn off RTS
    iFlags = TIOCM_RTS;
    ioctl(Fd, TIOCMBIC, &iFlags);

    // turn on DTR
    iFlags = TIOCM_DTR;
    ioctl(Fd, TIOCMBIS, &iFlags);

    // turn off DTR
    iFlags = TIOCM_DTR;
    ioctl(Fd, TIOCMBIC, &iFlags);

    return 0;
}

int
serial_end(void)
{
    tcsetattr(Fd, TCSANOW, &Origopts);
    if (close(Fd)) {
	fprintf(stderr, "ERROR: bad close on %s (%s)\n",
	    PORT, strerror(errno));
	return -1;
    }
    return 0;
}

void
sendcmd(int fd, unsigned char *s, int len)
{
    int n;
    char *bp;
    unsigned char buf[CMDLEN];
    int i;

    if (len > CMDLEN - 5) {
	len = CMDLEN - 5;
	screen_printf("Warning: command too long; truncating.\n");
    }

    if (len % 2) {
	screen_printf("Warning: command length is odd; truncating.\n");
	len--;
    }

    if (Direct) {
	// direct connection to flight system
	int i;
	for (i=0; i<len; i += 2) {
	    bp =(char*) buf;
#ifdef NOTDEF
	    *bp++ = 'p';
	    *bp++ = 'a';
	    *bp++ = 'u';
	    *bp++ = 'l';
	    *bp++ = ' ';
#endif
	    *bp++ = 0x10;
	    *bp++ = 0x14;
	    *bp++ = 2;
	    *bp++ = s[i];
	    *bp++ = s[i+1];
	    *bp++ = 0x03;
	    write (fd, buf, 6);
#ifdef NOTDEF
	    {
		int n;
		for (n=0; n<6; n++) {
		    screen_printf("0x%02x ", buf[n]);
		}
		screen_printf("\n");
	    }
#endif // NOTDEF
	}
	screen_printf("Direct to science computer.\n");
    } else {
	// connection via sip
	bp = (char*)buf;
	*bp++ = 0x10;
	*bp++ = Curlink;
	*bp++ = Curroute;
	*bp++ = len;
	for (i=0; i < len; i++) {
	    *bp++ = *s++;
	}
	*bp = 0x03;
	n = len + 5;
	write (fd, buf, n);
	wait_for_ack();
    }
    ///RJN hack for creating a more useful cmd output
    write_cmd_json_file(Logstr);
    cmd_log();

#ifdef PRINT_EXTRA_STUFF
    screen_printf("sending ");
#ifdef NOTDEF
    for (i=0; i<4; i++) {
	screen_printf("0x%02x ", buf[i]);
    }
    for ( ; i < n-1; i++) {
	screen_printf("%c ", buf[i]);
    }
    for ( ; i < n; i++) {
	screen_printf("0x%02x ", buf[i]);
    }
#else
    for (i=0; i<n; i++) {
	screen_printf("0x%02x ", buf[i]);
    }
    screen_printf("\n");
    for (i=0; i<n; i++) {
	screen_printf("%c", buf[i]);
    }
#endif /* NOTDEF */
    screen_printf("\n");
#endif /* PRINT_EXTRA_STUFF */

}

void
wait_for_ack(void)
{
    fd_set f;
    int n;
    int ret;
    struct timeval t;

    screen_printf("Awaiting CSBF response: ");

    t.tv_sec = Timeout;
    t.tv_usec = 0L;
    FD_ZERO(&f);
    FD_SET(Fd, &f);
    ret = select(Fd+1, &f, NULL, NULL, &t);
    if (ret == -1) {
	screen_printf("select() error (%s)\n", strerror(errno));
	log_out("select() error (%s)", strerror(errno));
	return;
    } else if (ret == 0 || !FD_ISSET(Fd, &f)) {
	screen_printf("no response after %ld seconds\n", Timeout);
	log_out("CSBF response; no response after %ld seconds", Timeout);
	return;
    }
    sleep(1);
    n = read(Fd, ibuf, IBUFSIZE);
    if (n > 0) {
	if (ibuf[0] != 0xFA || ibuf[1] != 0xF3) {
	    screen_printf("malformed response!\n%x\t\n",ibuf[0],ibuf[1]);
	    log_out("CSBF response; malformed response!");
	    return;
	}

	if (ibuf[2] == 0x00) {
	    screen_printf("OK\n");
	    log_out("CSBF response; OK");
	} else if (ibuf[2] == 0x0A) {
	    screen_printf("GSE operator disabled science commands.\n");
	    log_out("CSBF response; GSE operator disabled science commands.");
	} else if (ibuf[2] == 0x0B) {
	    screen_printf("routing address does not match selected link.\n");
	    log_out( "CSBF response; routing address does not match link.");
	} else if (ibuf[2] == 0x0C) {
	    screen_printf("selected link not enabled.\n");
	    log_out("CSBF response; selected link not enabled.");
	} else if (ibuf[2] == 0x0D) {
	    screen_printf("miscellaneous error.\n");
	    log_out("CSBF response; miscellaneous error.");
	} else {
	    screen_printf("unknown error code (0x%02x)", ibuf[2]);
	    log_out("CSBF response; unknown error code (0x%02x)", ibuf[2]);
	}
    } else {
	screen_printf("strange...no response read\n");
	log_out("NSBF response; strange...no response read");
    }
}

static void
accum_cmd(int key)
{
    char digits[4];
    int cmdCode=0;
    int n;

    screen_printf( "\nEnter digits for flight cmd. Press ESC to cancel.\n");
    digits[3] = '\0';

    n = 0;
    do {
	if (isdigit(key)) {
	    digits[n] = key;
	    screen_printf("%c", key);
	    n++;
	} else if (key == 0x1b) {
	    // escape key
	    screen_printf(" cancelled\n");
	    return;
	} else {
	    screen_beep();
	}
    } while (n < 3 && (key = GETKEY));

    cmdCode = atoi(digits);
    if (cmdCode > Csize - 1) {
	screen_printf(" No such command '%d'\n", cmdCode);
	return;
    }

    //screen_printf("       got '%d'\n", cmdCode);
    //screen_printf("       got '%s'\n", digits);

    if (Cmdarray[cmdCode].f == NULL) {
	screen_printf(" No such command '%d'\n", cmdCode);
	return;
    }
    screen_printf(" %s\n", Cmdarray[cmdCode].name);
    Cmdarray[cmdCode].f(cmdCode);
}

static void
show_cmds(void)
{
    int got = 0;
    int i,j;
    int val[3];

    int easyCmdArray[33]={1,2,3,11,132,150, 151, 152,153,154,155,
			  156,157,158,159,160, 161, 162, 163, 171,172,173,
			  174,175,182,183,210,230,231,235,238,239,255};

    for (j=0; j<33; j++) {
	i=easyCmdArray[j];

	if (Cmdarray[i].f != NULL) {
	    val[got] = i;
	    got++;
	}
	if (got == 3) {
	    screen_printf("%03d %s", val[0], Cmdarray[val[0]].name);
	    screen_printf("%03d %s", val[1], Cmdarray[val[1]].name);
	    screen_printf("%03d %s\n", val[2], Cmdarray[val[2]].name);
	    got = 0;
	}
    }
    if (got) {
	for (i=0; i<got; i++) {
	    screen_printf("%d %s", val[i], Cmdarray[val[i]].name);
	}
	screen_printf("\n");
    }
}


static void
expert(void)
{
    int got = 0;
    int i;
    int val[3];

    for (i=0; i<256; i++) {

	if (Cmdarray[i].f != NULL) {
	    val[got] = i;
	    got++;
	}
	if (got == 3) {
	    screen_printf("%03d %s", val[0], Cmdarray[val[0]].name);
	    screen_printf("%03d %s", val[1], Cmdarray[val[1]].name);
	    screen_printf("%03d %s\n", val[2], Cmdarray[val[2]].name);
	    got = 0;
	}
    }
    if (got) {
	for (i=0; i<got; i++) {
	    screen_printf("%d %s", val[i], Cmdarray[val[i]].name);
	}
	screen_printf("\n");
    }
}


static void
LOG_REQUEST_COMMAND(int cmdCode)
{
    char resp[32];
    short logNum=1;
    short numLines=500;
    short t;
    int fileNum=0;

    for(fileNum=LOG_FIRST_LOG;fileNum<LOG_NOT_A_LOG;fileNum++) {
	if(fileNum%2==1)
	    screen_printf("%d -- %s\t\t",fileNum,logRequestName(fileNum));
	else
	    screen_printf("%d -- %s\n",fileNum,logRequestName(fileNum));
    }
    screen_printf("\n");
    screen_dialog(resp, 31, "Which log? (-1 to cancel) [%d] ",logNum);
    if (resp[0] != '\0') {
	logNum = atoi(resp);

	if(logNum>=LOG_FIRST_LOG && logNum<LOG_NOT_A_LOG) {
	    //Good
	} else if (logNum == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be %d-%d, not %d.\n", LOG_FIRST_LOG,LOG_NOT_A_LOG,logNum);
	    return;
	}
    }
    screen_dialog(resp,31,"Max. number lines (-1 to cancel [%d]",numLines);
    if (resp[0] != '\0') {
	numLines = atoi(resp);

	if (numLines < 0) {
	    screen_printf("Cancelled\n");
	    return;
	}
    }


    unsigned char val=logNum;

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = val;
    Curcmd[4] = 2;
    val=(numLines&0xff);
    Curcmd[5] = val;
    Curcmd[6] = 3;
    val=((numLines&0xf00)>>8);
    Curcmd[7] = val;
    Curcmd[8] = 4;
    Curcmdlen = 8;

    set_cmd_log("%d; %d lines from %s", cmdCode, numLines,logRequestName(logNum));
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
JOURNALCTL_COMMAND(int cmdCode)
{
    char resp[32];
    short jcOpt=1;
    short jcArg=0;
    short numLines=500;
    short t;
    short det;
    int journalOpt=0;

    for(journalOpt=JOURNALCTL_OPT_COMM;journalOpt<=JOURNALCTL_NO_OPT;journalOpt++) {
	if(journalOpt%2==0)
	    screen_printf("%d -- %s\t\t",journalOpt,journalOptionName(journalOpt));
	else
	    screen_printf("%d -- %s\n",journalOpt,journalOptionName(journalOpt));
    }
    screen_printf("\n");
    screen_dialog(resp, 31, "Which journalctl option? (-1 to cancel) [%d] ",jcOpt);
    if (resp[0] != '\0') {
	jcOpt = atoi(resp);

	if(jcOpt>=JOURNALCTL_OPT_COMM && jcOpt<=JOURNALCTL_NO_OPT) {
	    //Good
	} else if (jcOpt == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be %d-%d, not %d.\n", JOURNALCTL_OPT_COMM,JOURNALCTL_NO_OPT,jcOpt);
	    return;
	}
    }
    screen_dialog(resp,31,"Max. number lines (-1 to cancel [%d]",numLines);
    if (resp[0] != '\0') {
	numLines = atoi(resp);

	if (numLines < 0) {
	    screen_printf("Cancelled\n");
	    return;
	}
    }



    if(jcOpt==JOURNALCTL_OPT_COMM) {

	screen_printf("1.  Acqd       6.  GPSd\n");
	screen_printf("2.  Archived   7.  Hkd\n");
	screen_printf("3.  Calibdd    8.  LOSd\n");
	screen_printf("4.  Cmdd       9.  Monitord\n");
	screen_printf("5.  Eventd     10. Prioritizerd\n");
	screen_printf("11. SIPd       12. Playbackd\n");
	screen_printf("13. LogWatchd  14. Ntud\n");
	screen_printf("15. Openportd  16. RTLd\n");
	screen_printf("17. Tuffd\n");
	screen_dialog(resp, 31, "Which Process Name? (-1 to cancel) [%d] ",
		      jcArg);
	if (resp[0] != '\0') {
	    det = atoi(resp);
	    if (1 <= det && det <= 17) {
		switch(det){
		    case 1:
			jcArg = ID_ACQD;
			break;
		    case 2:
			jcArg = ID_ARCHIVED;
			break;
		    case 3:
			jcArg = ID_CALIBD;
			break;
		    case 4:
			jcArg = ID_CMDD;
			break;
		    case 5:
			jcArg = ID_EVENTD;
			break;
		    case 6:
			jcArg = ID_GPSD;
			break;
		    case 7:
			jcArg = ID_HKD;
			break;
		    case 8:
			jcArg = ID_LOSD;
			break;
		    case 9:
			jcArg = ID_MONITORD;
			break;
		    case 10:
			jcArg = ID_PRIORITIZERD;
			break;
		    case 11:
			jcArg = ID_SIPD;
			break;
		    case 12:
			jcArg = ID_PLAYBACKD;
			break;
		    case 13:
			jcArg = ID_LOGWATCHD;
			break;
		    case 14:
			jcArg = ID_NTUD;
			break;
		    case 15:
			jcArg = ID_OPENPORTD;
			break;
		    case 16:
			jcArg = ID_RTLD;
			break;
                    case 17:
			jcArg = ID_TUFFD;
			break;

		    default: break;
		}
	    } else if (det == -1) {
		screen_printf("Cancelled\n");
		return;
	    } else {
		screen_printf("Value must be 1-17, not %d.\n", det);
		return;
	    }
	}
    }
    else if(jcOpt==JOURNALCTL_OPT_SYSLOG_FACILITY) {
	screen_dialog(resp, 31, "Which SYSLOG Facility? (-1 to cancel) [%d] ",
		      jcArg);
	if (resp[0] != '\0') {
	    det=atoi(resp);
	    if(det>=0 && det<=255) {
		jcArg=det;
	    } else if (det == -1) {
		screen_printf("Cancelled\n");
		return;
	    } else {
		screen_printf("Value must be 0-255, not %d.\n", det);
		return;
	    }
	}
    }
    else if(jcOpt==JOURNALCTL_OPT_PRIORITY) {
	screen_dialog(resp, 31, "Which SYSLOG Priority? (-1 to cancel) [%d] ",
		      jcArg);
	if (resp[0] != '\0') {
	    det=atoi(resp);
	    if(det>=1 && det<=7) {
		jcArg=det;
	    } else if (det == -1) {
		screen_printf("Cancelled\n");
		return;
	    } else {
		screen_printf("Value must be 1-7, not %d.\n", det);
		return;
	    }
	}
    }

    unsigned char val=jcOpt;

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = val;
    Curcmd[4] = 2;
    val=(numLines&0xff);
    Curcmd[5] = val;
    Curcmd[6] = 3;
    val=((numLines&0xf00)>>8);
    Curcmd[7] = val;
    Curcmd[8] = 4;
    val=(jcArg&0xff);
    Curcmd[9] = val;
    Curcmd[10] = 5;
    val=((jcArg&0xff00)>>8);
    Curcmd[11] = val;
    Curcmdlen = 12;

    set_cmd_log("%d; %d lines from %s", cmdCode, numLines,journalOptionName(jcOpt));
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CMD_DISABLE_DISK(int cmdCode)
{

    char resp[32];
    short det;
    short v;
    screen_printf("You may be about to do something very bad\n");
    screen_printf("Only disable or enable disks if you are really certain of what is going on\n");
    screen_printf("0: Enable a disk    1: Disable a disk\n");
    screen_printf("2: Set Bitmask\n");
    screen_dialog(resp, 31,
	"Which function (0, 1, or 2)  (-1,to cancel), [%d]",
	diskChoice);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 2) {
	    diskChoice= v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-2, not %d.\n", v);
	    return;
	}
    }

    diskBitMask=0;
    if(diskChoice==0 || diskChoice==1) {
	screen_printf("0: Helium1            1: Helium2\n");
	screen_printf(" 3: Ntu\n");
	screen_dialog(resp,31,"Which disk (-1, to cancel)\n");

	if(resp[0] != '\0') {
	    v = atoi(resp);
	    if (0 <= v && v <= 3) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-3, not %d.\n", v);
		return;
	    }
	} else return;
    }
    else {
	screen_printf("0: Helium1            1: Helium2\n");
	screen_printf("3: Ntu\n");
	screen_dialog(resp,31,"Which disk to add to mask (-1, to cancel) [mask: %#x]",diskBitMask);
	if(resp[0] != '\0') {
	    v = atoi(resp);
	    if (0 <= v && v <= 3) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-3, not %d.\n", v);
		return;
	    }
	} else return;

	while(1) {
	    screen_printf("0: Helium1            1: Helium2\n");
	    screen_printf("3: Ntu\n");
	    screen_dialog(resp,31,"Which disk to add? (5 to send, -1 to cancel) [mask: %#x]",diskBitMask);
	    if(resp[0] != '\0') {
		v = atoi(resp);
		if (0 <= v && v <= 3) {
		    diskBitMask |= diskBitMasks[v];
		} else if (v == -1) {
		    screen_printf("Cancelled.\n");
		    return;
		} else if (v == 5) {
		    break;
		}
		else {
		    screen_printf("Value must be 0-5, not %d.\n", v);
		    return;
		}
	    } else return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = diskChoice;
    Curcmd[4] = 2;
    Curcmd[5] = (diskBitMask&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = ((diskBitMask&0xff00)>>8);
    Curcmdlen = 8;
    set_cmd_log("%d; Send event disk bit mask command %d %#x.", cmdCode, diskChoice,
		diskBitMask);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
TAIL_MESSAGES(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Send the last X lines of /var/log/messages  (1-1000, -1 to cancel) [%d] ",
	numLines);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (1 <= t && t <= 1000) {
	    numLines = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Number of lines must be in range 1-1000, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (numLines&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((numLines&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Tail last %d lines of /var/log/messages.", cmdCode, numLines);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
TAIL_ANITA(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Send the last X lines of /var/log/anita.log  (1-1000, -1 to cancel) [%d] ",
	numLines);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (1 <= t && t <= 1000) {
	    numLines = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Number of lines must be in range 1-1000, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (numLines&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((numLines&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Tail last %d lines of /var/log/anita.log.", cmdCode, numLines);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CMD_START_NEW_RUN(int cmdCode)
{
    if (screen_confirm("Really start new run?")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Start new run.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
CMD_MOUNT_ARGH(int cmdCode)
{
    if (screen_confirm("Do you really want to try remonting the mtrons?")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Remount mtrons.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
CMD_MAKE_NEW_RUN_DIRS(int cmdCode)
{
    if (screen_confirm("Really make new run directories?")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Make new run directories.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
CMD_SIPD_REBOOT(int cmdCode)
{
    if (screen_confirm("Really reboot through SIPD?")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; SIPD Reboot.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
CMD_SHUTDOWN_HALT(int cmdCode)
{
    if (screen_confirm("Really shutdown the computer")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Shutdown and halt computer.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
CMD_REBOOT(int cmdCode)
{
    if (screen_confirm("Really reboot the computer")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Reboot.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
CMD_REALLY_KILL_PROGS(int cmdCode)
{
    short det;
    int i;
    char resp[32];


    screen_printf("1.  Acqd       6.  GPSd\n");
    screen_printf("2.  Archived   7.  Hkd\n");
    screen_printf("3.  Calibdd    8.  LOSd\n");
    screen_printf("4.  Cmdd       9.  Monitord\n");
    screen_printf("5.  Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd       12. Playbackd\n");
    screen_printf("13. LogWatchd  14. Ntud\n");
    screen_printf("15. Openportd  16. RTLd\n");
    screen_printf("17. Tuffd  \n");
    screen_printf("19. All (except SIPd and Cmdd)\n");
    screen_dialog(resp, 31, "Kill -9 which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Prog_det = ACQD_ID_MASK;
	    break;
          case 2:
            Prog_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Prog_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Prog_det = CMDD_ID_MASK;
	    break;
          case 5:
            Prog_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Prog_det = GPSD_ID_MASK;
	    break;
          case 7:
            Prog_det = HKD_ID_MASK;
	    break;
          case 8:
            Prog_det = LOSD_ID_MASK;
	    break;
          case 9:
            Prog_det = MONITORD_ID_MASK;
            break;
          case 10:
            Prog_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Prog_det = SIPD_ID_MASK;
	    break;
          case 12:
            Prog_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Prog_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Prog_det = NTUD_ID_MASK;
	    break;
          case 15:
            Prog_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Prog_det = RTLD_ID_MASK;
	    break;
          case 17:
            Prog_det = TUFFD_ID_MASK;
	    break;
          case 19:
            Prog_det = ALL_ID_MASK;
            Prog_det &= ~CMDD_ID_MASK;
            Prog_det &= ~SIPD_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Prog_det&0xff0000)>>16);
    Curcmdlen = 8;
    set_cmd_log("%d; Program  %d killed.", cmdCode, Prog_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}




static void
CMD_KILL_PROGS(int cmdCode)
{
    short det;
    int i;
    char resp[32];

    screen_printf("1.  Acqd       6.  GPSd\n");
    screen_printf("2.  Archived   7.  Hkd\n");
    screen_printf("3.  Calibdd    8.  LOSd\n");
    screen_printf("4.  Cmdd       9.  Monitord\n");
    screen_printf("5.  Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd       12. Playbackd\n");
    screen_printf("13. LogWatchd  14. Ntud\n");
    screen_printf("15. Openportd  16. RTLd\n");
    screen_printf("17. Tuffd  \n");
    screen_printf("19. All (except SIPd and Cmdd)\n");
    screen_dialog(resp, 31, "Kill which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Prog_det = ACQD_ID_MASK;
	    break;
          case 2:
            Prog_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Prog_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Prog_det = CMDD_ID_MASK;
	    break;
          case 5:
            Prog_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Prog_det = GPSD_ID_MASK;
	    break;
          case 7:
            Prog_det = HKD_ID_MASK;
	    break;
          case 8:
            Prog_det = LOSD_ID_MASK;
	    break;
          case 9:
            Prog_det = MONITORD_ID_MASK;
            break;
          case 10:
            Prog_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Prog_det = SIPD_ID_MASK;
	    break;
          case 12:
            Prog_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Prog_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Prog_det = NTUD_ID_MASK;
	    break;
          case 15:
            Prog_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Prog_det = RTLD_ID_MASK;
	    break;
          case 17:
            Prog_det = TUFFD_ID_MASK;
	    break;
          case 19:
            Prog_det = ALL_ID_MASK;
	    Prog_det &= ~CMDD_ID_MASK;
	    Prog_det &= ~SIPD_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }



    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Prog_det&0xff0000)>>16);
    Curcmdlen = 8;
    set_cmd_log("%d; Program  %d killed.", cmdCode, Prog_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CMD_RESPAWN_PROGS(int cmdCode)
{
    short det;
    int i;
    char resp[32];

    screen_printf("1.  Acqd       6.  GPSd\n");
    screen_printf("2.  Archived   7.  Hkd\n");
    screen_printf("3.  Calibdd    8.  LOSd\n");
    screen_printf("4.  Cmdd       9.  Monitord\n");
    screen_printf("5.  Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd       12. Playbackd\n");
    screen_printf("13. LogWatchd  14. Ntud\n");
    screen_printf("15. Openportd  16. RTLd\n");
    screen_printf("17. Tuffd \n");
    screen_printf("19. All (except SIPd and Cmdd)\n");
    screen_dialog(resp, 31, "Respawn which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Prog_det = ACQD_ID_MASK;
	    break;
          case 2:
            Prog_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Prog_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Prog_det = CMDD_ID_MASK;
	    break;
          case 5:
            Prog_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Prog_det = GPSD_ID_MASK;
	    break;
          case 7:
            Prog_det = HKD_ID_MASK;
	    break;
          case 8:
            Prog_det = LOSD_ID_MASK;
	    break;
          case 9:
            Prog_det = MONITORD_ID_MASK;
            break;
          case 10:
            Prog_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Prog_det = SIPD_ID_MASK;
	    break;
          case 12:
            Prog_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Prog_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Prog_det = NTUD_ID_MASK;
	    break;
          case 15:
            Prog_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Prog_det = RTLD_ID_MASK;
	    break;
          case 17:
            Prog_det = TUFFD_ID_MASK;
	    break;
          case 19:
            Prog_det = ALL_ID_MASK;
	    Prog_det &= ~CMDD_ID_MASK;
	    Prog_det &= ~SIPD_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }


    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Prog_det&0xff0000)>>16);
    Curcmdlen = 8;
    screen_printf("Sent %d %d %d\n",cmdCode,(Prog_det&0xff),((Prog_det&0xf00)>>8));
    set_cmd_log("%d; Program  %d respawned.", cmdCode, Prog_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
CMD_START_PROGS(int cmdCode)
{
    short det;
    int i;
    char resp[32];

    screen_printf("1.  Acqd       6.  GPSd\n");
    screen_printf("2.  Archived   7.  Hkd\n");
    screen_printf("3.  Calibdd    8.  LOSd\n");
    screen_printf("4.  Cmdd       9.  Monitord\n");
    screen_printf("5.  Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd       12. Playbackd\n");
    screen_printf("13. LogWatchd  14. Ntud\n");
    screen_printf("15. Openportd  16. RTLd\n");
    screen_printf("17. Tuffd  \n");
    screen_printf("19. All (except SIPd and Cmdd)\n");
    screen_dialog(resp, 31, "Start which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Prog_det = ACQD_ID_MASK;
	    break;
          case 2:
            Prog_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Prog_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Prog_det = CMDD_ID_MASK;
	    break;
          case 5:
            Prog_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Prog_det = GPSD_ID_MASK;
	    break;
          case 7:
            Prog_det = HKD_ID_MASK;
	    break;
          case 8:
            Prog_det = LOSD_ID_MASK;
	    break;
          case 9:
            Prog_det = MONITORD_ID_MASK;
            break;
          case 10:
            Prog_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Prog_det = SIPD_ID_MASK;
	    break;
          case 12:
            Prog_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Prog_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Prog_det = NTUD_ID_MASK;
	    break;
          case 15:
            Prog_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Prog_det = RTLD_ID_MASK;
	    break;
          case 17:
            Prog_det = TUFFD_ID_MASK;
	    break;
          case 19:
            Prog_det = ALL_ID_MASK;
	    Prog_det &= ~CMDD_ID_MASK;
	    Prog_det &= ~SIPD_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }



    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Prog_det&0xff0000)>>16);
    Curcmdlen = 8;
    set_cmd_log("%d; Program  %d started.", cmdCode, Prog_det);
    sendcmd(Fd, Curcmd, Curcmdlen);

}




static void
CMD_MOUNT_NEXT_SATA(int cmdCode)
{
    short bladeOrMini=0;
    short whichDrive;
    char resp[32];
    char *driveName[2]={"satablade","satamini"};

    short tempVal=0;
    screen_dialog(resp, 31,
		  "Helium1 (0) or Helium2 (1), -1 to cancel) [%d] ",
		  bladeOrMini);
    if (resp[0] != '\0') {
	tempVal = atoi(resp);
	if (0 <= tempVal && tempVal <= 1) {
	    bladeOrMini = tempVal;
	} else if (tempVal == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-1, not %d.\n", tempVal);
	    return;
	}
    }
    tempVal=0;
    screen_dialog(resp, 31,
		  "Drive number, 0 for next, -1 to cancel) [%d] ",
		  whichDrive);
    if (resp[0] != '\0') {
	tempVal = atoi(resp);
	if (0 <= tempVal && ((tempVal <= 8 && bladeOrMini==0)
			     || (tempVal <= 8 && bladeOrMini==1))) {
	    whichDrive = tempVal;
	} else if (tempVal == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-4(8), not %d.\n", tempVal);
	    return;
	}
    }

   if (screen_confirm("Really mount next %s?",driveName[bladeOrMini])) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmd[2] = 1;
	Curcmd[3] = bladeOrMini;
	Curcmd[4] = 2;
	Curcmd[5] = whichDrive;
	Curcmdlen = 6;
	screen_printf("\n");
	set_cmd_log("%d; mount next %s drive -- %d", cmdCode,driveName[bladeOrMini],whichDrive);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
CMD_MOUNT_NEXT_USB(int cmdCode)
{
    short WhichUsb=0;
    char resp[32];
    short det;
    short v;

    screen_dialog(resp, 31,
		  "Which USB drive  (0 for next, or 1-31, -1 to cancel) [%d] ",
		  WhichUsb);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 31) {
	    WhichUsb = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-31, not %d.\n", v);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = WhichUsb;
    Curcmdlen = 4;
    set_cmd_log("%d; Mount next USB drive %d.", cmdCode, WhichUsb);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CMD_EVENT_DISKTYPE(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    screen_printf("You may be about to do something very bad\n");
    screen_printf("Only change which disks events are written to if you are really certain of what is going on\n");
    screen_printf("0: Disable a disk    1: Enable a disk\n");
    screen_printf("2: Set Bitmask\n");
    screen_dialog(resp, 31,
	"Which function (0, 1, or 2)  (-1,to cancel), [%d]",
	diskChoice);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 2) {
	    diskChoice= v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-2, not %d.\n", v);
	    return;
	}
    }

    diskBitMask=0;
    if(diskChoice==0 || diskChoice==1) {
	screen_printf("0: Helium1            1: Helium2\n");
	screen_printf("3: Ntu\n");
	screen_dialog(resp,31,"Which disk (-1, to cancel)\n");

	if(resp[0] != '\0') {
	    v = atoi(resp);
	    if (0 <= v && v <= 3) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-3, not %d.\n", v);
		return;
	    }
	} else return;
    }
    else {
	screen_printf("0: Helium1            1: Helium2\n");
	screen_printf("3: Ntu\n");
	screen_dialog(resp,31,"Which disk to add to mask (-1, to cancel) [mask: %#x]",diskBitMask);
	if(resp[0] != '\0') {
	    v = atoi(resp);
	    if (0 <= v && v <= 3) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-3, not %d.\n", v);
		return;
	    }
	} else return;

	while(1) {
	    screen_printf("0: Helium1            1: Helium2\n");
	    screen_printf("3: Ntu\n");
	    screen_dialog(resp,31,"Which disk to add? (5 to send, -1 to cancel) [mask: %#x]",diskBitMask);
	    if(resp[0] != '\0') {
		v = atoi(resp);
		if (0 <= v && v <= 3) {
		    diskBitMask |= diskBitMasks[v];
		} else if (v == -1) {
		    screen_printf("Cancelled.\n");
		    return;
		} else if (v == 5) {
		    break;
		}
		else {
		    screen_printf("Value must be 0-5, not %d.\n", v);
		    return;
		}
	    } else return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = diskChoice;
    Curcmd[4] = 2;
    Curcmd[5] = (diskBitMask&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = ((diskBitMask&0xff00)>>8);
    Curcmdlen = 8;
    set_cmd_log("%d; Send event disk bit mask command %d %#x.", cmdCode, diskChoice,
		diskBitMask);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
CMD_HK_DISKTYPE(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    screen_printf("You may be about to do something very bad\n");
    screen_printf("Only change which disks housekeeping is written to if you are really certain of what is going on\n");
    screen_printf("0: Disable a disk    1: Enable a disk\n");
    screen_printf("2: Set Bitmask\n");
    screen_dialog(resp, 31,
	"Which function (0, 1, or 2)  (-1,to cancel), [%d]",
	diskChoice);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 2) {
	    diskChoice= v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-2, not %d.\n", v);
	    return;
	}
    }

    diskBitMask=0;
    if(diskChoice==0 || diskChoice==1) {
	screen_printf("0: Helium1       1: Helium2\n");
	screen_printf("3: Ntu\n");
	screen_dialog(resp,31,"Which disk (-1, to cancel)\n");

	if(resp[0] != '\0') {
	    v = atoi(resp);
	    if (0 <= v && v <= 3) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-3, not %d.\n", v);
		return;
	    }
	} else return;
    }
    else {
	screen_printf("0: Helium1       1: Helium2\n");
	screen_printf("3: Ntu\n");
	screen_dialog(resp,31,"Which disk to add to mask (-1, to cancel) [mask: %#x]",diskBitMask);
	if(resp[0] != '\0') {
	    v = atoi(resp);
	    if (0 <= v && v <= 3) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-3, not %d.\n", v);
		return;
	    }
	} else return;

	while(1) {
	    screen_printf("0: Helium1    1: Helium2\n");
	    screen_printf("3: Ntu\n");
	    screen_dialog(resp,31,"Which disk to add? (5 to send, -1 to cancel) [mask: %#x]",diskBitMask);
	    if(resp[0] != '\0') {
		v = atoi(resp);
		if (0 <= v && v <= 3) {
		    diskBitMask |= diskBitMasks[v];
		} else if (v == -1) {
		    screen_printf("Cancelled.\n");
		    return;
		} else if (v == 5) {
		    break;
		}
		else {
		    screen_printf("Value must be 0-5, not %d.\n", v);
		    return;
		}
	    } else return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = diskChoice;
    Curcmd[4] = 2;
    Curcmd[5] = (diskBitMask&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = ((diskBitMask&0xff00)>>8);
    Curcmdlen = 8;
    set_cmd_log("%d; Send hk disk bit mask command %d %#x.", cmdCode, diskChoice,
		diskBitMask);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_DECIMATION(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    unsigned int t=0;
    float fv;
    static int whichPri=1;
    static float decimateFrac=0;

    screen_dialog(resp, 31,
		  "Which priority (0-9) to decimate (-1 to cancel) [%d] ",
		  whichPri);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    whichPri = v;
	} else {
	    screen_printf("Value must be 0-9, not %d.\n", v);
	    return;
	}
    }



    screen_dialog(resp, 31,
		  "Enter decimation fraction 0.000 to 1.000 (-1 to cancel) [%f]",
		  decimateFrac);
    if (resp[0] != '\0') {
	fv = atof(resp);
	if (0 <= fv && fv <= 1) {
	    decimateFrac = fv;
	} else {
	    screen_printf("Value must be 0.000 to 1.000, not %f.\n", fv);
	    return;
	}
    }

    short decWord=(short)(1000.*decimateFrac);

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = whichPri;
    Curcmd[4] = 2;
    Curcmd[5] = (decWord&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = (decWord&0xff00)>>8;
    Curcmdlen = 8;
    set_cmd_log("%d; Set archived priority %d global decimation fraction to %f on.", cmdCode, whichPri,decimateFrac);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
ARCHIVE_STORAGE_TYPE(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    screen_printf("1: Raw Data       2: Ped Subbed Raw Data\n");
    screen_printf("3: Encoded Data   4: Ped Subbed Encoded Data\n");
    screen_dialog(resp, 31,
	"Set Storage type to (-1 to cancel) [%d] ",
	storageType);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (1 <= v && v <= 4) {
	    storageType = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 1-4, not %d.\n", v);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = storageType;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Storage Type to %d.", cmdCode, storageType);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
SET_SPECIAL_PRI(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    static int pps1Pri=2;
    static int pps2Pri=2;
    static int softPri=2;

    screen_dialog(resp, 31,
		  "Priority of PPS1  (0 to 9, -1 for Prioritizerd [%d] or -10 to cancel) ",
		  pps1Pri);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (-1 <= v && v <= 9) {
	    pps1Pri = v;
	}
	else if(v==-10){
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be -1-9, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp, 31,
		  "Priority of PPS2  (0 to 9, -1 for Prioritizerd [%d] ",
		  pps2Pri);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (-1 <= v && v <= 9) {
	    pps2Pri = v;
	} else {
	    screen_printf("Value must be -1-9, not %d.\n", v);
	    return;
	}
    }


    screen_dialog(resp, 31,
		  "Priority of Soft Trig.  (0 to 9, -1 for Prioritizerd [%d] ",
		  softPri);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (-1 <= v && v <= 9) {
	    softPri = v;
	} else {
	    screen_printf("Value must be -1-9, not %d.\n", v);
	    return;
	}
    }


    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = pps1Pri;
    Curcmd[4] = 2;
    Curcmd[5] = pps2Pri;
    Curcmd[6] = 3;
    Curcmd[7] = softPri;
    Curcmdlen = 8;
    set_cmd_log("%d; Set PPS Priorities to PPS1 %d and PPS2 %d.", cmdCode, pps1Pri,pps2Pri);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_SPECIAL_DECIMATE(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    float fv;
    static int whichPPS=1;
    static int telemOrDisk=1;
    static float decimateFrac=0;

    screen_dialog(resp, 31,
		  "Enter 1 for PPS1, 2 for PPS2, 3 for Soft (-1 to cancel) [%d] ",
		  whichPPS);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (1 <= v && v <= 3) {
	    whichPPS = v;
	} else {
	    screen_printf("Value must be 1 or 2, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp, 31,
		  "Enter 1 for disk decimation, 2 for telemetry decimation (-1 to cancel) [%d] ",
		  telemOrDisk);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (1 <= v && v <= 2) {
	    telemOrDisk = v;
	} else {
	    screen_printf("Value must be 1 or 2, not %d.\n", v);
	    return;
	}
    }



    screen_dialog(resp, 31,
		  "Enter decimation fraction 0.000 to 1.000 (-1 to cancel) [%f]",
		  decimateFrac);
    if (resp[0] != '\0') {
	fv = atof(resp);
	if (0 <= fv && fv <= 1) {
	    decimateFrac = fv;
	} else {
	    screen_printf("Value must be 0.000 to 1.000, not %f.\n", fv);
	    return;
	}
    }

    short decWord=(short)(1000.*decimateFrac);

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = telemOrDisk;
    Curcmd[4] = 2;
    Curcmd[5] = whichPPS;
    Curcmd[6] = 3;
    Curcmd[7] = (decWord&0xff);
    Curcmd[8] = 4;
    Curcmd[9] = (decWord&0xff00)>>8;
    Curcmdlen = 10;
    set_cmd_log("%d; Set Special Trigger %d decimation fraction to %f.", cmdCode, whichPPS,decimateFrac);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
ARCHIVE_DECIMATE_PRI(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    unsigned int t=0;
    float fv;
    static int whichPri=1;
    static float decimateFrac=0;

    screen_dialog(resp, 31,
		  "Which priority (0-9) to decimate (-1 to cancel) [%d] ",
		  whichPri);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    whichPri = v;
	} else {
	    screen_printf("Value must be 0-9, not %d.\n", v);
	    return;
	}
    }

    static unsigned int  diskMask=0;
    screen_dialog(resp,31,
		  "Which diskmask [%#x] ? (-1 to cancel)",diskMask);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	}

	t = strtoul(resp,NULL,16);
	diskMask=t&0xffff;
    }


    screen_dialog(resp, 31,
		  "Enter decimation fraction 0.000 to 1.000 (-1 to cancel) [%f]",
		  decimateFrac);
    if (resp[0] != '\0') {
	fv = atof(resp);
	if (0 <= fv && fv <= 1) {
	    decimateFrac = fv;
	} else {
	    screen_printf("Value must be 0.000 to 1.000, not %f.\n", fv);
	    return;
	}
    }

    short decWord=(short)(1000.*decimateFrac);

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = diskMask;
    Curcmd[4] = 2;
    Curcmd[5] = whichPri;
    Curcmd[6] = 3;
    Curcmd[7] = (decWord&0xff);
    Curcmd[8] = 4;
    Curcmd[9] = (decWord&0xff00)>>8;
    Curcmdlen = 10;
    set_cmd_log("%d; Set archived priority %d decimation fraction to %f on disk mask %#x.", cmdCode, whichPri,decimateFrac,diskMask);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
TELEM_TYPE(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    screen_printf("1: Raw Data       2: Ped Subbed Raw Data\n");
    screen_printf("3: Encoded Data   4: Ped Subbed Encoded Data\n");
    screen_dialog(resp, 31,
	"Set Telemetry type to (-1 to cancel) [%d] ",
	telemType);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (1 <= v && v <= 4) {
	    telemType = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 1-4, not %d.\n", v);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = telemType;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Telemetry Type to %d.", cmdCode, telemType);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
ARCHIVE_PRI_DISK(int cmdCode)
{
    char resp[32];
    short pri;
    short det;
    short v;
    int t;
    screen_dialog(resp,31,
		  "Which priority to change event disk bit mask for? (-1 to cancel)");
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    pri = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Priority must be 0-9, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp,31,
		  "To which bit mask [%#x] ? (-1 to cancel)",eventBitMask);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	}

	t = strtoul(resp,NULL,16);
	eventBitMask=t&0xffff;
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = pri;
    Curcmd[4] = 2;
    Curcmd[5] = eventBitMask&0xff;
    Curcmd[6] = 3;
    Curcmd[7] = (eventBitMask&0xff00)>>8;
    Curcmdlen = 8;
    set_cmd_log("%d; Set priDiskBitMask[%d] to %#x.", cmdCode, pri,eventBitMask);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
ARCHIVE_ALTERNATE_USB(int cmdCode)
{
    char resp[32];
    short pri;
    short det;
    short v;
    int t;
    screen_dialog(resp,31,
		  "Which priority(0-9) to change USB alternating for? (-1 to cancel)");
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    pri = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Priority must be 0-9, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp,31,
		  "Enable (1) or disable (0) alternating? (-1 to cancel)",altUsb);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if(v>=0 && v<=1) {
	    altUsb=v;
	}
	else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	}
	else {
	    screen_printf("Value must be 0 or 1 not %d.\n",v);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = pri;
    Curcmd[4] = 2;
    Curcmd[5] = altUsb;
    Curcmdlen = 6;
    set_cmd_log("%d; Set alternateUsb[%d] to %d.", cmdCode, pri,altUsb);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
RJNHIDE_SIPD_SEND_WAVE(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Send Wave Packets(0 is disable, 1 is enable, -1 to cancel) [%d] ",
	sendWave);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 1) {
	    sendWave = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = sendWave;
    Curcmdlen = 4;
    set_cmd_log("%d; SendWavePackets sets to %d.", cmdCode, sendWave);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
ARCHIVE_PRI_ENC_TYPE(int cmdCode)
{
    char resp[32];
    short pri;
    short det;
    short v;
    int t;
    screen_dialog(resp,31,
		  "Which priority(0-9) to change storage encoding type for? (-1 to cancel)");
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    pri = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Priority must be 0-9, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp,31,
		  "To which encoding type [%d] ? (-1 to cancel)",encType);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	}
	else {
	    encType=v;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = pri;
    Curcmd[4] = 2;
    Curcmd[5] = encType&0xff;
    Curcmd[6] = 3;
    Curcmd[7] = (encType&0xff00)>>8;
    Curcmdlen = 8;
    set_cmd_log("%d; Set priTelemEncodingType[%d] to %d.", cmdCode, pri,encType);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
TELEM_PRI_ENC_TYPE(int cmdCode)
{
    char resp[32];
    short pri;
    short det;
    short v;
    int t;
    int encTypeClock=0;
    screen_dialog(resp,31,
		  "Which priority(0-9) to change telemetry encoding type for? (-1 to cancel)");
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    pri = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Priority must be 0-9, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp,31,
		  "To which encoding type [%d] ? (-1 to cancel)",encType);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	}
	else {
	    encType=v;
	}
    }

    screen_dialog(resp,31,
		  "Clock to which encoding type [%d] ? (-1 to cancel)",encTypeClock);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	}
	else {
	    encTypeClock=v;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = pri;
    Curcmd[4] = 2;
    Curcmd[5] = encType&0xff;
    Curcmd[6] = 3;
    Curcmd[7] = (encType&0xff00)>>8;
    Curcmd[8] = 4;
    Curcmd[9] = encTypeClock&0xff;
    Curcmd[10] = 5;
    Curcmd[11] = (encTypeClock&0xff00)>>8;
    Curcmdlen = 12;
    set_cmd_log("%d; Set priDiskencodingType[%d] to %d. (clock %d)", cmdCode, pri,encType,encTypeClock);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
TURN_TUFFS_ON(int cmdCode)
{
    if (screen_confirm("Really turn on Tuffs")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on Tuffs.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_TUFFS_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off Tuffs")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off Tuffs.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_AMPAS_ON(int cmdCode)
{
    if (screen_confirm("Really turn on all Ampas")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on all Ampas", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_AMPAS_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off all Ampas")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off all Ampas.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_NTU_AMPAS_ON(int cmdCode)
{
    if (screen_confirm("Really turn on NTU Ampas")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on NTU Ampas.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_NTU_AMPAS_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off NTU Ampas")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off NTU Ampas.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_SHORT_BOARDS_ON(int cmdCode)
{
    if (screen_confirm("Really turn on Shorts")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on Shorts.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_SHORT_BOARDS_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off Shorts")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off Shorts.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_NTU_SSD_5V_ON(int cmdCode)
{
    if (screen_confirm("Really turn on NTU SSD 5V")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on NTU SSD 5V.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_NTU_SSD_5V_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off NTU SSD 5V")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off NTU SSD 5V.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_NTU_SSD_12V_ON(int cmdCode)
{
    if (screen_confirm("Really turn on NTU SSD 12V")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on NTU SSD 12V.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_NTU_SSD_12V_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off NTU SSD 12V")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off NTU SSD 12V.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_ALL_ON(int cmdCode)
{
    if (screen_confirm("Really turn on Tuffs, BZ Ampas, NTU Ampas, Shorts, NTU SSD 5V and NTU SSD 12V")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on Tuffs, BZ Ampas, NTU Ampas, Shorts, NTU SSD 5V and NTU SSD 12V", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_ALL_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off Tuffs, BZ Ampas, NTU Ampas, Shorts, NTU SSD 5V and NTU SSD 12V")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off Tuffs, BZ Ampas, NTU Ampas, Shorts, NTU SSD 5V and NTU SSD 12V", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
SET_ADU5_PAT_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set ADU5 Position and attitude readout period (in units of 100ms) to  (0-65535, -1 to cancel) [%d] ",
	ADU5PatPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    ADU5PatPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period in seconds must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (ADU5PatPer&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((ADU5PatPer&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set ADU5 position readout period to %d.", cmdCode, ADU5PatPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_ADU5_VTG_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set ADU5 Velocity and Course readout period (in units of seconds) to  (0-65535, -1 to cancel) [%d] ",
	ADU5VtgPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    ADU5VtgPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period in seconds must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (ADU5VtgPer&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((ADU5VtgPer&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set ADU5 velocity and course readout period to %d s.", cmdCode, ADU5VtgPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_G12_POS_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set G12 Position readout period (in units of ms) to  (0-65535, -1 to cancel) [%d] ",
	G12PosPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    G12PosPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period in seconds must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (G12PosPer&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((G12PosPer&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set G12 position readout period to %d ms.", cmdCode, G12PosPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
SET_ADU5_SAT_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set ADU5 Satellite lock readout period (in seconds) to  (0-65535, -1 to cancel) [%d] ",
	ADU5SatPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    ADU5SatPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period in seconds must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (ADU5SatPer&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((ADU5SatPer&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set ADU5 satellite lock readout period to %d.", cmdCode, ADU5SatPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_G12_PPS_OFFSET(int cmdCode)
{
    char resp[32];
    short det;
    unsigned short ms;
    unsigned short subms;
    unsigned short sign;
    float t,temp;

    screen_dialog(resp, 31,
	"Set G12 PPS period (in ms) to  (-999.9999, +999.9999) [%f] ",
	G12Offset);
    if (resp[0] != '\0') {
	t = atof(resp);
	if (-999.9999 <= t && t <= 999.9999) {
	    G12Offset = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period in ms must -999.9999 to + 999.9999, not %d.\n", t);
	    return;
	}
    }
    temp=G12Offset;
    sign=0;
    if(G12Offset<0) {
      temp=-1*G12Offset;
      sign=1;
    }
    ms=(unsigned short) temp;
    temp-=ms;
    temp*=10000;
    subms=(unsigned short) temp;

    //    temp=ms+((float)subms)/10000.;
    //    if(sign==1) temp*=-1;
    //    //    printf("ms %d, subms %d, sign %d -- %f\n",ms,subms,sign,temp);

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (ms&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((ms&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = (subms&0xff);
    Curcmd[8] = 4;
    Curcmd[9] = ((subms&0xff00)>>8);
    Curcmd[10] = 5;
    Curcmd[11] = (sign&0xff);
    Curcmdlen = 12;
    set_cmd_log("%d; Set G12 PPS offset to %3.4f ms.", cmdCode, G12Offset);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
SET_G12_PPS_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set G12 PPS period (in ms) to  (0-65535, -1 to cancel) [%d] ",
	G12PPSPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    G12PPSPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period in ms must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (G12PPSPer&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((G12PPSPer&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set G12 PPS period to %d ms.", cmdCode, G12PPSPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
ADU5_CAL_12(int cmdCode)
{
    int t;
    char resp[32];
    float v12[3]={0,3.088,0.035};
    short det[3];
    int whichAdu5=0;
    screen_printf("Are you sure you want to do this?.\n");
    screen_dialog(resp, 31,
		  "Which ADU5 (1 for A, 2 for B):  ",whichAdu5);
    if (resp[0] != '\0') {
	t=atoi(resp);
	if(t>=1 && t<=2)
	    whichAdu5=t;
	else
	    return;
    } else return;
    screen_dialog(resp,31,"Enter calibV12[0]: [%3.3f]  ",v12[0]);
    if (resp[0] != '\0') {
	v12[0]=atof(resp);
    }
    screen_dialog(resp,31,"Enter calibV12[1]: [%3.3f]  ",v12[1]);
    if (resp[0] != '\0') {
	v12[1]=atof(resp);
    }
    screen_dialog(resp,31,"Enter calibV12[2]: [%3.3f]  ",v12[2]);
    if (resp[0] != '\0') {
	v12[2]=atof(resp);
    }
    for(t=0;t<3;t++) {
      det[t]=((short)((v12[t]*1000.)+0.5));
    }
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = whichAdu5;
    Curcmd[4] = 2;
    Curcmd[5] = (det[0]&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = ((det[0]&0xff00)>>8);
    Curcmd[8] = 4;
    Curcmd[9] = (det[1]&0xff);
    Curcmd[10] = 5;
    Curcmd[11] = ((det[1]&0xff00)>>8);
    Curcmd[12] = 6;
    Curcmd[13] = (det[2]&0xff);
    Curcmd[14] = 7;
    Curcmd[15] = ((det[2]&0xff00)>>8);
    Curcmdlen = 16;
    set_cmd_log("%d; Set Adu5 %d Cal V12 to {%3.3f,%3.3f,%3.3f} {%d %d %d}.", cmdCode, whichAdu5,v12[0],v12[1],v12[2],det[0],det[1],det[2]);
    sendcmd(Fd, Curcmd, Curcmdlen);

  return;
}

static void
ADU5_CAL_13(int cmdCode)
{
    int t;
    char resp[32];
    float v13[3]={-1.538,1.558,-0.001};
    //    float v13[3]={0,3.088,0.035};
    short det[3];
    int whichAdu5=0;
    screen_printf("Are you sure you want to do this?.\n");
    screen_dialog(resp, 31,
		  "Which ADU5 (1 for A, 2 for B):  ",whichAdu5);
    if (resp[0] != '\0') {
	t=atoi(resp);
	if(t>=1 && t<=2)
	    whichAdu5=t;
	else
	    return;
    } else return;
    screen_dialog(resp,31,"Enter calibV13[0]: [%3.3f]  ",v13[0]);
    if (resp[0] != '\0') {
	v13[0]=atof(resp);
    }
    screen_dialog(resp,31,"Enter calibV13[1]: [%3.3f]  ",v13[1]);
    if (resp[0] != '\0') {
	v13[1]=atof(resp);
    }
    screen_dialog(resp,31,"Enter calibV13[2]: [%3.3f]  ",v13[2]);
    if (resp[0] != '\0') {
	v13[2]=atof(resp);
    }
    for(t=0;t<3;t++) {
      det[t]=((short)((v13[t]*1000.)+0.5));
    }
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = whichAdu5;
    Curcmd[4] = 2;
    Curcmd[5] = (det[0]&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = ((det[0]&0xff00)>>8);
    Curcmd[8] = 4;
    Curcmd[9] = (det[1]&0xff);
    Curcmd[10] = 5;
    Curcmd[11] = ((det[1]&0xff00)>>8);
    Curcmd[12] = 6;
    Curcmd[13] = (det[2]&0xff);
    Curcmd[14] = 7;
    Curcmd[15] = ((det[2]&0xff00)>>8);
    Curcmdlen = 16;
    set_cmd_log("%d; Set Adu5 %d Cal V13 to {%3.3f,%3.3f,%3.3f} {%d %d %d}.", cmdCode, whichAdu5,v13[0],v13[1],v13[2],det[0],det[1],det[2]);

    sendcmd(Fd, Curcmd, Curcmdlen);

  return;


}

static void
ADU5_CAL_14(int cmdCode)
{
    float v14[3]={1.543,1.553,-0.024};
    int t;
    char resp[32];
    short det[3];
    int whichAdu5=0;
    screen_printf("Are you sure you want to do this?.\n");
    screen_dialog(resp, 31,
		  "Which ADU5 (1 for A, 2 for B):  ",whichAdu5);
    if (resp[0] != '\0') {
	t=atoi(resp);
	if(t>=1 && t<=2)
	    whichAdu5=t;
	else
	    return;
    } else return;
    screen_dialog(resp,31,"Enter calibV14[0]: [%3.3f]  ",v14[0]);
    if (resp[0] != '\0') {
	v14[0]=atof(resp);
    }
    screen_dialog(resp,31,"Enter calibV14[1]: [%3.3f]  ",v14[1]);
    if (resp[0] != '\0') {
	v14[1]=atof(resp);
    }
    screen_dialog(resp,31,"Enter calibV14[2]: [%3.3f]  ",v14[2]);
    if (resp[0] != '\0') {
	v14[2]=atof(resp);
    }
    for(t=0;t<3;t++) {
      det[t]=((short)((v14[t]*1000.)+0.5));
    }
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = whichAdu5;
    Curcmd[4] = 2;
    Curcmd[5] = (det[0]&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = ((det[0]&0xff00)>>8);
    Curcmd[8] = 4;
    Curcmd[9] = (det[1]&0xff);
    Curcmd[10] = 5;
    Curcmd[11] = ((det[1]&0xff00)>>8);
    Curcmd[12] = 6;
    Curcmd[13] = (det[2]&0xff);
    Curcmd[14] = 7;
    Curcmd[15] = ((det[2]&0xff00)>>8);
    Curcmdlen = 16;
    set_cmd_log("%d; Set Adu5 %d Cal V14 to {%3.3f,%3.3f,%3.3f} {%d %d %d}.", cmdCode, whichAdu5,v14[0],v14[1],v14[2],det[0],det[1],det[2]);
    sendcmd(Fd, Curcmd, Curcmdlen);

  return;

}

static void
SET_HK_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set Housekeeping readout period (in 100ms units) to  (0-65535, -1 to cancel) [%d] ",
	HskPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    HskPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (HskPer&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((HskPer&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set Housekeeping readout period to %d (100ms units).", cmdCode, HskPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
SET_HK_CAL_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set Housekeeping calibration readout period (in seconds) to  (0-65535, -1 to cancel) [%d] ",
	HskCalPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    HskCalPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (HskCalPer&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((HskCalPer&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set Housekeeping calibration readout period to %d.", cmdCode, HskCalPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_HK_TELEM_EVERY(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set Housekeeping telem every (0-255)in HK events (-1 to cancel) [%d] ",
	HskTelemEvery);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    HskTelemEvery = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-255, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (HskTelemEvery&0xff);
    Curcmdlen = 4;
    set_cmd_log("%d; Set Housekeeping telem to every %d.", cmdCode, HskTelemEvery);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
RJNHIDE_SIPD_THROTTLE_RATE(int cmdCode)
{
    char resp[32];
    short det;
    int t;

    screen_dialog(resp, 31,
	"Set data rate to CSBF in approx Bytes/s to  (0-680, -1 to cancel) [%d] ",
	HskPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 680) {
	    sipThrottle = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Throttle rate must be 0-680, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (sipThrottle&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((sipThrottle&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Sip throttle rate set to %d.", cmdCode, sipThrottle);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
RJNHIDE_SIPD_PRIORITY_BANDWIDTH(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    float fv;
    static int whichPri=1;
    static short bandFrac=0;

    screen_dialog(resp, 31,
		  "Which priority (0-9) to set bandwidth for (-1 to cancel) [%d] ",
		  whichPri);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    whichPri = v;
	} else {
	    screen_printf("Value must be 0-9, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp, 31,
		  "Enter bandwidth fraction 0 to 100 (-1 to cancel) [%f]",
		  bandFrac);
    if (resp[0] != '\0') {
	fv = atof(resp);
	if (0 <= fv && fv <= 100) {
	    bandFrac = fv;
	} else {
	    screen_printf("Value must be 0 to 100, not %f.\n", fv);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = whichPri;
    Curcmd[4] = 2;
    Curcmd[5] = (bandFrac&0xff);
    Curcmdlen = 6;
    set_cmd_log("%d; Set sip priority %d bandwidth fraction to %f.", cmdCode, whichPri,bandFrac);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
RJN_HIDE_LOSD_PRIORITY_BANDWIDTH(int cmdCode)
{
    char resp[32];
    short det;
    short v;
    float fv;
    static int whichPri=1;
    static short bandFrac=0;

    screen_dialog(resp, 31,
		  "Which priority (0-9) to set bandwidth for (-1 to cancel) [%d] ",
		  whichPri);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 9) {
	    whichPri = v;
	} else {
	    screen_printf("Value must be 0-9, not %d.\n", v);
	    return;
	}
    }

    screen_dialog(resp, 31,
		  "Enter bandwidth fraction 0 to 100 (-1 to cancel) [%f]",
		  bandFrac);
    if (resp[0] != '\0') {
	fv = atof(resp);
	if (0 <= fv && fv <= 100) {
	    bandFrac = fv;
	} else {
	    screen_printf("Value must be 0 to 100, not %f.\n", fv);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = whichPri;
    Curcmd[4] = 2;
    Curcmd[5] = (bandFrac&0xff);
    Curcmdlen = 6;
    set_cmd_log("%d; Set los priority %d bandwidth fraction to %f.", cmdCode, whichPri,bandFrac);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
RJN_HIDE_LOSD_SEND_DATA(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Send LOS Data(0 is disable, 1 is enable, -1 to cancel) [%d] ",
	losSendData);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 1) {
	    losSendData = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = losSendData;
    Curcmdlen = 4;
    set_cmd_log("%d;LOS Send Data set to %d.", cmdCode, losSendData);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
CLEAN_DIRS(int cmdCode)
{
     screen_printf("Not an available command.\n");
     return;
 /* short det;
    int i;
    char resp[32];

    screen_printf("0. Dir 0    3. Dir 3\n");
    screen_printf("1. Dir 1    4. Dir 4\n");
    screen_printf("2. Dir 2    5. Dir 5\n");
    screen_printf("6. All of the above directories\n");
    screen_dialog(resp, 31, "Clean which directory? (-1 to cancel) [%d] ",
	Dir_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (0 <= det && det <= 6) {
	    Dir_det = det;
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 0-6, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = Dir_det;
    Curcmdlen = 4;
    set_cmd_log("%d; Directory  %d cleaned.", cmdCode, Dir_det);
    sendcmd(Fd, Curcmd, Curcmdlen);*/
}


static void
CLEAR_RAMDISK(int cmdCode)
{
    if (screen_confirm("Really clear ramdisk")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Clear RAMdisk.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
SEND_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];


    screen_printf("1.  Acqd.config       6.  GPSd.config\n");
    screen_printf("2.  Archived.config   7.  Hkd.config\n");
    screen_printf("3.  Calibd.config   8.  LOSd.config\n");
    screen_printf("4.  Cmdd.config       9.  Monitord.config\n");
    screen_printf("5.  Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config       12. Playbackd.config\n");
    screen_printf("13. LogWatchd.config  14. Ntud.config\n");
    screen_printf("15. Openportd.config  16. RTLd.config\n");
    screen_printf("17. Tuffd.config \n");
    screen_printf("19. All of the above\n");
    screen_dialog(resp, 31, "Which config File? (-1 to cancel) [%u] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Config_det = ACQD_ID_MASK;
	    break;
          case 2:
            Config_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Config_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Config_det = CMDD_ID_MASK;
	    break;
          case 5:
            Config_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Config_det = GPSD_ID_MASK;
	    break;
          case 7:
            Config_det = HKD_ID_MASK;
	    break;
          case 8:
            Config_det = LOSD_ID_MASK;
	    break;
          case 9:
            Config_det = MONITORD_ID_MASK;
            break;
          case 10:
            Config_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Config_det = SIPD_ID_MASK;
	    break;
          case 12:
            Config_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Config_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Config_det = NTUD_ID_MASK;
	    break;
          case 15:
            Config_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Config_det = RTLD_ID_MASK;
	    break;
          case 17:
            Config_det = TUFFD_ID_MASK;
	    break;
          case 19:
            Config_det = ALL_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Config_det&0xff0000)>>16);
    Curcmdlen = 8;
    set_cmd_log("%d; Config  %u sent.", cmdCode, Config_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
DEFAULT_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];


    screen_printf("1.  Acqd.config       6.  GPSd.config\n");
    screen_printf("2.  Archived.config   7.  Hkd.config\n");
    screen_printf("3.  Calibd.config   8.  LOSd.config\n");
    screen_printf("4.  Cmdd.config       9.  Monitord.config\n");
    screen_printf("5.  Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config       12. Playbackd.config\n");
    screen_printf("13. LogWatchd.config  14. Ntud.config\n");
    screen_printf("15. Openportd.config  16. RTLd.config\n");
    screen_printf("17. Tuffd.config \n");
    screen_printf("19. All of the above\n");
    screen_dialog(resp, 31, "Which config file to return to default? (-1 to cancel) [%d] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Config_det = ACQD_ID_MASK;
	    break;
          case 2:
            Config_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Config_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Config_det = CMDD_ID_MASK;
	    break;
          case 5:
            Config_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Config_det = GPSD_ID_MASK;
	    break;
          case 7:
            Config_det = HKD_ID_MASK;
	    break;
          case 8:
            Config_det = LOSD_ID_MASK;
	    break;
          case 9:
            Config_det = MONITORD_ID_MASK;
            break;
          case 10:
            Config_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Config_det = SIPD_ID_MASK;
	    break;
          case 12:
            Config_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Config_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Config_det = NTUD_ID_MASK;
	    break;
          case 15:
            Config_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Config_det = RTLD_ID_MASK;
	    break;
          case 17:
            Config_det = TUFFD_ID_MASK;
	    break;
          case 19:
            Config_det = ALL_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }


    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Config_det&0xff0000)>>16);
    Curcmdlen = 8;
    set_cmd_log("%d; Config  %d set to default.", cmdCode, Config_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
LAST_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];
    screen_printf("1.  Acqd.config       6.  GPSd.config\n");
    screen_printf("2.  Archived.config   7.  Hkd.config\n");
    screen_printf("3.  Calibd.config     8.  LOSd.config\n");
    screen_printf("4.  Cmdd.config       9.  Monitord.config\n");
    screen_printf("5.  Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config       12. Playbackd.config\n");
    screen_printf("13. LogWatchd.config  14. Ntud.config\n");
    screen_printf("15. Openportd.config  16. RTLd.config\n");
    screen_printf("17. Tuffd.config \n");
    screen_printf("19. All of the above\n");
    screen_dialog(resp, 31, "Which config file to return to last config? (-1 to cancel) [%d] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Config_det = ACQD_ID_MASK;
	    break;
          case 2:
            Config_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Config_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Config_det = CMDD_ID_MASK;
	    break;
          case 5:
            Config_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Config_det = GPSD_ID_MASK;
	    break;
          case 7:
            Config_det = HKD_ID_MASK;
	    break;
          case 8:
            Config_det = LOSD_ID_MASK;
	    break;
          case 9:
            Config_det = MONITORD_ID_MASK;
            break;
          case 10:
            Config_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Config_det = SIPD_ID_MASK;
	    break;
          case 12:
            Config_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Config_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Config_det = NTUD_ID_MASK;
	    break;
          case 15:
            Config_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Config_det = RTLD_ID_MASK;
	    break;
          case 17:
            Config_det = TUFFD_ID_MASK;
	    break;

          case 19:
            Config_det = ALL_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Config_det&0xff0000)>>16);
    Curcmdlen = 8;
    set_cmd_log("%d; Config  %d set to last.", cmdCode, Config_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SWITCH_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];


    screen_printf("1.  Acqd.config       6.  GPSd.config\n");
    screen_printf("2.  Archived.config   7.  Hkd.config\n");
    screen_printf("3.  Calibd.config     8.  LOSd.config\n");
    screen_printf("4.  Cmdd.config       9.  Monitord.config\n");
    screen_printf("5.  Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config       12. Playbackd.config\n");
    screen_printf("13. LogWatchd.config  14. Ntud.config\n");
    screen_printf("15. Openportd.config  16. RTLd.config\n");
    screen_printf("17. Tuffd.config \n");
    screen_printf("19. All of the above\n");
    screen_dialog(resp, 31, "Which config file to switch? (-1 to cancel) [%d] ",
	Config_det);
      if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Config_det = ACQD_ID_MASK;
	    break;
          case 2:
            Config_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Config_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Config_det = CMDD_ID_MASK;
	    break;
          case 5:
            Config_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Config_det = GPSD_ID_MASK;
	    break;
          case 7:
            Config_det = HKD_ID_MASK;
	    break;
          case 8:
            Config_det = LOSD_ID_MASK;
	    break;
          case 9:
            Config_det = MONITORD_ID_MASK;
            break;
          case 10:
            Config_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Config_det = SIPD_ID_MASK;
	    break;
          case 12:
            Config_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Config_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Config_det = NTUD_ID_MASK;
	    break;
          case 15:
            Config_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Config_det = RTLD_ID_MASK;
	    break;
          case 17:
            Config_det = TUFFD_ID_MASK;
	    break;

          case 19:
            Config_det = ALL_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }


    screen_dialog(resp, 31, "Which config number? (0-255, -1 to cancel) [%d] ",switchConfig);
    if (resp[0] != '\0') {
      det = atoi(resp);
      if (0 <= det && det <= 255) {
	switchConfig=det;
      }
      else if (det == -1) {
	screen_printf("Cancelled\n");
	    return;
      } else {
	screen_printf("Value must be 0-255, not %d.\n", det);
	return;
      }
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Config_det&0xff0000)>>16);
    Curcmd[8] = 4;
    Curcmd[9] = (switchConfig&0xff);
    Curcmdlen = 10;
    set_cmd_log("%d; Config  %d set to %d.", cmdCode, Config_det,switchConfig);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
SAVE_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];
    static int configNum;
//    static int whichConfig;


    screen_printf("1.  Acqd.config       6.  GPSd.config\n");
    screen_printf("2.  Archived.config   7.  Hkd.config\n");
    screen_printf("3.  Calibd.config     8.  LOSd.config\n");
    screen_printf("4.  Cmdd.config       9.  Monitord.config\n");
    screen_printf("5.  Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config       12. Playbackd.config\n");
    screen_printf("13. LogWatchd.config  14. Ntud.config\n");
    screen_printf("15. Openportd.config  16. RTLd.config\n");
    screen_printf("17. Tuffd.config \n");
    screen_printf("19. All of the above\n");
    screen_dialog(resp, 31, "Which config file to switch? (-1 to cancel) [%d] ",
	Config_det);
      if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 19) {
	  switch(det){
	  case 1:
            Config_det = ACQD_ID_MASK;
	    break;
          case 2:
            Config_det = ARCHIVED_ID_MASK;
	    break;
          case 3:
            Config_det = CALIBD_ID_MASK;
	    break;
          case 4:
	    Config_det = CMDD_ID_MASK;
	    break;
          case 5:
            Config_det = EVENTD_ID_MASK;
	    break;
          case 6:
            Config_det = GPSD_ID_MASK;
	    break;
          case 7:
            Config_det = HKD_ID_MASK;
	    break;
          case 8:
            Config_det = LOSD_ID_MASK;
	    break;
          case 9:
            Config_det = MONITORD_ID_MASK;
            break;
          case 10:
            Config_det = PRIORITIZERD_ID_MASK;
            break;
          case 11:
            Config_det = SIPD_ID_MASK;
	    break;
          case 12:
            Config_det = PLAYBACKD_ID_MASK;
	    break;
          case 13:
            Config_det = LOGWATCHD_ID_MASK;
	    break;
          case 14:
            Config_det = NTUD_ID_MASK;
	    break;
          case 15:
            Config_det = OPENPORTD_ID_MASK;
	    break;
          case 16:
            Config_det = RTLD_ID_MASK;
	    break;
          case 17:
            Config_det = TUFFD_ID_MASK;
	    break;
          case 19:
            Config_det = ALL_ID_MASK;
	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-19, not %d.\n", det);
	    return;
	}
    }

    screen_dialog(resp, 31, "Which config number? (10-255, -1 to cancel) [%d] ",configNum);
    if (resp[0] != '\0') {
      det = atoi(resp);
      if (10 <= det && det <= 255) {
	configNum=det;
      }
      else if (det == -1) {
	screen_printf("Cancelled\n");
	    return;
      } else {
	screen_printf("Value must be 10-255, not %d.\n", det);
	return;
      }
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = ((Config_det&0xff0000)>>16);
    Curcmd[8] = 4;
    Curcmd[9] = (configNum&0xff);
    Curcmdlen = 10;
    set_cmd_log("%d; Current config %d saved as %d.", cmdCode, Config_det,configNum);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
ACQD_ADU5_TRIG_FLAG(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set SURF Trigger on ADU5 Flag, 0 disable, 1 enable, -1 to cancel) [%d] ", TrigADU5);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 == t || t == 1) {
	    TrigADU5 = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period must be 0 or 1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = TrigADU5;
    Curcmdlen = 4;
    set_cmd_log("%d; Set SURF ADU5 Trigger Flag to %d.", cmdCode,TrigADU5);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
ACQD_G12_TRIG_FLAG(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set SURF Trigger on G12 Flag, 0 disable, 1 enable, -1 to cancel) [%d] ", TrigG12);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 == t || t == 1) {
	    TrigG12 = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period must be 0 or 1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = TrigG12;
    Curcmdlen = 4;
    set_cmd_log("%d; Set SURF G12 Trigger Flag to %d.", cmdCode,TrigG12);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
ACQD_SOFT_TRIG_FLAG(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set Trigger on Software Flag, 0 disable, 1 enable, -1 to cancel) [%d] ", TrigSoft);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 == t || t == 1) {
	    TrigSoft = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period must be 0 or 1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = TrigSoft;
    Curcmdlen = 4;
    set_cmd_log("%d; Set SURF Software Trigger Flag to %d.", cmdCode,TrigSoft);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
ACQD_SOFT_TRIG_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set software trigger period (in seconds) to  (0-255, -1 to cancel) [%d] ",
	SoftTrigPer);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    SoftTrigPer = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Period must be 0-255, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = SoftTrigPer;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Software trigger period to %d.", cmdCode, SoftTrigPer);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
ACQD_ENABLE_CHAN_SERVO(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Enable Chan Servo  (0 is disable, 1 is enable, -1 to cancel) [%d] ",
	enableChanServo);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 1) {
	    enableChanServo = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = enableChanServo;
    Curcmdlen = 4;
    set_cmd_log("%d; Enable Chan Servo Set to %d.", cmdCode, enableChanServo);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_PID_GOAL(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set Servo Scaler Goal  (0-16000, -1 to cancel) [%d] ",
	pidGoal);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 16000) {
	    pidGoal = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-16000, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (pidGoal&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((pidGoal&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting pidGoal to %d.", cmdCode, pidGoal);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
ACQD_PEDESTAL_RUN(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Take Pedestal Run  (0 is disable, 1 is enable, -1 to cancel) [%d] ",
	pedestalRun);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 1) {
	    pedestalRun = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = pedestalRun;
    Curcmdlen = 4;
    set_cmd_log("%d; Take Pedestal Run %d.", cmdCode, pedestalRun);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
THRESHOLD_SCAN(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Take Threshold Scan  (0 is disable, 1 is enable, -1 to cancel) [%d] ",
	thresholdRun);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 1) {
	    thresholdRun = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = thresholdRun;
    Curcmdlen = 4;
    set_cmd_log("%d; Take Threshold Scan %d.", cmdCode, thresholdRun);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
ACQD_REPROGRAM_TURF(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Reprogram TURF when Acqd restarts  (0 is disable, 1 is enable, -1 to cancel) [%d] ",
	reprogramTurf);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 1) {
	    reprogramTurf = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = reprogramTurf;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Reprogram Turf to %d.", cmdCode, reprogramTurf);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
SURFHK_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set surf housekeeping recording period (in secs) (0-255, -1 to cancel) [%d] ",
	surfhkPeriod);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    surfhkPeriod = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-255, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (surfhkPeriod&0xff);
    Curcmdlen = 4;
    set_cmd_log("%d; Setting surfhkPeriod to %d.", cmdCode, surfhkPeriod);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SURFHK_TELEM_EVERY(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set surfhk telemetry reduction (in surfhk events) (0-255, -1 to cancel) [%d] ",
	surfhkTelemEvery);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    surfhkTelemEvery = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-255, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (surfhkTelemEvery&0xff);
    Curcmdlen = 4;
    set_cmd_log("%d; Setting surfhkTelemEvery to %d.", cmdCode, surfhkTelemEvery);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
TURFHK_TELEM_EVERY(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set turfhk telemetry reduction (in turfhk events) (0-255, -1 to cancel) [%d] ",
	turfhkTelemEvery);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    turfhkTelemEvery = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-255, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (turfhkTelemEvery&0xff);
    Curcmdlen = 4;
    set_cmd_log("%d; Setting turfhkTelemEvery to %d.", cmdCode, turfhkTelemEvery);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
NUM_PED_EVENTS(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set Number of Events in Pedestal Run  (0-10000, -1 to cancel) [%d] ",
	numPedEvents);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 10000) {
	    numPedEvents = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-10000, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (numPedEvents&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((numPedEvents&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting number of pedestal events to %d.", cmdCode, numPedEvents);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
THRESH_SCAN_STEP_SIZE(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set threshold scan step size (0-255, -1 to cancel) [%d] ",
	threshScanStepSize);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    threshScanStepSize = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-255, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (threshScanStepSize&0xff);
    Curcmdlen = 4;
    set_cmd_log("%d; Setting threshold scan step size to %d.", cmdCode, threshScanStepSize);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
THRESH_SCAN_REPEAT(int cmdCode)
{
    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set threshold scan points per step (0-255, -1 to cancel) [%d] ",
	threshScanPointsPerStep);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    threshScanPointsPerStep = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-255, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (threshScanPointsPerStep&0xff);
    Curcmdlen = 4;
    set_cmd_log("%d; Setting threshold scan points per step to %d.", cmdCode, threshScanPointsPerStep);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
RAMDISK_KILL_ACQD(int cmdCode)
{
    char resp[32];
    short det;
    short t;
    static short megaBytes=50;

    screen_dialog(resp, 31,
	"Ramdisk Acqd Kill Threshold in MB (-1 to cancel) [%d] ", megaBytes);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    megaBytes = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set Acqd Kill threshold to %d.\n", t);
	    //	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = megaBytes;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Ramdisk Acqd kill threshold to %d MB.", cmdCode,megaBytes);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
RAMDISK_DUMP_DATA(int cmdCode)
{
    char resp[32];
    short det;
    short t;
    static short megaBytes=50;

    screen_dialog(resp, 31,
	"Ramdisk Dump Data Threshold in MB (-1 to cancel) [%d] ", megaBytes);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    megaBytes = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set Ramdisk Data Dump Threshold to %d MB.\n", t);
	    //	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = megaBytes;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Ramdisk Data Dump Threshold to %d MB.", cmdCode,megaBytes);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
EVENTD_MATCH_GPS(int cmdCode)
{
    char resp[32];
    short det;
    short t=0;
    short val=0;

    screen_dialog(resp, 31,
		  "Enable (1) or disable (0) gps matching (-1 to cancel) [%d] ",
		  val);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t<=1) {
	    val = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0 or 1, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = val;
    Curcmdlen = 4;
    set_cmd_log("%d; Setting event match GPS to %d.", cmdCode, val);
    sendcmd(Fd, Curcmd, Curcmdlen);
}

static void
MONITORD_ACQD_WAIT(cmdCode){
    char resp[32];
    short det;

    short t;

    screen_dialog(resp, 31,
	"Set seconds for max wait time to restart acqd after ramdisk filling  (0-65535, -1 to cancel) [%d] ",
	acqdWait);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t) {
	    acqdWait = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-65535, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (acqdWait&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((acqdWait&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting maxAcqdWaitPeriod to %d seconds.", cmdCode, acqdWait);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
MONITORD_PERIOD(cmdCode){
    char resp[32];
    short det;
    short t;
    static short monPeriod=30;

    screen_dialog(resp, 31,
	"Set the monitor period in seconds (0-256, -1 to cancel) [%d] ", monPeriod);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 256) {
	    monPeriod = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set monitor period to  %d seconds.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = monPeriod;
    Curcmdlen = 4;
    set_cmd_log("%d; Set monitor period to  %d seconds.", cmdCode, monPeriod);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
USB_CHANGE_THRESH(cmdCode){
    char resp[32];
    short det;
    short t;
    static short usbMegaBytes=50;

    screen_dialog(resp, 31,
	"Set the threeshold to switch USB drives in MB (0-255,-1 to cancel) [%d] ", usbMegaBytes);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    usbMegaBytes = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set usb switch threshold to %d MB.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = usbMegaBytes;
    Curcmdlen = 4;
    set_cmd_log("%d; Set usb switch threshold to %d MB.", cmdCode, usbMegaBytes);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
SATA_CHANGE_THRESH(cmdCode){
    char resp[32];
    short det;
    short t;
    short val=0;
    static short satabladeMegaBytes=120;
    char *driveName[2]={"satablade","satamini"};
    screen_dialog(resp, 31,
		  "Blades(0) or Minis (1) (-1 to cancel) [%d] ",
		  val);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t<=1) {
	  val = t;
	} else if (t == -1) {
	  screen_printf("Cancelled.\n");
	  return;
	} else {
	  screen_printf("Value must be 0 or 1, not %d.\n", t);
	  return;
	}
    }

    screen_dialog(resp, 31,
		  "Set %s switch threshold in MB (0-255, -1 to cancel) [%d] ", driveName[val],satabladeMegaBytes);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    satabladeMegaBytes = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set satablade switch threshold to %d MB.\n", t);
	    //	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = val;
    Curcmd[4] = 2;
    Curcmd[5] = satabladeMegaBytes;
    Curcmdlen = 6;
    set_cmd_log("%d; Set %s switch threshold %d MB.", cmdCode,driveName[val],satabladeMegaBytes);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
MAX_QUEUE_LENGTH(cmdCode){

    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set size for max event queue (0-65535, -1 to cancel) [%d] ",
	maxQueue);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t) {
	    maxQueue = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-65535, not %d.\n", t);
	    //	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (maxQueue&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((maxQueue&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting maximum event queue to %d.", cmdCode, maxQueue);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
INODES_KILL_ACQD(cmdCode){

    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set the inodesKillAcqd (0-65535, -1 to cancel) [%d] ",
	inodesKill);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t ) {
	    inodesKill = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-65535, not %d.\n", t);
	    //	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (inodesKill&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((inodesKill&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting inodesKillAcqd %d.", cmdCode, inodesKill);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
INODES_DUMP_DATA(cmdCode){

    char resp[32];
    short det;
    short t;

    screen_dialog(resp, 31,
	"Set inodesDumpData variable(0-65535, -1 to cancel) [%d] ",
	inodesDump);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t ) {
	    inodesDump = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-65535, not %d.\n", t);
	    //	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (inodesDump&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((inodesDump&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting inodesDumpData to %d.", cmdCode, inodesDump);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
ACQD_SET_RATE_SERVO(cmdCode){
    static unsigned short eventRate;
    char resp[32];
    short det;
    short t;
    float ft=0;
    float rate=0;
    short enabler=0;

    screen_dialog(resp, 31,
	"Set Desired Event Rate  (0 to disable servo, -1 to cancel) [%f] ",
	rate);
    if (resp[0] != '\0') {
	 ft = atof(resp);
	if (0 < ft && ft <= 10) {
	    enabler=1;
	    rate = ft;
	    rate*=1000;
	    eventRate=(int)rate;
	} else if (ft < 0) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    enabler=0;
	    rate=0;
	    eventRate=0;
	    //	    return;
	}
    }


    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = enabler;
    Curcmd[4] = 2;
    Curcmd[5] = (eventRate&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = ((eventRate&0xff00)>>8);
    Curcmdlen = 8;
    set_cmd_log("%d; Setting event rate goal to %d %f.", cmdCode, enabler,eventRate);
    sendcmd(Fd, Curcmd, Curcmdlen);

}


static void
ACQD_SET_NICE_VALUE(cmdCode){
    char resp[32];
    short det;
    short t;
    static short acqdNice=-10;

    screen_dialog(resp, 31,
	"Set Acqd nice value(-20 to 19, 100 to cancel) [%d] ", acqdNice);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (-20 <= t && t <=19) {
	    acqdNice = t;
	    acqdNice+=20;
	} else if (t == 100) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set Acqd Nice value to %d .\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = acqdNice;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Acqd nice value to %d.", cmdCode,acqdNice-20);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
PRIORITIZERD_COMMAND(cmdCode){
  screen_printf("Enter Prioritizerd command (1-16)\n");

  char resp[32];
  short det;
  int t;
  float ft;
  unsigned char cmdBytes[8]={0};

  unsigned short extraCode = 0;
  unsigned short priMaxQueueLength = 5000;
  unsigned char priorityBin = 1;
  float lowBinEdge = 0;
  float highBinEdge = 0;
  unsigned short priSlopeImageHilbert = 0;


  screen_printf("1. PRI_PANIC_QUEUE_LENGTH\n");
  screen_printf("2. PRI_PARAMS_LOW_BIN_EDGE\n");
  screen_printf("3. PRI_PARAMS_HIGH_BIN_EDGE\n");
  screen_printf("4. PRI_SLOPE_IMAGE_HILBERT\n");
  screen_printf("5. PRI_INTERCEPT_IMAGE_HILBERT\n");
  /* screen_printf("6. PRI_BIN_TO_BIN_THRESH\n"); */
  /* screen_printf("7. PRI_ABS_MAG_THRESH\n"); */
  screen_printf("8. PRI_THETA_ANGLE_DEMOTION_LOW\n");
  screen_printf("9. PRI_THETA_ANGLE_DEMOTION_HIGH\n");
  screen_printf("10. PRI_THETA_ANGLE_DEMOTION_PRIORITY\n");
  screen_printf("11. PRI_POWER_SPECTRUM_PERIOD\n");
  screen_printf("12. PRI_ANT_PHI_POS\n");
  screen_printf("13. PRI_ANT_R_POS\n");
  screen_printf("14. PRI_ANT_Z_POS\n");
  screen_printf("15. PRI_POS_SATUATION\n");
  screen_printf("16. PRI_NEG_SATUATION\n");
  screen_printf("17. PRI_ASYM_SATURATION\n");
  screen_printf("18. PRI_SLEEP_TIME_KILL_X\n");
  screen_printf("19. PRI_DISABLE_GPU\n");
  screen_printf("20. PRI_CALIB_VERSION\n");
  screen_printf("21. PRI_INVERT_TOP_RING_SOFTWARE\n");
  screen_printf("22. PRI_DEBUG_MODE\n");
  screen_printf("23. PRI_SKIP_BLAST_RATIO_HPOL\n");
  screen_printf("24. PRI_SKIP_BLAST_RATIO_VPOL\n");
  screen_printf("25. PRI_BLAST_RATIO_MAX\n");
  screen_printf("26. PRI_BLAST_RATIO_MIN\n");
  screen_printf("27. PRI_BLAST_GRADIENT\n");
  screen_printf("28. PRI_STATIC_NOTCH_LOW_EDGE\n");
  screen_printf("29. PRI_STATIC_NOTCH_HIGH_EDGE\n");
  screen_printf("30. PRI_USE_LONG_DYNAMIC_FILTERING\n");
  screen_printf("31. PRI_START_DYNAMIC_FREQUENCY\n");
  screen_printf("32. PRI_STOP_DYNAMIC_FREQUENCY\n");
  screen_printf("33. PRI_CONSERVATIVE_START\n");
  screen_printf("34. PRI_THRESH_DB\n");
  screen_printf("35. PRI_DELTA_PHI_TRIG\n");


  screen_dialog(resp,31,"Select extra code %d (-1 to cancel)\n",extraCode);
  if (resp[0] != '\0') {
    t = atoi(resp);
    if (1<= t && t <=34) {
      extraCode = t;
    } else if (t == -1) {
      screen_printf("Cancelled.\n");
      return;
    } else {
      screen_printf("Not a valid command\n");
      return;
    }
  }
  if(extraCode==PRI_PANIC_QUEUE_LENGTH){
    screen_dialog(resp,31,"Enter maximum Prioritizerd queue length before writing all events as priority 7 (0-65535) (-1 to cancel)\n",priMaxQueueLength);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=65535) {
	priMaxQueueLength = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0]=priMaxQueueLength & 0xff;
    cmdBytes[1]=(priMaxQueueLength & 0xff00)>>8;
  }
  if(extraCode==PRI_PARAMS_LOW_BIN_EDGE) {
    screen_dialog(resp, 31,
		  "Select Priority Bin to modify (0-9) (-1 to cancel) [%d] ",
		  priorityBin);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 <= t && t <= 9) {
	priorityBin = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-9, not %d.\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter Low Bin Edge Value (-1 to cancel) [%f] ",
		  lowBinEdge);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0 <= ft && ft <= 6553) {
	lowBinEdge = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-6553, not %f.\n", ft);
	return;
      }
    }
    cmdBytes[0]=priorityBin;
    cmdBytes[1]=0;
    unsigned short convertedVal = (unsigned short)(lowBinEdge*100.0);
    cmdBytes[2]=(convertedVal & 0xff);
    cmdBytes[3]=(convertedVal & 0xff00) >> 8;
  }
  if(extraCode==PRI_PARAMS_HIGH_BIN_EDGE) {
    screen_dialog(resp, 31,
		  "Select priority bin to modify (0-9) (-1 to cancel) [%d] ",
		  priorityBin);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 <= t && t <= 9) {
	priorityBin = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-9, not %d.\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter high bin edge value (-1 to cancel) [%f] ",
		  lowBinEdge);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0 <= ft && ft <= 6553) {
	highBinEdge = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-6553, not %f.\n", ft);
	return;
      }
    }
    cmdBytes[0]=priorityBin;
    cmdBytes[1]=0;
    unsigned short convertedVal = (unsigned short)(highBinEdge*100.0);
    cmdBytes[2]=(convertedVal & 0xff);
    cmdBytes[3]=(convertedVal & 0xff00) >> 8;
  }
  if(extraCode==PRI_SLOPE_IMAGE_HILBERT){
    screen_dialog(resp,31,"Enter Prioritizerd slope for image peak and hilbert peak parameter space (0-65535) (-1 to cancel) [%hu]\n",priSlopeImageHilbert);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=65535) {
	priSlopeImageHilbert = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[2]=priSlopeImageHilbert & 0xff;
    cmdBytes[3]=(priSlopeImageHilbert & 0xff00)>>8;
  }

  if(extraCode==PRI_INTERCEPT_IMAGE_HILBERT){
    unsigned short priInterceptImageHilbert = 0;
    screen_dialog(resp,31,"Enter Prioritizerd intercept for image peak and hilbert peak parameter space (0-65535) (-1 to cancel)[%hu]\n",priInterceptImageHilbert);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=65535) {
	priInterceptImageHilbert = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[2]=priInterceptImageHilbert & 0xff;
    cmdBytes[3]=(priInterceptImageHilbert & 0xff00)>>8;
  }

  if(extraCode==PRI_BIN_TO_BIN_THRESH){
    screen_printf("Option %d is currently disabled for ANITA-IV.\n", extraCode);
    return;

    float priBinToBinThresh = 10;
    screen_dialog(resp,31,"Enter Prioritizerd bin-to-bin threshold for CW cut (-1 to cancel) [%f]\n", priBinToBinThresh);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=6553) {
	priBinToBinThresh = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(priBinToBinThresh*10);
    cmdBytes[2]=convertedVal & 0xff;
    cmdBytes[3]=(convertedVal & 0xff00)>>8;
  }

  if(extraCode==PRI_ABS_MAG_THRESH){
    screen_printf("Option %d is disabled for ANITA-IV.\n", extraCode);
    return;

    float priAbsMagThresh = 6553;
    screen_dialog(resp,31,"Enter Prioritizerd absolute threshold (dBm) for CW cut (-1 to cancel) [%f]\n", priAbsMagThresh);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=6553) {
	priAbsMagThresh = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(priAbsMagThresh*10);
    cmdBytes[2]=convertedVal & 0xff;
    cmdBytes[3]=(convertedVal & 0xff00)>>8;
  }

  if(extraCode==PRI_THETA_ANGLE_DEMOTION_LOW){
    unsigned short thetaAngleDemotionLow = -30;
    screen_dialog(resp,31,"Enter low theta angle for priority demotion (-90 - 90) (-100 to cancel) [%hu]\n", thetaAngleDemotionLow);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (-90<= t && t <=90) {
	thetaAngleDemotionLow = t;
      } else if (t == -100) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(thetaAngleDemotionLow + 90);
    cmdBytes[2]=convertedVal & 0xff;
    cmdBytes[3]=(convertedVal & 0xff00)>>8;
  }

  if(extraCode==PRI_THETA_ANGLE_DEMOTION_HIGH){
    unsigned short thetaAngleDemotionHigh = 10;
    screen_dialog(resp,31,"Enter high theta angle for priority demotion (-90 - 90) (-100 to cancel) [%hu]\n", thetaAngleDemotionHigh);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (-90<= t && t <=90) {
	thetaAngleDemotionHigh = t;
      } else if (t == -100) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(thetaAngleDemotionHigh);
    cmdBytes[2]=convertedVal & 0xff;
    cmdBytes[3]=(convertedVal & 0xff00)>>8;
  }

  if(extraCode==PRI_THETA_ANGLE_DEMOTION_PRIORITY){
    unsigned char thetaAngleDemotionPriority = 1;
    screen_dialog(resp,31,"Enter priority demotion value, number added to priority if outside the set theta range (0-6)  (-1 to cancel) [%hu]\n", thetaAngleDemotionPriority);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=6) {
	thetaAngleDemotionPriority = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0]=thetaAngleDemotionPriority;
  }

  if(extraCode==PRI_POWER_SPECTRUM_PERIOD){
    unsigned short priPowerSpectrumPeriod = 30;
    screen_dialog(resp,31,"Enter number of seconds to make average power spectrum over before writing to disk (-1 to cancel) [%hu]\n", priPowerSpectrumPeriod);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 < t && t <=65535) {
	priPowerSpectrumPeriod = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[2]=(priPowerSpectrumPeriod & 0xff);
    cmdBytes[3]=(priPowerSpectrumPeriod & 0xff00)>>8;
  }

  if(extraCode==PRI_ANT_PHI_POS) {
    float priAntPhiPos = 0;
    unsigned char ant = 0;

    screen_dialog(resp, 31,
		  "Select antenna index to modify position (0-47) (-1 to cancel) [%hhu] ",
		  ant);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 <= t && t <= 47) {
	ant = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-47, not %f.\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter antenna phi position (degrees) (-1 to cancel) [%f] ",
		  lowBinEdge);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0 <= ft && ft < 360) {
	priAntPhiPos = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-360, not %f.\n", ft);
	return;
      }
    }
    cmdBytes[0]=ant;
    cmdBytes[1]=0;
    unsigned short convertedVal = (unsigned short)(nearbyint(priAntPhiPos*100.0));
    cmdBytes[2]=(convertedVal & 0xff);
    cmdBytes[3]=(convertedVal & 0xff00) >> 8;
  }
  if(extraCode==PRI_ANT_R_POS) {
    float priAntRPos = 0;
    unsigned char ant = 0;
    screen_dialog(resp, 31,
		  "Select antenna index to modify position (0-47) (-1 to cancel) [%hhu] ",
		  ant);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 <= t && t <= 47) {
	ant = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-47, not %f.\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter antenna r position (meters) (-1 to cancel) [%f] ",
		  lowBinEdge);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0 <= ft && ft <= 100) {
	priAntRPos = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-100, not %f.\n", ft);
	return;
      }
    }
    cmdBytes[0]=ant;
    cmdBytes[1]=0;
    unsigned short convertedVal = (unsigned short)(nearbyint(priAntRPos*1000.0));
    cmdBytes[2]=(convertedVal & 0xff);
    cmdBytes[3]=(convertedVal & 0xff00) >> 8;
  }

  if(extraCode==PRI_ANT_Z_POS) {
    float priAntZPos = 0;
    unsigned char ant = 0;
    screen_dialog(resp, 31,
		  "Select antenna index to modify position (0-47) (-1 to cancel) [%hhu] ",
		  ant);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 <= t && t <= 47) {
	ant = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-47, not %f.\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter antenna z position (meters) (+1 to cancel) [%f] ",
		  lowBinEdge);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (-6.5536 <= ft && ft <= 0) {
	priAntZPos = ft;
      } else if (ft == +1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be -6.5536 - 0, not %f.\n", ft);
	return;
      }
    }
    cmdBytes[0]=ant;
    cmdBytes[1]=0;
    unsigned short convertedVal = (unsigned short)(nearbyint(-1.0*priAntZPos*10000.0));
    screen_printf("priAntZPos = %f, convertedVal = %hu\n", priAntZPos, convertedVal);
    cmdBytes[2]=(convertedVal & 0xff);
    cmdBytes[3]=(convertedVal & 0xff00) >> 8;
  }

  if(extraCode==PRI_POS_SATUATION){
    unsigned short priPosSaturation = 1000;
    screen_dialog(resp,31,"Enter mV for positive saturation, goes in priority 9 (0 - 2000) (-1 to cancel) [%hu]\n",priPosSaturation);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=2000) {
	priPosSaturation = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[2]=priPosSaturation & 0xff;
    cmdBytes[3]=(priPosSaturation & 0xff00)>>8;
  }

  if(extraCode==PRI_NEG_SATUATION){
    short priNegSaturation = -1000;
    screen_dialog(resp,31,"Enter mV for negative saturation, goes in priority 9 (0 - -2000) (+1 to cancel) [%hu]\n",priNegSaturation);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0>= t && t >=-2000) {
	priNegSaturation = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = -1*priNegSaturation;
    cmdBytes[2]= convertedVal & 0xff;
    cmdBytes[3]=(convertedVal & 0xff00)>>8;
  }


  /* case PRIORITIZERD_COMMAND: */
  /*   ivalue=theCmd->cmd[1]; //Command num */
  /*   ivalue2=theCmd->cmd[2]+(theCmd->cmd[3]<<8); //command value */
  /*   ivalue3=theCmd->cmd[4]+(theCmd->cmd[5]<<8); //command value */
  /*   return executePrioritizerdCommand(ivalue,ivalue2,ivalue3); */
  /* int executePrioritizerdCommand(int command, int value, int value2) */

  if(extraCode==PRI_ASYM_SATURATION){
    unsigned short priAsymSaturation = 700;

    screen_dialog(resp,31,"Enter mV asymmetry for saturation, goes in priority 9 (0 - 2000) (-1 to cancel) [%hu]\n", priAsymSaturation);
    if (resp[0] != '\0') {
      t = atoi(resp);
      screen_printf("I got %hu\n", t);
      if (0<= t && t <=2000) {
	priAsymSaturation = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0]= priAsymSaturation & 0xff;
    cmdBytes[1]=(priAsymSaturation & 0xff00)>>8;
  }

  if(extraCode==PRI_SLEEP_TIME_KILL_X){
    unsigned short priSleepTimeKillX = 5;

    screen_dialog(resp, 31, "Enter sleep time in the Prioritizerd before killing X if it fails to connect. Don't set this too high. (0 - 60) (-1 to cancel) [%hu]\n", priSleepTimeKillX);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=60) {
	priSleepTimeKillX = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0]= priSleepTimeKillX & 0xff;
    cmdBytes[1]=(priSleepTimeKillX & 0xff00)>>8;
  }

  if(extraCode==PRI_DISABLE_GPU){
    unsigned short priDisableGpu = 0;
    screen_dialog(resp, 31, "Enter flag to disable GPU. 0 is GPU on. 1 is GPU off. (0 - 1) (-1 to cancel) [%hu]\n", priDisableGpu);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=1) {
	priDisableGpu = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = priDisableGpu & 0xff;
    cmdBytes[1] = 0;
  }

  if(extraCode==PRI_CALIB_VERSION){
    unsigned short priCalibVersion = 4;
    screen_dialog(resp, 31, "Enter version of the ANITA calibration to use. 3 is ANITA-III. 4 is ANITA-IV. (3 - 4) (-1 to cancel) [%hu]\n", priCalibVersion);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (3<= t && t <=4) {
	priCalibVersion = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = priCalibVersion & 0xff;
    cmdBytes[1] = 0; //(priCalibVersion & 0xff00)>>8; // uncomment this for ANITA-256
  }


  unsigned short priInvertTopRingSoftware = 0;
  if(extraCode==PRI_INVERT_TOP_RING_SOFTWARE){
    screen_dialog(resp, 31, "Adds a factor of -1 in front of the top ring antennas if enabled.. 1 is on (ANITA-III). 0 is off (ANITA-IV). (0 - 1) (-1 to cancel) [%hu]\n", priInvertTopRingSoftware);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=1) {
        priInvertTopRingSoftware = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = priInvertTopRingSoftware & 0xff;
    cmdBytes[1] = 0;
  }

  if(extraCode==PRI_DEBUG_MODE){
    unsigned char priDebugMode = 0;
    screen_dialog(resp, 31, "Switch on debug mode. Make sure this is off (0) during flight. 1 is on. 0 is off. (0 - 1) (-1 to cancel) [%hu]\n", priDebugMode);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=1) {
	priDebugMode = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = priDebugMode & 0xff;
    cmdBytes[1] = 0;
  }


  if(extraCode==PRI_SKIP_BLAST_RATIO_HPOL){
    unsigned char priSkipBlastRatioHPol = 0;
    unsigned char ant = 0;

    screen_dialog(resp, 31,
		  "Select phi-sector index to modify (0-15) (-1 to cancel) [%hhu] ",
		  ant);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (t >= 0 && t <= 15) {
	ant = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-15, not %hu\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter switch value for ratio skip. 1 is skip. 0 is don't skip. (-1 to cancel) [%hhu] ",
		  priSkipBlastRatioHPol);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (t >= 0 && t <= 1) {
	priSkipBlastRatioHPol = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option.\n", t);
	return;
      }
    }
    cmdBytes[0]=ant;
    cmdBytes[1]=0;
    /* screen_printf("priAntZPos = %f, convertedVal = %hu\n", priAntZPos, convertedVal); */
    cmdBytes[2]=(priSkipBlastRatioHPol & 0xff);
    cmdBytes[3]=0;
  }

  if(extraCode==PRI_SKIP_BLAST_RATIO_VPOL){
    unsigned char priSkipBlastRatioVPol = 0;
    unsigned char ant = 0;
    screen_dialog(resp, 31,
		  "Select phi-sector index to modify (0-15) (-1 to cancel) [%hhu] ",
		  ant);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (t >= 0 && t <= 15) {
	ant = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-47, not %hu\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter switch value for ratio skip. 1 is skip. 0 is don't skip. (-1 to cancel) [%hhu] ",
		  priSkipBlastRatioVPol);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (t >= 0 && t <= 1) {
	priSkipBlastRatioVPol = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option.\n", t);
	return;
      }
    }
    cmdBytes[0]=ant;
    cmdBytes[1]=0;
    /* screen_printf("priAntZPos = %f, convertedVal = %hu\n", priAntZPos, convertedVal); */
    cmdBytes[2]=(priSkipBlastRatioVPol & 0xff);
    cmdBytes[3]=0;

  }
  if(extraCode==PRI_BLAST_RATIO_MAX){
    float priMaxRatioBlast = 2.7;
    screen_dialog(resp, 31, "Ratio of peak-to-peaks in bottom ring divided by top ring, above which we assign event as a self triggered blast. (1 - 10) (-1 to cancel) [%.1f]\n", priMaxRatioBlast);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0<= ft && ft <=10) {
        priMaxRatioBlast = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(nearbyint(priMaxRatioBlast*100));
    cmdBytes[0]=(convertedVal & 0xff);
    cmdBytes[1]=(convertedVal & 0xff00) >> 8;
  }
  if(extraCode==PRI_BLAST_RATIO_MIN){
    float priMinRatioBlast = 1;
    screen_dialog(resp, 31, "Ratio of peak-to-peaks in bottom ring divided by top ring, below  which we assign event as a \"self triggered blast\". (0 - 10) (-1 to cancel) [%.1f]\n", priMinRatioBlast);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0<= ft && ft <=10) {
        priMinRatioBlast = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(nearbyint(priMinRatioBlast*100));
    cmdBytes[0]=(convertedVal & 0xff);
    cmdBytes[1]=(convertedVal & 0xff00) >> 8;
  }
  if(extraCode==PRI_BLAST_GRADIENT){
    float priBlastGradient = 2500;
    screen_dialog(resp, 31, "Gradient that defines a line in rotated cross-correlation parameter space, above which we assign event as a self triggered blast. Set to zero to switch this off. (0-65535) (-1 to cancel) [%.1f]\n", priBlastGradient);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0<= ft && ft <=65535) {
        priBlastGradient = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(nearbyint(priBlastGradient));
    cmdBytes[0]=(convertedVal & 0xff);
    cmdBytes[1]=(convertedVal & 0xff00) >> 8;
  }
  if(extraCode==PRI_STATIC_NOTCH_LOW_EDGE){
    unsigned short notchLowFreq = 0;
    unsigned char notchIndex = 0;

    screen_dialog(resp, 31,
		  "Select notch to modify low edge of (0-9) (-1 to cancel) [%hhu] ",
		  notchIndex);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 <= t && t <= 9) {
	notchIndex = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-9, not %hu\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter the frequency (MHz) to put the low edge of the notch (0-1300) (-1 to cancel) [%hu].",
		  notchLowFreq);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (t >= 0 && t <= 1300) {
	notchLowFreq = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option.\n", t);
	return;
      }
    }
    cmdBytes[0]=notchIndex;
    cmdBytes[1]=0;
    /* screen_printf("priAntZPos = %f, convertedVal = %hu\n", priAntZPos, convertedVal); */
    cmdBytes[2]=(notchLowFreq & 0xff);
    cmdBytes[3]=(notchLowFreq & 0xff00)>>8;
  }
  if(extraCode==PRI_STATIC_NOTCH_HIGH_EDGE){
    unsigned short notchHighFreq = 0;
    unsigned char notchIndex = 0;

    screen_dialog(resp, 31,
		  "Select notch to modify low edge of (0-9) (-1 to cancel) [%hhu] ",
		  notchIndex);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0 <= t && t <= 9) {
	notchIndex = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Value must be 0-9, not %hu\n", t);
	return;
      }
    }
    screen_dialog(resp, 31,
		  "Enter the frequency (MHz) to put the high edge of the notch (0-1300) (-1 to cancel) [%hu].",
		  notchHighFreq);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (t >= 0 && t <= 1300) {
	notchHighFreq = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option.\n", t);
	return;
      }
    }
    cmdBytes[0]=notchIndex;
    cmdBytes[1]=0;
    /* screen_printf("priAntZPos = %f, convertedVal = %hu\n", priAntZPos, convertedVal); */
    cmdBytes[2]=(notchHighFreq & 0xff);
    cmdBytes[3]=(notchHighFreq & 0xff00)>>8;
  }
  if(extraCode==PRI_USE_LONG_DYNAMIC_FILTERING){
    unsigned short useLongDynamicFiltering = 0;
    screen_dialog(resp, 31, "Use dynamic filtering. 1 is on. 0 is off. (0 - 1) (-1 to cancel) [%hu]\n", useLongDynamicFiltering);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=1) {
	useLongDynamicFiltering = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = (useLongDynamicFiltering & 0xff);
    cmdBytes[1] = 0;
  }
  if(extraCode==PRI_START_DYNAMIC_FREQUENCY){
    unsigned short startDynamicFilteringFrequency = 0;
    screen_dialog(resp, 31, "Select frequency above which to use dynamic filtering (0 - 1300) (-1 to cancel) [%hu]\n", startDynamicFilteringFrequency);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=1300) {
	startDynamicFilteringFrequency = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = (startDynamicFilteringFrequency & 0xff);
    cmdBytes[1] = (startDynamicFilteringFrequency & 0xff00) >> 8;
  }
  if(extraCode==PRI_STOP_DYNAMIC_FREQUENCY){
    unsigned short stopDynamicFilteringFrequency = 1300;
    screen_dialog(resp, 31, "Select frequency at which dynamic filtering is stopped (0 - 1300) (-1 to cancel) [%hu]\n", stopDynamicFilteringFrequency);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=1300) {
	stopDynamicFilteringFrequency = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = (stopDynamicFilteringFrequency & 0xff);
    cmdBytes[1] = (stopDynamicFilteringFrequency & 0xff00) >> 8;
  }
  if(extraCode==PRI_CONSERVATIVE_START){
    unsigned short conservativeStart = 0;
    screen_dialog(resp, 31, "Set all priorities to 6 until the first GPU power spectrum has been made for dynamic filtering. 1 is on. 0 is off. (0 - 1) (-1 to cancel) [%hu]\n", conservativeStart);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=1) {
	conservativeStart = t;
      } else if (t == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = (conservativeStart & 0xff);
    cmdBytes[1] = 0;

  }
  if(extraCode==PRI_THRESH_DB){
    float priThresh_dB = 0;
    screen_dialog(resp, 31, "Threshold spike size for dynamic filtering (0-655)  (-1 to cancel) [%.2f]\n", priThresh_dB);
    if (resp[0] != '\0') {
      ft = atof(resp);
      if (0<= ft && ft <=655) {
	priThresh_dB = ft;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    unsigned short convertedVal = (unsigned short)(nearbyint(100*priThresh_dB));
    cmdBytes[0] = (convertedVal & 0xff);
    cmdBytes[1] = (convertedVal & 0xff00)>>8;
  }
  if(extraCode==PRI_DELTA_PHI_TRIG){
    unsigned short deltaPhiSectTrig = 1;
    screen_dialog(resp, 31, "Number of phi-sectors adjacent to L3 trigger to include in map (0-15)  (-1 to cancel) [%hu]\n", deltaPhiSectTrig);
    if (resp[0] != '\0') {
      t = atoi(resp);
      if (0<= t && t <=15) {
	 deltaPhiSectTrig = t;
      } else if (ft == -1) {
	screen_printf("Cancelled.\n");
	return;
      } else {
	screen_printf("Not a valid option\n");
	return;
      }
    }
    cmdBytes[0] = (deltaPhiSectTrig & 0xff);
    cmdBytes[1] = (deltaPhiSectTrig & 0xff00)>>8;
  }


  Curcmd[0] = 0;
  Curcmd[1] = cmdCode;
  Curcmd[2] = 1;
  Curcmd[3] = extraCode;
  int ind=0;
  for(ind=0;ind<8;ind++) {
    Curcmd[4+2*ind]=ind+2;
    Curcmd[5+2*ind]=cmdBytes[ind];
  }
  Curcmdlen = 20;
  set_cmd_log("%d; Prioritizerd command %d (%d %d %d %d %d %d %d %d)", cmdCode,extraCode,cmdBytes[0],cmdBytes[1],cmdBytes[2],cmdBytes[3],cmdBytes[4],cmdBytes[5],cmdBytes[6],cmdBytes[7]);
  sendcmd(Fd, Curcmd, Curcmdlen);

  return;
}


static void
GPSD_EXTRA_COMMAND(cmdCode){
    char resp[32];
    short det;
    short t;
    unsigned char cval=0;
    float fval=0;
    static short extraCode=0;
    static short whichGps=0;
    static short value=0;
     screen_printf("130 -- GPS_SET_GGA_PERIOD\n");
     screen_printf("131 -- GPS_SET_PAT_TELEM_EVERY\n");
     screen_printf("132 -- GPS_SET_VTG_TELEM_EVERY\n");
     screen_printf("133 -- GPS_SET_SAT_TELEM_EVERY\n");
     screen_printf("134 -- GPS_SET_GGA_TELEM_EVERY\n");
     screen_printf("135 -- GPS_SET_POS_TELEM_EVERY\n");
     screen_printf("136 -- GPS_SET_INI_RESET_FLAG\n");
     screen_printf("137 -- GPS_SET_ELEVATION_MASK\n");
     screen_printf("138 -- GPS_SET_CYC_PHASE_ERROR\n");
     screen_printf("139 -- GPS_SET_MXB_BASELINE_ERROR\n");
     screen_printf("140 -- GPS_SET_MXM_PHASE_ERROR\n");
     screen_dialog(resp, 31,
		   "Select Extra Code (-1 to cancel) [%d] ", extraCode);

     if (resp[0] != '\0') {
	 t = atoi(resp);
	 if (130<= t && t <=140) {
	     extraCode = t;
	 } else if (t == -1) {
	     screen_printf("Cancelled.\n");
	     return;
	 } else {
	     screen_printf("Not a valid command code\n");
	     return;
	}
    }
     screen_dialog(resp, 31,
		   "Select GPS 1-ADU5A, 2-ADU5B, 3-G12 (-1 to cancel) [%d] ", whichGps);
     if (resp[0] != '\0') {
	 t = atoi(resp);
	 if (1<= t && t <=3) {
	     whichGps = t;
	 } else if (t == -1) {
	     screen_printf("Cancelled.\n");
	     return;
	 } else {
	     screen_printf("Not a vali GPS\n");
	     return;
	}
     }
     if(extraCode<138) {
       screen_dialog(resp, 31,
		   "Enter value (-1 to cancel) [%d] ", value);
       if (resp[0] != '\0') {
	 t = atoi(resp);
	 if (0<= t && t <=255) {
	   value = t;
	 } else if (t == -1) {
	   screen_printf("Cancelled.\n");
	   return;
	 } else {
	   screen_printf("Not a valid GPS\n");
	   return;
	 }
       }
       cval=value;
     }
     else {
       screen_dialog(resp, 31,
		   "Enter value (-1 to cancel [%f] ", fval);
       if (resp[0] != '\0') {
	 fval = atof(resp);
	 if(fval<0) {
	   screen_printf("Cancelled\n");
	   return;
	 }
       }

       if(extraCode==GPS_SET_CYC_PHASE_ERROR) {
	 cval=round((float)(500*fval));
       }
       else {
	 cval=round((float)(1000*fval));
       }
     }





    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extraCode;
    Curcmd[4] = 2;
    Curcmd[5] = whichGps;
    Curcmd[6] = 3;
    Curcmd[7] = cval;
    Curcmdlen = 8;
    set_cmd_log("%d; Extra GPS command %d %d to %d.", cmdCode,extraCode,whichGps,cval);
    sendcmd(Fd, Curcmd, Curcmdlen);

    return;
}

static void
SIPD_CONTROL_COMMAND(cmdCode){
    char resp[32];
    short det;
    int t;
    unsigned char cval=0;
    static short extraCode=127;
    static short whichInd=0;
    static unsigned short value=0;
    screen_printf("127. SIPD_SEND_WAVE\n");
    screen_printf("128. SIPD_THROTTLE_RATE\n");
    screen_printf("129. SIPD_PRIORITY_BANDWIDTH\n");
    screen_printf("130. SIPD_HEADERS_PER_EVENT\n");
    screen_printf("131. SIPD_HK_TELEM_ORDER\n");
    screen_printf("132. SIPD_HK_TELEM_MAX_PACKETS\n");
    screen_dialog(resp,31,"Select extra code %d (-1 to cancel)\n",extraCode);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (127<= t && t <=132) {
	    extraCode = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Not a valid command\n");
	    return;
	}
    }
    int extraByte=0;
    if(extraCode==SIPD_PRIORITY_BANDWIDTH) {
	extraByte=1;
	screen_dialog(resp,31,"Select priority %d (-1 to cancel)\n",whichInd);
    }
    else if(extraCode==SIPD_HK_TELEM_ORDER ||
	    extraCode==SIPD_HK_TELEM_MAX_PACKETS) {
	extraByte=1;
	screen_dialog(resp,31,"Select hk index %d (-1 to cancel)\n",whichInd);
    }
    if(extraByte) {
	if (resp[0] != '\0') {
	    t= atoi(resp);
	    if (0<= t && t <=20) {
		whichInd = t;
	    } else if (t == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Not a valid index\n");
		return;
	    }
	}
    }
    screen_dialog(resp,31,"Enter value (-1 to cancel)\n",value);
    if (resp[0] != '\0') {
	t= atoi(resp);
	if (t < 0) {
	    screen_printf("Cancelled.\n");
	    return;
	}
	value=t;
    }



    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extraCode;
    Curcmd[4] = 2;
    Curcmd[6] = 3;
    if(extraByte) {
	Curcmd[5] = whichInd;
	Curcmd[7] = value&0xff;
    }
    else {
	Curcmd[5] = value&0xff;
	Curcmd[7] = 0;
	if(extraCode==SIPD_THROTTLE_RATE) {
	    Curcmd[7] = (value&0xff00)>>8;
	}
    }
    Curcmdlen = 8;
    set_cmd_log("%d; Extra SIPd command %d %d to %d.", cmdCode,extraCode,whichInd,value);
    sendcmd(Fd, Curcmd, Curcmdlen);


     return;
}

static void
LOSD_CONTROL_COMMAND(cmdCode){
    char resp[32];
    short det;
    int t;
    unsigned char firstByte=0;
    unsigned char secondByte=0;
    static short extraCode=1;
    static short whichPri=0;
    static unsigned short value=0;
    screen_printf("1. LOSD_SEND_DATA\n");
    screen_printf("2. LOSD_PRIORITY_BANDWIDTH\n");
    screen_printf("3. --- \n");
    screen_printf("4. LOSD_MIN_WAIT_TIME_B\n");
    screen_printf("5. LOSD_MIN_WAIT_TIME_M\n");
    screen_dialog(resp,31,"Select extra code %d (-1 to cancel)\n",extraCode);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if ((1<= t && t <=2) || (t == 4)) {
	    extraCode = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Not a valid command\n");
	    return;
	}
    }
    if(extraCode==LOSD_SEND_DATA) {
	screen_dialog(resp, 31,
		      "Send LOS Data(0 is disable, 1 is enable, -1 to cancel) [%d] ",
		      losSendData);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0 <= t && t <= 1) {
		losSendData = t;
	    } else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	    } else {
		screen_printf("Value must be 0-1, not %d.\n", t);
		return;
	    }
	}
	firstByte=losSendData;
	secondByte=0;
    }
    if(extraCode==LOSD_PRIORITY_BANDWIDTH) {
	screen_dialog(resp, 31,
		      "Which priority (0-9) to set bandwidth for (-1 to cancel) [%d] ",
		      whichPri);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0 <= t && t <= 9) {
		whichPri = t;
	    } else {
		screen_printf("Value must be 0-9, not %d.\n", t);
		return;
	    }
	}

	value=20;
	screen_dialog(resp, 31,
		      "Enter bandwidth fraction 0 to 100 (-1 to cancel) [%d]",
		      value);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0 <= t && t <= 100) {
		value=t;
	    } else {
		screen_printf("Value must be 0 to 100, not %d.\n", t);
		return;
	    }
	}
	firstByte=whichPri;
	secondByte=value;
    }

    if (extraCode == LOSD_MIN_WAIT_TIME_B)
    {
        char wait_time = 5;
        screen_printf(" You have selected LOSD_MIN_WAIT_TIME_B\n");
        screen_printf(" This is intercept of the minimum time that must elapse between LOSd messages\n");
        screen_printf(" Otherwise we miss messages on the ground. \n");
        screen_printf(" Units are in ms. \n\n");

        screen_dialog(resp, 31, "Enter minimum wait time( 0-255) (-1 to cancel) [%d]\n", wait_time);

        if (resp[0] != '\0')
        {
            t = atoi(resp);
            if (0 <= t && t <=255)
            {
                wait_time = t;
            }
            else if (t ==-1)
            {
                screen_printf(" cancelled. \n");
                return;
            }
            else
            {
                screen_printf("bad value: %s\n", resp);
                return;
            }

        }

        firstByte  = wait_time;
        secondByte = 0;

    }

    if (extraCode == LOSD_MIN_WAIT_TIME_M)
    {
        char wait_time = 5;
        screen_printf(" You have selected LOSD_MIN_WAIT_TIME_M\n");
        screen_printf(" This is slope of the  minimum time that must elapse between LOSd messages\n");
        screen_printf(" Otherwise we miss messages on the ground. \n");
        screen_printf(" Units are in us/kbyte. \n\n");

        screen_dialog(resp, 31, "Enter minimum wait time( 0-255) (-1 to cancel) [%d]\n", wait_time);

        if (resp[0] != '\0')
        {
            t = atoi(resp);
            if (0 <= t && t <=255)
            {
                wait_time = t;
            }
            else if (t ==-1)
            {
                screen_printf(" cancelled. \n");
                return;
            }
            else
            {
                screen_printf("bad value: %s\n", resp);
                return;
            }

        }

        firstByte  = wait_time;
        secondByte = 0;

    }
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extraCode;
    Curcmd[4] = 2;
    Curcmd[5] = firstByte;
    Curcmd[6] = 3;
    Curcmd[7] = secondByte;
    Curcmdlen = 8;
    set_cmd_log("%d; Extra SIPd command %d %d to %d.", cmdCode,extraCode,firstByte,secondByte);
    sendcmd(Fd, Curcmd, Curcmdlen);

    return;
}

static void
ACQD_EXTRA_COMMAND(cmdCode){
    char resp[32];
    short det;
    int t;
    unsigned char firstByte=0;
    unsigned char secondByte=0;
    static short extraCode=1;
    static short enableOrDisable=0;
    static short surfNumber=0;
    static unsigned short value=0;
    //    screen_printf("127. DISABLE_SURF\n");
    screen_printf("128. SET_TURF_RATE_AVERAGE\n");
    screen_printf("140. SET_PHOTO_SHUTTER_MASK\n");
    screen_printf("141. SET_PPS_SOURCE\n");
    screen_printf("142. SET_REF_CLOCK_SOURCE\n");
    screen_dialog(resp,31,"Select extra code %d (-1 to cancel)\n",extraCode);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (128== t || t==140 || t==141 || t==142) {
	    extraCode = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Not a valid command\n");
	    return;
	}
    }
    if(extraCode==ACQD_DISABLE_SURF) {
	screen_dialog(resp, 31,
		      "Enable or disable(0 is disable, 1 is enable, -1 to cancel) [%d] ",
		      enableOrDisable);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0 <= t && t <= 1) {
		enableOrDisable = t;
	    } else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	    } else {
		screen_printf("Value must be 0-1, not %d.\n", t);
		return;
	    }
	}
	screen_dialog(resp, 31,
		      "Enter SURF number (1-10), -1 to cancel) [%d] ",
		      surfNumber);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (1 <= t && t <= 10) {
		surfNumber = t;
	    } else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	    } else {
		screen_printf("Value must be 1-10, not %d.\n", t);
		return;
	    }
	}
	firstByte=enableOrDisable;
	secondByte=surfNumber;
    }
    if(extraCode==ACQD_SET_TURF_RATE_AVERAGE) {
	value=60;
	screen_dialog(resp, 31,
		      "How many Turf Rates to Average? (-1 to cancel) [%d] ",
		      value);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (1 <= t && t <= 255) {
		value = t;
	    } else {
		screen_printf("Value must be 1-255, not %d.\n", t);
		return;
	    }
	}
	firstByte=value;
	secondByte=0;
    }
    if(extraCode==ACQD_SET_PHOTO_SHUTTER_MASK) {
	value=0;
	screen_dialog(resp, 31,
		      "Enter decimal value 0 is all on, 7 is all off? (-1 to cancel) [%d] ",
		      value);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0 <= t && t <= 7) {
		value = t;
	    } else {
		screen_printf("Value must be 0-7, not %d.\n", t);
		return;
	    }
	}
	firstByte=value;
	secondByte=0;
    }
    if(extraCode==ACQD_SET_PPS_SOURCE) {
	value=0;
	screen_dialog(resp, 31,
		      "Enter 0 for ADU5, 1 for ADU5B, 2 for G12? (-1 to cancel) [%d] ",
		      value);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0 <= t && t <= 2) {
		value = t;
	    } else {
		screen_printf("Value must be 0-2, not %d.\n", t);
		return;
	    }
	}
	firstByte=value;
	secondByte=0;
    }
    if(extraCode==ACQD_SET_REF_CLOCK_SOURCE) {
	value=0;
	screen_dialog(resp, 31,
		      "Enter 0 for 125MHz, 1 for 33MHz? (-1 to cancel) [%d] ",
		      value);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0 <= t && t <= 1) {
		value = t;
	    } else {
		screen_printf("Value must be 0-1, not %d.\n", t);
		return;
	    }
	}
	firstByte=value;
	secondByte=0;
    }


    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extraCode;
    Curcmd[4] = 2;
    Curcmd[5] = firstByte;
    Curcmdlen = 6;
    set_cmd_log("%d; Extra Acqd command %d %d.", cmdCode,extraCode,firstByte);
    sendcmd(Fd, Curcmd, Curcmdlen);
    return;
}

static void
ACQD_RATE_COMMAND(cmdCode){
    char resp[32];
    short det;
    int t;
    float ft;
    unsigned char cmdBytes[8]={0};
    static short extraCode=1;
    static short enableOrDisable=0;
    static short surfNumber=0;
    static short dacNumber=0;
    static float scaleFactor=0;
    static float fvalue=0;
    static unsigned short usvalue=0;
    static unsigned int uivalue=0;
    static unsigned int l2TrigMask=0;
    static unsigned short phiTrigMask=0;
    static unsigned short surfTrigMask=0;
    static unsigned short eventRate=0;




     screen_printf("1. ENABLE_CHAN_SERVO\n");
     screen_printf("2. SET_PID_GOALS\n");
     screen_printf("3. SET_L2_TRIG_MASK\n");
     screen_printf("4. SET_PHI_MASK\n");
     /* screen_printf("5. SET_SURF_BAND_TRIG_MASK\n"); */
     screen_printf("6. SET_CHAN_PID_GOAL_SCALE\n");
     //     screen_printf("7. SET_RATE_SERVO\n");
     screen_printf("8. ENABLE_DYNAMIC_PHI_MASK\n");
     screen_printf("9. ENABLE_DYNAMIC_L2_MASK\n");
     screen_printf("10. SET_DYNAMIC_PHI_MASK_OVER\n");
     screen_printf("11. SET_DYNAMIC_PHI_MASK_UNDER\n");
     screen_printf("12. SET_DYNAMIC_L2_MASK_OVER\n");
     screen_printf("13. SET_DYNAMIC_L2_MASK_UNDER\n");
     screen_printf("14. SET_GLOBAL_THRESHOLD\n");
     //     screen_printf("16. SET_NADIR_PID_GOALS\n");
     screen_printf("17. SET_PID_PGAIN\n");
     screen_printf("18. SET_PID_IGAIN\n");
     screen_printf("19. SET_PID_DGAIN\n");
     screen_printf("20. SET_PID_IMAX\n");
     screen_printf("21. SET_PID_IMIN\n");
     /* screen_printf("22. SET_PID_AVERAGE\n"); */
     /* screen_printf("23. SET_PHI_MASK_HPOL\n"); */
     screen_printf("25. SET_DYNAMIC_L2_MASK_OVER_WINDOW\n");
     screen_printf("26. SET_DYNAMIC_L2_MASK_UNDER_WINDOW\n");

     screen_dialog(resp,31,"Select extra code %d (-1 to cancel)\n",extraCode);
     if (resp[0] != '\0') {
	 t = atoi(resp);
	if (1<= t && t <=26) {
	    extraCode = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Not a valid command\n");
	    return;
	}
     }
     if(extraCode==ACQD_RATE_ENABLE_CHAN_SERVO) {

	 screen_dialog(resp,31,"1 for enable, 0 for disable %d (-1 to cancel)\n",enableOrDisable);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=1) {
		 enableOrDisable = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=enableOrDisable;


     }
     if(extraCode==ACQD_RATE_SET_PID_GOALS) {
	 screen_dialog(resp,31,"Enter top ring rate in kHz (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=65535) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue&0xff;
	 cmdBytes[1]=(usvalue&0xff00)>>8;
	 screen_dialog(resp,31,"Enter middle ring rate in kHz (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=65535) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[2]=usvalue&0xff;
	 cmdBytes[3]=(usvalue&0xff00)>>8;
	 screen_dialog(resp,31,"Enter bottom ring rate in kHz (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=65535) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[4]=usvalue;
	 cmdBytes[5]=(usvalue&0xff00)>>8;
	 cmdBytes[6]=usvalue&0xff;
	 cmdBytes[7]=(usvalue&0xff00)>>8;

     }
     if(extraCode==ACQD_RATE_SET_L2_TRIG_MASK) {
	 unsigned int t;
	 unsigned int test;
	 int fred;
	 int phiAdd=-1;
	 int allOn=0;
	 l2TrigMask=0;

	 screen_dialog(resp, 31,"Add First phi sector to mask  (0 for all on)( -1 to cancel) [%d] ", phiAdd);

	 if (resp[0] != '\0') {
	     phiAdd=atoi(resp);
	     if(phiAdd==0){
		 allOn=1;
	     }
	     else if(phiAdd>=1 && phiAdd<=16) {
	     }

	     else if(phiAdd==-1) {
		 screen_printf("Cancelled.\n");
		 return;
	     }
	     else {
		 screen_printf("Not a valid phi sector number");
		 return;
	     }
	 }
	 else {
	     screen_printf("Cancelled.\n");
	     return;
	 }


	 int bitShift=phiAdd-1;
	 test=(1<<bitShift);
	 if(allOn==1){
	     test=0;
	 }
	 l2TrigMask|=test;

	 while(1) {
	     phiAdd=-1;
	     screen_dialog(resp, 31,
			   "Add next phi sector to mask  ( -1 to cancel, 0 to finish) [%d]",phiAdd);

	     if (resp[0] != '\0') {
		 phiAdd=atoi(resp);
		 if(phiAdd==0) break;
		 if(phiAdd>=1 && phiAdd<=16) {

		 }
		 else {
		     screen_printf("Not a valid phi sector number");
		   continue;
		 }
		 if(phiAdd==-1) {
		   screen_printf("Cancelled.\n");
		   return;
		 }
	     }
	     else {
		 screen_printf("Cancelled.\n");
		 return;
	     }
	     bitShift=phiAdd-1;
	     test=(1<<bitShift);
	     l2TrigMask|=test;
	 }


	 if (screen_confirm("Really Set l2TrigMask to: %#06x",l2TrigMask)) {
	     cmdBytes[0]=(l2TrigMask&0xff);
	     cmdBytes[1]=((l2TrigMask&0xff00)>>8);
	     /* cmdBytes[2]=((l2TrigMask&0xff0000)>>16); */
	     /* cmdBytes[3]=((l2TrigMask&0xff000000)>>24); */
	 } else {
	     screen_printf("\nCancelled\n");
	     return;
	 }
     }
     if(extraCode==ACQD_RATE_SET_PHI_MASK) {
	 unsigned int t;
	 unsigned int test;
	 int fred;
	 int phiAdd=-1;
	 int allOn=0;
	 phiTrigMask=0;

	 screen_dialog(resp, 31,"Add First Phi Sector to mask  (0 for all on/off)( -1 to cancel) [%d] ", phiAdd);

	 if (resp[0] != '\0') {
	     phiAdd=atoi(resp);
	     if(phiAdd==0){
	       screen_dialog(resp, 31, "1 for enable all, 0 for disable all (-1 to cancel) [%d]", allOn);
	       allOn = atoi(resp);
	       if(allOn == 0 || allOn == 1){
	       }
	       else if(allOn==-1){
		 screen_printf("Cancelled.\n");
		 return;
	       }
	       else{
		 screen_printf("Not a valid option, must be 0 or 1");
		 return;
	       }
	     }
	     else if(phiAdd>=1 && phiAdd<=16) {
	     }
	     else if(phiAdd==-1) {
		 screen_printf("Cancelled.\n");
		 return;
	     }
	     else {
		 screen_printf("Not a valid phi sector");
		 return;
	     }
	 }
	 else {
	     screen_printf("Cancelled.\n");
	     return;
	 }

	 test=(1<<(phiAdd-1));
	 if(allOn==1){
	     test=0xffff;
	 }
	 phiTrigMask|=test;

	 while(1) {
	     phiAdd=-1;
	     screen_dialog(resp, 31,
			   "Add next phi sector to mask  ( -1 to cancel, 0 to finish) [%d]",phiAdd);

	     if (resp[0] != '\0') {
		 phiAdd=atoi(resp);
		 if(phiAdd==0) break;
		 if(phiAdd>=1 && phiAdd<=16) {

		 }
		 else {
		     screen_printf("Not a valid phi sector");
		   continue;
		 }
		 if(phiAdd==-1) {
		   screen_printf("Cancelled.\n");
		   return;
		 }
	     }
	     else {
		 screen_printf("Cancelled.\n");
		 return;
	     }
	     test=(1<<(phiAdd-1));
	     phiTrigMask|=test;
	 }



	 if (screen_confirm("Really Set phiTrigMask to: %#010x",phiTrigMask)) {
	     cmdBytes[0]=(phiTrigMask&0xff);
	     cmdBytes[1]=((phiTrigMask&0xff00)>>8);

	 } else {
	     screen_printf("\nCancelled\n");
	     return;
	 }


     }
     /* if(extraCode==ACQD_RATE_SET_PHI_MASK_HPOL) { */
     /* 	 unsigned int t; */
     /* 	 unsigned int test; */
     /* 	 int fred; */
     /* 	 int phiAdd=-1; */
     /* 	 int allOn=0; */
     /* 	 phiTrigMask=0; */

     /* 	 screen_dialog(resp, 31,"Add First Phi Sector to mask  (0 for all on)( -1 to cancel) [%d] ", phiAdd); */

     /* 	 if (resp[0] != '\0') { */
     /* 	     phiAdd=atoi(resp); */
     /* 	     if(phiAdd==0){ */
     /* 	       screen_dialog(resp, 31, "1 for enable all, 0 for disable all (-1 to cancel) [%d]", allOn); */
     /* 	       allOn = atoi(resp); */
     /* 	       if(allOn == 0 || allOn == 1){ */
     /* 	       } */
     /* 	       else if(allOn==-1){ */
     /* 		 screen_printf("Cancelled.\n"); */
     /* 		 return; */
     /* 	       } */
     /* 	       else{ */
     /* 		 screen_printf("Not a valid option, must be 0 or 1"); */
     /* 		 return; */
     /* 	       } */
     /* 	     } */
     /* 	     else if(phiAdd>=1 && phiAdd<=16) { */
     /* 	     } */
     /* 	     else if(phiAdd==-1) { */
     /* 		 screen_printf("Cancelled.\n"); */
     /* 		 return; */
     /* 	     } */
     /* 	     else { */
     /* 		 screen_printf("Not a valid phi sector"); */
     /* 		 return; */
     /* 	     } */
     /* 	 } */
     /* 	 else { */
     /* 	     screen_printf("Cancelled.\n"); */
     /* 	     return; */
     /* 	 } */

     /* 	 test=(1<<(phiAdd-1)); */
     /* 	 if(allOn==1){ */
     /* 	     test=0xffff; */
     /* 	 } */
     /* 	 phiTrigMask|=test; */

     /* 	 while(1) { */
     /* 	     phiAdd=-1; */
     /* 	     screen_dialog(resp, 31, */
     /* 			   "Add next phi sector to mask  ( -1 to cancel, 0 to finish) [%d]",phiAdd); */

     /* 	     if (resp[0] != '\0') { */
     /* 		 phiAdd=atoi(resp); */
     /* 		 if(phiAdd==0) break; */
     /* 		 if(phiAdd>=1 && phiAdd<=16) { */

     /* 		 } */
     /* 		 else { */
     /* 		     screen_printf("Not a valid phi sector"); */
     /* 		   continue; */
     /* 		 } */
     /* 		 if(phiAdd==-1) { */
     /* 		   screen_printf("Cancelled.\n"); */
     /* 		   return; */
     /* 		 } */
     /* 	     } */
     /* 	     else { */
     /* 		 screen_printf("Cancelled.\n"); */
     /* 		 return; */
     /* 	     } */
     /* 	     test=(1<<(phiAdd-1)); */
     /* 	     phiTrigMask|=test; */
     /* 	 } */



     /* 	 if (screen_confirm("Really Set phiTrigMaskH to: %#010x",phiTrigMask)) { */
     /* 	     cmdBytes[0]=(phiTrigMask&0xff); */
     /* 	     cmdBytes[1]=((phiTrigMask&0xff00)>>8); */

     /* 	 } else { */
     /* 	     screen_printf("\nCancelled\n"); */
     /* 	     return; */
     /* 	 } */
     /* } */

     if(extraCode==ACQD_RATE_SET_SURF_BAND_TRIG_MASK) {

	 screen_dialog(resp, 31,
		       "Which SURF to change trigBandMask  (1-10, -1 to cancel) [%ul] ",
		       surfNumber);
	 if (resp[0] != '\0') {
	     t=atoi(resp);
	     if(t>=1 && t<=9) {
		 surfNumber=t-1;
	     }
	     else if(t==-1) {
		 screen_printf("Cancelled.\n");
		 return;
	     }
	     else {
		 screen_printf("SURF must be between 1 and 10.\n");
		 return;
	     }
	 }

	 screen_dialog(resp, 31,
		       "Hex bitmask for 16 channels  (0 - 0xffff, -1 to cancel) [%ul] ",
		       surfTrigMask);
	 if (resp[0] != '\0') {
	     t=atoi(resp);
	     if(t==-1) {
		 screen_printf("Cancelled.\n");
		 return;
	     }
	     t=strtol(resp,NULL,16);
	     if(t>=0 && t<=0xffff) {
		 surfTrigMask=t;
	     }
	     else {
		 screen_printf("SURF must be between 0 and 0xffff (not %#x).\n",t);
		 return;
	     }
	 }

	 cmdBytes[0]=surfNumber;
	 cmdBytes[1]=surfTrigMask&0xff;
	 cmdBytes[2]=(surfTrigMask&0xff00)>>8;
     }
     if(extraCode==ACQD_RATE_SET_CHAN_PID_GOAL_SCALE) {

	 screen_printf("Use this command to change the scale factor for a single trigger band\n");
	 screen_printf("Obviously use with caution\n");
	 screen_dialog(resp, 31,
		       "Which SURF, 3-10, to change (-1 to cancel) [%d] ",
		       surfNumber);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (3 <= t && t <= 10) {
		 surfNumber = t-1;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Value must be 3-10, not %d.\n", t);
		 return;
	     }
	 }
	 screen_dialog(resp, 31,
		       "Which DAC, 1-12, to change (-1 to cancel) [%d] ",
		       dacNumber+1);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1 <= t && t <= 12) {
		 dacNumber = t-1;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Value must be 1-12, not %d.\n", t);
		 return;
	     }
	 }


	 screen_dialog(resp, 31,
		       "To what scale factor (1 is normal, <1 decreases rate, >1 increases rate (-1 to cancel) [%f] ",
		       scaleFactor);
	 if (resp[0] != '\0') {
	     ft = atof(resp);
	     if (ft >= 0) {
		 scaleFactor = ft;
	     } else {
		 screen_printf("Cancelled.\n");
		 return;
	     }
	 }

	 usvalue = ((unsigned short) (scaleFactor*1000.));
	 //    screen_printf("scaleFactor %f\tvalue %u\n",scaleFactor,value);
	 cmdBytes[0]=surfNumber;
	 cmdBytes[1]=dacNumber;
	 cmdBytes[2]=usvalue&0xff;
	 cmdBytes[3]=(usvalue&0xff00)>>8;

     }
     if(extraCode==ACQD_RATE_SET_RATE_SERVO) {
	 float rate=0;
	 short enabler=0;
	 eventRate=0;
	 screen_dialog(resp, 31,
		       "Set Desired Event Rate  (0 to disable servo, -1 to cancel) [%f] ",
		       rate);
	 if (resp[0] != '\0') {
	     ft = atof(resp);
	     if (0 < ft && ft <= 10) {
		 enabler=1;
		 rate = ft;
		 rate*=1000;
		 eventRate=(int)rate;
	     } else if (ft < 0) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 enabler=0;
		 rate=0;
		 eventRate=0;
	     }
	 }
	 cmdBytes[0]=eventRate&0xff;
	 cmdBytes[1]=(eventRate&0xff00)>>8;
     }
     if(extraCode==ACQD_RATE_ENABLE_DYNAMIC_PHI_MASK) {
	 screen_dialog(resp,31,"1 for enable, 0 for disable %d (-1 to cancel)\n",enableOrDisable);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=1) {
	    enableOrDisable = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	     cmdBytes[0]=enableOrDisable;
	 }

     }
     if(extraCode==ACQD_RATE_ENABLE_DYNAMIC_L2_MASK) {
	 screen_dialog(resp,31,"1 for enable, 0 for disable %d (-1 to cancel)\n",enableOrDisable);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=1) {
	    enableOrDisable = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=enableOrDisable;

     }
     if(extraCode==ACQD_RATE_SET_DYNAMIC_PHI_MASK_OVER) {
	 usvalue=20;
	 screen_dialog(resp,31,"Set L3 phi  over threshold rate (1-255Hz) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t && t <=255) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid rate\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;

	 usvalue=20;
	 screen_dialog(resp,31,"Set L3 phi threshold window (1-255) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t && t <=255) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid time window\n");
		 return;
	     }
	 }
	 cmdBytes[1]=usvalue;

     }
     if(extraCode==ACQD_RATE_SET_DYNAMIC_PHI_MASK_UNDER) {
	 usvalue=2;
	 screen_dialog(resp,31,"Set L3 phi under threshold rate (1-255Hz) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t && t <=255) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid rate\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;

	 usvalue=40;
	 screen_dialog(resp,31,"Set L3 phi threshold window (1-255) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t && t <=255) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid time window\n");
		 return;
	     }
	 }
	 cmdBytes[1]=usvalue;

     }
     if(extraCode==ACQD_RATE_SET_DYNAMIC_L2_MASK_OVER) {
	 uivalue=100000;
	 screen_dialog(resp,31,"Set L2 over threshold rate (Hz) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t ) {
		 uivalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid rate\n");
		 return;
	     }
	 }
	 cmdBytes[0]=(uivalue&0xff);
	 cmdBytes[1]=(uivalue&0xff00)>>8;
	 cmdBytes[2]=(uivalue&0xff0000)>>16;
	 cmdBytes[3]=(uivalue&0xff000000)>>24;

	 usvalue=40;
	 screen_dialog(resp,31,"Set L2 phi sector (1-16) (-1 to cancel) [%d]\n",usvalue);

	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t && t <=16) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid phi sector\n");
		 return;
	     }
	 }
	 cmdBytes[4]=usvalue-1;

     }
     if(extraCode==ACQD_RATE_SET_DYNAMIC_L2_MASK_UNDER) {

	 uivalue=100000;
	 screen_dialog(resp,31,"Set L2 under threshold rate (Hz) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t ) {
		 uivalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid rate\n");
		 return;
	     }
	 }
	 cmdBytes[0]=(uivalue&0xff);
	 cmdBytes[1]=(uivalue&0xff00)>>8;
	 cmdBytes[2]=(uivalue&0xff0000)>>16;
	 cmdBytes[3]=(uivalue&0xff000000)>>24;

	 usvalue=40;
	 screen_dialog(resp,31,"Set L2 phi sector (1-16) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (1<= t && t <=16) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid phi sector\n");
		 return;
	     }
	 }
	 cmdBytes[4]=usvalue-1;

     }
     if(extraCode==ACQD_RATE_SET_DYNAMIC_L2_MASK_UNDER_WINDOW) {

	 usvalue=60;
	 screen_dialog(resp,31,"Set L2 threshold window (1-60) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	   t = atoi(resp);
	   if (1<= t && t <=60) {
	     usvalue = t;
	   } else if (t == -1) {
	     screen_printf("Cancelled.\n");
	     return;
	   } else {
	     screen_printf("Not a valid time window\n");
	     return;
	   }
	 }

	 cmdBytes[0]=(usvalue);

     }

     if(extraCode==ACQD_RATE_SET_DYNAMIC_L2_MASK_OVER_WINDOW) {

	 usvalue=60;
	 screen_dialog(resp,31,"Set L2 threshold over window (1-60) (-1 to cancel) [%d]\n",usvalue);
	 if (resp[0] != '\0') {
	   t = atoi(resp);
	   if (1<= t && t <=60) {
	     usvalue = t;
	   } else if (t == -1) {
	     screen_printf("Cancelled.\n");
	     return;
	   } else {
	     screen_printf("Not a valid time window\n");
	     return;
	   }
	 }

	 cmdBytes[0]=usvalue;

     }


     if(extraCode==ACQD_RATE_SET_GLOBAL_THRESHOLD) {
	 usvalue=0;
	 screen_dialog(resp,31,"Enter global threshold (0 to disable, -1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=4095) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue&0xff;
	 cmdBytes[1]=(usvalue&0xff00)>>8;
     }
     if(extraCode==ACQD_SET_NADIR_PID_GOALS) {
	 screen_dialog(resp,31,"Enter low band goal (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=65535) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue&0xff;
	 cmdBytes[1]=(usvalue&0xff00)>>8;
	 screen_dialog(resp,31,"Enter mid band goal (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=65535) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[2]=usvalue&0xff;
	 cmdBytes[3]=(usvalue&0xff00)>>8;
	 screen_dialog(resp,31,"Enter high band goal (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=65535) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[4]=usvalue&0xff;
	 cmdBytes[5]=(usvalue&0xff00)>>8;
	 screen_dialog(resp,31,"Enter full band goal (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=65535) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[6]=usvalue&0xff;
	 cmdBytes[7]=(usvalue&0xff00)>>8;


     }
     if(extraCode==ACQD_SET_PID_PGAIN) {
	 usvalue=0;
	 screen_dialog(resp,31,"Enter ring: 0-Top, 1-Middle, 2-Bottom (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=2) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;
	 fvalue=0.01;
	 screen_dialog(resp,31,"Enter Gain -- [%f]\n",fvalue);
	 if (resp[0] != '\0') {
	     fvalue = atof(resp);
	     if(fvalue<0) {
		 screen_printf("Not a valid gain\n");
		 return;
	     }
	 }
	 uivalue=fvalue*10000;
	 cmdBytes[1]=uivalue&0xff;
	 cmdBytes[2]=(uivalue&0xff00)>>8;
	 cmdBytes[3]=(uivalue&0xff0000)>>16;
	 cmdBytes[4]=(uivalue&0xff000000)>>24;


     }
     if(extraCode==ACQD_SET_PID_IGAIN) {
	 usvalue=0;
	 screen_dialog(resp,31,"Enter ring 0-Top, 1-Middle, 2-Bottom (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=2) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;
	 fvalue=0.01;
	 screen_dialog(resp,31,"Enter Gain -- [%f]\n",fvalue);
	 if (resp[0] != '\0') {
	     fvalue = atof(resp);
	     if(fvalue<0) {
		 screen_printf("Not a valid gain\n");
		 return;
	     }
	 }
	 uivalue=fvalue*10000;
	 cmdBytes[1]=uivalue&0xff;
	 cmdBytes[2]=(uivalue&0xff00)>>8;
	 cmdBytes[3]=(uivalue&0xff0000)>>16;
	 cmdBytes[4]=(uivalue&0xff000000)>>24;


     }
     if(extraCode==ACQD_SET_PID_DGAIN) {
	 usvalue=0;
	 screen_dialog(resp,31,"Enter ring 0-Top, 1-Middle, 2-Bottom (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=2) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;
	 fvalue=0.01;
	 screen_dialog(resp,31,"Enter Gain -- [%f]\n",fvalue);
	 if (resp[0] != '\0') {
	     fvalue = atof(resp);
	     if(fvalue<0) {
		 screen_printf("Not a valid gain\n");
		 return;
	     }
	 }
	 uivalue=fvalue*10000;
	 cmdBytes[1]=uivalue&0xff;
	 cmdBytes[2]=(uivalue&0xff00)>>8;
	 cmdBytes[3]=(uivalue&0xff0000)>>16;
	 cmdBytes[4]=(uivalue&0xff000000)>>24;


     }
     if(extraCode==ACQD_SET_PID_IMAX) {
	 usvalue=0;
	 screen_dialog(resp,31,"Enter ring 0-Top, 1-Middle, 2-Bottom (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=2) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;
	 fvalue=0.01;
	 screen_dialog(resp,31,"Enter Gain -- [%f]\n",fvalue);
	 if (resp[0] != '\0') {
	     fvalue = atof(resp);
	     if(fvalue<0) {
		 screen_printf("Not a valid gain\n");
		 return;
	     }
	 }
	 uivalue=fvalue;
	 cmdBytes[1]=uivalue&0xff;
	 cmdBytes[2]=(uivalue&0xff00)>>8;
	 cmdBytes[3]=(uivalue&0xff0000)>>16;
	 cmdBytes[4]=(uivalue&0xff000000)>>24;


     }
     if(extraCode==ACQD_SET_PID_IMIN) {
	 usvalue=0;
	 screen_dialog(resp,31,"Enter ring 0-Top, 1-Middle, 2-Bottom (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t && t <=2) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;
	 fvalue=0.01;
	 screen_dialog(resp,31,"Enter Gain -- [%f]\n",fvalue);
	 if (resp[0] != '\0') {
	     fvalue = atof(resp);
	     if(fvalue<0) {
		 screen_printf("Not a valid gain\n");
		 return;
	     }
	 }
	 uivalue=fvalue;
	 cmdBytes[1]=uivalue&0xff;
	 cmdBytes[2]=(uivalue&0xff00)>>8;
	 cmdBytes[3]=(uivalue&0xff0000)>>16;
	 cmdBytes[4]=(uivalue&0xff000000)>>24;


     }
     if(extraCode==ACQD_SET_PID_AVERAGE) {
	 usvalue=0;
	 screen_dialog(resp,31,"PID Surf Hk Average (-1 to cancel)\n",usvalue);
	 if (resp[0] != '\0') {
	     t = atoi(resp);
	     if (0<= t) {
		 usvalue = t;
	     } else if (t == -1) {
		 screen_printf("Cancelled.\n");
		 return;
	     } else {
		 screen_printf("Not a valid option\n");
		 return;
	     }
	 }
	 cmdBytes[0]=usvalue;
     }


     Curcmd[0] = 0;
     Curcmd[1] = cmdCode;
     Curcmd[2] = 1;
     Curcmd[3] = extraCode;
     int ind=0;
     for(ind=0;ind<8;ind++) {
	 Curcmd[4+2*ind]=ind+2;
	 Curcmd[5+2*ind]=cmdBytes[ind];
     }
     Curcmdlen = 20;
     set_cmd_log("%d; Acqd Rate command %d (%d %d %d %d %d %d %d %d)", cmdCode,extraCode,cmdBytes[0],cmdBytes[1],cmdBytes[2],cmdBytes[3],cmdBytes[4],cmdBytes[5],cmdBytes[6],cmdBytes[7]);
     sendcmd(Fd, Curcmd, Curcmdlen);

     return;
}

static void
GPS_PHI_MASK_COMMAND(cmdCode){
    char resp[32];
    short det;
    int t;
    float ft;
    unsigned char cmdBytes[5]={0};
    static short extraCode=1;
    static short enableOrDisable=0;
    static float fvalue=0;
    static unsigned short usvalue=0;
    static unsigned int uivalue=0;

  screen_printf("1. GPS_PHI_MASK_ENABLE\n");
  screen_printf("2. GPS_PHI_MASK_UPDATE_PERIOD\n");
  screen_printf("3. GPS_PHI_MASK_SET_SOURCE_LATITUDE\n");
  screen_printf("4. GPS_PHI_MASK_SET_SOURCE_LONGITUDE\n");
  screen_printf("5. GPS_PHI_MASK_SET_SOURCE_ALTITUDE\n");
  screen_printf("6. GPS_PHI_MASK_SET_SOURCE_HORIZON\n");
  screen_printf("7. GPS_PHI_MASK_SET_SOURCE_WIDTH\n");
  screen_dialog(resp,31,"Select extra code %d (-1 to cancel)\n",extraCode);
  if (resp[0] != '\0') {
      t = atoi(resp);
	if (1<= t && t <=7) {
	    extraCode = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Not a valid command\n");
	    return;
	}
     }


  if(extraCode== GPS_PHI_MASK_ENABLE) {
      screen_dialog(resp,31,"1 for enable, 0 for disable %d (-1 to cancel)\n",enableOrDisable);
      if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (0<= t && t <=1) {
	      enableOrDisable = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[0]=enableOrDisable;
  }
  if(extraCode==GPS_PHI_MASK_UPDATE_PERIOD)
  {
      usvalue=30;
      screen_dialog(resp,31,"Set update period [%d] (-1 to cancel)\n",usvalue);
      if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (1<= t && t <=65535) {
	      usvalue = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[0]=usvalue&0xff;
      cmdBytes[1]=(usvalue&0xff00)>>8;

  }
  if(extraCode==GPS_PHI_MASK_SET_SOURCE_LATITUDE)
  {
      usvalue=30;
      screen_dialog(resp,31,"Select source (1-20) [%d] (-1 to cancel)\n",usvalue);
      if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (1<= t && t <=20) {
	      usvalue = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[0]=usvalue&0xff;
      fvalue=-90;
      screen_dialog(resp,31,"Enter latitude [%f] (-1 to cancel)\n",fvalue);
      if (resp[0] != '\0') {
	  ft = atof(resp);
	  if (-180<= ft && ft <=180) {
	      uivalue = (180+ft)*1e7;
	  } else if (ft == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[1]=(uivalue&0xff);
      cmdBytes[2]=(uivalue&0xff00)>>8;
      cmdBytes[3]=(uivalue&0xff0000)>>16;
      cmdBytes[4]=(uivalue&0xff000000)>>24;

  }
  if(extraCode==GPS_PHI_MASK_SET_SOURCE_LONGITUDE)
  {
      usvalue=30;
      screen_dialog(resp,31,"Select source (1-20) [%d] (-1 to cancel)\n",usvalue);
      if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (1<= t && t <=20) {
	      usvalue = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[0]=usvalue&0xff;
      fvalue=-90;
      screen_dialog(resp,31,"Enter longitude [%f] (-1 to cancel)\n",fvalue);
      if (resp[0] != '\0') {
	  ft = atof(resp);
	  if (-180<= ft && ft <=180) {
	      uivalue = (180+ft)*1e7;
	  } else if (ft == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[1]=(uivalue&0xff);
      cmdBytes[2]=(uivalue&0xff00)>>8;
      cmdBytes[3]=(uivalue&0xff0000)>>16;
      cmdBytes[4]=(uivalue&0xff000000)>>24;

  }
  if(extraCode==GPS_PHI_MASK_SET_SOURCE_ALTITUDE)
  {
      usvalue=30;
      screen_dialog(resp,31,"Select source (1-20) [%d] (-1 to cancel)\n",usvalue);
      if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (1<= t && t <=20) {
	      usvalue = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[0]=usvalue&0xff;
      fvalue=-90;
      screen_dialog(resp,31,"Enter altitude (in m) [%f] (-1 to cancel)\n",fvalue);
      if (resp[0] != '\0') {
	  ft = atof(resp);
	  if (0<= ft && ft <=45000) {
	      uivalue = ft*1e3;
	  } else if (ft == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[1]=(uivalue&0xff);
      cmdBytes[2]=(uivalue&0xff00)>>8;
      cmdBytes[3]=(uivalue&0xff0000)>>16;
      cmdBytes[4]=(uivalue&0xff000000)>>24;

  }
  if(extraCode==GPS_PHI_MASK_SET_SOURCE_HORIZON)
  {
      usvalue=30;
      screen_dialog(resp,31,"Select source (1-20) [%d] (-1 to cancel)\n",usvalue);
      if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (1<= t && t <=20) {
	      usvalue = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[0]=usvalue&0xff;
      fvalue=-90;
      screen_dialog(resp,31,"Enter source horizon (in m) [%f] (-1 to cancel)\n",fvalue);
      if (resp[0] != '\0') {
	  ft = atof(resp);
	  if (0<= ft && ft <=1e6) {
	      uivalue = ft;
	  } else if (ft == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[1]=(uivalue&0xff);
      cmdBytes[2]=(uivalue&0xff00)>>8;
      cmdBytes[3]=(uivalue&0xff0000)>>16;
      cmdBytes[4]=(uivalue&0xff000000)>>24;

  }
  if(extraCode==GPS_PHI_MASK_SET_SOURCE_WIDTH)
  {
      usvalue=30;
      screen_dialog(resp,31,"Select source (1-20) [%d] (-1 to cancel)\n",usvalue);
      if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (1<= t && t <=20) {
	      usvalue = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[0]=usvalue&0xff;
      usvalue=1;
      screen_dialog(resp,31,"Enter mask width (in phi sectors) [%d] (-1 to cancel)\n",usvalue);
       if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (1<= t && t <=16) {
	      usvalue = t;
	  } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	  } else {
	      screen_printf("Not a valid option\n");
	      return;
	  }
      }
      cmdBytes[1]=(usvalue&0xff);

  }

  Curcmd[0] = 0;
  Curcmd[1] = cmdCode;
  Curcmd[2] = 1;
  Curcmd[3] = extraCode;
  int ind=0;
  for(ind=0;ind<5;ind++) {
      Curcmd[4+2*ind]=ind+2;
      Curcmd[5+2*ind]=cmdBytes[ind];
  }
  Curcmdlen = 14;
  set_cmd_log("%d; GPS Phi Mask command %d (%d %d %d %d %d)", cmdCode,extraCode,cmdBytes[0],cmdBytes[1],cmdBytes[2],cmdBytes[3],cmdBytes[4]);
  sendcmd(Fd, Curcmd, Curcmdlen);
  return;
}

static void
PLAYBACKD_COMMAND(cmdCode){
    char resp[32];
    short det;
    int t;
    float ft;
    unsigned char cmdBytes[5]={0};
    static short extraCode=1;
    static short enableOrDisable=0;
    static float fvalue=0;
    static unsigned short usvalue=0;
    static unsigned int uivalue=0;
    screen_printf("1. PLAY_GET_SINGLE_EVENT\n");
    screen_printf("2. PLAY_START_PRIORITY\n");
    screen_printf("3. PLAY_STOP_PRIORITY\n");
    //    screen_printf("4. PLAY_START_EVENT_NUMBER\n");
    screen_printf("5. PLAY_USE_DISK\n");
    screen_printf("6. PLAY_START_PLAYBACK\n");
    screen_printf("7. PLAY_STOP_PLAYBACK\n");
    screen_printf("8. PLAY_SLEEP_PERIOD\n");
    screen_printf("9. PLAY_MODE\n");
    screen_printf("10. PLAY_STARTING_RUN\n");

    screen_dialog(resp,31,"Select extra code %d (-1 to cancel)\n",extraCode);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (1<= t && t <=10) {
	    extraCode = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Not a valid command\n");
	    return;
	}
     }
    if(extraCode==PLAY_GET_EVENT) {
	uivalue=1000;
        screen_dialog(resp,31,"Enter event number [%d] (-1 to cancel)\n",uivalue);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0<= t) {
		uivalue = t;
	    } else if (t == -1) {
		screen_printf("Cancelled.\n");
	    return;
	    } else {
		screen_printf("Not a valid event number\n");
		return;
	    }
	}
	usvalue=1;
        screen_dialog(resp,31,"Enter desired priority [%d] (-1 to cancel)\n",usvalue);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (1<= t && t<11) {
		usvalue = t;
	    } else if (t == -1) {
		screen_printf("Cancelled.\n");
	    return;
	    } else {
		screen_printf("Not a valid priority\n");
		return;
	    }
	}
	cmdBytes[0]=(uivalue&0xff);
	cmdBytes[1]=(uivalue&0xff00)>>8;
	cmdBytes[2]=(uivalue&0xff0000)>>16;
	cmdBytes[3]=(uivalue&0xff000000)>>24;
	cmdBytes[4]=usvalue&0xff;
    }

    if(extraCode==PLAY_SLEEP_PERIOD) {
	uivalue=1000;
        screen_dialog(resp,31,"Enter sleep period in ms (i.e 1/rate) [%d] (-1 to cancel)\n",uivalue);
	if (resp[0] != '\0') {
	  t = atoi(resp);
	  if (0<= t) {
	    uivalue = t;
	  } else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	  } else {
	    screen_printf("Not a valid event number\n");
	    return;
	  }
	}
	usvalue=0;
	cmdBytes[0]=(uivalue&0xff);
	cmdBytes[1]=(uivalue&0xff00)>>8;
	cmdBytes[2]=(uivalue&0xff0000)>>16;
	cmdBytes[3]=(uivalue&0xff000000)>>24;
	cmdBytes[4]=usvalue&0xff;
    }
    if(extraCode==PLAY_START_PLAY) {
	if (!screen_confirm("Really Start Playback?")) {
	    return;
	}
    }
    if(extraCode==PLAY_STOP_PLAY) {
	if (!screen_confirm("Really Stop Playback?")) {
	    return;
	}
    }
    if(extraCode==PLAY_USE_DISK) {
	usvalue=0;
	screen_printf("0. Helium1\n1. Helium2\n2. USB\n");
        screen_dialog(resp,31,"Select playback drive [%d] (-1 to cancel)\n",usvalue);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0<= t && t<3) {
		usvalue = t;
	    } else if (t == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Not a valid drive\n");
		return;
	    }
	}
	cmdBytes[0]=(usvalue&0xff);
    }
    if(extraCode==PLAY_START_PRI) {
	usvalue=0;
        screen_dialog(resp,31,"Enter priority to start playing back (0-9) [%d] (-1 to cancel)\n",usvalue);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0<= t && t<9) {
	      usvalue = t;
	    } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	    } else {
	      screen_printf("Not a valid drive\n");
	      return;
	    }
	}
	cmdBytes[0]=(usvalue&0xff);
	cmdBytes[1]=0;
	cmdBytes[2]=0;
	cmdBytes[3]=0;
	cmdBytes[4]=0;
    }
    if(extraCode==PLAY_STOP_PRI) {
	usvalue=0;
        screen_dialog(resp,31,"Enter priority to stop playing back (0-9) [%d] (-1 to cancel)\n",usvalue);
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0<= t && t<9) {
	      usvalue = t;
	    } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	    } else {
	      screen_printf("Not a valid drive\n");
	      return;
	    }
	}
	cmdBytes[0]=(usvalue&0xff);
	cmdBytes[1]=0;
	cmdBytes[2]=0;
	cmdBytes[3]=0;
	cmdBytes[4]=0;
    }

    if (extraCode==PLAY_MODE)
    {
      usvalue = 1; 
      screen_dialog(resp,31,"Enter playback mode (0-1) [%d] (-1 to cancel)\n", usvalue); 
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0<= t && t<=1) {
	      usvalue = t;
	    } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	    } else {
	      screen_printf("Not a valid mode\n");
	      return;
	    }
	}
	cmdBytes[0]=(usvalue&0xff);
	cmdBytes[1]=0;
	cmdBytes[2]=0;
	cmdBytes[3]=0;
	cmdBytes[4]=0; 
    }


    if (extraCode==PLAY_STARTING_RUN)
    {
      usvalue = 1; 
      screen_dialog(resp,31,"Enter starting run (0-65535) [%d] (-1 to cancel)\n", usvalue); 
	if (resp[0] != '\0') {
	    t = atoi(resp);
	    if (0<= t && t<=65535) {
	      usvalue = t;
	    } else if (t == -1) {
	      screen_printf("Cancelled.\n");
	      return;
	    } else {
	      screen_printf("Not a valid run\n");
	      return;
	    }
	}
	cmdBytes[0]=(usvalue&0xff);
	cmdBytes[1]=(usvalue & 0xff00) >> 8;
	cmdBytes[2]=0;
	cmdBytes[3]=0;
	cmdBytes[4]=0; 
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extraCode;
    int ind=0;
    for(ind=0;ind<5;ind++) {
	Curcmd[4+2*ind]=ind+2;
	Curcmd[5+2*ind]=cmdBytes[ind];
    }
    Curcmdlen = 14;
    set_cmd_log("%d; Playback command %d (%d %d %d %d %d)", cmdCode,extraCode,cmdBytes[0],cmdBytes[1],cmdBytes[2],cmdBytes[3],cmdBytes[4]);
    sendcmd(Fd, Curcmd, Curcmdlen);
    return;

}

#define NBITS_FOR_RTL_INDEX 3
#define NUM_RTLSDR 6

static void RTLD_COMMAND(cmdCode)
{
  char resp[32];
  char extra_code = 1;
  int t;
  double real;
  unsigned char cmdBytes[2] = {0};
  unsigned char telemEvery = 1;
  unsigned short startFrequency = 180;
  unsigned short endFrequency = 1300;
  unsigned short stepFrequency = 300;
  unsigned char gainTarget = 1;
  double gain = 20;
  short sgain = 200;
  short gracefulTimeout = 60;
  short failTimeout = 5;
  short failThreshold = 5;
  char disable_index = 0;
  char disable_value = 0;

  screen_printf("Enter RTLd command (1-5):  \n\n");
  screen_printf("  1.  RTL_SET_TELEM_EVERY      --- set telemetry interval  \n");
  screen_printf("  2.  RTL_SET_START_FREQUENCY  --- set power spectrum start frequency  \n");
  screen_printf("  3.  RTL_SET_END_FREQUENCY    --- set power spectrum end frequency  \n");
  screen_printf("  4.  RTL_SET_GAIN             --- set RTL-SDR gains \n");
  screen_printf("  5.  RTL_SET_FREQUENCY_STEP   --- set power spectrum frequency step \n");
  screen_printf("  6.  RTL_SET_GRACEFUL_TIMEOUT --- set scan timeout\n");
  screen_printf("  7.  RTL_SET_FAIL_TIMEOUT     --- set kill -9 timeout (after graceful fails)\n");
  screen_printf("  8.  RTL_SET_MAX_FAILS        --- maximum number of fails until soft disable \n");
  screen_printf("  9.  RTL_SET_DISABLED         --- soft disable devices (or enable) \n");
  screen_dialog(resp, 31, "Select command [%d] (-1 to cancel)\n", extra_code);


  if (resp[0] != '\0')
  {
    t = atoi(resp);

    if ( 1 <=t && t <=9)
    {
      extra_code = t;
    }
    else if (t == -1)
    {
      screen_printf("Cancelled. Have a nice day!\n");
      return;
    }
    else
    {
      screen_printf("Not a valid command.\n");
      return;
    }


    if (extra_code == RTL_SET_TELEM_EVERY)
    {
      screen_printf("[ You have selected RTL_SET_TELEM_EVERY  ]\n");
      screen_printf("   This controls how often RTL-SDR power spectra are telemetered.\n");
      screen_printf("   0 means never, otherwise every Nth spectrum is telemetered (i.e. 1 means every)\n");
      screen_dialog(resp, 31, "Enter telemetry interval (0-255) [%d]  (-1 to cancel)\n\n", telemEvery);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=255)
        {
          telemEvery = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("Not a valid option\n");
          return;
        }

      }

      cmdBytes[0] = telemEvery;
    }

    else if (extra_code == RTL_SET_START_FREQUENCY)
    {
      screen_printf("[ You have selected RTL_SET_START_FREQUENCY ]\n");
      screen_printf("   This sets the start frequency of the scan, in __MHz__\n");
      screen_printf("   Be aware that the larger the scan, the longer it will take\n");
      screen_dialog(resp, 31, "Enter start frequency in MHz [%d] (50 - 1699) (-1 to cancel)\n\n", startFrequency);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if  (50 <=t && t <=1699)
        {
          startFrequency = t;
        }
        else if (t == -1)
        {
          screen_printf("Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d is outside the range of valid start frequencies (50-1699)\n\n", t);
          return;
        }
      }

      cmdBytes[0] = startFrequency & 0xff;
      cmdBytes[1] = (startFrequency & 0xff00) >> 8;
    }

    else if (extra_code == RTL_SET_END_FREQUENCY)
    {
      screen_printf(" [You have selected RTL_SET_END_FREQUENCY] \n");
      screen_printf("   This sets the end frequency of the scan, in __MHz__\n");
      screen_printf("   Be aware that the larger the scan, the longer it will take\n");
      screen_dialog(resp, 31, "Enter end frequency in MHz [%d] (51 - 1700) (-1 to cancel)\n\n", endFrequency);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if  (51 <=t && t <=1700)
        {
          endFrequency = t;
        }
        else if (t == -1)
        {
          screen_printf("Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d is outside the range of valid end frequencies (51-1700)\n", t);
          return;
        }
      }

      cmdBytes[0] = endFrequency & 0xff;
      cmdBytes[1] = (endFrequency & 0xff00) >> 8;
    }

    else if (extra_code == RTL_SET_FREQUENCY_STEP)
    {
      screen_printf("   You have selected RTL_FREQUENCY_STEP \n");
      screen_printf("   This sets the frequency of the scan, in __kHz__\n\n");
      screen_printf("   This parameter behaves somewhat unexpectedly due to the \n");
      screen_printf("   intricacies of RTL-SDR bandwidth and powers of two.\n");
      screen_printf("   An unwise setting can result in a frequency spectrum\n    that is truncated or has uneven spacing.\n");
      screen_printf("   It is therefore highly recommended that you don't touch\n    this unless you know what you're doing. \n");
      screen_printf("   ... Unless you really know what you're doing...\n     (i.e. you have read and understand what RTLd is doing) \n");
      screen_printf("      [ This warning may be removed if the handling code becomes smarter ]  \n\n");
      screen_dialog(resp, 31, "Enter frequency step in __kHz__ [%d] (1-65535) (-1 to cancel)\n", stepFrequency);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if  (1 <=t && t <=65535)
        {
          stepFrequency = t;
        }
        else if (t == -1)
        {
          screen_printf("Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d is outside the range of valid frequency steps (1-65535)\n", t);
          return;
        }
      }

      cmdBytes[0] = stepFrequency & 0xff;
      cmdBytes[1] = (stepFrequency & 0xff00) >> 8;
    }

    else if (extra_code == RTL_SET_GAIN)
    {
      screen_printf("   You have selected RTL_SET_GAIN. ");
      screen_printf("   This program believes that there are %d RTL-SDR's.\n   Hopefully that's true.\n", NUM_RTLSDR);
      screen_printf("   This program indexes the RTL-SDR's according to their serials \n   (e.g. RTL1, RTL2, etc.).\n");
      screen_dialog(resp, 32, "Select the RTL-SDR that you want to set the gain for [%d] (1-%d) (-1 to cancel)\n", gainTarget, NUM_RTLSDR);

      if (resp[0] != '\0')
      {
        t = atoi(resp);
        if (1 <=t && t <= NUM_RTLSDR)
        {
          gainTarget = t-1;
        }
        else if (t == -1)
        {
          screen_printf("Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d is outside the range of valid RTL's (1-%d)\n", t, NUM_RTLSDR);
          return;
        }
      }

      screen_printf("       [[ You are setting the gain for RTL%d  ]] \n\n", gainTarget);
      screen_printf("   The RTL-SDR's have various LNA gains available between 0-50 dB.\n");
      screen_printf("   However, as the input impedance of the units appears to change with gain, \n   we recommend you don't go above 20 dB\n");
      screen_printf("   In fact, this program will limit you to 30 dB \n");
      screen_printf("   The nearest valid gain to the selected gain will be used. \n");
      screen_dialog(resp, 32, "Select the LNA gain you want for RTL%d in dB [%f] (0-30 dB) (-1 to cancel)\n", gainTarget, gain);
      if (resp[0] != '\0')
      {
        real = atof(resp);

        if ( 0<= real <= 30) //soft enforce limit here
        {
          gain = real;
        }
        else if (real == -1)
        {
          screen_printf("Cancelled.");
          return;
        }
        else
        {
          screen_printf("%f is outside the range (0-30)\n", real);
          return;
        }

      }

      //convert to short
      sgain = 0.5 + gain * 10;

      //shift over so that index can be packed in
      sgain <<= NBITS_FOR_RTL_INDEX;
      sgain |=  (gainTarget & ( ~(0xffff << NBITS_FOR_RTL_INDEX)));

      cmdBytes[0] = sgain & 0xff;
      cmdBytes[1] = (sgain & 0xff00) >> 8;
    }
    else if (extra_code == RTL_SET_GRACEFUL_TIMEOUT)
    {
      screen_printf("[ You have selected RTL_SET_GRACEFUL_TIMEOUT  ]\n");
      screen_printf("   This controls the maximum time RTLd will wait for a scan to complete..\n");
      screen_printf("   After this time, a SIGTERM will  sent which should truncate the scan \n");
      screen_dialog(resp, 31, "Enter graceful timeout  (0-65535) [%d]  (-1 to cancel)\n\n", gracefulTimeout);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=65535)
        {
          gracefulTimeout = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("Not a valid option: %s \n", resp);
          return;
        }

      }

      cmdBytes[0] = gracefulTimeout & 0xff;
      cmdBytes[1] = (gracefulTimeout & 0xff00) >> 8;
    }
    else if (extra_code == RTL_SET_FAIL_TIMEOUT)
    {
      screen_printf("[ You have selected RTL_SET_FAILTIMEOUT  ]\n");
      screen_printf("   This controls how long we allow a scan to gracefully terminate .\n");
      screen_printf("   After this time, a SIGKILL will sent which destroy the scan\n");
      screen_dialog(resp, 31, "Enter fail timeout  (0-65535) [%d]  (-1 to cancel)\n\n", failTimeout);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=65535)
        {
          failTimeout = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("Not a valid option: %s \n", resp);
          return;
        }

      }

      cmdBytes[0] = failTimeout & 0xff;
      cmdBytes[1] = (failTimeout & 0xff00) >> 8;
    }

    else if (extra_code == RTL_SET_MAX_FAIL)
    {
      screen_printf("[ You have selected RTL_SET_MAX_FAIL ]\n");
      screen_printf("   This controls how many times we allow an RTL to fail before giving up \n");
      screen_dialog(resp, 31, "Enter max fails  (0-65535) [%d]  (-1 to cancel)\n\n", failThreshold);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=65535)
        {
          failThreshold = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("Not a valid option: %s \n", resp);
          return;
        }

      }

      cmdBytes[0] = failThreshold & 0xff;
      cmdBytes[1] = (failThreshold & 0xff00) >> 8;
    }

    else if (extra_code == RTL_SET_DISABLED)
    {
      screen_printf("   You have selected RTL_SET_DISABLED. ");
      screen_printf("   This program thinks there are %d RTL-SDR's.\n   Hopefully that's true.\n", NUM_RTLSDR);
      screen_printf("   This program indexes the RTL-SDR's according to their serials \n   (e.g. RTL1, RTL2, etc.).\n");
      screen_dialog(resp, 32, "Select the RTL-SDR that you want to toggle disable [%d] (1-%d) (-1 to cancel)\n", disable_index, NUM_RTLSDR);

      if (resp[0] != '\0')
      {
        t = atoi(resp);
        if (1 <=t && t <= NUM_RTLSDR)
        {
          disable_index = t-1;
        }
        else if (t == -1)
        {
          screen_printf("Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d is outside the range of valid RTL's (1-%d)\n", t, NUM_RTLSDR);
          return;
        }
      }

      screen_printf("       [[ You are toggling RTL%d  ]] \n\n", disable_index+1);
      screen_printf("   You are setting the disabled state, so 1 disables, 0 enables\n");
      screen_dialog(resp, 32, "Select if you you want to disable RTL%d  [%d]  (-1 to cancel)\n", disable_index+1, disable_value);
      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if ( 0<= t <= 1) //soft enforce limit here
        {
          disable_value = t;
        }
        else if (t == -1)
        {
          screen_printf("Cancelled.");
          return;
        }
        else
        {
          screen_printf("%d is outside the range (0-30)\n", t);
          return;
        }

      }

      //convert to short

      cmdBytes[0] = disable_index;
      cmdBytes[1] = disable_value;



    }

    //Build and send the command

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extra_code;
    int ind=0;
    for(ind=0;ind<2;ind++)
    {
      Curcmd[4+2*ind]=ind+2;
      Curcmd[5+2*ind]=cmdBytes[ind];
    }

    Curcmdlen = 8;
    set_cmd_log("%d; RTLd command %d (%d %d)", cmdCode,extra_code,cmdBytes[0],cmdBytes[1]);
    sendcmd(Fd, Curcmd, Curcmdlen);
  }
}

#define NUM_TUFF_NOTCHES 3
#define NUM_RFCM 4
static const unsigned int notch_freqs[NUM_TUFF_NOTCHES] = { 260,360,450 };
static const unsigned char notch_default_start[NUM_TUFF_NOTCHES] = { 0,16,16 };
static const unsigned char notch_default_end[NUM_TUFF_NOTCHES] = { 15,16,16 };

static void TUFFD_COMMAND(cmdCode)
{
  char resp[32];
  char extra_code = 1;
  char cmdBytes[6];
  unsigned char start[NUM_TUFF_NOTCHES];
  unsigned char end[NUM_TUFF_NOTCHES];
  int inotch;

  memcpy(start, notch_default_start, sizeof(start));
  memcpy(end, notch_default_end, sizeof(end));

  int t;

  screen_printf("Enter Tuffd Command (1-11): \n\n");
  screen_printf(" 1. TUFF_SET_NOTCH                            ---  Set notches by phi sector\n");
  screen_printf(" 2. TUFF_SEND_RAW                             ---  (ADVANCED) send a raw command\n");
  screen_printf(" 3. TUFF_SET_SLEEP_AMOUNT                     ---  Set sleep amount \n");
  screen_printf(" 4. TUFF_READ_TEMPERATURE                     ---  Toggle temperature readout \n");
  screen_printf(" 5. TUFF_TELEM_EVERY                          ---  Modify telemetry interval \n");
  screen_printf(" 6. TUFF_SET_TELEM_AFTER_CHANGE               ---  Toggle post-change telemetry \n");
  screen_printf(" 7. TUFF_ADJUST_ACCORDING_TO_HEADING          ---  (EXPERIMENTAL) enable heading servo\n");
  screen_printf(" 8. TUFF_DEGREES_FROM_NORTH_TO_NOTCH          ---  (EXPERIMENTAL) heading servo width\n");
  screen_printf(" 9. TUFF_SLOPE_THRESHOLD_TO_NOTCH_NEXT_SECTOR ---  (EXPERIMENTAL++)\n");
  screen_printf(" 10. TUFF_PHI_SECTOR_OFFSET_FROM_NORTH        ---  (EXPERIMENTAL) phi sector offset from north \n");
  screen_printf(" 11. TUFF_MAX_HEADING_AGE                     ---  (EXPERIMENTAL) oldest allowed heading for servo\n");
  screen_printf(" 12. TUFF_SET_CAPS_ON_STARTUP                 ---  (EXPERIMENTAL) Enable changing of TUFF caps\n");
  screen_printf(" 13. TUFF_SET_CAP_VALUE                       ---  (EXPERIMENTAL) Change TUFF cap values\n");
  screen_dialog(resp,31, "Select command [%d] (-1 to cancel)\n", extra_code);


  if (resp[0]!='\0')
  {
    t = atoi(resp);

    if ( 1 <=t && t <= 13)
    {
      extra_code = t;
    }
    else if (t == -1)
    {
      screen_printf("Cancelled.\n");
      return;
    }
    else
    {
      screen_printf("Not a valid command.\n");
      return;
    }


    if (extra_code == TUFF_SET_NOTCH)
    {

      screen_printf(" [ You have selected TUFF_SET_NOTCH ]\n");
      screen_printf("   This program will now ask you for start and end phi sectors for each notch.\n");
      screen_printf("   To disable the notch for all phi-sectors, type \"disable\" for the start.\n");
      screen_printf("   For the purposes of this program, the phi-sectors are 1-indexed. \n");
      screen_printf("   As a reminder, the notches are: \n\n");
      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        screen_printf("      notch %d : ~%d MHz\n", inotch, notch_freqs[inotch]);
      }

      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        screen_dialog(resp, 31,  "\nEnter start sector for notch %d (~%d MHz) [%d] (1-16 or disable) (-1 cancels)\n",inotch, notch_freqs[inotch], start[inotch]);

        if (resp[0]!='\0')
        {
          char * disabled = strcasestr(resp,  "disable");
          if (disabled)
          {
            start[inotch] = 16;
            end[inotch] = 16;
            continue;
          }

          t = atoi(resp);

          if ( 1<=t && t<=16)
          {
            start[inotch] = t-1;
          }
          else if (t == -1)
          {
            screen_printf(" Cancelled! \n");
            return;
          }
          else
          {
            screen_printf (" Bad input: %s... try again! \n", resp);
            inotch--;
            continue;
          }
        }

        screen_dialog(resp, 31,  "\nEnter end phi sector for notch %d (~%d MHz) [%d] (1-16) (-1 to cancel)\n",inotch, notch_freqs[inotch], end[inotch]);
        if (resp[0]!='\0')
        {
          t = atoi(resp);

          if ( 1<=t && t<=16)
          {
            end[inotch] = t-1;
          }
          else if (t == -1)
          {
            screen_printf(" Cancelled! \n");
            return;
          }
          else
          {
            screen_printf (" Bad input: %s... try again! \n", resp);
            inotch--;
            continue;
          }
        }
      }


      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        cmdBytes[2*inotch] = start[inotch];
        cmdBytes[2*inotch+1] = end[inotch];
      }
    }

    else if (extra_code == TUFF_SEND_RAW)
    {
      int irfcm = 0;
      int tuff_stack = 0;
      short cmd = 0;

      screen_printf(" [ You have selected TUFF_SEND_RAW ]\n\n");
      screen_printf("   This command is meant for ADVANCED USERS ONLY   \n\n");
      screen_printf("   Familiarize yourself with the tuff firmware prior to attempting this command. \n");
      screen_printf("   It is at https://github.com/barawn/tuff-slave-usi/ \n\n");
      screen_printf("   If you don't know how to read that, find someone who does. \n");

      screen_dialog(resp, 31, "Please enter the target RFCM [%d] (0-%d) (-1 to cancel)", irfcm, NUM_RFCM -1 );
      if (resp[0] != '\0')
      {
        t = atoi(resp);
        if (0 <= t && t <=NUM_RFCM)
        {
          irfcm =t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled\n");
          return;
        }
        else
        {
          screen_printf("  Bad value: %d\n", t);
          return;
        }
      }


      screen_dialog(resp, 31, "Please enter the target Tuff Stack [%d] (0-1) (-1 to cancel)", tuff_stack);

      if (resp[0] != '\0')
      {
        t = atoi(resp);
        if (0 <= t && t <=1)
        {
          tuff_stack =t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled\n");
          return;
        }
        else
        {
          screen_printf("  Bad value: %d\n", t);
          return;
        }
      }

      screen_dialog(resp,31, "Please enter the raw command. [0x%x] (0x0000 - 0xFFFF) (-1 to cancel)\n", cmd);

      if (resp[0] != '\0')
      {
        char * endptr;
        // set errno to zero
        errno = 0;
        t = strtol(resp, &endptr, 0);

        if (t == -1)
        {
          screen_printf("Cancelled\n");
          return;
        }
        else if (!(*resp != '\0' && *endptr == '\0' ) || errno || t > 0xffff || t < 0 )
        {
          screen_printf(" Bad value: %s\n", resp);
          return;
        }
        else
        {
          cmd = t;
        }

        cmdBytes[0] =  (irfcm << 1) | (tuff_stack & 1);
        cmdBytes[1] =  cmd & 0xff;
        cmdBytes[2] = (cmd & 0xff00) >> 8;

      }
    }
    else if (extra_code == TUFF_SET_SLEEP_AMOUNT)
    {
      char sleep_amt = 1;
      screen_printf("[ You have selected TUFF_SET_SLEEP_AMOUNT  ]\n");
      screen_printf("   This controls the length that Tuffd spends sleeping. \n");
      screen_printf("   Changing other parameters will wake raise signals that stir Tuffd\n");
      screen_printf("   from its slumber, so, modulo rare but theoretically possible race\n");
      screen_printf("   conditions, this mostly affects the rate of temperature reading.\n");
      screen_printf("   Time is in seconds\n");
      screen_dialog(resp, 31, "Enter TUFF sleep amount (0-255) [%d]  (-1 to cancel)\n", sleep_amt);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=255)
        {
          sleep_amt = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d not a valid option\n", t);
          return;
        }

      }

      cmdBytes[0] = sleep_amt;
    }

    else if (extra_code == TUFF_SET_READ_TEMPERATURE)
    {
      char rdtemp = 1;
      screen_printf("[ You have selected TUFF_SET_READ_TEMPERATURE  ]\n");
      screen_printf("   This is used to toggle Tuffd temperature readings. \n");
      screen_printf("   Since it takes some time to communicate to the Tuff, \n");
      screen_printf("   it's possible that you may want to temporarily disable this occassionally. \n");
      screen_dialog(resp, 31, "Enter whether the TUFF should read temperatures (0-1) [%d]  (-1 to cancel)\n", rdtemp);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=1)
        {
          rdtemp = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d not a valid option\n", t);
          return;
        }

      }

      cmdBytes[0] = rdtemp;
    }

    else if (extra_code == TUFF_SET_TELEM_EVERY)
    {
      char telem_every = 1;
      screen_printf("[ You have selected TUFF_SET_TELEM_EVERY  ]\n");
      screen_printf("   This is used to set the Tuffd telemetry interval. \n");
      screen_dialog(resp, 31, "Enter Tuffd telemetry interval (0-255) [%d]  (-1 to cancel)\n", telem_every);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=255)
        {
          telem_every = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d not a valid option\n", t);
          return;
        }

      }

      cmdBytes[0] = telem_every;
    }
    else if (extra_code == TUFF_SET_TELEM_AFTER_CHANGE)
    {
      char telem_after = 1;
      screen_printf("[ You have selected TUFF_SET_TELEM_AFTER_CHANGE  ]\n");
      screen_printf("   This determines if Tuffd will telemeter after every change. \n");
      screen_dialog(resp, 31, "Enter Tuffd telemetry interval (0-1) [%d]  (-1 to cancel)\n", telem_after);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=1)
        {
          telem_after = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d not a valid option\n", t);
          return;
        }

      }

      cmdBytes[0] = telem_after;
    }


    else if (extra_code == TUFF_ADJUST_ACCORDING_TO_HEADING)
    {
      char adjust[3];
      memset(adjust,0,sizeof(adjust));

      screen_printf("[ You have selected TUFF_ADJUST_ACCORDING_TO_HEADING  ]\n");
      screen_printf("   This option is marked EXPERIMENTAL.  \n");
      screen_printf("   That means you probably should't use it.   \n");
      screen_printf("   It might set your house on fire. Who knows? It's experimental.   \n\n");
      screen_printf("   This command enables servo of which notches are enabled based on heading.\n");
      screen_printf("   It has only been lightly tested and could result in strange notch behavior.\n");
      screen_printf("   Especially if the GPS's are becoming erratic.\n");


      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        screen_dialog(resp, 31, "Should the notch %d be servoed? (0-1) [%d]  (-1 to cancel)\n", inotch, adjust[inotch]);
        if (resp[0] !=0)
        {
          t = atoi(resp);
          if (0 <= t && t <=1)
          {
            adjust[inotch] = t;
          }

          else if (t == -1)
          {
            screen_printf("Cancelled\n");
            return;
          }
          else
          {
            screen_printf("%d not a valid option\n");
            inotch--;
          }
        }

        memcpy(cmdBytes, adjust, sizeof(adjust));
      }
    }
    else if (extra_code == TUFF_DEGREES_FROM_NORTH_TO_NOTCH)
    {
      char adjust[3];

      screen_printf("[ You have selected TUFF_DEGREES_FROM_NORTH_TO_NOTCH  ]\n");
      screen_printf("   This option is marked EXPERIMENTAL.  \n");
      screen_printf("   That means you probably should't use it.   \n");
      screen_printf("   It might set your chair on fire. Who knows? It's experimental.   \n\n");
      screen_printf("   This command sets how many degrees from north phi sectors have this notch applied\n");
      screen_printf("   It has only been lightly tested and could result in strange notch behavior.\n");
      screen_printf("   Especially if the GPS's are becoming erratic.\n");

      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        adjust[inotch] = 90;
        screen_dialog(resp, 31, "How many degrees for notch %d? (0-255) [%d]  (-1 to cancel)\n", inotch, adjust[inotch]);
        if (resp[0] !=0)
        {
          t = atoi(resp);
          if (0 <= t && t <=255)
          {
            adjust[inotch] = t;
          }

          else if (t == -1)
          {
            screen_printf("Cancelled\n");
            return;
          }
          else
          {
            screen_printf("%d not a valid option\n");
            inotch--;
          }
        }
        memcpy(cmdBytes, adjust, sizeof(adjust));
      }
    }

    else if (extra_code == TUFF_SLOPE_THRESHOLD_TO_NOTCH_NEXT_SECTOR)
    {
      char adjust[3];

      screen_printf("[ You have selected TUFF_SLOPE_THRESHOLD_TO_NOTCH_NEXT_SECTOR  ]\n");
      screen_printf("   This option is marked EXPERIMENTAL++.  \n");
      screen_printf("   That means you probably definitely should't use it.   \n");
      screen_printf("   It might set your hair on fire. Who knows? It's experimental.   \n\n");
      screen_printf("   This command sets the slope threshold to notch the next phi sector in rotation direction..\n");
      screen_printf("   It's probably not that stable. Also, because heading is analyzed after each sleep, it's optimal value depends on sleepAmount.\n");
      screen_printf("   It has only been lightly tested and could result in strange notch behavior.\n");
      screen_printf("   Especially if the GPS's are becoming erratic.\n");

      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        adjust[inotch] = 20;
        screen_dialog(resp, 31, "Slope threshold in deg /s for notch %d? (0-255) [%d]  (-1 to cancel)\n", inotch, adjust[inotch]);
        if (resp[0] !=0)
        {
          t = atoi(resp);
          if (0 <= t && t <=255)
          {
            adjust[inotch] = t;
          }

          else if (t == -1)
          {
            screen_printf("Cancelled\n");
            return;
          }
          else
          {
            screen_printf("%d not a valid option\n");
            inotch--;
          }
        }
        memcpy(cmdBytes, adjust, sizeof(adjust));
      }
    }

    else if (extra_code == TUFF_PHI_SECTOR_OFFSET_FROM_NORTH)
    {
      char adjust[3];

      screen_printf("[ You have selected TUFF_TUFF_PHI_SECTOR_OFFSET_FROM_NORTH  ]\n");
      screen_printf("   This option is marked EXPERIMENTAL  \n");
      screen_printf("   That means you probably should't use it.   \n");
      screen_printf("   It might set your pants on fire. Who knows? It's experimental.   \n\n");
      screen_printf("   This command sets the offset of the GPS heading from North\n");
      screen_printf("   This might be useful if you want to mask out a specific satellite slightly off north.");
      screen_printf("   It has only been lightly tested and could result in strange notch behavior.\n");
      screen_printf("   Especially if the GPS's are becoming erratic.\n");
      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        adjust[inotch] = 45;
        screen_dialog(resp, 31, "Offset from north in deg for notch %d? (0-255) [%d]  (-1 to cancel)\n", inotch, adjust[inotch]);
        if (resp[0] !=0)
        {
          t = atoi(resp);
          if (0 <= t && t <=45)
          {
            adjust[inotch] = t;
          }

          else if (t == -1)
          {
            screen_printf("Cancelled\n");
            return;
          }
          else
          {
            screen_printf("%d not a valid option\n");
            inotch--;
          }
        }
        memcpy(cmdBytes, adjust, sizeof(adjust));
      }
    }
    else if (extra_code == TUFF_MAX_HEADING_AGE)
    {
      char max_age = 120;
      screen_printf("   This option is marked EXPERIMENTAL  \n");
      screen_printf("   That means you probably should't use it.   \n");
      screen_printf("   It might set your cat on fire. Who knows? It's experimental.   \n\n");
      screen_printf("   This command sets the maximum age of a GPS heading that can be used. 0 means no max age.\n");

      screen_dialog(resp, 31, "Enter max heading age (0-255) [%d]  (-1 to cancel)\n", max_age);

      if (resp[0] != '\0')
      {
        t = atoi(resp);

        if (0<= t && t <=255)
        {
          max_age = t;
        }
        else if (t == -1)
        {
          screen_printf(" Cancelled.\n");
          return;
        }
        else
        {
          screen_printf("%d not a valid option\n", t);
          return;
        }

      }

      cmdBytes[0] = max_age;
    }
    else if (extra_code == TUFF_SET_CAPS_ON_STARTUP)
    {
      char adjust[3];
      memset(adjust,0,sizeof(adjust));

      screen_printf("[ You have selected TUFF_SET_CAPS_ON_STARTUP ]\n");
      screen_printf("   This option is marked EXPERIMENTAL.  \n");
      screen_printf("   That means you probably should't use it.   \n");
      screen_printf("   It might set your beard on fire. Even if you don't have a beard. Who knows? It's experimental.   \n\n");
      screen_printf("   This command enables setting of TUFF cap values on Tuffd Startup from the config array.\n");
      screen_printf("   It has only been lightly tested.\n");
      screen_printf("   This won't take effect until the next Tuffd restart.\n");


      for (inotch = 0; inotch < NUM_TUFF_NOTCHES; inotch++)
      {
        screen_dialog(resp, 31, "Should the caps for notch %d be read from config (0-1) [%d]  (-1 to cancel)\n", inotch, adjust[inotch]);
        if (resp[0] !=0)
        {
          t = atoi(resp);
          if (0 <= t && t <=1)
          {
            adjust[inotch] = t;
          }

          else if (t == -1)
          {
            screen_printf("Cancelled\n");
            return;
          }
          else
          {
            screen_printf("%d not a valid option\n");
            inotch--;
          }
        }

        memcpy(cmdBytes, adjust, sizeof(adjust));
      }
    }
    else if (extra_code == TUFF_SET_CAP_VALUE)
    {
      char notch, rfcm, chan, cap; 
      notch = 2; 
      rfcm = -2; 
      chan = -2; 
      cap = 16; 
      screen_printf("[ You have selected TUFF_SET_CAP_VALUE ]\n");
      screen_printf("   This option is marked EXPERIMENTAL.  \n");
      screen_printf("   That means you probably should't use it.   \n");
      screen_printf("   It might set your shoes on fire. Who knows? It's experimental.   \n\n");
      screen_printf("   This command sets the TUFF cap config arrays that Tuffd will read on startup\n");
      screen_printf("   These are only used if TUFF_SET_CAPS_ON_STARTUP is enabled for a notch\n");
      screen_printf("   and only take effect on the next Tuffd restart\n");

      screen_dialog(resp, 31, "Which notch would you like to modify [%d] (0-2) (-1 to cancel, -2 for all)\n", notch);
      if (resp[0] !=0)
      {
        t = atoi(resp);
        if (0 <= t && t <=2)
        {
          notch = t; 
        }

        else if (t == -1)
        {
          screen_printf("Cancelled\n");
          return;
        }
        else if (t == -2)
        {
          notch = -1; 
        }
        else
        {
          screen_printf("%d not a valid option. Aborting.\n"); 
          return; 
        }
      }
      screen_dialog(resp, 31, "Which rfcm would you like to modify [%d] (0-3) (-1 to cancel, -2 for all)\n", rfcm);
      if (resp[0] !=0)
      {
        t = atoi(resp);
        if (0 <= t && t <=3)
        {
          rfcm = t;
        }

        else if (t == -1)
        {
          screen_printf("Cancelled\n");
          return;
        }
        else if (t == -2)
        {
          rfcm = -1; 
        }
        else
        {
          screen_printf("%d not a valid option. Aborting.\n"); 
          return; 
        }
      }
      screen_dialog(resp, 31, "Which chan would you like to modify [%d] (0-3) (-1 to cancel, -2 for all)\n", chan);
      if (resp[0] !=0)
      {
        t = atoi(resp);
        if (0 <= t && t <=23)
        {
         chan = t;
        }

        else if (t == -1)
        {
          screen_printf("Cancelled\n");
          return;
        }
        else if (t == -2)
        {
          chan = -1; 
        }
        else
        {
          screen_printf("%d not a valid option. Aborting.\n"); 
          return; 
        }
      }
      screen_dialog(resp, 31, "Which should the capacitor value be? [%d] (0-31) (-1 to cancel)\n", cap );
      if (resp[0] !=0)
      {
        t = atoi(resp);
        if (0 <= t && t <=31)
        {
         cap = t;
        }

        else if (t == -1)
        {
          screen_printf("Cancelled\n");
          return;
        }
        else
        {
          screen_printf("%d not a valid option. Aborting.\n"); 
          return; 
        }
      }


      cmdBytes[0] = notch; 
      cmdBytes[1] = rfcm; 
      cmdBytes[2] = chan; 
      cmdBytes[3] = cap; 

    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extra_code;
    int ind=0;
    for(ind=0;ind<6;ind++)
    {
      Curcmd[4+2*ind]=ind+2;
      Curcmd[5+2*ind]=cmdBytes[ind];
    }

    Curcmdlen = 16;
    set_cmd_log("%d; Tuffd command %d (%d %d %d %d %d %d)", cmdCode,extra_code,cmdBytes[0],cmdBytes[1], cmdBytes[2], cmdBytes[3], cmdBytes[4], cmdBytes[5]);
    sendcmd(Fd, Curcmd, Curcmdlen);

  } //check that response was sensical
}


void
clr_cmd_log(void)
{
    Logstr[0] = '\0';
}

void
set_cmd_log(char *fmt, ...)
{
  get_next_cmd_count();
  va_list ap;
  if (fmt != NULL) {
    va_start(ap, fmt);
    sprintf(Logstr, "CMD ");
    vsnprintf(Logstr+4, LOGSTRSIZE-4, fmt, ap);
    va_end(ap);
  }
}

void
cmd_log(void)
{
    log_out(Logstr);
}

int
log_init(char *filename, char *msg)
{
    snprintf(Logfilename, LOGSTRSIZE, "%s", filename);
    Logfp = fopen(Logfilename, "a");
    if (Logfp == NULL) {
	return -1;
    }
    if (msg != NULL) {
	log_out(msg);
    } else {
	log_out("Starting up");
    }
    return 0;
}

void
log_out(char *fmt, ...)
{
    if (fmt != NULL) {
	va_list ap;
	char timestr[64];
	time_t t;

	/* output the date */
	t = time(NULL);
	sprintf(timestr, "%s", ctime(&t));
	timestr[strlen(timestr)-1] = '\0';	// delete \n
	fprintf(Logfp, "%s\t", timestr);

	/* output the message */
	va_start(ap, fmt);
	vfprintf(Logfp, fmt, ap);
	fprintf(Logfp, "\n");
	fflush(Logfp);
	va_end(ap);
    }
}

void
log_close(void)
{
    if (Logfp) {
	log_out("Finished");
	fclose(Logfp);
    }
}

/* check_pidfile - return 0 if no pid file is found, else return the value
 * of the PID found in it.
 */
int
check_pidfile()
{
#define PSIZ 8
    FILE *fp = fopen(PIDFILENAME, "r");
    int pid;
    char pidtxt[PSIZ];

    if (fp == NULL) {
	return 0;
    }

    if (fgets(pidtxt, PSIZ, fp) == NULL) {
	return 0;
    }

    pid = atoi(pidtxt);
    return (pid);
}

/* make_pidfile - create a pid file with our PID in it. Return 0 on
 * success, else -1.
 */
int
make_pidfile()
{
    FILE *fp = fopen(PIDFILENAME, "w");
    if (fp == NULL) {
	return -1;
    }
    fprintf(fp, "%d\n", getpid());
    fclose(fp);
    return 0;
}

void get_next_cmd_count()
{
  FILE *fp = fopen(CMD_COUNT_FILE,"r");
  if (fp == NULL) {
    curCmdNumber=0;
  }
  else {
    fscanf(fp,"%u",&curCmdNumber);
  fclose(fp);

  }
  curCmdNumber++;
  fp = fopen(CMD_COUNT_FILE,"w");
  if (fp == NULL) {
  }
  else {
    fprintf(fp,"%u\n",curCmdNumber);
    fclose(fp);
  }


}

void write_cmd_json_file(char *fmt, ...)
{
  char fileName[FILENAME_MAX];
  sprintf(fileName,"%s/cmd_%u.json",JSON_LOG_DIR,curCmdNumber);
  FILE *fp = fopen(fileName,"w");
  int i=0;
  time_t t;

  /* output the date */
  t = time(NULL);

  fprintf(fp,"{\n");
  fprintf(fp,"\"cmdNumber\": %u,\n",curCmdNumber );
  fprintf(fp,"\"time\": %u,\n",(unsigned int)t );
  fprintf(fp,"\"cmdLink\": %d,\n",(int)Curlink );
  fprintf(fp,"\"cmdRoute\": %d,\n",(int)Curroute);

  if (fmt != NULL) {
    fprintf(fp,"\"cmdLog\": \"");
    va_list ap;
    /* output the message */
    va_start(ap, fmt);
    vfprintf(fp, fmt, ap);
    fprintf(fp, "\",\n");
    fflush(fp);
    va_end(ap);
  }


  fprintf(fp,"\"response\": [%d,%d,%d],\n",(int)ibuf[0],(int)ibuf[1],(int)ibuf[2]);
  fprintf(fp,"\"cmd\":[");
  for(i=0;i<Curcmdlen;i++) {
    if(i>0) fprintf(fp,",");
    fprintf(fp,"%d",(int)Curcmd[i]);
  }
  fprintf(fp,"]\n");
  fprintf(fp,"}\n");
  fclose(fp);
}
