/* cmd - interface to NSBF for sending LDB commands
 *
 * USAGE: cmd [-d] [-l link] [-r route] [-g logfile]
 *
 * Marty Olevitch, June '01, editted KJP 7/05
 * 	12/27/06 Check for another 'cmd' running at startup. (MAO)
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>	/* strlen, strerror */
#include <stdlib.h>	/* malloc */
#include <curses.h>
#include <signal.h>

#include <fcntl.h>
#include <termios.h>

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


int diskBitMasks[DISK_TYPES]={SATABLADE_DISK_MASK,SATAMINI_DISK_MASK,USB_DISK_MASK,PMC_DISK_MASK,NEOBRICK_DISK_MASK};

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
"%c=set_timeout  %c=new_cmd %c=quit  %c=link %c=route %c=show_cmds",
};

#define CMDLEN		25	/* max length of command buffer */
#define LINK_LOS	0
#define LINK_TDRSS	1
#define LINK_HF		2
#define LINK_LOS	0
//#define PORT		"/dev/ttyUSB0"
#define PORT		"/dev/ttyS0"
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

unsigned char Curcmd[32];
int Curcmdlen = 0;
unsigned char Curlink = LINK_TDRSS;
unsigned char Curroute = ROUTE_COMM1;
int Fd;
struct termios Origopts;
long Timeout = 10L;

static short numLines=0;

static short Dir_det =0;
static short Prog_det =0;
static char Which_RFCM =0;
static short Config_det =0;
static short switchConfig =0;
static short Priorit_det =0;
static short CalPulserSwitch =0;
static short disableSatablade=0;
static short disableUsbInt=0;
static short disableUsbExt=0;
static short calPulserAtten =0;
static short cpAttenLoopPeriod =0;
static short cpSwitchLoopPeriod =0;
static short cpOffLoopPeriod =0;
static short calibWritePeriod=0;
static long ADU5PatPer =0;
static long ADU5SatPer =0;
static long ADU5VtgPer =0;
static long G12PPSPer =0;
static long G12PosPer =0;
static float G12Offset =0;
static unsigned short HskPer =0;
static unsigned short HskCalPer =0;
static short HskTelemEvery=0;
static short sendWave=1;
static unsigned short sipThrottle=680;
static short losSendData=0;
static long SoftTrigPer =0;
static short TrigADU5 =0;
static short TrigG12 =0;
static short TrigSoft =0;
static short enableChanServo=0;
static short pidGoal=0;
static short pedestalRun=0;
static short thresholdRun=0;
static unsigned long antTrigMask=0;
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
static char Logfilename[LOGSTRSIZE];
static FILE *Logfp;

#define PIDFILENAME "/tmp/cmd.pid"
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
		snprintf(logname, LOGSTRSIZE, optarg);
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
	    	"Can't make PID file '%s' (%s). Press <ret> to continue.\n",
		PIDFILENAME);

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
    sprintf(progmenu[0], menuformat[0] , TIMEOUT, NEWCMD, QUIT,
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
    char s[512];
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

int
serial_init(void)
{
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

    newopts.c_iflag &= ~(INLCR | ICRNL);
    newopts.c_iflag &= ~(IXON | IXOFF | IXANY);	/* no XON/XOFF */

    newopts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	/* raw input */
    newopts.c_oflag &= ~OPOST;	/* raw output */

    tcsetattr(Fd, TCSANOW, &newopts);
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
#define IBUFSIZE 32
    unsigned char ibuf[IBUFSIZE];
    int n;
    int ret;
    struct timeval t;

    screen_printf("Awaiting NSBF response: ");

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
	log_out("NSBF response; no response after %ld seconds", Timeout);
	return;
    }
    sleep(1);
    n = read(Fd, ibuf, IBUFSIZE);
    if (n > 0) {
	if (ibuf[0] != 0xFA || ibuf[1] != 0xF3) {
	    screen_printf("malformed response!\n%x\t\n",ibuf[0],ibuf[1]);
	    log_out("NSBF response; malformed response!");
	    return;
	}

	if (ibuf[2] == 0x00) {
	    screen_printf("OK\n");
	    log_out("NSBF response; OK");
	} else if (ibuf[2] == 0x0A) {
	    screen_printf("GSE operator disabled science commands.\n");
	    log_out("NSBF response; GSE operator disabled science commands.");
	} else if (ibuf[2] == 0x0B) {
	    screen_printf("routing address does not match selected link.\n");
	    log_out( "NSBF response; routing address does not match link.");
	} else if (ibuf[2] == 0x0C) {
	    screen_printf("selected link not enabled.\n");
	    log_out("NSBF response; selected link not enabled.");
	} else if (ibuf[2] == 0x0D) {
	    screen_printf("miscellaneous error.\n");
	    log_out("NSBF response; miscellaneous error.");
	} else {
	    screen_printf("unknown error code (0x%02x)", ibuf[2]);
	    log_out("NSBF response; unknown error code (0x%02x)", ibuf[2]);
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
    int val[2];
    
    int easyCmdArray[]={1,2,3,132,152,153,154,155,
			156,157,158,159,171,172,173,
			174,175,182,183,210,230,231,235,238,239};

    for (j=0; j<25; j++) {
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
    int val[2];
    
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
    screen_printf("Not implemented\n ");

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
    val=(numLines&0xf);
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
CMD_DISABLE_DISK(int cmdCode)
{
    
    char resp[32];
    short det;
    short v;
    screen_printf("You may be about to do something very bad\n");
    screen_printf("Only disable or enable disks if you are really certain of what is going on\n");
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
	screen_printf("0: Satablade            1: Satamini\n");
	screen_printf("2: USB     3: Neobrick\n");
	screen_printf("4: PMC Drive\n");
	screen_dialog(resp,31,"Which disk (-1, to cancel)\n");
    
	if(resp[0] != '\0') {
	    v = atoi(resp);	
	    if (0 <= v && v <= 4) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-4, not %d.\n", v);
		return;
	    }       
	} else return;
    }
    else {
	screen_printf("0: Satablade            1: Satamini\n");
	screen_printf("2: USB     3: Neobrick\n");
	screen_printf("4: PMC Drive\n");
	screen_dialog(resp,31,"Which disk to add to mask (-1, to cancel) [mask: %#x]",diskBitMask);
	if(resp[0] != '\0') {
	    v = atoi(resp);	
	    if (0 <= v && v <= 4) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-4, not %d.\n", v);
		return;
	    }       
	} else return;

	while(1) {
	    screen_printf("0: Satablade            1: Satamini\n");
	    screen_printf("2: USB     3: Neobrick\n");
	    screen_printf("4: PMC Drive\n");
	    screen_dialog(resp,31,"Which disk to add? (5 to send, -1 to cancel) [mask: %#x]",diskBitMask);
	    if(resp[0] != '\0') {
		v = atoi(resp);	
		if (0 <= v && v <= 4) {
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
    Curcmd[7] = ((diskBitMask&0xff00)<<8);
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
    set_cmd_log("%d; Tail last %d lines of /var/log/messages.", cmdCode, ADU5SatPer);
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
    set_cmd_log("%d; Tail last %d lines of /var/log/anita.log.", cmdCode, ADU5SatPer);
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

    
    screen_printf("1. Acqd       6. GPSd\n");
    screen_printf("2. Archived   7. Hkd\n");
    screen_printf("3. Calibdd    8. LOSd\n");
    screen_printf("4. Cmdd       9. Monitord\n");
    screen_printf("5. Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd\n");
    screen_printf("12. All of the above\n");
    screen_dialog(resp, 31, "Kill which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 12) {
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
	    screen_printf("Not allowed\n");
	    return;
	    //            Prog_det = CMDD_ID_MASK;
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
	    screen_printf("Not allowed\n");
	    return;
          case 12:
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
	    screen_printf("Value must be 1-12, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xf00)>>8); 
    Curcmdlen = 6;
    set_cmd_log("%d; Program  %d killed.", cmdCode, Prog_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}




static void
CMD_KILL_PROGS(int cmdCode)
{
    short det;
    int i;
    char resp[32];
    
    screen_printf("1. Acqd       6. GPSd\n");
    screen_printf("2. Archived   7. Hkd\n");
    screen_printf("3. Calibdd    8. LOSd\n");
    screen_printf("4. Cmdd       9. Monitord\n");
    screen_printf("5. Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd\n");
    screen_printf("12. All of the above\n");
    screen_dialog(resp, 31, "Kill which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 12) {
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
	    screen_printf("Not allowed\n");
	    return;
	    //            Prog_det = CMDD_ID_MASK;
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
	       // Prog_det = SIPD_ID_MASK;
	    screen_printf("Not allowed\n");
	    return;
          case 12:
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
	    screen_printf("Value must be 1-12, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xf00)>>8); 
    Curcmdlen = 6;
    set_cmd_log("%d; Program  %d killed.", cmdCode, Prog_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CMD_RESPAWN_PROGS(int cmdCode)
{
    short det;
    int i;
    char resp[32];

    screen_printf("1. Acqd       6. GPSd\n");
    screen_printf("2. Archived   7. Hkd\n");
    screen_printf("3. Calibdd    8. LOSd\n");
    screen_printf("4. Cmdd       9. Monitord\n");
    screen_printf("5. Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd\n");
    screen_printf("12. All of the above\n");
    screen_dialog(resp, 31, "Respawn which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 12) {
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
	      //Prog_det = CMDD_ID_MASK;
	      //break;
	    screen_printf("Not allowed\n");
	    return;
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
          case 12:
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
	    screen_printf("Value must be 1-12, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xf00)>>8); 
    Curcmdlen = 6;
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


    screen_printf("1. Acqd       6. GPSd\n");
    screen_printf("2. Archived   7. Hkd\n");
    screen_printf("3. Calibdd    8. LOSd\n");
    screen_printf("4. Cmdd       9. Monitord\n");
    screen_printf("5. Eventd     10. Prioritizerd\n");
    screen_printf("11. SIPd\n");
    screen_printf("12. All of the above\n");
    screen_dialog(resp, 31, "Start which daemon? (-1 to cancel) [%d] ",
	Prog_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 12) {
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
	      //Prog_det = CMDD_ID_MASK;
	      //break;
	    screen_printf("Not allowed\n");
	    return;
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
          case 12:
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
	    screen_printf("Value must be 1-12, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Prog_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Prog_det&0xf00)>>8); 
    Curcmdlen = 6;
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
		  "Satablade (0) or Satamini (1), -1 to cancel) [%d] ",
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
			     || (tempVal <= 4 && bladeOrMini==1))) {
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
	screen_printf("0: Satablade            1: Satamini\n");
	screen_printf("2: USB     3: Neobrick\n");
	screen_printf("4: PMC Drive\n");
	screen_dialog(resp,31,"Which disk (-1, to cancel)\n");
    
	if(resp[0] != '\0') {
	    v = atoi(resp);	
	    if (0 <= v && v <= 4) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-4, not %d.\n", v);
		return;
	    }       
	} else return;
    }
    else {
	screen_printf("0: Satablade            1: Satamini\n");
	screen_printf("2: USB     3: Neobrick\n");
	screen_printf("4: PMC Drive\n");
	screen_dialog(resp,31,"Which disk to add to mask (-1, to cancel) [mask: %#x]",diskBitMask);
	if(resp[0] != '\0') {
	    v = atoi(resp);	
	    if (0 <= v && v <= 4) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-4, not %d.\n", v);
		return;
	    }       
	} else return;

	while(1) {
	    screen_printf("0: Satablade            1: Satamini\n");
	    screen_printf("2: USB     3: Neobrick\n");
	    screen_printf("4: PMC Drive\n");
	    screen_dialog(resp,31,"Which disk to add? (5 to send, -1 to cancel) [mask: %#x]",diskBitMask);
	    if(resp[0] != '\0') {
		v = atoi(resp);	
		if (0 <= v && v <= 4) {
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
    Curcmd[7] = ((diskBitMask&0xff00)<<8);
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
	screen_printf("0: Satablade       1: Satamini\n");
	screen_printf("2: USB             3: Neobrick\n");
	screen_printf("4: PMC Drive\n");
	screen_dialog(resp,31,"Which disk (-1, to cancel)\n");
    
	if(resp[0] != '\0') {
	    v = atoi(resp);	
	    if (0 <= v && v <= 4) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-4, not %d.\n", v);
		return;
	    }       
	} else return;
    }
    else {
	screen_printf("0: Satablade       1: Satamini\n");
	screen_printf("2: USB             3: Neobrick\n");
	screen_printf("4: PMC Drive\n");
	screen_dialog(resp,31,"Which disk to add to mask (-1, to cancel) [mask: %#x]",diskBitMask);
	if(resp[0] != '\0') {
	    v = atoi(resp);	
	    if (0 <= v && v <= 4) {
		diskBitMask = diskBitMasks[v];
	    } else if (v == -1) {
		screen_printf("Cancelled.\n");
		return;
	    } else {
		screen_printf("Value must be 0-4, not %d.\n", v);
		return;
	    }       
	} else return;

	while(1) {
	    screen_printf("0: Satablade    1: Satamini\n");
	    screen_printf("2: USB          3: Neobrick\n");
	    screen_printf("4: PMC Drive\n");
	    screen_dialog(resp,31,"Which disk to add? (5 to send, -1 to cancel) [mask: %#x]",diskBitMask);
	    if(resp[0] != '\0') {
		v = atoi(resp);	
		if (0 <= v && v <= 4) {
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
    Curcmd[7] = ((diskBitMask&0xff00)<<8);
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
    Curcmd[4] = 3;
    Curcmd[5] = (decWord&0xff);
    Curcmd[6] = 4;
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
    Curcmd[6] = 2;
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
    static float decimateFrac=0;

    screen_dialog(resp, 31,
		  "Enter 1 for G12, 2 for ADU5, 3 for Soft (-1 to cancel) [%d] ",
		  whichPPS);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (1 <= v && v <= 2) {
	    whichPPS = v;
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
    Curcmd[3] = whichPPS;
    Curcmd[4] = 2;
    Curcmd[5] = (decWord&0xff);
    Curcmd[6] = 3;
    Curcmd[7] = (decWord&0xff00)>>8;
    Curcmdlen = 8;
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
TURN_GPS_ON(int cmdCode)
{
    if (screen_confirm("Really turn on GPS/Magnetometer")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on GPS/Magnetometer.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_GPS_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off GPS/Magnetometer")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off GPS/Magnetometer.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_RFCM_ON(int cmdCode)
{
    char resp[32];
    short det;        
    char rfcmVal;
    screen_printf("1. RFCM Manifold 1 \n");
    screen_printf("2. RFCM Manifold 2\n");
    screen_printf("3. RFCM Manifold 3\n");
    screen_printf("4. RFCM Manifold 4\n");
    screen_printf("8. All RFCM Manifolds\n");
    screen_dialog(resp, 31, "Which RFCM Manifold to turn ON? (-1 to cancel) [%d] ",
		  Which_RFCM);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 4) {
	    rfcmVal=1<<(det-1);
	} else if (det == 8) {
	    rfcmVal=0xf;      
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-4, or 8, not %d.\n", det);
	    return;
	}
    }

    
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = rfcmVal;
    Curcmdlen = 4;
    screen_printf("\n");
    set_cmd_log("%d; Turn on RFCM with mask %#x.", cmdCode,rfcmVal);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
TURN_RFCM_OFF(int cmdCode)
{
    char resp[32];
    short det;
    char rfcmVal;
    screen_printf("1. RFCM Manifold 1 \n");
    screen_printf("2. RFCM Manifold 2\n");
    screen_printf("3. RFCM Manifold 3\n");
    screen_printf("4. RFCM Manifold 4\n");
    screen_printf("5. All RFCM Manifolds\n");
    screen_dialog(resp, 31, "Which RFCM Manifold to turn OFF? (-1 to cancel) [%d] ",
		  Which_RFCM);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 4) {
	    rfcmVal=1<<(det-1);
	} else if (det == 5) {
	    rfcmVal=0xf;      
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-4, or 8, not %d.\n", det);
	    return;
	}
    }

    
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = rfcmVal;
    Curcmdlen = 4;
    screen_printf("\n");
    set_cmd_log("%d; Turn off RFCM with mask %#x.", cmdCode,rfcmVal);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
TURN_CALPULSER_ON(int cmdCode)
{
    if (screen_confirm("Really turn on CalPulser")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on CalPulser.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_CALPULSER_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off CalPulser")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off CalPulser.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}

static void
TURN_NADIR_ON(int cmdCode)
{
    if (screen_confirm("Really turn on Nadir RFCMs")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on Nadir RFCMs.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_NADIR_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off Nadir RFCMs")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off Nadir RFCMs.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_ALL_ON(int cmdCode)
{
    if (screen_confirm("Really turn on GPS, RFCM, CalPulser and Nadir")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn on GPS, RFCM, CalPulser, and Nadir.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
TURN_ALL_OFF(int cmdCode)
{
    if (screen_confirm("Really turn off GPS, RFCM, CalPulser and Nadir")) {
	Curcmd[0] = 0;
	Curcmd[1] = cmdCode;
	Curcmdlen = 2;
	screen_printf("\n");
	set_cmd_log("%d; Turn off GPS, RFCM, CalPulser and Nadir.", cmdCode);
	sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
    }
}


static void
SET_CALPULSER_SWITCH(int cmdCode)
{
    char resp[32];
    short det;
    short v;
     
    screen_dialog(resp, 31,
	"Set CalPulser to  (0 for loop, 1-4 for ports 1-4, -1 to cancel) [%d] ",
	CalPulserSwitch);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 4) {
	    CalPulserSwitch = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-4, not %d.\n", v);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = CalPulserSwitch;
    Curcmdlen = 4;
    set_cmd_log("%d; Set CalPulser Switch to? %d.", cmdCode, CalPulserSwitch);
    sendcmd(Fd, Curcmd, Curcmdlen);
}



static void
SET_CALPULSER_ATTEN(int cmdCode)
{
    char resp[32];
    short det;
    short v;
     
    screen_printf("Approx. dB's based on UCI measurement\n");
    screen_printf("0. 0dB     4. 18dB\n");
    screen_printf("1. 3dB     5. 22dB\n");
    screen_printf("2. 8dB     6. 28dB\n");
    screen_printf("3. 12dB    7. 33dB\n");
    screen_dialog(resp, 31,
	"Set Cal Pulser Atten to  (0-7, 8 for loop, -1 to cancel) [%d] ",
	calPulserAtten);
    if (resp[0] != '\0') {
	v = atoi(resp);
	if (0 <= v && v <= 8) {
	    calPulserAtten = v;
	} else if (v == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Atten Value must be 0-8, not %d.\n", v);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = calPulserAtten;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Cal Pulser Atten to %d.", cmdCode, calPulserAtten);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CP_ATTEN_LOOP_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;
     
    screen_dialog(resp, 31,
	"Set Attenuator Loop Period in seconds  (0-65535, -1 to cancel) [%d] ",
	cpAttenLoopPeriod);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    cpAttenLoopPeriod = t;
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
    Curcmd[3] = (cpAttenLoopPeriod&0xff);
    Curcmd[4] = 2; 
    Curcmd[5] = ((cpAttenLoopPeriod&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set Attenuator loop period to %d secs.", cmdCode, cpAttenLoopPeriod);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CP_SWITCH_LOOP_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;
     
    screen_dialog(resp, 31,
	"Set RF Switch Loop Period in seconds  (0-65535, -1 to cancel) [%d] ",
	cpSwitchLoopPeriod);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    cpSwitchLoopPeriod = t;
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
    Curcmd[3] = (cpSwitchLoopPeriod&0xff);
    Curcmd[4] = 2; 
    Curcmd[5] = ((cpSwitchLoopPeriod&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set RF switch loop period to %d secs.", cmdCode, cpSwitchLoopPeriod);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CP_PULSER_OFF_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;
     
    screen_dialog(resp, 31,
	"Set CalPulser off (between RF switch loops) period in seconds  (0-65535, -1 to cancel) [%d] ",
	cpOffLoopPeriod);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    cpOffLoopPeriod = t;
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
    Curcmd[3] = (cpOffLoopPeriod&0xff);
    Curcmd[4] = 2; 
    Curcmd[5] = ((cpOffLoopPeriod&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set CalPulser off (between rf switch loops) period to %d secs.", cmdCode, cpOffLoopPeriod);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
CP_CALIB_WRITE_PERIOD(int cmdCode)
{
    char resp[32];
    short det;
    int t;
     
    screen_dialog(resp, 31,
	"Set Calibd Data Write period in seconds  (0-65535, -1 to cancel) [%d] ",
	calibWritePeriod);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 65535) {
	    calibWritePeriod = t;
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
    Curcmd[3] = (calibWritePeriod&0xff);
    Curcmd[4] = 2; 
    Curcmd[5] = ((calibWritePeriod&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Set Calibd data write period to %d secs.", cmdCode, calibWritePeriod);
    sendcmd(Fd, Curcmd, Curcmdlen);
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
    set_cmd_log("%d; Set G12 PPS offset to %d ms.", cmdCode, G12PPSPer);
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
	det[t]=((short)v12[t]*1000.);
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
    set_cmd_log("%d; Set Adu5 %d Cal V12 to {%3.3f,%3.3f,%3.3f}.", cmdCode, whichAdu5,det[0],
		det[1],det[2]);
    sendcmd(Fd, Curcmd, Curcmdlen);

  return;
}

static void 
ADU5_CAL_13(int cmdCode)
{
    float v13[3]={-1.538,1.558,-0.001};
    int t;
    char resp[32];
    short det[3];
    screen_printf("Are you sure you want to do this?.\n");
    screen_dialog(resp, 31,
		  "Press -1, to cancel (later you will have to cntl-c out):  ",NULL);
    if (resp[0] != '\0') {
	t=atoi(resp);
	if(t==-1) return;
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
	det[t]=((short)v13[t]*1000.);
    }
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (det[0]&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((det[0]&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = (det[1]&0xff);
    Curcmd[8] = 4;
    Curcmd[9] = ((det[1]&0xff00)>>8);
    Curcmd[10] = 5;
    Curcmd[11] = (det[2]&0xff);
    Curcmd[12] = 6;
    Curcmd[13] = ((det[2]&0xff00)>>8);
    Curcmdlen = 14;
    set_cmd_log("%d; Set Adu5 Cal V13 to {%3.3f,%3.3f,%3.3f}.", cmdCode, det[0],
		det[1],det[2]);
    sendcmd(Fd, Curcmd, Curcmdlen);



//  screen_printf("Not implemented\n.\n");
  return;
}

static void 
ADU5_CAL_14(int cmdCode)
{
    float v14[3]={1.543,1.553,-0.024};

    int t;
    char resp[32];
    short det[3];
    screen_printf("Are you sure you want to do this?.\n");
    screen_dialog(resp, 31,
		  "Press -1, to cancel (later you will have to cntl-c out):  ",NULL);
    if (resp[0] != '\0') {
	t=atoi(resp);
	if(t==-1) return;
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
	det[t]=((short)v14[t]*1000.);
    }
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (det[0]&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((det[0]&0xff00)>>8);
    Curcmd[6] = 3;
    Curcmd[7] = (det[1]&0xff);
    Curcmd[8] = 4;
    Curcmd[9] = ((det[1]&0xff00)>>8);
    Curcmd[10] = 5;
    Curcmd[11] = (det[2]&0xff);
    Curcmd[12] = 6;
    Curcmd[13] = ((det[2]&0xff00)>>8);
    Curcmdlen = 14;
    set_cmd_log("%d; Set Adu5 Cal V14 to {%3.3f,%3.3f,%3.3f}.", cmdCode, det[0],
		det[1],det[2]);
    sendcmd(Fd, Curcmd, Curcmdlen);

//  screen_printf("Not implemented\n.\n");
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


    screen_printf("1. Acqd.config      6. GPSd.config\n");
    screen_printf("2. Archived.config   7. Hkd.config\n");
    screen_printf("3. Calibd.config    8. LOSd.config\n");
    screen_printf("4. Cmdd.config       9. Monitord.config\n");
    screen_printf("5. Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config\n");
    screen_printf("12. All of the above\n");
    screen_dialog(resp, 31, "Which config File? (-1 to cancel) [%d] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 12) {
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
          case 12:
            Config_det = ALL_ID_MASK;

	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-12, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xf00)>>8); 
    Curcmdlen = 6;
    set_cmd_log("%d; Config  %d sent.", cmdCode, Config_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
DEFAULT_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];


    screen_printf("1. Acqd.config      6. GPSd.config\n");
    screen_printf("2. Archived.config   7. Hkd.config\n");
    screen_printf("3. Calibd.config    8. LOSd.config\n");
    screen_printf("4. Cmdd.config       9. Monitord.config\n");
    screen_printf("5. Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config\n");
    screen_printf("12. All of the above\n");
    screen_dialog(resp, 31, "Which config file to return to default? (-1 to cancel) [%d] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 12) {
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
          case 12:
            Config_det = ALL_ID_MASK;

	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-12, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xf00)>>8); 
    Curcmdlen = 6;
    set_cmd_log("%d; Config  %d set to default.", cmdCode, Config_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
LAST_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];


    screen_printf("1. Acqd.config      6. GPSd.config\n");
    screen_printf("2. Archived.config   7. Hkd.config\n");
    screen_printf("3. Calibd.config    8. LOSd.config\n");
    screen_printf("4. Cmdd.config       9. Monitord.config\n");
    screen_printf("5. Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config\n");
    screen_printf("12. All of the above\n");
    screen_dialog(resp, 31, "Which config file to return to last config? (-1 to cancel) [%d] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 12) {
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
          case 12:
            Config_det = ALL_ID_MASK;

	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-12, not %d.\n", det);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (Config_det&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((Config_det&0xf00)>>8); 
    Curcmdlen = 6;
    set_cmd_log("%d; Config  %d set to last.", cmdCode, Config_det);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SWITCH_CONFIG(int cmdCode)
{
    short det;
    int i;
    char resp[32];


    screen_printf("1. Acqd.config      6. GPSd.config\n");
    screen_printf("2. Archived.config   7. Hkd.config\n");
    screen_printf("3. Calibd.config    8. LOSd.config\n");
    screen_printf("4. Cmdd.config       9. Monitord.config\n");
    screen_printf("5. Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config\n");
    screen_dialog(resp, 31, "Which config file to switch? (-1 to cancel) [%d] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 11) {
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
          case 12:
            Config_det = ALL_ID_MASK;

	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-12, not %d.\n", det);
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
    Curcmd[5] = ((Config_det&0xf00)>>8); 
    Curcmd[6] = 3;
    Curcmd[7] = (switchConfig&0xff); 
    Curcmdlen = 8;
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

    screen_printf("1. Acqd.config      6. GPSd.config\n");
    screen_printf("2. Archived.config   7. Hkd.config\n");
    screen_printf("3. Calibd.config    8. LOSd.config\n");
    screen_printf("4. Cmdd.config       9. Monitord.config\n");
    screen_printf("5. Eventd.config     10. Prioritizerd.config\n");
    screen_printf("11. SIPd.config\n");
    screen_dialog(resp, 31, "Which config file to switch? (-1 to cancel) [%d] ",
	Config_det);
    if (resp[0] != '\0') {
	det = atoi(resp);
	if (1 <= det && det <= 11) {
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
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
	    //	    screen_printf("Not allowed\n");
	    //	    return;
          case 12:
            Config_det = ALL_ID_MASK;

	    break;
          default: break;
	  }
	} else if (det == -1) {
	    screen_printf("Cancelled\n");
	    return;
	} else {
	    screen_printf("Value must be 1-12, not %d.\n", det);
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
    Curcmd[5] = ((Config_det&0xf00)>>8); 
    Curcmd[6] = 3;
    Curcmd[7] = (configNum&0xff); 
    Curcmdlen = 8;
    set_cmd_log("%d; Current coonfig %d saved as %d.", cmdCode, Config_det,configNum);
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
	"Set SURF Trigger on Software Flag, 0 disable, 1 enable, -1 to cancel) [%d] ", TrigSoft);
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
	"Set SURFsoftware trigger period (in seconds) to  (0-255, -1 to cancel) [%d] ",
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
SET_ANT_TRIG_MASK(int cmdCode)
{

    char resp[32];
    short det;
    unsigned long t;
    unsigned long test;
    int fred; 
    int antAdd=-1;
    int allOn=0;
    antTrigMask=0;
    
    screen_dialog(resp, 31,"Add First antenna to mask  (0 for all on)( -1 to cancel) [%d] ", antAdd);
    
    if (resp[0] != '\0') {
	 
	 antAdd=atoi(resp);
	 if(antAdd==0){
	      allOn=1;
	 }
	 else if(antAdd>=1 && antAdd<=32) {
	
	 }
	 
	 else if(antAdd==-1) {
	      screen_printf("Cancelled.\n");
	      return;
	 }
	 else {
	      screen_printf("Not a valid antenna number");
	      return;
	 }
    }
    else { 
	 screen_printf("Cancelled.\n");
	 return;
    }
	 	 
    int bit=(antAdd%4);
    if(bit) bit=4-bit;
    int nibble=7-(antAdd-1)/4;
    
    int bitShift=bit+4*nibble;
    test=(1<<bitShift);
    if(allOn==1){
	 test=0;
    }
    antTrigMask|=test;

    while(1) {
	 antAdd=-1;
	 screen_dialog(resp, 31, 
		       "Add next antenna to mask  ( -1 to cancel, 0 to finish) [%d]",antAdd);
      
	 if (resp[0] != '\0') {
	      antAdd=atoi(resp);
	      if(antAdd==0) break;
	      if(antAdd>=1 && antAdd<=32) {
	  
	      }
	      else {
		   screen_printf("Not a valid antenna number");
		   continue;
	      }
	      if(antAdd==-1) {
		   screen_printf("Cancelled.\n");
		   return;
	      }
	 }
	 else { 
	      screen_printf("Cancelled.\n");
	      return;
	 }
	 bit=(antAdd%4);
	 if(bit) bit=4-bit;
	 nibble=7-(antAdd-1)/4;
    
	 bitShift=bit+4*nibble;
	 test=(1<<bitShift);
	 antTrigMask|=test;
    }
   
      
    if (screen_confirm("Really Set antTrigMask to: %#010x",antTrigMask)) {

      Curcmd[0] = 0;
      Curcmd[1] = cmdCode;
      Curcmd[2] = 1;
      Curcmd[3] = (antTrigMask&0xff);
      Curcmd[4] = 2;
      Curcmd[5] = ((antTrigMask&0xff00)>>8);
      Curcmd[6] = 3;
      Curcmd[7] = ((antTrigMask&0xff0000)>>16);
      Curcmd[8] = 4;
      Curcmd[9] = ((antTrigMask&0xff000000)>>24);
      Curcmdlen = 10;
      set_cmd_log("%d; Set antTrigMask %#x.", cmdCode, antTrigMask);
      sendcmd(Fd, Curcmd, Curcmdlen);
    } else {
	screen_printf("\nCancelled\n");
	return;
    }
     
}


static void
SET_SURF_TRIG_MASK(int cmdCode)
{
    char resp[32];
    short det;
    int t;
    int fred; 

    
    screen_dialog(resp, 31,
	"Which SURF to change trigBandMask  (1-9, -1 to cancel) [%ul] ",
	antTrigMask);
    if (resp[0] != '\0') {
      t=atoi(resp);
      if(t>=1 && t<=9) {
	surfTrigBandSurf=t-1;
      }
      else if(t==-1) {
	screen_printf("Cancelled.\n");
	return;
      }
      else {	
	screen_printf("SURF must be between 1 and 9.\n");
	return;
      }	
    }

    screen_dialog(resp, 31,
	"Hex bitmask for lower 16 channels  (0 - 0xffff, -1 to cancel) [%ul] ",
	antTrigMask);
    if (resp[0] != '\0') {
      t=atoi(resp);
      if(t==-1) {
	screen_printf("Cancelled.\n");
	return;	
      }
      t=strtol(resp,NULL,16);
      if(t>=0 && t<=0xffff) {
	surfTrigBandVal1=t;
      }
      else {	
	screen_printf("SURF must be between 0 and 0xffff (not %#x).\n",t);
	return;
      }	
    }

    screen_dialog(resp, 31,
	"Hex bitmask for upper 16 channels  (0 - 0xffff, -1 to cancel) [%ul] ",
	antTrigMask);
    if (resp[0] != '\0') {
      t=atoi(resp);
      if(t==-1) {
	screen_printf("Cancelled.\n");
	return;	
      }
      t=strtol(resp,NULL,16);
      if(t>=0 && t<=0xffff) {
	surfTrigBandVal2=t;
      }
      else {	
	screen_printf("SURF must be between 0 and 0xffff (not %#x).\n",t);
	return;
      }	
    }



    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (surfTrigBandSurf&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((surfTrigBandVal1&0xff));
    Curcmd[6] = 3;
    Curcmd[7] = ((surfTrigBandVal1&0xff00)>>8);
    Curcmd[8] = 4;
    Curcmd[9] = ((surfTrigBandVal2&0xff));
    Curcmd[10] = 5;
    Curcmd[11] = ((surfTrigBandVal2&0xff00)>>8);
    Curcmdlen = 12;
    set_cmd_log("%d; Take SURF trig band mask %d -- %#x %#x.", cmdCode, surfTrigBandSurf,surfTrigBandVal1,surfTrigBandVal2);
    sendcmd(Fd, Curcmd, Curcmdlen);
}


static void
SET_GLOBAL_THRESHOLD(int cmdCode)
{
    char resp[32];
    short det;
    short t;
     
    screen_dialog(resp, 31,
	"Set Global Threshold  (0 means disable, 1-4095, -1 to cancel) [%d] ",
	globalThreshold);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 4095) {
	    globalThreshold = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 0-4095, not %d.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = (globalThreshold&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((globalThreshold&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting globalThreshold to %d.", cmdCode, globalThreshold);
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
BAND_SCALE_FACTOR(int cmdCode)
{
    char resp[32];
    short det;
    short t;
    float ft;
    unsigned short value;
    
    screen_printf("Use this command to change the scale factor for a single trigger band\n");
    screen_printf("Obviously use with caution\n");    
    screen_dialog(resp, 31,
	"Which SURF, 1-8, to change (-1 to cancel) [%d] ",
	whichSurf+1);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (1 <= t && t <= 8) {
	    whichSurf = t-1;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 1-8, not %d.\n", t);
	    return;
	}
    }   
    screen_dialog(resp, 31,
	"Which DAC, 1-32, to change (-1 to cancel) [%d] ",
	whichDac+1);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (1 <= t && t <= 32) {
	    whichDac = t-1;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Value must be 1-32, not %d.\n", t);
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
    
    value = ((unsigned short) (scaleFactor*1000.));
    //    screen_printf("scaleFactor %f\tvalue %u\n",scaleFactor,value);
       
    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = whichSurf&0xff;
    Curcmd[4] = 2;
    Curcmd[5] = whichDac&0xff;
    Curcmd[6] = 3;
    Curcmd[7] = value&0xff;
    Curcmd[8] = 4;
    Curcmd[9] = ((value&0xff00)>>8);
    Curcmdlen = 10;
    set_cmd_log("%d; Set Band Scale Factor SURF %d, DAC %d, %f.", cmdCode, whichSurf,whichDac,scaleFactor);
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
	if (0 <= t && t <= 200) {
	    megaBytes = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set Acqd Kill threshold to %d.\n", t);
	    return;
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
	if (0 <= t && t <= 200) {
	    megaBytes = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set Ramdisk Data Dump Threshold to %d MB.\n", t);
	    return;
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
    screen_printf("Not implemented yet");
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
    static short satabladeMegaBytes=120;

    screen_dialog(resp, 31,
	"Set Satablade switch threshold in MB (0-255, -1 to cancel) [%d] ", satabladeMegaBytes);
    if (resp[0] != '\0') {
	t = atoi(resp);
	if (0 <= t && t <= 255) {
	    satabladeMegaBytes = t;
	} else if (t == -1) {
	    screen_printf("Cancelled.\n");
	    return;
	} else {
	    screen_printf("Set satablade switch threshold to %d MB.\n", t);
	    return;
	}
    }

    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = satabladeMegaBytes;
    Curcmdlen = 4;
    set_cmd_log("%d; Set Satablade switch threshold %d MB.", cmdCode,satabladeMegaBytes);
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
	    return;
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
	    return;
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
	    return;
	}
    }


    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = enabler;
    Curcmd[3] = (eventRate&0xff);
    Curcmd[4] = 2;
    Curcmd[5] = ((eventRate&0xff00)>>8);
    Curcmdlen = 6;
    set_cmd_log("%d; Setting event rate goal to %f.", cmdCode, eventRate);
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
    screen_printf("Not yet Implemented in cmdSend.\n");


     return;
}


static void
GPSD_EXTRA_COMMAND(cmdCode){
    char resp[32];
    short det;
    short t;
    unsigned char cval=0;
    static short extraCode=0;
    static short whichGps=0;
    static short value=0;
     screen_printf("130 -- GPS_SET_GGA_PERIOD\n");
     screen_printf("131 -- GPS_SET_PAT_TELEM_EVERY\n");
     screen_printf("132 -- GPS_SET_VTG_TELEM_EVERY\n");
     screen_printf("133 -- GPS_SET_SAT_TELEM_EVERY\n");
     screen_printf("134 -- GPS_SET_GGA_TELEM_EVERY\n");
     screen_printf("135 -- GPS_SET_POS_TELEM_EVERY\n");
     screen_dialog(resp, 31,
		   "Select Extra Code (-1 to cancel) [%d] ", extraCode);
     
     if (resp[0] != '\0') {
	 t = atoi(resp);
	 if (129<= t && t <=135) {
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
     


    Curcmd[0] = 0;
    Curcmd[1] = cmdCode;
    Curcmd[2] = 1;
    Curcmd[3] = extraCode;
    Curcmd[4] = 1;
    Curcmd[5] = whichGps;
    Curcmd[6] = 1;
    Curcmd[7] = cval;
    Curcmdlen = 8;
    set_cmd_log("%d; Extra GPS command %d %d to %d.", cmdCode,extraCode,whichGps,value);
    sendcmd(Fd, Curcmd, Curcmdlen);
    
    return;
}

static void
SIPD_CONTROL_COMMAND(cmdCode){
     screen_printf("Not yet Implemented in cmdSend.\n");
     return;
}

static void
LOSD_CONTROL_COMMAND(cmdCode){
    screen_printf("Not yet Implemented in cmdSend.\n");
    return;
}

static void
ACQD_EXTRA_COMMAND(cmdCode){
     screen_printf("Not yet Implemented in cmdSend.\n");
     return;
}

static void
ACQD_RATE_COMMAND(cmdCode){
     screen_printf("Not yet Implemented in cmdSend.\n");
     return;
}

static void
GPS_PHI_MASK_COMMAND(cmdCode){
     screen_printf("Not yet Implemented in cmdSend.\n");
     return;
}

static void
PLAYBACKD_COMMAND(cmdCode){
     screen_printf("Not yet Implemented in cmdSend.\n");
     return;
}


void
clr_cmd_log(void)
{
    Logstr[0] = '\0';
}

void
set_cmd_log(char *fmt, ...)
{
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
