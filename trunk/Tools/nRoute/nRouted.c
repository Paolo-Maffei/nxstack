/*
    NanoStack: MCU software and PC tools for IP-based wireless sensor networking.
		
    Copyright (C) 2006-2007 Sensinode Ltd.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

		Address:
		Sensinode Ltd.
		Teknologiantie 6	
		90570 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <getopt.h>

#include <signal.h>

#include "nRouted.h"
#include "nrp.h"

#include "port.h"

typedef void (*sighandler_t)(int);

extern struct nRdconfig_t nRdconf;
extern FILE *logfp;
extern FILE *serialfp;
extern int haltsystem;

extern struct nRd_conn_table_t *nRd_conn_table;

unsigned char nrp_proto_table[32][64] = {
	{ "IEEE 802.15.4 MAC data" },
	{ "nUDP" },
	{ "6LoWPAN" },
	{ "ZigBee" },
	{ "6LoWPAN ICMP" }
};

unsigned char nrp_addr_table[32][128] = {
	{ "IEEE hardware identifier (48 bits)" },			// 0x00
	{ "802.15.4 device long address (64 bits)" },		// 0x01
	{ "802.15.4 PAN ID (16 bits)" },					// 0x02
	{ "802.15.4 short address (16 bits)" },				// 0x03
	{ "" },												// 0x04		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x05		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x06		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x07		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x08		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x09		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0A		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0B		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0C		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0D		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0E		DO NOT REMOVE THE EMPTY LINES!!!
	{ "" },												// 0x0F		DO NOT REMOVE THE EMPTY LINES!!!
	{ "IP address (string, 6-14 bytes)" },				// 0x10
	{ "IPv4 address (32 bits)" },						// 0x11
	{ "IPv6 address (128 bits)" }						// 0x12
};

#define OPTIONS_STRING "dmp:"
/* long option list */
static struct option long_options[] =
{
  {"daemon", 0, NULL, 'd'},                  
  {"monitor", 0, NULL, 'm'},                 
  {"protocol", 1, NULL, 'p'},
  {0, 0, 0, 0}
};

/**	@file nRouted.c
 *	The nRouted daemon functionality.
 *
 *	This file contains all the initialization functionality that is required for the nRouted daemon
 *	to start, open log files, create lock file and to connect to syslog.
 */

/** The nRouted configuration file.
 *
 *	This file contains all the neccessary information for the nRouted daemon to work properly. If the file
 *	does not exist the daemon will try to use the built-in default parameters. These default parameters are
 *	not even designed to work on all possible distributions. As the development is done mostly on Fedora Core
 *	(currently version 5) Fedora users have the best chances of nRouted working without the config file. The 
 *	format of the file and all the keywords are documented in nRouted Readme.
 *
 */
//#define CONFIG_FILE "/etc/nRoute/nRouted.conf"
#define CONFIG_FILE "nRouted.conf"

/** The nRouted lock file.
 *
 *	When nRouted is started it eventually checks if a lock file exists. This is done so that only one instance
 *	of nRouted is running simultaneously.
 *
 **/
char nRouted_lockfile[64];
#define LOCK_FILE_BASENAME "/tmp/.nRouted-lock-"
#define LOCK_DIR "/tmp/"

int nrouted_daemon=0;
int saved_stdout;

int parse_config(void);
int parse_opts(int count, char* param[]);
void logger(int loglevel, const char *tolog, ...);
void rawlogger(int loglevel, const char *tolog, ...);
void serial_server(void);
void serial_write(void);
int tcp_server(void);
void usage(void);
void sig_interrupt(void);
void sigsegv_interrupt(void);

/** The nRouted main function.
 *
 *	If nRoute is started with parameter -d the parent thread exits (almost) immediately but the daemon keeps
 *	on running in the child process.
 *
 *	@return 0 on success and -1 on error when it failed to create a child process
 *
 */
int main(int argc, char **argv)
{
	pid_t process_id;		//	The pid (Process ID) variable
	pid_t session_id;		//	The sid (Session ID) variable for the child process

	int rval;				//	A variable where some return values are temporarily stored
	struct stat bf;

	pthread_t serial_server_th;
	pthread_t serial_write_th;

	haltsystem = 0;
	monitor_mode = 0;

	nRdconf.protocol	= PROTO_6LOWPAN;

	if (parse_opts(argc, argv) != 0)
	{
		usage();
		exit -1;
	}
/*	Install a new handler for keyboard interrupt. */
	if((signal(SIGINT, (sighandler_t)sig_interrupt)) == SIG_ERR)
	{
		syslog(LOG_ERR, "nRouted error: failed to install SIGINT handler. Exit.\n");
		exit(EXIT_FAILURE);
	}

/*	Install a new handler for SIGSEGV. */
	if((signal(SIGSEGV, (sighandler_t)sigsegv_interrupt)) == SIG_ERR)
	{
		syslog(LOG_ERR, "nRouted error: failed to install SIGSEGV handler. Exit.\n");
		exit(EXIT_FAILURE);
	}

/*	Install a new handler for SIGIO.
 *
 *	According to man 7 signal this signal is by default ignored by most systems.
 *	It seems that pre FC7 this was true for Fedoras also. We have noticed that at least
 *	on some FC7 installations the default action has changed. We avoid abnormal program 
 *	exits by defining the SIGIO as SIG_IGN (ignore). - mjs
 */
	if(signal(SIGIO, SIG_IGN) == SIG_ERR)
	{
		syslog(LOG_ERR, "nRouted error: failed to install SIGIO handler. Exit.\n");
		exit(EXIT_FAILURE);
	}

/*	Check if LOCK_DIR exists */
	if((stat(LOCK_DIR, &bf)) != 0)
	{
		syslog(LOG_INFO, "nRouted info: nRouted lock file directory nonexistent, creating it.\n");
		if((rval = mkdir(LOCK_DIR, S_IRUSR | S_IWUSR)) != 0)
		{
			if(nrouted_daemon == 0)
				printf("nRouted error: failed to create lock file directory %s.\n", LOCK_DIR);
			syslog(LOG_ERR, "nRouted error: failed to create lock file directory %s. Exit.\n", LOCK_DIR);
			exit(EXIT_FAILURE);
		}
	}


	//	Try to open and parse the config file contents
	if((rval = parse_config()) == -1)
	{			
		syslog(LOG_ERR, "nRouted error: Exiting.\n");

		if((rval = remove(nRouted_lockfile)) != 0)
		{
			logger(0, "Failed to remove lock file (%s), this might cause trouble when nRouted is started next time (stale lock file).\n", nRouted_lockfile);
		}

		printf("Exit with failure...\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		logger(2, "Configuration file parsed.\n");
	}

	if(nrouted_daemon == 0)
		printf("Everything fine, nRouted version %s starting using nRP version %d.\n", NROUTED_VERSION, NRP_VERSION);

	logger(0, "Everything fine, nRouted version %s starting using nRP version %d.\n", NROUTED_VERSION, NRP_VERSION);

	//	Fork the process.
	if(nrouted_daemon == 1)
	{
		process_id = fork();

		if(process_id < 0)
		{
			//	The parent failed to create a child process, exiting completely...
			exit(EXIT_FAILURE);
		}

		if(process_id > 0)
		{
			/*	The parent succeeded in creating a child. Because the fork() returns childs PID in the parents
				thread and 0 in childs thread this if() statement causes only the parent to exit. */
			exit(EXIT_SUCCESS);
		}
	}
	
	/*	The file permissions must be changed in order the child process to be able to write e.g. log files.
		There is no need for checking the return value as this system call always succeeds. */
	umask(0);


	//	Now we have most of the things done, but still some left so on with it!
	if(nrouted_daemon == 1)
	{
		//	First we must get an unique SID (Session ID) for the child process
		session_id = setsid();
		if(session_id < 0)
		{
			logger(0, "Failed to get a valid SID (%d) for child process, exiting.\n", session_id);
			exit(EXIT_FAILURE);
		}
	}


/*	First check if the lock file exists */
	if((stat(nRouted_lockfile, &bf)) == 0)
	{
		if(nrouted_daemon == 0)
			printf("nRouted error: nRouted already running or stale lock file %s\n", nRouted_lockfile);
			
		syslog(LOG_ERR, "nRouted error: nRouted already running or stale lock file %s\n", nRouted_lockfile);
		return(-1);
	}
	else
	{
		if((rval = creat(nRouted_lockfile, S_IRUSR | S_IWUSR)) == -1)
		{
			if(nrouted_daemon == 0)
				printf("nRouted error: failed to create lock file %s.\n", nRouted_lockfile);
			syslog(LOG_ERR, "nRouted error: failed to create lock file. Exit.\n");
			exit(EXIT_FAILURE);
		}
		else
		{
			syslog(LOG_INFO, "nRouted info: Lock file created.\n");
		}
	}


	/*	Then we must close some basic file descriptors. This is done because we don't want the daemon to output anything to stderr
	 *	or anything. Probably everyone has experienced badly coded programs that do this and most certainly no-one likes it! 
	 *	Ofcourse we first check the daemon var.
	 */
	if(nrouted_daemon == 1)
	{
		/*	Before closing the stdout, make a copy of it. */
		saved_stdout = dup(STDOUT_FILENO);
		logger(1, "Closing standard file I/O\n");
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);
	}

	/*	Attempting to create a separate thread for the serial server function. This threads sole responsibility is to poll the 
	 *	serial port and process the nRP packets.
	 */
	logger(0, "Starting serial server thread.\n");
	if((rval = pthread_create(&serial_server_th, NULL, (void *)&serial_server, NULL)) != 0)
	{
		logger(0, "Failed to create serial thread, exiting.\n");
		exit(EXIT_FAILURE);
	}

	/*	Here we create the thread that simply sets up a message queue and waits for messages to be sent to serial port
	 *
	 */
	logger(0, "Starting serial write thread.\n");
	if((rval = pthread_create(&serial_write_th, NULL, (void *)&serial_write, NULL)) != 0)
	{
		sleep(1);
		logger(0, "Failed to create serial thread, exiting.\n");
		exit(EXIT_FAILURE);
	}

	logger(0, "Starting TCP server thread.\n");
	//	Here we start the eth loop which creates a simple TCP server
	if((rval = tcp_server()) ==-1)
	{
		logger(2, "TCP server exit with error, stopping nRouted.\n");
		exit(EXIT_FAILURE);
	}		
	
	pthread_join(serial_server_th, NULL);
	pthread_join(serial_write_th, NULL);

/*	Remove the lock file but leave the LOCK_DIR intact. */
	if((rval = remove(nRouted_lockfile)) != 0)
	{
		logger(0, "Failed to remove lock file (%s), this might cause trouble when nRouted is started next time (stale lock file).\n", nRouted_lockfile);
	}
/*	We close the log file here, so no using the logger() function after this point. Thus if the closing fails we log the error just in case 
 *	into the syslog.
 */	
	if((rval = fclose(logfp)) != 0)
	{
		syslog(LOG_WARNING, "nRouted warning: failed to close log file %s.\n", nRdconf.logfile);
	}

	return(1);
}

/**	The nRouted config file parsing function.
 *
 *	This function parses the nRouted config file.
 *
 *	@return 0 when config file was successfully parsed, 1 when no config file was found and nRouted will try to use default parameters
 *	-1 when config file was found but it contained errors. nRouted will try to continue operation with return values 0 and 1. With
 *	return value -1 the daemon will exit.
 *
 */

int parse_config(void)
{
	FILE *cf;
	char keyword[32];
	char *value;
	int i;

	int rval;
	int ret=1;
	struct stat bf;

	//	Set the default values to some of the configuration variables
	strcpy(nRdconf.wdir, "/tmp/nRouted/");
//	strcpy(nRdconf.serialport, "/dev/ttyUSB0");
	nRdconf.uselogging = 1;
	nRdconf.loglevel=0;
	strcpy(nRdconf.logfile, "/tmp/nRouted.log");
	nRdconf.tcpport = 21780;
	nRdconf.tcpallowmulti = 0;
	nRdconf.tcpmaxconn = 1;

	nRdconf.serialbufsize = 20;


	value = (char *)malloc(128);


	if((cf = fopen(CONFIG_FILE, "r")) != NULL)
	{
		//	The config file was found and next we'll try to parse it
		while(fscanf(cf, "%[^=]=%[^\n\r]\n", keyword, value) == 2)
		{
			if((strncmp(keyword, "WORKDIR", 7) == 0) && strlen(keyword) == 7)
			{
				strcpy(nRdconf.wdir, value);
			}
			else if((strncmp(keyword, "SERIALPORT", 10) == 0) && strlen(keyword) == 10)
			{
				char *lock_file;
				char lock_file_tmp[128];
				char *svptr;
				char *tmpbuffer;
				int str_i;
				
				lock_file = (char *)malloc(128);
				tmpbuffer = (char *)malloc(128);
								
				strcpy(nRdconf.serialport, value);
				strcpy(nRouted_lockfile, LOCK_FILE_BASENAME);
				strcpy(tmpbuffer, value);
				
				for(str_i=1;str_i<999;str_i++, tmpbuffer = NULL)
				{
					if((lock_file = strtok_r(tmpbuffer, "/", &svptr)) == NULL && str_i<999)
					{
						strcpy(&(nRouted_lockfile[strlen(nRouted_lockfile)]), lock_file_tmp);
						str_i = 999;
					}
					else
					{
						strcpy(lock_file_tmp, lock_file);
					}
				}
				
				free(tmpbuffer);
				free(lock_file);
			}
			else if((strncmp(keyword, "TCPADDR", 7) == 0) && strlen(keyword) == 7)
			{
				strcpy(nRdconf.tcpaddr, value);
			}
			else if((strncmp(keyword, "TCPPORT", 7) == 0) && strlen(keyword) == 7)
			{
				nRdconf.tcpport = atoi(value);
			}
			else if((strncmp(keyword, "TCPMAXCONN", 10) == 0) && strlen(keyword) == 10)
			{	
				nRdconf.tcpmaxconn = atoi(value);
			}
			else if((strncmp(keyword, "TCPALLOWMULTI", 13) == 0) && strlen(keyword) == 13)
			{	
				nRdconf.tcpallowmulti = atoi(value);
			}
			else if((strncmp(keyword, "USELOGGING", 10) == 0) && strlen(keyword) == 10)
			{
				nRdconf.uselogging = atoi(value);
			}
			else if((strncmp(keyword, "LOGLEVEL", 8) == 0) && strlen(keyword) == 8)
			{
				nRdconf.loglevel = atoi(value);
			}
			else if((strncmp(keyword, "LOGFILE", 7) == 0) && strlen(keyword) == 7)
			{
				strcpy(nRdconf.logfile, value);
			}
			else if((strncmp(keyword, "SERIALBUFSIZE", 13) == 0) && strlen(keyword) == 13)
			{
				nRdconf.serialbufsize = atoi(value);
			}
		}
		//	Lets adjust the value of variable ret to 0 as
		ret = 0;
	} else {
		/*	The config file was not where it was supposed to be. Now we must check if we can use the default
			paths for working directories etc. -> Some work... */
		printf("We shouldn't be here!!! Do something.\n");
		fflush(stdout);
	}

	if((rval = chdir(nRdconf.wdir)) != 0)
	{
		if(nrouted_daemon == 0)
			printf("nRouted error: Failed to chdir into working directory %s\n", nRdconf.wdir);
			
		syslog(LOG_ERR, "nRouted error: Failed to chdir into working directory %s\n", nRdconf.wdir);
		return(-1);
	}

	if((stat(nRdconf.serialport, &bf)) != 0)
	{
		if(nrouted_daemon == 0)
			printf("nRouted error: Failed to stat serial port dev file %s\n", nRdconf.serialport);
		syslog(LOG_ERR, "nRouted error: Failed to stat serial port dev file %s\n", nRdconf.serialport);
		return(-1);
	}

	if(nRdconf.uselogging == 1)
	{
		if((logfp = fopen(nRdconf.logfile, "w+")) == NULL)
		{
			printf("nRouted error: Failed to open log file %s\n", nRdconf.logfile);
			syslog(LOG_ERR, "nRouted error: Failed to open log file %s\n", nRdconf.logfile);
			return(-1);
		}
	}

	nRd_conn_table = NULL;
	nRd_conn_table = (struct nRd_conn_table_t *)malloc(nRdconf.tcpmaxconn*sizeof(struct nRd_conn_table_t));
	if(nRd_conn_table == NULL)
	{
		if(nrouted_daemon == 0)
			printf("nRouted error: Failed to allocate memory for the connection array. Giving up...\n");
		logger(0, "nRouted error: Failed to allocate memory for the connection array. Giving up...\n");
		return(-1);
	}
	
	for(i=0;i<nRdconf.tcpmaxconn;i++)
	{
		nRd_conn_table[i].fd = 0;
		nRd_conn_table[i].active = 0;
		nRd_conn_table[i].proto = 0xFF;
		nRd_conn_table[i].address_type = 0xFE;
		nRd_conn_table[i].source_port = 0xFFFF;
		nRd_conn_table[i].destination_port = 0xFFFF;
	}


	free(value);

	return(0);
}
	

/**	The nRouted general/multipurpose logger function.
 *
 *	This function gives a nice and simple interface to output entries into the nRouted log file.
 *	
 */

void logger(int loglevel, const char *tolog, ...)
{
	va_list ap;
	struct timeval *tval;
	struct tm *time;

	//	Log only if the defined loglevel is greater or equal to the loglevel defined in configuration
	if(loglevel <= nRdconf.loglevel)
	{
		//	Get the time for logging
		tval = (struct timeval *)malloc(sizeof(struct timeval));

		tval->tv_sec = 0;
		tval->tv_usec = 0;

		gettimeofday(tval, NULL);

		//	Convert the timeval to local timezone
		time = localtime(&(tval->tv_sec));

		//	Print the date and time to logfile
		fprintf(logfp, "%d-%.2d-%.2d %.2d:%.2d.%.2d  ", time->tm_year+1900, time->tm_mon+1, time->tm_mday+1, time->tm_hour, time->tm_min, time->tm_sec);


		//	Process the variable argument list
		va_start(ap, tolog);
		vfprintf(logfp, tolog, ap);
		va_end(ap);
		fflush(logfp);
		
		free(tval);
	}
	return;	
}

/**	The nRouted command line parsing function.
 *
 *	This function parses the nRouted command line.
 *
 *  @param count argc
 *  @param param argv
 *
 *	@return 0 when command line was successfully parsed
 *
 */
int parse_opts(int count, char* param[])
{
	int opt;
	int option_index = 0;
	
  while ((opt = getopt_long(count, param, OPTIONS_STRING,
                            long_options, &option_index)) != -1)
  {
		fflush(stdout);
    switch(opt)
		{
			case 'd':
				nrouted_daemon = 1;
				break;
				
			case 'm':
				syslog(LOG_INFO, "nRouted info: enable monitor mode.\n");
				printf("nRouted is in monitor mode.\n");
				monitor_mode = 1;
				break;

			case 'p':
				if (strcmp(optarg, "nudp") == 0)
				{
					nRdconf.protocol = PROTO_NUDP;
				}
				else if (strcmp(optarg, "6lowpan") == 0)
				{
					nRdconf.protocol = PROTO_6LOWPAN;
				}
				else if (strcmp(optarg, "mac") == 0)
				{
					nRdconf.protocol = PROTO_IEEE_802_15_4_RAW;
				}
				else return -1;
				break;

			default:
				return -1;				
		}
	}
	
	return 0;
}

void rawlogger(int loglevel, const char *tolog, ...)
{
	va_list ap;

	//	Log only if the defined loglevel is greater or equal to the loglevel defined in configuration
	if(loglevel <= nRdconf.loglevel)
	{
		//	Process the variable argument list
		va_start(ap, tolog);
		vfprintf(logfp, tolog, ap);
		va_end(ap);
		fflush(logfp);
	}
	return;	
}

void sig_interrupt(void)
{
	int i=3;
	int rval;

/*	We first set this variable to 1. Other threads know to stop their operation. TCP server does not
 *	accept new connections and serial server stops polling the serial port.
 */
	haltsystem = 1;

/*	Because if the nRoute is running as a daemon we can not receive keyboard interrupt we know that STDOUT is available for printf. */
	printf("Trying to close all threads and exit decently. Please wait 3 seconds...");
	while(i>0)
	{
		printf("\b\b\b\b\b\b\b\b\b\b\b\b%d seconds...", i);
		fflush(stdout);
		sleep(1);
		i--;
	}
	printf("\b\b\b\b\b\b\b\b\b\b\b\b0 seconds...\n");
	fflush(stdout);
	
/*	Close all open TCP connections. the tcp_conn_handler() should take care of all this when it exits but just to make things sure we check the
 *	connection table here also. 
 */

/*	for(i=0;i<nRdconf.tcpmaxconn;i++)
	{
		if(nRd_conn_table[i].active == 1)
			close(nRd_conn_table[i].fd);
	}
*/

/*	Close the serial port. */
	port_close();

/*	Remove the lock file but leave the LOCK_DIR intact. */

	if((rval = remove(nRouted_lockfile)) != 0)
	{
		logger(0, "Failed to remove lock file (%s), this might cause trouble when nRouted is started next time (stale lock file).\n", nRouted_lockfile);
	}
	
	printf("All done.\n");
	
	exit(1);
}

void sigsegv_interrupt(void)
{
	int rval;
	FILE *fp;

	fp = (FILE *)malloc(sizeof(FILE));

	if(nrouted_daemon == 1)
	{
		fp = fdopen(saved_stdout, "w");
	}

	fprintf(fp, "Caught Segmentation fault!\nExit.\n");

	if((rval = remove(nRouted_lockfile)) != 0)
	{
		fprintf(fp, "Failed to remove lock file (%s), this might cause trouble when nRouted is started next time (stale lock file).\n", nRouted_lockfile);
	}
	
	free(fp);
	
	exit(1);
}

void usage(void)
{
	printf("\nUSAGE: ./nRouted [-d] [-m] [-p protocol]\n\n");
	printf("If the -d parameter is given to nRouted then the program will start as a background daemon.\n");
	printf("If the -m parameter is given to nRouted then the program will print out received packets.\n");
	printf("If the -p parameter selects active protocol on startup (default 6lowpan).\n");
	printf("Available protocols: mac, nudp, 6lowpan.\n");
	return;
}

void hex_dump(unsigned char *data, int length)
{
	int i;
	
	for (i=0; i<length; i++)
	{
		printf("%2.2X", data[i]);
		if ((i & 0x0F) == 0x0F) printf("\n");
		else printf(":");
	}
	if ((length & 0x0F) != 0) printf("\n");
}

