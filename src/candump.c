/*
 *  $Id$
 */

/*
 * candump.c
 *
 * Copyright (c) 2002-2009 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "terminal.h"
#include "lib.h"

#include <wiringPi.h>
#include "kbhit.h"
#include "ABE_ADCPi.h"
#include "pthread.h"
#include "candump.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>

#include "tmr.h"







#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

#define SILENT_INI 42 /* detect user setting on commandline */
#define SILENT_OFF 0  /* no silent mode */
#define SILENT_ANI 1  /* silent mode with animation */
#define SILENT_ON  2  /* silent mode (completely silent) */

#define BOLD    ATTBOLD
#define RED     ATTBOLD FGRED
#define GREEN   ATTBOLD FGGREEN
#define YELLOW  ATTBOLD FGYELLOW
#define BLUE    ATTBOLD FGBLUE
#define MAGENTA ATTBOLD FGMAGENTA
#define CYAN    ATTBOLD FGCYAN

#define MAX_CANFRAME      "12345678#01.23.45.67.89.AB.CD.EF"

const char col_on [MAXCOL][19] = {BLUE, RED, GREEN, BOLD, MAGENTA, CYAN};
const char col_off [] = ATTRESET;

static char *cmdlinename[MAXSOCK];
static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 

#define MAXANI 4
const char anichar[MAXANI] = {'|', '/', '-', '\\'};

extern int optind, opterr, optopt;

static volatile int running = 0;
/*
Variables for digital IOs communication
*/
int inputs_SIZE;
int analogs_SIZE;
int *inputs_0; //First array of inputs
double *analogs_0; //Second array of inputs
int array_index = 0; //Actual index of the array
int array_index_analog = 0;
int trigger = 0;
int analog = 0;
int rising_edge_captured = 0;
int timestep;
int timestep_analog;
int timestep_can;
const int input_pins[] = {18,27,22,23,5,6,13,16,20,21}; //GPIO 1,2,3,6,21,22,23,27,28,29 (PHYS 12,13,15,22,29,31,33,36,38,40) 
int NUM_DIGITALS;
int NUM_ANALOGS;
const int TRIGGER_PIN = 21; //GPIO 29 (PHYS 40)
const int STOP_TRIGGER_PIN = 20; //GPIO 28 (PHYS 38)
FILE *inputFILE = NULL;
FILE *analogFILE = NULL;
int save = 0;
struct timeval time_init;
char **current_times;
char **actual_times;
char **current_times_analog;
char **actual_times_analog;
char dir_dir[sizeof("/home/pi/Exo_beta/data/2006-11-20/14_25_12")+1];
char dir[sizeof("/home/pi/Exo_beta/data/2006-11-20")+1];
int key_trigger = 2;
int stop_trigger = 0;
int digital_time_sampling = 0;
int analog_time_sampling = 0;
int flag_digital = 1;
int flag_analog = 1;


char *logf110[sizeof("/home/pi/Exo_beta/data/2019-01-16/17_50_00/candump_110.csv")+1];
char *logf120[sizeof("/home/pi/Exo_beta/data/2019-01-16/17_50_00/candump_110.csv")+1];
char *logf130[sizeof("/home/pi/Exo_beta/data/2019-01-16/17_50_00/candump_110.csv")+1];
char *logf140[sizeof("/home/pi/Exo_beta/data/2019-01-16/17_50_00/candump_110.csv")+1];
char *csv_name[sizeof("/home/pi/Exo_beta/data/2006-11-20/candump-2006-11-20_202026.csv")+1];

clock_t digital_timestamp[10];
int digital_time = 0;
clock_t analog_timestamp[10];
int analog_time = 0;


/*********************************/



void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s [options] <CAN interface>+\n", prg);
	fprintf(stderr, "  (use CTRL-C to terminate %s)\n\n", prg);
	fprintf(stderr, "Options: -t <type>   (timestamp: (a)bsolute/(d)elta/(z)ero/(A)bsolute w date)\n");
	fprintf(stderr, "         -c          (increment color mode level)\n");
	fprintf(stderr, "         -i          (binary output - may exceed 80 chars/line)\n");
	fprintf(stderr, "         -a          (enable additional ASCII output)\n");
	fprintf(stderr, "         -S          (swap byte order in printed CAN data[] - marked with '%c' )\n", SWAP_DELIMITER);
	fprintf(stderr, "         -s <level>  (silent mode - %d: off (default) %d: animation %d: silent)\n", SILENT_OFF, SILENT_ANI, SILENT_ON);
	fprintf(stderr, "         -b <can>    (bridge mode - send received frames to <can>)\n");
	fprintf(stderr, "         -B <can>    (bridge mode - like '-b' with disabled loopback)\n");
	fprintf(stderr, "         -u <usecs>  (delay bridge forwarding by <usecs> microseconds)\n");
	fprintf(stderr, "         -l          (log CAN-frames into file. Sets '-s %d' by default)\n", SILENT_ON);
	fprintf(stderr, "         -L          (use log file format on stdout)\n");
	fprintf(stderr, "         -n <count>  (terminate after receiption of <count> CAN frames)\n");
	fprintf(stderr, "         -r <size>   (set socket receive buffer to <size>)\n");
	fprintf(stderr, "         -d          (monitor dropped CAN frames)\n");
	fprintf(stderr, "         -e          (dupm CAN error frames in human-readable format)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Up to %d CAN interfaces with optional filter sets can be specified\n", MAXSOCK);
	fprintf(stderr, "on the commandline in the form: <ifname>[,filter]*\n");
	fprintf(stderr, "\nComma separated filters can be specified for each given CAN interface:\n");
	fprintf(stderr, " <can_id>:<can_mask> (matches when <received_can_id> & mask == can_id & mask)\n");
	fprintf(stderr, " <can_id>~<can_mask> (matches when <received_can_id> & mask != can_id & mask)\n");
	fprintf(stderr, " #<error_mask>       (set error frame filter, see include/linux/can/error.h)\n");
	fprintf(stderr, "\nCAN IDs, masks and data content are given and expected in hexadecimal values.\n");
	fprintf(stderr, "When can_id and can_mask are both 8 digits, they are assumed to be 29 bit EFF.\n");
	fprintf(stderr, "Without any given filter all data frames are received ('0:0' default filter).\n");
	fprintf(stderr, "\nUse interface name '%s' to receive from all CAN interfaces.\n", ANYDEV);
	fprintf(stderr, "\nExamples:\n");
	fprintf(stderr, "%s -c -c -ta can0,123:7FF,400:700,#000000FF can2,400~7F0 can3 can8\n", prg);
	fprintf(stderr, "%s -l any,0~0,#FFFFFFFF    (log only error frames but no(!) data frames)\n", prg);
	fprintf(stderr, "%s -l any,0:0,#FFFFFFFF    (log error frames and also all data frames)\n", prg);
	fprintf(stderr, "%s vcan2,92345678:DFFFFFFF (match only for extended CAN ID 12345678)\n", prg);
	fprintf(stderr, "%s vcan2,123:7FF (matches CAN ID 123 - including EFF and RTR frames)\n", prg);
	fprintf(stderr, "%s vcan2,123:C00007FF (matches CAN ID 123 - only SFF and non-RTR frames)\n", prg);
	fprintf(stderr, "\n");
}

void sigterm(int signo)
{
	running = 0;
}

int idx2dindex(int ifidx, int socket) {

	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */

	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	if (i == MAXIFNAMES) {
		fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
		       MAXIFNAMES);
		exit(1);
	}

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
	printf("new index %d (%s)\n", i, devname[i]);
#endif

	return i;
}



void reconfig(){
   char *file_name;
   FILE *fp;
   int reconfig = 0;
   int linenum = 0;
   char* joint_control = (char*)malloc((50) * sizeof(char));
   char* min_angles = (char*)malloc((50) * sizeof(char));
   char* max_angles = (char*)malloc((50) * sizeof(char));
   char* CAN_data = (char*)malloc((50) * sizeof(char));

   int motor_id=-1,type_of_control=-1,position_torque_set_point=-1,stiffness_set_point=-1,right_hip_min_angle=-1,right_knee_min_angle=-1,right_ankle_min_angle=-1,left_hip_min_angle=-1,left_knee_min_angle=-1,left_ankle_min_angle=-1,right_hip_max_angle=-1,right_knee_max_angle=-1,right_ankle_max_angle=-1,left_hip_max_angle=-1,left_knee_max_angle=-1,left_ankle_max_angle=-1,start_stop_can_data=-1;
   int datas[] = {motor_id,type_of_control,position_torque_set_point,stiffness_set_point,right_hip_min_angle,right_knee_min_angle,right_ankle_min_angle,left_hip_min_angle,left_knee_min_angle,left_ankle_min_angle,right_hip_max_angle,right_knee_max_angle,right_ankle_max_angle,left_hip_max_angle,left_knee_max_angle,left_ankle_max_angle,start_stop_can_data};
   char *options[] = {"MOTOR_ID=","TYPE_OF_CONTROL=","POSITION/TORQUE_SET_POINT=","STIFFNESS_SET_POINT=","RIGHT_HIP_MIN_ANGLE=","RIGHT_KNEE_MIN_ANGLE=","RIGHT_ANKLE_MIN_ANGLE=","LEFT_HIP_MIN_ANGLE=","LEFT_KNEE_MIN_ANGLE=","LEFT_ANKLE_MIN_ANGLE=","RIGHT_HIP_MAX_ANGLE=","RIGHT_KNEE_MAX_ANGLE=","RIGHT_ANKLE_MAX_ANGLE=","LEFT_HIP_MAX_ANGLE=","LEFT_KNEE_MAX_ANGLE=","LEFT_ANKLE_MAX_ANGLE=","START/STOP_CAN_DATA="};
 
   file_name = "/home/pi/Exo_beta/src/reconfig.txt";
 
   fp = fopen(file_name, "r"); // read mode
 
   if (fp == NULL)
   {
      perror("Error while opening the file.\n");
      exit(EXIT_FAILURE);
   }
 
   //printf("The contents of %s file are:\n", file_name);
   char line[256];
   while(fgets(line, 256, fp) != NULL)
	{
        char option[256], data[256];

        linenum++;
        if(line[0] == '#') continue;

        if(sscanf(line, "%s %s", option, data) != 2)
        {
                fprintf(stderr, "Syntax error, line %d\n", linenum);
                continue;
        }
        //printf("%s %s",option,data);
        fflush(stdout);
        //printf("%i",strcmp(option,"NUM_DIGITALS="));
        if(strcmp(option,"NUM_DIGITALS=")==0){
        	NUM_DIGITALS = stringtoint(data)+1;
        	if(NUM_DIGITALS>10)
        		NUM_DIGITALS = 10;
        	if(NUM_DIGITALS<-1)
        		NUM_DIGITALS = 1;
        	inputs_SIZE = NUM_DIGITALS * 10;
        	fflush(stdout);
        	continue;
        }

        if(strcmp(option,"NUM_ANALOGS=")==0){
        	NUM_ANALOGS = stringtoint(data)+1;
        	if(NUM_ANALOGS>10)
        		NUM_ANALOGS = 10;
        	if(NUM_ANALOGS<-1)
        		NUM_ANALOGS = 1;
        	analogs_SIZE = NUM_ANALOGS * 10;
        	fflush(stdout);
        	continue;
        }

        if(strcmp(option,"KEY_OR_TRIGGER=")==0){
        	key_trigger = stringtoint(data);
        		if(key_trigger > 2 || key_trigger < 0)
        			key_trigger = 2;
        		if(key_trigger == 0 ||key_trigger == 2)
        			printf("KEYBOARD ENABLED\n");
        		if(key_trigger == 1 ||key_trigger == 2)
        			printf("TRIGGER ENABLED\n");
        	fflush(stdout);
        	continue;
        }
        if(strcmp(option,"DIGITAL_SAMPLES_PER_SECOND=")==0){
        	digital_time_sampling = 1000000/stringtoint(data);
        		if( digital_time_sampling < 20000)
        			digital_time_sampling = 22000;
        	//printf("%i\n",NUM_DIGITALS );
        	fflush(stdout);
        	continue;
        }
        if(strcmp(option,"ANALOG_SAMPLES_PER_SECOND=")==0){
        	analog_time_sampling = 1000000/stringtoint(data);
        		if( analog_time_sampling < 20000)
        			analog_time_sampling = 22000;
        	//printf("%i\n",NUM_DIGITALS );
        	fflush(stdout);
        	continue;
        }
        if(strcmp(option,"STOP_TRIGGER=")==0){
        	stop_trigger = stringtoint(data);
        		if(stop_trigger > 1 || stop_trigger < 0)
        			stop_trigger = 0;
        	//printf("%i\n",NUM_DIGITALS );
        	fflush(stdout);
        	continue;
        }

        if(strcmp(option,"RECONFIG=")==0){
        	if(strcmp(data,"TRUE")==0){
        		reconfig = 1;
        	}
        }else{
        		for(int i = 0;i<sizeof(datas)-1;i++){
        			//printf("Comparing %s to %s\n is %i ",option,options[i], strcmp(option,options[i]));
        			fflush(stdout);
        			if(reconfig){
        		
        			if(strcmp(option,options[i])==0){
						//printf("Estoy aquí");
        				fflush(stdout);
        				datas[i] = stringtoint(data);
        				//printf("Changing value of %s to %i\n",options[i],datas[i] );
        				break;
        			}
        		}
        	}
        }
   	}
   	sprintf(joint_control,"%s%02X%02X%02X%02X%02X%02X","070#",datas[0],datas[1],datas[2],datas[3],0,0);
   	sprintf(min_angles,"%s%02X%02X%02X%02X%02X%02X","075#",datas[4],datas[5],datas[6],datas[7],datas[8],datas[9]);
   	sprintf(max_angles,"%s%02X%02X%02X%02X%02X%02X","080#",datas[10],datas[11],datas[12],datas[13],datas[14],datas[15]);
   	sprintf(CAN_data,"%s%02X%02X%02X%02X%02X%02X","085#",datas[16],0,0,0,0,0);
	if(reconfig){
   		main_cansend(joint_control);
   		main_cansend(min_angles);
   		main_cansend(max_angles);
   		main_cansend(CAN_data);
	}
   	fflush(stdout);
}

void wait_until(struct timeval start, int usecs){
	struct timeval stop;
	gettimeofday(&start, NULL);
	while(1){
		gettimeofday(&stop, NULL);
		if((stop.tv_usec - start.tv_usec) > usecs)
			return;
	}
}

int stringtoint(char *string){
	int len = strlen(string);
	int dec = 0;
	for(int i = 0; i<len; i++){
		dec = dec * 10 + ( string[i] - '0' );
	}
	return dec;
}


void sprint_canframe_csv(char *buf , struct can_frame *cf, int sep) {
	/* documentation see lib.h */

	int i,offset;
	int dlc = (cf->can_dlc > 8)? 8 : cf->can_dlc;

	if (cf->can_id & CAN_ERR_FLAG) {
		sprintf(buf, "%08X; ", cf->can_id & (CAN_ERR_MASK|CAN_ERR_FLAG));
		offset = 9;
	} else if (cf->can_id & CAN_EFF_FLAG) {
		sprintf(buf, "%08X; ", cf->can_id & CAN_EFF_MASK);
		offset = 9;
	} else {
		sprintf(buf, "%03X; ", cf->can_id & CAN_SFF_MASK);
		offset = 4;
	}

	if (cf->can_id & CAN_RTR_FLAG) /* there are no ERR frames with RTR */
		sprintf(buf+offset, "R");
	else{
		sprintf(buf+offset, " ");
		offset++;
		for (i = 0; i < dlc; i++) {
			sprintf(buf+offset, "%02X", cf->data[i]);
			offset += 2;
			if (sep && (i+1 < dlc))
				sprintf(buf+offset++, ".");
		}
	}
		//sprintf(buf+offset,";%i",timestep_can);
}

void fprint_canframe_csv(FILE *stream , struct can_frame *cf, char *eol, int sep) {
	/* documentation see lib.h */

	char buf[sizeof(MAX_CANFRAME)+1]; /* max length */
	//printf("%s\n\n",t );
	fflush(stdout);
	fprintf(stream,"%ld; ",clock());
	fprintf(stream,"%s;",getactualtime());
	fprintf(stream," %s; %i; %i; ",gettimeprog(), 0 ,timestep_can);
	sprint_canframe_csv(buf, cf, sep);
	fprintf(stream, "%s", buf);
	if (eol)
		fprintf(stream, "%s", eol);
}

void shellcmd(){
	FILE *fp;
	char *command;

	command="sudo ip link set up can0 type can bitrate 1000000";

	fp = popen (command,"w");

	fclose(fp);

	//command = "sudo ip link set up can1 type can bitrate 1000000";
	//fp = popen (command,"w");

	//fclose(fp);

	/*command = "sudo rm candump_110.csv candump_120.csv candump_130.csv candump_140.csv candump-* inputs*";
	fp = popen (command,"w");

	fclose(fp);*/
}

void config(){
	debounceTime=DEBOUNCE_TIME;
	wiringPiSetupGpio();
	for(int i = 0; i < NUM_DIGITALS-1; i++){
		pinMode(input_pins[i], INPUT);
		int in = digitalRead(input_pins[i]);
		//printf("PIN %i was configured as INPUT and its value is %i \n", input_pins[i],in);
		fflush(stdout);
	}
	pinMode(TRIGGER_PIN,INPUT);
	pinMode(STOP_TRIGGER_PIN,INPUT);
	//printf("PIN %i was configured as INPUT and its value is %i \n", TRIGGER_PIN, digitalRead(TRIGGER_PIN));
	fflush(stdout);
	
    inputs_0 = (int*)malloc(inputs_SIZE*sizeof(int));
    analogs_0 = (int*)malloc(analogs_SIZE*sizeof(double));

	current_times = (char**)malloc(inputs_SIZE*sizeof(char*));
	actual_times = (char**)malloc(inputs_SIZE*sizeof(char*));
	current_times_analog = (char**)malloc(analogs_SIZE*sizeof(char*));
	actual_times_analog = (char**)malloc(analogs_SIZE*sizeof(char*));
	
	for (int i = 0; i < inputs_SIZE; i++){
		current_times[i]=(char*)malloc((50) * sizeof(char));
		actual_times[i]=(char*)malloc((50) * sizeof(char));
		current_times_analog[i]=(char*)malloc((50) * sizeof(char));
		actual_times_analog[i]=(char*)malloc((50) * sizeof(char));
	}
	
}

pthread_t thread1;
pthread_t thread2;
pthread_t thread3;
pthread_t thread4;
pthread_mutex_t runningMutex;
pthread_mutex_t triggerMutex;
pthread_mutex_t analogMutex;


FILE *log110 = NULL;
FILE *log120 = NULL;
FILE *log130 = NULL;
FILE *log140 = NULL;
FILE *csv = NULL;



void *explorateclado() {
	int teclaPulsada;

	printf(" -------------------------------------------------------\n USER MANUAL\n -------------------------------------------------------\n Press q to exit\n Press c to capture inputs and CAN frames\n Press s to stop capturing inputs and CAN frames\n ");

	while (1) {
		//printf(" Listening...\n");
		sleep(1); // Wiring Pi function: pauses program execution for at least 10 ms
		if (kbhit()) {
			teclaPulsada = kbread();

			switch (teclaPulsada) {
			case 'q':
				printf("\nKEY q PRESSED. EXITING...\n");
				fflush(stdout);
				fclose(log110);
				fclose(log120);
				fclose(log130);
				fclose(log140);
				fclose(csv);
				parseCANdata(logf110, dir_dir, 1);
				parseCANdata(logf120, dir_dir, 2);
				parseCANdata(logf130, dir_dir, 3);
				parseCANdata(logf140, dir_dir, 4);
				parseCANdata(csv_name,dir_dir, 5);
				exit(0);
				break;
			case 'c':
			if(key_trigger == 0 || key_trigger == 2){
				printf("\nKEY c PRESSED. CAPTURING ENTRIES AND CAN FRAMES\n");
				fflush(stdout);
				pthread_mutex_lock(&triggerMutex);
				trigger = 1;
				//printf("Trigger = %i",trigger);
				pthread_mutex_unlock(&triggerMutex);
				pthread_mutex_lock(&runningMutex);
				running = 1;
				//printf("Running = %i",running);
				pthread_mutex_unlock(&runningMutex);

				pthread_mutex_lock(&analogMutex);
				analog = 1;
				//printf("Analog = %i",running);
				pthread_mutex_unlock(&analogMutex);

				fflush(stdout);}
				break;
			case 'v':
				//printf("\nKEY v PRESSED. CAPTURING CAN FRAMES\n");
				//fflush(stdout);
				//pthread_mutex_lock(&runningMutex);
				//running = 1;
				//printf("Running = %i",running);
				//pthread_mutex_unlock(&runningMutex);
				//fflush(stdout);
				break;
			case 's':
				if(key_trigger == 0 || key_trigger == 2){
				printf("\nKEY s PRESSED. STOPPED CAPTURING INPUTS AND CAN FRAMES\n");
				fflush(stdout);
				pthread_mutex_lock(&triggerMutex);
				trigger = 0;
				rising_edge_captured = 0;
				//printf("Trigger = %i",trigger);
				fflush(stdout);
				pthread_mutex_unlock(&triggerMutex);
				pthread_mutex_lock(&runningMutex);
				running = 0;
				//printf("Running = %i",running);
				pthread_mutex_unlock(&runningMutex);}
				break;
			case 'd':
				//printf("\nKEY d PRESSED. STOPPED CAPTURING CAN FRAMES\n");
				//fflush(stdout);
				//pthread_mutex_lock(&runningMutex);
				//running = 0;
				//printf("Running = %i",running);
				//pthread_mutex_unlock(&runningMutex);
				//fflush(stdout);
				break;

			case 'p':
				main_cansend("8C#DEADBEEFFFFF");
				printf("\nMESSAGE SENT\n");
				fflush(stdout);
				break;

			case 'o':
				main_cansend("82#DEADBEEFFFFF");
				printf("\nMESSAGE SENT\n");
				fflush(stdout);
				break;
			case 'i':
				main_cansend("78#DEADBEEFFFFF");
				printf("\nMESSAGE SENT\n");
				fflush(stdout);
				break;
			case 'u':
				main_cansend("6E#DEADBEEFFFFF");
				printf("\nMESSAGE SENT\n");
				fflush(stdout);
				break;
			default:
				printf("\nPress q to exit\n Press c to capture inputs and CAN frames\n Press s to stop capturing inputs and CAN frames \n To send CAN frames: \n u - 110   i - 120   o - 130   p - 140\n");
				fflush(stdout);
				break;
			}
		}

	}
}



int *main_candump()
{
	
	int argc = 5;
	char *argv[argc];
	argv[0] = "./candump";
	argv[1] = "-l";
	argv[2] = "-s";
	argv[3] = "0";
	argv[4] = "can0";	


	fd_set rdfs;
	int s[MAXSOCK];
	int bridge = 0;
	useconds_t bridge_delay = 0;
	unsigned char timestamp = 0;
	unsigned char dropmonitor = 0;
	unsigned char silent = SILENT_INI;
	unsigned char silentani = 0;
	unsigned char color = 0;
	unsigned char view = 0;
	unsigned char log = 1;
	unsigned char logfrmt = 0;
	int count = 0;
	int rcvbuf_size = 0;
	int opt, ret;
	int currmax, numfilter;
	char *ptr, *nptr;
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *cmsg;
	struct can_filter *rfilter;
	can_err_mask_t err_mask;
	struct can_frame frame;
	int i;
	ssize_t nbytes;
	struct ifreq ifr;
	struct timeval tv, last_tv;
	FILE *logfile = NULL;

	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);

	last_tv.tv_sec  = 0;
	last_tv.tv_usec = 0;

	//Aquí se produce la violacion de segmento//
	while ((opt = getopt(argc, argv, "t:ciaSs:b:B:u:ldLn:r:he?")) != -1) {
		//printf("%s",argv);
	fflush(stdout);
		switch (opt) {
		case 't':
			timestamp = optarg[0];
			if ((timestamp != 'a') && (timestamp != 'A') &&
			    (timestamp != 'd') && (timestamp != 'z')) {
				fprintf(stderr, "%s: unknown timestamp mode '%c' - ignored\n",
				       basename(argv[0]), optarg[0]);
				timestamp = 0;
			}
			break;

		case 'c':
			color++;
			break;

		case 'i':
			view |= CANLIB_VIEW_BINARY;
			break;

		case 'a':
			view |= CANLIB_VIEW_ASCII;
			break;

		case 'S':
			view |= CANLIB_VIEW_SWAP;
			break;

		case 'e':
			view |= CANLIB_VIEW_ERROR;
			break;

		case 's':
			silent = atoi(optarg);
			if (silent > SILENT_ON) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'b':
		case 'B':
			if (strlen(optarg) >= IFNAMSIZ) {
				fprintf(stderr, "Name of CAN device '%s' is too long!\n\n", optarg);
				return 1;
			} else {
				bridge = socket(PF_CAN, SOCK_RAW, CAN_RAW);
				if (bridge < 0) {
					perror("bridge socket");
					return 1;
				}
				addr.can_family = AF_CAN;
				strcpy(ifr.ifr_name, optarg);
				if (ioctl(bridge, SIOCGIFINDEX, &ifr) < 0)
					perror("SIOCGIFINDEX");
				addr.can_ifindex = ifr.ifr_ifindex;
		
				if (!addr.can_ifindex) {
					perror("invalid bridge interface");
					return 1;
				}

				/* disable default receive filter on this write-only RAW socket */
				setsockopt(bridge, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

				if (opt == 'B') {
					const int loopback = 0;

					setsockopt(bridge, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
						   &loopback, sizeof(loopback));
				}

				if (bind(bridge, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
					perror("bridge bind");
					return 1;
				}
			}
			break;
	    
		case 'u':
			bridge_delay = (useconds_t)strtoul(optarg, (char **)NULL, 10);
			break;

		case 'l':
			log = 1;
			break;

		case 'd':
			dropmonitor = 1;
			break;

		case 'L':
			logfrmt = 1;
			break;

		case 'n':
			count = atoi(optarg);
			if (count < 1) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		case 'r':
			rcvbuf_size = atoi(optarg);
			if (rcvbuf_size < 1) {
				print_usage(basename(argv[0]));
				exit(1);
			}
			break;

		default:
			print_usage(basename(argv[0]));
			exit(1);
			break;
		}
	}

	if (optind == argc) {
		print_usage(basename(argv[0]));
		exit(0);
	}
	
	if (logfrmt && view) {
		fprintf(stderr, "Log file format selected: Please disable ASCII/BINARY/SWAP options!\n");
		exit(0);
	}

	if (silent == SILENT_INI) {
		if (log) {
			fprintf(stderr, "Disabled standard output while logging.\n");
			silent = SILENT_ON; /* disable output on stdout */
		} else
			silent = SILENT_OFF; /* default output */
	}

	currmax = argc - optind; /* find real number of CAN devices */

	if (currmax > MAXSOCK) {
		fprintf(stderr, "More than %d CAN devices given on commandline!\n", MAXSOCK);
		return 1;
	}

	for (i=0; i < currmax; i++) {

		ptr = argv[optind+i];
		nptr = strchr(ptr, ',');

#ifdef DEBUG
		printf("open %d '%s'.\n", i, ptr);
#endif

		s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s[i] < 0) {
			perror("socket");
			return 1;
		}

		cmdlinename[i] = ptr; /* save pointer to cmdline name of this socket */

		if (nptr)
			nbytes = nptr - ptr;  /* interface name is up the first ',' */
		else
			nbytes = strlen(ptr); /* no ',' found => no filter definitions */

		if (nbytes >= IFNAMSIZ) {
			fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
			return 1;
		}

		if (nbytes > max_devname_len)
			max_devname_len = nbytes; /* for nice printing */

		addr.can_family = AF_CAN;

		memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
		strncpy(ifr.ifr_name, ptr, nbytes);

#ifdef DEBUG
		printf("using interface name '%s'.\n", ifr.ifr_name);
#endif

		if (strcmp(ANYDEV, ifr.ifr_name)) {
			if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0) {
				perror("SIOCGIFINDEX");
				exit(1);
			}
			addr.can_ifindex = ifr.ifr_ifindex;
		} else
			addr.can_ifindex = 0; /* any can interface */

		if (nptr) {

			/* found a ',' after the interface name => check for filters */

			/* determine number of filters to alloc the filter space */
			numfilter = 0;
			ptr = nptr;
			while (ptr) {
				numfilter++;
				ptr++; /* hop behind the ',' */
				ptr = strchr(ptr, ','); /* exit condition */
			}

			rfilter = malloc(sizeof(struct can_filter) * numfilter);
			if (!rfilter) {
				fprintf(stderr, "Failed to create filter space!\n");
				return 1;
			}

			numfilter = 0;
			err_mask = 0;

			while (nptr) {

				ptr = nptr+1; /* hop behind the ',' */
				nptr = strchr(ptr, ','); /* update exit condition */

				if (sscanf(ptr, "%x:%x",
					   &rfilter[numfilter].can_id, 
					   &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (sscanf(ptr, "%x~%x",
						  &rfilter[numfilter].can_id, 
						  &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_id |= CAN_INV_FILTER;
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (sscanf(ptr, "#%x", &err_mask) != 1) { 
					fprintf(stderr, "Error in filter option parsing: '%s'\n", ptr);
					return 1;
				}
			}

			if (err_mask)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
					   &err_mask, sizeof(err_mask));

			if (numfilter)
				setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FILTER,
					   rfilter, numfilter * sizeof(struct can_filter));

			free(rfilter);

		} /* if (nptr) */

		if (rcvbuf_size) {

			int curr_rcvbuf_size;
			socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

			/* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
			if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUFFORCE,
				       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
#ifdef DEBUG
				printf("SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
#endif
				if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
					perror("setsockopt SO_RCVBUF");
					return 1;
				}

				if (getsockopt(s[i], SOL_SOCKET, SO_RCVBUF,
					       &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0) {
					perror("getsockopt SO_RCVBUF");
					return 1;
				}

				/* Only print a warning the first time we detect the adjustment */
				/* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
				if (!i && curr_rcvbuf_size < rcvbuf_size*2)
					fprintf(stderr, "The socket receive buffer size was "
						"adjusted due to /proc/sys/net/core/rmem_max.\n");
			}
		}

		if (timestamp || log || logfrmt) {

			const int timestamp_on = 1;

			if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMP,
				       &timestamp_on, sizeof(timestamp_on)) < 0) {
				perror("setsockopt SO_TIMESTAMP");
				return 1;
			}
		}

		if (dropmonitor) {

			const int dropmonitor_on = 1;

			if (setsockopt(s[i], SOL_SOCKET, SO_RXQ_OVFL,
				       &dropmonitor_on, sizeof(dropmonitor_on)) < 0) {
				perror("setsockopt SO_RXQ_OVFL not supported by your Linux Kernel");
				return 1;
			}
		}

		if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			return 1;
		}
	}

	if (log) {
		time_t currtime;
		struct tm now;
		char fname[sizeof("/home/pi/Exo_beta/data/2006-11-20/candump-2006-11-20_202026.log")+1];
		char fname_csv[sizeof("/home/pi/Exo_beta/data/2006-11-20/candump-2006-11-20_202026.csv")+1];

		if (time(&currtime) == (time_t)-1) {
			perror("time");
			return 1;
		}

		localtime_r(&currtime, &now);
		//char dir[sizeof("2006-11-20")+1];
		//sprintf(dir, "%02d-%02d-%04d",now.tm_mday,now.tm_mon+1,now.tm_year+1900);
		//mkdir(dir,0777);

		sprintf(fname, "%s/candump-%04d-%02d-%02d_%02d%02d%02d.log",
			dir_dir,
			now.tm_year + 1900,
			now.tm_mon + 1,
			now.tm_mday,
			now.tm_hour,
			now.tm_min,
			now.tm_sec);

		if (silent != SILENT_ON)
			//printf("\nWarning: console output active while logging!");

		//fprintf(stderr, "\nEnabling Logfile '%s'\n\n", fname);
	
		logfile = fopen(fname, "w");
		if (!logfile) {
			perror("logfile");
			return 1;
		}

		sprintf(fname_csv, "%s/candump-%04d-%02d-%02d_%02d%02d%02d.csv",
			dir_dir,
			now.tm_year + 1900,
			now.tm_mon + 1,
			now.tm_mday,
			now.tm_hour,
			now.tm_min,
			now.tm_sec);
		csv = fopen(fname_csv, "w");
		if (!csv) {
			perror("csv");
			return 1;
		}

		sprintf(csv_name, "%s/candump-%04d-%02d-%02d_%02d%02d%02d.csv",
			dir_dir,
			now.tm_year + 1900,
			now.tm_mon + 1,
			now.tm_mday,
			now.tm_hour,
			now.tm_min,
			now.tm_sec);

		fprintf(csv,"TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE ; ID; DATA;\n");
		
		sprintf(logf110,"%s/candump_110.csv",dir_dir);
		log110 = fopen(logf110, "w");
		if (!log110) {
			perror("log110");
			return 1;
		}
		fprintf(log110,"TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; ID; DATA\n ");
		sprintf(logf120,"%s/candump_120.csv",dir_dir);
		log120 = fopen(logf120, "w");
		if (!log120) {
			perror("log120");
			return 1;
		}
		fprintf(log120,"TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; ID; DATA\n ");
		sprintf(logf130,"%s/candump_130.csv",dir_dir);
		log130 = fopen(logf130, "w");
		if (!log130) {
			perror("log130");
			return 1;
		}
		fprintf(log130,"TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; ID; DATA\n ");
		sprintf(logf140,"%s/candump_140.csv",dir_dir);
		log140 = fopen(logf140, "w");
		if (!log140) {
			perror("log140");
			return 1;
		}
		fprintf(log140,"TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; ID; DATA\n ");
	}



	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	while (1) {
	//usleep(100);
	pthread_mutex_lock(&runningMutex);
	//printf("%s\n", "CAN" );
		fflush(stdout);
	if(running){
		
		FD_ZERO(&rdfs);

		for (i=0; i<currmax; i++){
			FD_SET(s[i], &rdfs);
		}
		
		

		 for (i=0; i<currmax; i++) {  /* check all CAN RAW sockets */

		 	if (FD_ISSET(s[i], &rdfs)) {

				int idx;

				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);  
				msg.msg_flags = 0;

				nbytes = recvmsg(s[i], &msg, MSG_DONTWAIT );
				//printf("%i bytes received\n",nbytes );
				fflush(stdout);

				if (nbytes < 0) {
					//perror("read\n");
					continue;
				}

		 		if (nbytes < sizeof(struct can_frame)) {
		 			fprintf(stderr, "read: incomplete CAN frame\n");
		 			return 1;
		 		}


		 		if (count && (--count == 0))
		 			running = 0;

		 		if (bridge) {
		 			if (bridge_delay)
		 				usleep(bridge_delay);

		 			nbytes = write(bridge, &frame, sizeof(struct can_frame));
		 			if (nbytes < 0) {
		 				perror("bridge write");
		 				return 1;
		 			} else if (nbytes < sizeof(struct can_frame)) {
		 				fprintf(stderr,"bridge write: incomplete CAN frame\n");
		 				return 1;
		 			}
		 		}
		    
		 		for (cmsg = CMSG_FIRSTHDR(&msg);
		 		     cmsg && (cmsg->cmsg_level == SOL_SOCKET);
		 		     cmsg = CMSG_NXTHDR(&msg,cmsg)) {
		 			if (cmsg->cmsg_type == SO_TIMESTAMP)
		 				tv = *(struct timeval *)CMSG_DATA(cmsg);
		 			else if (cmsg->cmsg_type == SO_RXQ_OVFL)
		 				dropcnt[i] = *(__u32 *)CMSG_DATA(cmsg);
		 		}

		 		/* check for (unlikely) dropped frames on this specific socket */
		 		if (dropcnt[i] != last_dropcnt[i]) {

		 			__u32 frames;

		 			if (dropcnt[i] > last_dropcnt[i])
		 				frames = dropcnt[i] - last_dropcnt[i];
		 			else
		 				frames = 4294967295U - last_dropcnt[i] + dropcnt[i]; /* 4294967295U == UINT32_MAX */

		 			if (silent != SILENT_ON)
		 				printf("DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
		 				       frames, (frames > 1)?"s":"", cmdlinename[i], dropcnt[i]);

		 			if (log)
		 				fprintf(logfile, "DROPCOUNT: dropped %d CAN frame%s on '%s' socket (total drops %d)\n",
		 					frames, (frames > 1)?"s":"", cmdlinename[i], dropcnt[i]);

		 			last_dropcnt[i] = dropcnt[i];
		 		}

		 		idx = idx2dindex(addr.can_ifindex, s[i]);

		 		if (log) {
		 			/* log CAN frame with absolute timestamp & device */
		 			fprintf(logfile, "(%ld.%06ld) ", tv.tv_sec, tv.tv_usec);
		 			fprintf(logfile, "%*s ", max_devname_len, devname[idx]);

		 			//printf("I'm in log if");
		 			//fflush(stdout);
		 			/* without seperator as logfile use-case is parsing */
		 			int identificator =frame.can_id;
		 			//printf(""identificator)
					fprintf(logfile,"Timestep:%i\n",timestep_can);
		 			timestep_can++;
		 			fprint_canframe(logfile, &frame, "\n", 0);
		 			fprint_canframe_csv(csv, &frame, "\n",0);
		 			//fprintf(logfile,"%i\n",identificator);

		 			

		 			switch(identificator){
		 				case 110:
		 					fprint_canframe_csv(log110, &frame, "\n",0);
		 					break;
		 				case 120:
		 					fprint_canframe_csv(log120, &frame, "\n",0);
		 					break;
		 				case 130:
		 					fprint_canframe_csv(log130, &frame, "\n",0);
		 					break;
		 				case 140:
		 					fprint_canframe_csv(log140, &frame, "\n",0);
		 					break;
		 				default:
		 					break;
		 			}
		 		}

		 		if (logfrmt) {
		 			/* print CAN frame in log file style to stdout */
		 			printf("(%ld.%06ld) ", tv.tv_sec, tv.tv_usec);
		 			printf("%*s ", max_devname_len, devname[idx]);
		 			fprint_canframe(stdout, &frame, "\n", 0);
		 			goto out_fflush; /* no other output to stdout */
		 		}

		 		if (silent != SILENT_OFF){
		 			if (silent == SILENT_ANI) {
		 				printf("%c\b", anichar[silentani%=MAXANI]);
		 				silentani++;
		 			}
		 			goto out_fflush; /* no other output to stdout */
		 		}
		      
		 		printf(" %s", (color>2)?col_on[idx%MAXCOL]:"");

		 		switch (timestamp) {

		 		case 'a': /* absolute with timestamp */
		 			printf("(%ld.%06ld) ", tv.tv_sec, tv.tv_usec);
		 			break;

		 		case 'A': /* absolute with date */
		 		{
		 			struct tm tm;
		 			char timestring[25];

		 			tm = *localtime(&tv.tv_sec);
		 			strftime(timestring, 24, "%Y-%m-%d %H:%M:%S", &tm);
		 			printf("(%s.%06ld) ", timestring, tv.tv_usec);
		 		}
		 		break;

		 		case 'd': /* delta */
		 		case 'z': /* starting with zero */
		 		{
		 			struct timeval diff;

		 			if (last_tv.tv_sec == 0)   /* first init */
		 				last_tv = tv;
		 			diff.tv_sec  = tv.tv_sec  - last_tv.tv_sec;
		 			diff.tv_usec = tv.tv_usec - last_tv.tv_usec;
		 			if (diff.tv_usec < 0)
		 				diff.tv_sec--, diff.tv_usec += 1000000;
		 			if (diff.tv_sec < 0)
		 				diff.tv_sec = diff.tv_usec = 0;
		 			printf("(%03ld.%06ld) ", diff.tv_sec, diff.tv_usec);
				
		 			if (timestamp == 'd')
		 				last_tv = tv; /* update for delta calculation */
		 		}
		 		break;

		 		default: /* no timestamp output */
		 			break;
		 		}

		 		printf(" %s", (color && (color<3))?col_on[idx%MAXCOL]:"");
		 		printf("%*s", max_devname_len, devname[idx]);
		 		printf("%s  ", (color==1)?col_off:"");

		 		fprint_long_canframe(stdout, &frame, NULL, view);

		 		printf("%s", (color>1)?col_off:"");
		 		printf("\n");
			}
			

		 out_fflush:
			fflush(stdout);
		 }
	}
	pthread_mutex_unlock(&runningMutex);
}

	for (i=0; i<currmax; i++)
		close(s[i]);

	if (bridge)
		close(bridge);

	if (log)
		fclose(logfile);

	return 0;
}


void *exploraentradas(){
	time_t currtime;
		struct tm now;
		char inputfname[sizeof("/home/pi/Exo_beta/data/digital-2006-11-20_202026.csv")+1];
		tmr_t* d_tmr = tmr_new(d_isr);

		if (time(&currtime) == (time_t)-1) {
			perror("time");
			return 1;
		}

		localtime_r(&currtime, &now);
		sprintf(inputfname, "%s/digital-%04d-%02d-%02d_%02d%02d%02d.csv",
			dir_dir,
			now.tm_year + 1900,
			now.tm_mon + 1,
			now.tm_mday,
			now.tm_hour,
			now.tm_min,
			now.tm_sec);
	
				inputFILE = fopen(inputfname, "w");
				if (!inputFILE) {
					error("inputFILE");
					return 1;
				}
				fprintf(inputFILE, "%s","TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE" );

				for(int i = 0; i<NUM_DIGITALS-1;i++){
					if(i==9 && (key_trigger==1 || key_trigger ==2)){
						fprintf(inputFILE, "; TRIGGER");
					}else{if(i==8 && (key_trigger==1 || key_trigger ==2) && stop_trigger==1){
							fprintf(inputFILE, "; STOP_TRIGGER");
						}else
						fprintf(inputFILE, "; INPUT_%i", i+1 );
					
					}
				}
				fprintf(inputFILE, "\n" );
				//printf("\n");
				//printf("%s\n","TIMESTEP     PIN 1    PIN 4     PIN 5     PIN 6     PIN 7" );
				fflush(stdout);

				int wait_time = digital_time_sampling;
					if(wait_time<0)
						wait_time = 0;
	while(1){
		pthread_mutex_lock(&triggerMutex);
		//printf("%s\n", "DIGITAL" );
		fflush(stdout);
		if(trigger && flag_digital){
			/*printf("%s\n","Inside if_d" );
			fflush(stdout);*/
			flag_digital = 0;
			/*printf("%s\n","Flag_d to 0" );
			fflush(stdout);*/
			tmr_startus(d_tmr,wait_time);
			/*printf("%s\n","Timer_d started" );
			fflush(stdout);*/
			actual_times[array_index] = getactualtime();
			current_times[array_index] = gettimeprog();
			fflush(stdout);
			digital_timestamp[digital_time++] = clock();

			for(int i = 0; i < NUM_DIGITALS ; i++){
				if(i == 0){
					inputs_0[array_index] = timestep;
					array_index++;
				}else{
				inputs_0[array_index] = digitalRead(input_pins[i-1]);
				array_index++;
				}
			}
			if( array_index >= inputs_SIZE){
				for(int i = 0; i < inputs_SIZE/NUM_DIGITALS; i++){
					fprintf(inputFILE, "%ld; ", digital_timestamp[i] );
					fprintf(inputFILE, "%s; ", actual_times[i*NUM_DIGITALS]);
					fprintf(inputFILE, "%s; ", current_times[i*NUM_DIGITALS]);
					fprintf(inputFILE, "%i", 0 );
					for(int j = 0; j < NUM_DIGITALS; j++){
						fprintf(inputFILE,"; %i",inputs_0[i*NUM_DIGITALS+j]);
						//printf("INPUT_%i = %i    ",j,inputs_0[i*NUM_DIGITALS+j]);
					}
					fprintf(inputFILE, "\n");
					//printf("\n");
					fflush(stdout);
				}
				digital_time = 0;
				array_index = 0;
			}
			timestep++;
			
		}
		pthread_mutex_unlock(&triggerMutex);		
	}
}

void *exploraanalogicas(){
	time_t currtime;
		struct tm now;
		char inputfname[sizeof("/home/pi/Exo_beta/data/analog-2006-11-20_202026.csv")+1];

		tmr_t* a_tmr = tmr_new(a_isr);

		if (time(&currtime) == (time_t)-1) {
			perror("time");
			return 1;
		}

		localtime_r(&currtime, &now);
		sprintf(inputfname, "%s/analog-%04d-%02d-%02d_%02d%02d%02d.csv",
			dir_dir,
			now.tm_year + 1900,
			now.tm_mon + 1,
			now.tm_mday,
			now.tm_hour,
			now.tm_min,
			now.tm_sec);
	
				analogFILE = fopen(inputfname, "w");
				if (!analogFILE) {
					error("analogFILE");
					return 1;
				}
				fprintf(analogFILE, "%s","TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE" );

				for(int i = 0; i<NUM_ANALOGS-1;i++)
					fprintf(analogFILE, "; ANALOG_%i", i+1 );

				fprintf(analogFILE, "\n" );
				//printf("%s\n","TIMESTEP     PIN 1    PIN 4     PIN 5     PIN 6     PIN 7" );
				fflush(stdout);
				int wait_time = analog_time_sampling;
				if(wait_time < 0)
					wait_time = 0;
		while(1){
		pthread_mutex_lock(&triggerMutex);
		//printf("%s\n", "ANALOG" );
		fflush(stdout);
		if(trigger && flag_analog){
			/*printf("%s\n","Inside if analog" );
			fflush(stdout);*/
			flag_analog = 0;
			/*printf("%s\n","Flag to 0 analog" );
			fflush(stdout);*/
			tmr_startus(a_tmr,wait_time);
			/*printf("%s\n","Timer started analog" );
			fflush(stdout);*/
			actual_times_analog[array_index_analog] = getactualtime();
			current_times_analog[array_index_analog] = gettimeprog();

			analog_timestamp[analog_time++] = clock();

			for(int i = 0; i < NUM_ANALOGS ; i++){
				if(i == 0){
					analogs_0[array_index_analog] = timestep_analog;
					array_index_analog++;
				}else{
				if(i<5)
					analogs_0[array_index_analog] = read_voltage(0x68,i%5, 12, 1, 1);
				else
					analogs_0[array_index_analog] = read_voltage(0x69,i%5+1, 12, 1, 1);
				array_index_analog++;
				}

			}
			if( array_index_analog >= analogs_SIZE){
				for(int i = 0; i < analogs_SIZE/NUM_ANALOGS; i++){
					fprintf(analogFILE, "%ld; ", analog_timestamp[i] );
					fprintf(analogFILE, "%s; ", actual_times_analog[i*NUM_ANALOGS]);
					fprintf(analogFILE, "%s; ", current_times_analog[i*NUM_ANALOGS]);
					fprintf(analogFILE, "%i", 0 );
					for(int j = 0; j < NUM_ANALOGS; j++){
						if(j==0){
							fprintf(analogFILE,"; %i",(int)analogs_0[i*NUM_ANALOGS+j]);
							
						}else{fprintf(analogFILE,"; %G",analogs_0[i*NUM_ANALOGS+j]);
							//printf("INPUT_%i = %i    ",j,(int)analogs_0[i*NUM_ANALOGS+j]);
							}
					}
					fprintf(analogFILE, "\n");
					//printf("\n");
					fflush(stdout);
				}

				analog_time = 0;
				array_index_analog = 0;
			}
			timestep_analog++;
		}
		pthread_mutex_unlock(&triggerMutex);		
	}
}

int main_cansend(char *message)
{
	printf("Sending %s\n",message);
	fflush(stdout);
	int argc = 3;
	char *argv[argc];
	argv[0] = "./cansend";
	argv[1] = "can0";
	argv[2] = message;

	int s; /* can raw socket */ 
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	/* check command line options */
	if (argc != 3) {
		fprintf(stderr, "Usage: %s <device> <can_frame>.\n", argv[0]);
		return 1;
	}

	/* parse CAN frame */
	if (parse_canframe(argv[2], &frame)){
		fprintf(stderr, "\nWrong CAN-frame format!\n\n");
		fprintf(stderr, "Try: <can_id>#{R|data}\n");
		fprintf(stderr, "can_id can have 3 (SFF) or 8 (EFF) hex chars\n");
		fprintf(stderr, "data has 0 to 8 hex-values that can (optionally)");
		fprintf(stderr, " be seperated by '.'\n\n");
		fprintf(stderr, "e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / ");
		fprintf(stderr, "5AA# /\n     1F334455#1122334455667788 / 123#R ");
		fprintf(stderr, "for remote transmission request.\n\n");
		return 1;
	}

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, argv[1]);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		return 1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	/* disable default receive filter on this RAW socket */
	/* This is obsolete as we do not read from the socket at all, but for */
	/* this reason we can remove the receive list in the Kernel to save a */
	/* little (really a very little!) CPU usage.                          */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	/* send frame */
	if ((nbytes = write(s, &frame, sizeof(frame))) != sizeof(frame)) {
		perror("write");
		return 1;
	}

	//fprint_long_canframe(stdout, &frame, "\n", 0);

	close(s);

	return 0;
}

 char* gettimeprog() {
 	char *date = malloc(25+1);
 	struct timeval time_now;
 	gettimeofday(&time_now,NULL);
 	int diff = (time_now.tv_sec*1000000 + time_now.tv_usec) - (time_init.tv_sec*1000000 + time_init.tv_usec);
 	int microsecs = diff%1000000;
 	diff = diff/1000000;
 	int secs = diff%60;
 	int mins = (diff/60)%60;
 	int hours = diff/3600;

 	
 	sprintf(date,"%i:%i:%i.%06d",hours,mins,secs,microsecs);
 	//printf("%s\n\n", date);

 	return date;
 }

 char* getactualtime() {
 	char *date = malloc(30+1);
 	time_t currtime;
	struct tm now;

		if (time(&currtime) == (time_t)-1) {
			perror("time");
			return 1;
		}

		localtime_r(&currtime, &now);

		sprintf(date, "%02d-%02d-%04d; %02d:%02d:%02d",
			now.tm_mday,			
			now.tm_mon + 1,
			now.tm_year + 1900,
			now.tm_hour,
			now.tm_min,
			now.tm_sec);

 	return date;
 }
#define DEBOUNCE_TIME 20;
 double debounceTime;

 void stoptriggerDetected(void){
 	if (millis () < debounceTime) {
		debounceTime = millis () + DEBOUNCE_TIME ;
		return;
	}
	if(rising_edge_captured){
 	pthread_mutex_lock(&triggerMutex);
 	trigger=0;
 	//printf("Trigger is %i\n",trigger );
 	pthread_mutex_unlock(&triggerMutex);
 	pthread_mutex_lock(&runningMutex);
 	running=0;
 	//printf("Running is %i\n",running );
 	pthread_mutex_unlock(&runningMutex);
 	printf("RISING EDGE DETECTED ON STOP TRIGGER. STOPPING CAPTURING INPUTS AND CAN FRAMES\n");
 	fflush(stdout);
 	rising_edge_captured = 0;
 	}
 }

 void triggerDetected(void){
 	if (millis () < debounceTime) {
		debounceTime = millis () + DEBOUNCE_TIME ;
		return;
	}
	if(!rising_edge_captured){
 	pthread_mutex_lock(&triggerMutex);
 	trigger=1;
 	//printf("Trigger is %i\n",trigger );
 	pthread_mutex_unlock(&triggerMutex);
 	pthread_mutex_lock(&runningMutex);
 	running=1;
 	//printf("Running is %i\n",running );
 	pthread_mutex_unlock(&runningMutex);
 	pthread_mutex_lock(&analogMutex);
 	analog=1;
 	//printf("Running is %i\n",running );
 	pthread_mutex_unlock(&analogMutex);
 	printf("RISING EDGE DETECTED. CAPTURING INPUTS AND CAN FRAMES\n");
 	fflush(stdout);
 	rising_edge_captured = 1;
 	}
 }

 void a_isr(){
 	//printf("a_isr\n");
 	fflush(stdout);
 	pthread_mutex_lock(&triggerMutex);
 	flag_analog = 1;
 	pthread_mutex_unlock(&triggerMutex);
 	return;
 }

 void d_isr(){
 	//printf("d_isr wtf\n");
 	fflush(stdout);
 	pthread_mutex_lock(&triggerMutex);
 	flag_digital = 1;
 	pthread_mutex_unlock(&triggerMutex);
 	return;
 }

 void debug_isr(){
 	printf("%s\n","Timer interruption" );
 	fflush(stdout);
 }

 void parseCANdata(char* file, char* dir, int id){
 	FILE *fp;
	char *command = malloc(256*sizeof(char));

	sprintf(command,"/home/pi/Exo_beta/src/parse %s %s/ -%i", file, dir, id);
	printf("%s\n", command);
	fflush(stdout);

	fp = popen (command,"w");

	fclose(fp);

	printf("File %s parsed\n", file);
	fflush(stdout);
 }



int main (int argc, char **argv){
	shellcmd();
	reconfig();
	config();
	time_t currtime;
	struct tm now;
	
	if (time(&currtime) == (time_t)-1) {
		perror("time");
		return 1;
	}

	localtime_r(&currtime, &now);
	sprintf(dir,"/home/pi/Exo_beta/data/%04d-%02d-%02d",now.tm_year+1900,now.tm_mon+1,now.tm_mday);
	sprintf(dir_dir, "/home/pi/Exo_beta/data/%04d-%02d-%02d/%02d_%02d_%02d",now.tm_year+1900,now.tm_mon+1,now.tm_mday,now.tm_hour,now.tm_min,now.tm_sec);
	mkdir(dir,0777);
	mkdir(dir_dir,0777);

	gettimeofday(&time_init,NULL);

	if(key_trigger == 1 || key_trigger == 2)
		wiringPiISR(TRIGGER_PIN,INT_EDGE_RISING,triggerDetected);
	if((key_trigger == 1 || key_trigger == 2) && stop_trigger==1)
		wiringPiISR(STOP_TRIGGER_PIN, INT_EDGE_RISING,stoptriggerDetected);


	pthread_create(&thread2,NULL,*explorateclado,NULL);
	pthread_create(&thread1,NULL,*main_candump,NULL);
	pthread_create(&thread3,NULL,*exploraentradas,NULL);
	pthread_create(&thread4,NULL,*exploraanalogicas,NULL);
	pthread_join(thread2, NULL);
	pthread_join(thread1, NULL);
	pthread_join(thread3, NULL);
	pthread_join(thread4, NULL);
	printf("Threads joined");
	return 0;
}

