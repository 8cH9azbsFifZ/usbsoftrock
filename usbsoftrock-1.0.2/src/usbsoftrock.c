/***
 * SoftRock USB I2C host control program
 * Copyright (C) 2009 Andrew Nilsson (andrew.nilsson@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Based on powerSwitch.c by Christian Starkjohann,
 * and usbtemp.c by Mathias Dalheimer
 * of Objective Development Software GmbH (2005)
 * (see http://www.obdev.at/avrusb)
 */

/*
   General Description:
   This program queries and controls the AVR Si570 hardware.
   It must be linked with libusb, a library for accessing the USB bus from
   Linux, FreeBSD, Mac OS X and other Unix operating systems. Libusb can be
   obtained from http://libusb.sourceforge.net/.
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <getopt.h>

#ifdef HAVE_LIBNCURSES
#include <ncurses.h>
#endif

/* Needed to compile on older FC3 systems */
#if defined __linux
#include <sys/types.h>
#include <linux/limits.h>
#endif

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */

#include "operations.h"
#include "config_ops.h"
#include "interactive.h"

#define USBDEV_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   0x05DC  /* Obdev's free shared PID */
/* Use obdev's generic shared VID/PID pair and follow the rules outlined
 * in firmware/usbdrv/USBID-License.txt.
 */


// Number of attempts to complete an USB operation before giving up.
#define USB_MAX_RETRIES		3

double multiplier = 4;

#ifndef HAVE_LIBCONFIG
/* If no libconfig, only static vendor config */
# define VENDOR_NAME				"www.obdev.at"
# define PRODUCT_NAME			"DG8SAQ-I2C"
#endif

extern char    serialNumberString[256];

#define MAX_COMMAND_ARGS 3

int verbose = 0;

int i2cAddress = SI570_I2C_ADDR;
double fXtall = SI570_NOMINAL_XTALL_FREQ;
double startupFreq = SI570_DEFAULT_STARTUP_FREQ;

int setByValue = 0;
int PTT = 0;
int keys = 0;		// paddle status 0 : none pressed    1:  dit pressed    2: dah pressed   3: both
int firmware_PTT = 0;
int CW_tone = 700;

int major;
int minor;

static void usage(char *name)
{
  fprintf(stderr, "usbsoftrock %s\n", VERSION);
  fprintf(stderr, "usage: %s [OPTION] COMMAND\n\n", name);
  fprintf(stderr, "OPTION is one or more of\n");
  fprintf(stderr, "  -a                             Advanced firmware present\n");
  fprintf(stderr, "                                 i.e. let the firmware calculate registers\n");
  fprintf(stderr, "  -d                             Enter a mode that listens for commands via UDP.\n");
//  fprintf(stderr, "  -h <freq MHz>                  Enable subharmonic (/3) sampling from frequency (DEFAULT off)\n");
  fprintf(stderr, "  -h                             PTT status by reading hardware port\n");
  fprintf(stderr, "                                 Mobo only.\n");
  fprintf(stderr, "  -i <address>                   I2C address in DECIMAL (DEFAULT = 85 (0x55))\n"); 
  fprintf(stderr, "  -m <multiplier>                Multiplication factor for frequency (DEFAULT = 4)\n");
  fprintf(stderr, "  -p <port num>                  Port to listen for UDP datagrams (DEFAULT = 19004)\n");
  fprintf(stderr, "  -s <startup frequency MHz>     Factory programmed startup frequency (DEFAULT = 56.32)\n");
  fprintf(stderr, "  -u <serial number>             Serial Number of Device\n");
  fprintf(stderr, "  -v                             Verbose output (fairly useful)\n");
  fprintf(stderr, "  -vv                            Even more verbose output (debugging)\n");
  fprintf(stderr, "  -x <calibrated xtall freq MHz> Corrected XTALL frequency of Si570 device calculated\n");
  fprintf(stderr, "                                 through the use of the calibrate command immediately\n");
  fprintf(stderr, "                                 after startup.\n");
  fprintf(stderr, "COMMAND is one of\n");
  fprintf(stderr, "  calibrate (may require -s option)\n");
  fprintf(stderr, "  getfreq\n");
  fprintf(stderr, "  getregisters\n");
#ifdef HAVE_LIBNCURSES
  fprintf(stderr, "  interactive\n");
#endif
  fprintf(stderr, "  getptt                                 (-h option for Mobo only)\n");
  fprintf(stderr, "  getkeys                                (PE0FKO+TF3LJ+Mobo)\n");
  fprintf(stderr, "  gettone\n");
  fprintf(stderr, "  ptt {on|off}\n");
  fprintf(stderr, "  set bpf {on|off}                       (PE0FKO+TF3LJ+Mobo)\n");
  fprintf(stderr, "  set bpf_addr <band> <filter>           (PE0FKO >= 15.12+Mobo)\n");
  fprintf(stderr, "  set bpf_point <crossover> <f in MHz>   (PE0FKO+TF3LJ+Mobo)\n");
  fprintf(stderr, "  set lpf {on|off}                       (TF3LJ+Mobo only)\n");
  fprintf(stderr, "  set lpf_addr <band> <filter>                   \"     \n");
  fprintf(stderr, "  set lpf_point <crossover> <f in MHz>           \"     \n");
  fprintf(stderr, "  set freq <frequency in MHz>\n");
  fprintf(stderr, "  set si570_addr <i2c address in decimal>\n");
  fprintf(stderr, "  set si570_multiplier [band] <decimal factor>  (PE0FKO>=15.12+Mobo)\n");
  fprintf(stderr, "  set startup <frequency in MHz>         (PE0FKO+TF3LJ+Mobo)\n");
  fprintf(stderr, "  set xtall <frequency in MHz>           (PE0FKO+TF3LJ+Mobo)\n");
  fprintf(stderr, "  status\n\n");
  fprintf(stderr, "  where TF3LJ = Lofturs AtMega168 derivative\n");
  fprintf(stderr, "        Mobo  = Mobo 4.3 Project AT90USB162 Firmware\n");
}


int do_command(usb_dev_handle *handle, char **argv, int argc, char *result) {

  sprintf(result, "ok");

  if (strcmp(argv[0], "get") == 0) {
    if(argv[1][0] == 'p'){
	if (firmware_PTT) PTT = getPTT(handle);	// this reads the PTT status from the connected Mobo
        sprintf(result, "ok %d", PTT);
	}
    else if (argv[1][0] == 'k'){
	keys = getkeys(handle);
        sprintf(result, "ok %d", keys);
	}
    else if(argv[1][0] == 'f'){
      double freq; 
      if (setByValue)
       freq = readFrequencyByValue(handle);
      else
        freq = getFrequency(handle);

      if (freq != 0.00) {
        if (verbose >= 2) printf("Frequency   : %f (x %.2f)\n", freq / multiplier, multiplier);
        sprintf(result, "ok %f", freq / multiplier);
      	} 
      else
	{
        sprintf(result, "error");
        return -1;
        }
      } 
    else if(argv[1][0] == 't'){
        sprintf(result, "ok %d", CW_tone);
        }
    else if (strcmp(argv[1], "si570_multiplier") == 0) {
        double sub, mul;
	// Just works for non per-band multipliers
        if (readMultiplyLO(handle, 0, &mul, &sub) == 0)
            sprintf(result, "ok %f", mul);
        else {
            sprintf(result, "error");
	    return -1;
	}
    } 
    else if (strcmp(argv[1], "local_multiplier") == 0) {
        sprintf(result, "ok %f", multiplier);
    } 
    else {
        sprintf(result, "error");
	return -1;
    } // end if "local multiplier"

  } // end if "get"

else if ((strcmp(argv[0], "set") == 0) && (argc >= 2)) {

    if ((strcmp(argv[1], "ptt") == 0) && (argc >= 1)) {

	PTT = (strncmp(argv[2],"on",2) == 0) ? 1 : 0;
        setPTT(handle, PTT);
      
    } else if ((strcmp(argv[1], "bpf") == 0)) {

	setBPF(handle, (strncmp(argv[2],"on",2) == 0) ? 1 : 0);

    } else if (strcmp(argv[1], "freq") == 0) {

      if (setByValue)
        setFreqByValue(handle, atof(argv[2]));
      else
        setFrequency(handle, atof(argv[2]));
    } else if (strcmp(argv[1], "tone") == 0) {
        CW_tone = atof(argv[2]);
      
    } else if (strcmp(argv[1], "local_multiplier") == 0) {
      multiplier = atof(argv[2]);
    } else {
      sprintf(result, "error");
      return -1;
    } 
  } else {
    sprintf(result, "error");
    return -1;
  }
  
  return 0;
}



/**
 * Main routine. Parse commandline args and trigger actions.
 */
int main(int argc, char **argv) {
  usb_dev_handle      *handle = NULL;
  char * usbSerialID = NULL;
  int c;

// moved this malloc() here instead of within the while(1) loop
// to prevent memory leakage problem
// as *args is not free'ed.

  char **args = malloc(MAX_COMMAND_ARGS * sizeof(char *));
  int port = 19004;
  int daemon = 0;

  // Read options
  while ( (c = getopt(argc, argv, "adhi:m:p:s:u:vx:")) != -1) {
    switch (c) {
    case 'i':
      i2cAddress = atoi(optarg);
      break;
    case 'a':
      setByValue = 1;
      break;
    case 'd':
      daemon = 1;
      break;
    case 'h':
      firmware_PTT = 1;
      break;
    case 'm':
      multiplier = atof(optarg);
      break;
    case 'p': 
      port = atoi(optarg);
      break;
    case 's':
      startupFreq = atof(optarg);
      break;
    case 'x':
      fXtall = atof(optarg);
      break;
    case 'u':
      usbSerialID = optarg;
      break;
    case 'v':
      verbose++;
      break;
    default: /* '?' */
      usage(argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  if (verbose) {
    printf("I2C Address = %X\n", i2cAddress);
    printf("fXtall = %f\n", fXtall);
    printf("multiplier = %f\n", multiplier);
    printf("startupFreq = %f\n", startupFreq);
  }

  if((argc <= optind) && (daemon == 0)){
	usage(argv[0]);
	exit(1);
  }

  usb_init();
  char attempt=0, error=0;
  do {
	attempt++;
// lsusb -d 16c0:05dc
// iManufacturer           1 www.ov-lennestadt.de
// iProduct                2 FiFi-SDR
// iSerial                 3 F50000064CE267E3535220E80B0B0F1F
#define VENDOR_NAME "www.ov-lennestadt.de"
#define PRODUCT_NAME "FiFi-SDR"
//#define usbSerialID "F50000064CE267E3535220E80B0B0F1F"


	error=usbOpenDevice(&handle, USBDEV_SHARED_VENDOR, VENDOR_NAME, USBDEV_SHARED_PRODUCT, PRODUCT_NAME, usbSerialID);
	if(error != 0){
	  fprintf(stderr, "Could not open USB device \"%s\" with vid=0x%x pid=0x%x, retrying\n", PRODUCT_NAME, USBDEV_SHARED_VENDOR, USBDEV_SHARED_PRODUCT);
	  sleep(2*attempt);
	}
  } while (error && attempt < USB_MAX_RETRIES);
  if (error) {
	fprintf(stderr, "Permanent problem opening usb device. Giving up.\n");
	exit(1);
  }

  unsigned short version = readVersion(handle);
  major = (version & 0xFF00) >> 8;
  minor = (version & 0xFF);

  /* Relocate lower later */
  if (daemon) {
    printf("Starting daemon...\n");

    int socket_desc;

    socket_desc=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (socket_desc==-1)
      perror("Create socket");

    struct sockaddr_in address;
    /* type of socket created in socket() */
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    /* bind the socket to the port specified above */
    int retval;
    if (bind(socket_desc,(struct sockaddr *)&address,sizeof(address)) != 0) {
      fprintf(stderr, "Error binding to port %d\n", port);
      exit(0); 
    }

    while (1) {
      ssize_t bytes;
      char buffer[1024];
      struct sockaddr_in clnt;
      socklen_t clnt_len;
      clnt_len = sizeof(clnt);

      bytes = recvfrom (socket_desc, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&clnt, &clnt_len);
      if (bytes > 0) {
        buffer[bytes] = 0;
        if (verbose >= 2)
          printf("Returned %d bytes from %s: %s\n", (int)bytes, inet_ntoa(clnt.sin_addr), buffer);

        if (strncmp(buffer, "quit", 4) == 0) {
          if (verbose)
            printf("Quit command received\n");
          exit(0);
        }

        char *saveptr;
        char *token;
        int argn = 0;

        for (int i=0; i < MAX_COMMAND_ARGS;i++)
          args[i] = NULL;

        token = strtok_r(buffer, " ", &saveptr);
        while (token != NULL) {
          args[argn] = strcpy(malloc(strlen(token) + 1), token);
          argn++;          
          token = strtok_r(NULL, " ", &saveptr);
        }

        // Execute command here
        char result[100];
        do_command(handle, args, argn, result);

        // Cleanup
        for (int i=0; i < MAX_COMMAND_ARGS;i++) {
          if (args[i] != NULL)
            free(args[i]);
        }

        int retlen = strlen(result);
        if (sendto(socket_desc, result, retlen,0, (struct sockaddr *) &clnt, clnt_len) != retlen) {
          perror("Failed to send ack");
          exit(1);
        }

      } else {
      	fprintf(stderr, "recvfrom returned %d\n", (int)bytes);
      }
    }

    close(socket_desc);
    exit(0);
  }
 
  /* Device has been opened - perform the requested operation */
  if (strcmp(argv[optind], "getregisters") == 0) {

	getRegisters(handle);

  } else if(strcmp(argv[optind], "getfreq") == 0){
	double freq; 
	if (setByValue)
		freq = readFrequencyByValue(handle);
	else
		freq = getFrequency(handle);

	if (freq != 0.00)
		printf("Frequency   : %f (x %.2f)\n", freq / multiplier, multiplier);

#ifdef HAVE_LIBNCURSES
  } else if (strcmp(argv[optind], "interactive") == 0) {
      run_interactive(handle);
#endif

  } else if (strcmp(argv[optind], "getptt") == 0){
	if (firmware_PTT) PTT = getPTT(handle);
        printf("PTT   : %d\n", PTT);

 } else if (strcmp(argv[optind], "getkeys") == 0){
	keys = getkeys(handle);
        printf("Paddles: %d\n", keys);

 } else if (strcmp(argv[optind], "gettone") == 0){
        printf("CW Tone: %d\n", CW_tone);

  } else if ((strcmp(argv[optind], "ptt") == 0) && (argc >= optind + 1)) {

	PTT = (strcmp(argv[optind+1],"on") == 0) ? 1: 0;
	setPTT(handle, PTT);
	printf("PTT set to %d\n", PTT);

  } else if (strcmp(argv[optind], "calibrate") == 0) {
	
	calibrate(handle);

  } else if ((strcmp(argv[optind], "set") == 0) && (argc >= optind + 2)) {

    if ((strcmp(argv[optind+1], "bpf_addr") == 0) && (argc >= optind + 3)) {
      // set bpf_addr index value
      setBPFAddress(handle, atoi(argv[optind+2]), atoi(argv[optind+3]));
      
    } else if ((strcmp(argv[optind+1], "bpf_point") == 0) && (argc >= optind + 3)) {
      // set bpf_point index (int) value (float)
      setBPFCrossOver(handle, atoi(argv[optind+2]), atof(argv[optind+3]));
      
    } else if ((strcmp(argv[optind+1], "bpf") == 0) && (argc >= optind + 2)) {

        setBPF(handle, (strcmp(argv[optind+2],"on") == 0) ? 1 : 0);

    } else if ((strcmp(argv[optind+1], "lpf") == 0) && (argc >= optind + 2)) {

        setLPF(handle, (strcmp(argv[optind+2],"on") == 0) ? 1 : 0);

    } else if ((strcmp(argv[optind+1], "lpf_addr") == 0) && (argc >= optind + 3)) {
      // set bpf_addr index value
      setBPFAddress(handle, atoi(argv[optind+2]), atoi(argv[optind+3]));
      displayLPFs(handle);
      
    } else if ((strcmp(argv[optind+1], "lpf_point") == 0) && (argc >= optind + 3)) {
      // set lpf_point index (int) value (float)
      setLPFCrossOver(handle, atoi(argv[optind+2]), atof(argv[optind+3]));
            
    } else if (strcmp(argv[optind+1], "freq") == 0) {

      if (setByValue)
        setFreqByValue(handle, atof(argv[optind+2]));
      else
        setFrequency(handle, atof(argv[optind+2]));
      
    } else if ((strcmp(argv[optind+1], "si570_addr") == 0) && (argc >= optind + 2)) {
      
      setSi570Address(handle, atoi(argv[optind+2]));
      
    } else if (strcmp(argv[optind+1], "si570_multiplier") == 0) {
        
	int index = 0;
	int valueIndex = optind+2;
	// If there are 2 args after the variable name, one is index
	if (argc >= optind + 3) {
	  index = atoi(argv[optind+2]);
	  valueIndex++;
	}
	
        double sub, mul;
        readMultiplyLO(handle, index, &mul, &sub);
        mul = atof(argv[valueIndex]);
        setMultiplyLo(handle, index, mul, sub);
        if (verbose)
            printf("Set multiply [%d] to : %f\n", index, mul);
        
    } else if (strcmp(argv[optind+1], "xtall") == 0) {

        setXtallFrequency(handle, atof(argv[optind+2]));

    } else if (strcmp(argv[optind+1], "startup") == 0) {

	setStartupFrequency(handle, atof(argv[optind+2]));

    } else {
	usage(argv[0]);
	exit(1);
    }

  } else if (strcmp(argv[optind], "status") == 0) {

	printf("USB SerialID: %s\n", serialNumberString);

	if (major >= 15) {
		readFrequencyByValue(handle);
		readStartupFreq(handle);
		readXtallFreq(handle);
		readSmoothTunePPM(handle);
		if (major >= 16 || minor >= 12) {
			readSi570Address(handle);
		}

		displayBands(handle);
		displayLPFs(handle);
		
		/*
		if (major >= 16 || ((major >= 15) && (minor >= 12))) {
		  displayBands(handle);
		} else if (minor >= 10) {
		  double sub, mul;
		  readMultiplyLO(handle, 0, &mul, &sub);
		  printf("LO Subtract : %f\n", sub);
		  printf("Multiply    : %f\n", mul);
		}
		//displayBPFFilters(handle);
		//displayLPFFilters(handle);*/
	}
  } else {
	usage(argv[0]);
	exit(1);
  }
  usb_close(handle);
  return 0;
}

