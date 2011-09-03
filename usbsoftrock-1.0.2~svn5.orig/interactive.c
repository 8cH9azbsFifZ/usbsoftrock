#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_LIBNCURSES
#include <ncurses.h>
#endif

/* Needed to compile on older FC3 systems */
#if defined __linux
#include <sys/types.h>
#include <linux/limits.h>
#endif

#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */

#include "operations.h"
#include "interactive.h"

extern int setByValue;
extern double multiplier;

void run_interactive(usb_dev_handle *handle) {
	char ch;
	int row, col;

	initscr();
	getmaxyx(stdscr,row,col);
	keypad(stdscr, TRUE);
	cbreak();
	noecho();

	double currentFreq;
	if (setByValue == 1)
		currentFreq = readFrequencyByValue(handle);
	else 
		currentFreq = getFrequency(handle);
	currentFreq = currentFreq / multiplier;

	attron(A_BOLD);
	mvprintw(1, 0, "%3.6f MHz", currentFreq);
	attroff(A_BOLD);
	mvprintw(3, 0, "q/a = Up/Down 100 KHz\n");
	printw("w/s = Up/Down  10 KHz\n");
	printw("e/d = Up/Down   1 KHz\n");
	printw("r/f = Up/Down 100 Hz\n");
	printw("t/g = Up/Down  10 Hz\n");
	printw("x   = Exit");
 
	refresh();

	int ptt = 0;
	
	while ((ch = getch()) != 'x') {		
		int inc = 0;
		switch (ch) {
		case 'q':
			inc = 100000;
			break;
		case 'a':
			inc = -100000;
			break;
		case 'w':
			inc = 10000;
			break;
		case 's':
			inc = -10000;
			break;
		case 'e': 
			inc = 1000;
			break;
		case 'd':
			inc = -1000;
			break;
		case 'r':
			inc = 100;
			break;
		case 'f':
			inc = -100;
			break;
		case 't':
			inc = 10;
			break;
		case 'g':
			inc = -10;
			break;
		case 'p':
			ptt = ptt == 0 ? 1 : 0;
			setPTT(handle, ptt);
		default:
			inc = 0;
		}
		currentFreq = currentFreq + inc / 1000000.0;

		/* Now set the new freq */
		if (setByValue)
			setFreqByValue(handle, currentFreq);
		else
			setFrequency(handle, currentFreq);

		attron(A_BOLD);
		mvprintw(1, 0, "%3.6f MHz", currentFreq);
		attroff(A_BOLD);
		refresh();
	}

	endwin();
}
