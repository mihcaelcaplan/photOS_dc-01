/*
 * debug_shell.h
 *
 *  Created on: Jun 4, 2025
 *      Author: mcaplan
 */

#ifndef DEBUG_SHELL_H_
#define DEBUG_SHELL_H_

//variables
typedef struct _input {
	char inputString[64];
	char commandString[32];
	char argString[32];
} shell_input_t;

#define SHELL_HISTORY_LEN 10

//interface that will just run and block the bm loop
 void SHELL_WaitForInput(void);

// execution logic
 void handleInput(void);

// compare inputString to any command string
 int commandEquals(char* commandString);
 int argEquals(char* argString);

// parse args
void parseInput(void);


#endif /* DEBUG_SHELL_H_ */
