/*
 * debug_shell.c
 *
 *  Created on: Jun 4, 2025
 *      Author: mcaplan
 */


#include "debug_shell.h"
#include "fsl_debug_console.h"

// variables
shell_input_t input;
char c;

char history[SHELL_HISTORY_LEN][64];
int hist_i = 0;
int hist_len  = 0;


//interface that will just run and block the bm loop
 void SHELL_WaitForInput(void){

	 PRINTF("photOS $ ");

	 int i = 0;

	 while(1){
		 c = (char)GETCHAR();

		 if(c == '\r' || c == '\n') {  // Enter pressed
		             PUTCHAR('\r');
		             PUTCHAR('\n');
//		             save to history
		             if( hist_len < SHELL_HISTORY_LEN ){
						 hist_i = hist_len;
						 hist_len++;
						 strcpy(history[hist_i], input.inputString);
		             }
		             break;
		         }
		 	 	 else if(c == 127){ //backspace
		 	 		 if(i>0){
		 	 			PUTCHAR('\b');
		 	 			PUTCHAR(' ');
		 	 			PUTCHAR('\b');
		 	 			i--;
		 	 			input.inputString[i] = '\0';
		 	 		 }

		 	 	 }
		 	 	 else if (c == 91) { //arrow key finder
		 	 		 char dir =  GETCHAR();
		 	 		 if(dir == 'A'){
//		 	 			 chill, the index is already lagging

		 	 		 }
		 	 		 else if(dir == 'B'){
						 if (hist_i < hist_len){
						 hist_i++;
						 }

		 	 		 }

		 	 		PRINTF("\r\033[K");           // Go to start + clear line
		 	 		PRINTF("photOS $ %s", history[hist_i]);  // Reprint prompt + history
		 	 		// Update your input buffer too
		 	 		strcpy(input.inputString, history[hist_i]);
		 	 		i = strlen(history[hist_i]);

		 	 	 }
		         else if (i < (sizeof(input.inputString)-1) && c >= 32 && c <= 126){  // Normal character
		        	 input.inputString[i] = c;
		        	 PUTCHAR(c);  // Echo character back
		        	 i++;
		         }
	 }

//	 after break, go handle the input
	 parseInput();
	 handleInput();

//	 clear the input
	 //	 clear the input
	 for(int j = 0; j < (sizeof(input.inputString)-1); j++){
		 input.inputString[j] = '\0';
	 }

//clear the arg
	 for(int j = 0; j < (sizeof(input.argString)-1); j++){
	 		 input.argString[j] = '\0';
	 	 }

 }

// execution logic
 void handleInput(void){

	 if (commandEquals("hello") || commandEquals("hi")) {
		 PRINTF("hi to you! \r\n");
	 }
	 else if (commandEquals("transfer")) {
		 if( argEquals("start") ){
			 PRINTF("starting MSC device");
			 // start the usb app
			 USB_DeviceAppStart();

		 }
		 else if( argEquals("stop") ){
			 PRINTF("stopping MSC device");
			 USB_DeviceAppStop();
		 }
		 else{
			 PRINTF("usage: transfer {start | stop}");
		 }
	 }
	 else if (commandEquals("screen")) {
	 		 if( argEquals("on") ){
	 			 PRINTF("initializing screen");
	 			 DISPLAY_Init();
	 		 }
	 		 else if( argEquals("off") ){
//	 			 PRINTF("stopping MSC device");
//	 			 USB_DeviceAppStop();
	 		 }
	 		 else{
	 			 PRINTF("usage: screen {on | off}");
	 		 }
	 	 }
	 else {
		 PRINTF("received input %s, nothing to do", input.inputString);
	 }

	 //	 newline
	 PRINTF("\r\n");
 }

 void parseInput(void) {
     char* space_pos = strchr(input.inputString, ' ');

     if (space_pos) {
         // Copy command part to struct
         int cmd_len = space_pos - input.inputString;
         strncpy(input.commandString, input.inputString, cmd_len);

         // Copy args part (skip the space)
         strcpy(input.argString, space_pos + 1);

     } else {
         // No args, just command
         strcpy(input.commandString, input.inputString);
     }
 }


// wrap the strcmp and invert it for syntax niceness
 int commandEquals(char* commandString){
	 return !strcmp(input.commandString, commandString);
}

 int argEquals(char* argString){
	 return !strcmp(input.argString, argString);
}
