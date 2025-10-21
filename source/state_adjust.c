/*
 * state_adjust.c
 *
 *  Created on: Sep 2, 2025
 *      Author: mcaplan
 */

#include "state_adjust.h"

#include "fsl_debug_console.h"
#include "fsl_csi.h"
#include "fsl_elcdif.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include <stdbool.h>
#include "global_buffers.h"
#include "lcd_st7701.h"
#include "camera_ov5640.h"
#include "utils.h"
#include "timer.h"
#include "image_processing.h"

static inline int clampi(int v, int low, int high){
	if( v < low ) return low;
	if (v > high) return high;
	return v;
}

// init things
zoom_level_t last_zoom_level = zoom_level_1;
volatile struct inputMailbox adjustMailbox = {
		.encoder_a = 0,
		.encoder_b = 0,
		.encoder_c = 0,
		.multifunction_toggle = 0
};

encoder_state_t _encoder_a = 0;
encoder_state_t _encoder_b = 0;
encoder_state_t _encoder_c = 0;


// moving for smooth input
int current_gain = 0;
int last_gain = 0;


//void adjustExposure(int exposure)

void STATE_Adjust_Enter(void) {
    PRINTF("ADJUST: Entering adjust state\r\n");

	// check if multifunction toggle is requested and switch encoder control as necessary


    // store local values
    _encoder_a = adjustMailbox.encoder_a;
    _encoder_b = adjustMailbox.encoder_b;
    _encoder_c = adjustMailbox.encoder_c;
	//clear mailboxes
    adjustMailbox.encoder_a = 0;
    adjustMailbox.encoder_b = 0;
    adjustMailbox.encoder_c = 0;

	//    check the mailbox values rcved and update if necessary
	if(_encoder_a == CW){
		current_gain = db_options.r_gain+=10;
		last_gain = current_gain;
	}
	if(_encoder_a == CCW){
		current_gain = db_options.r_gain-=10;
		last_gain = current_gain;
	}

	if(_encoder_b == CW){
		current_gain = db_options.g_gain+=10;
	   last_gain = current_gain;
	}
	if(_encoder_b == CCW){
	current_gain = db_options.g_gain-=10;
	   last_gain = current_gain;
	}

	if(_encoder_c == CW){
		current_gain = db_options.b_gain+=10;
		  last_gain = current_gain;
	  }
	if(_encoder_c == CCW){
	current_gain = db_options.b_gain-=10;
	  last_gain = current_gain;
	}


    PRINTF("ADJUST R: %d\r\n", db_options.r_gain);
    PRINTF("ADJUST G: %d\r\n", db_options.g_gain);
    PRINTF("ADJUST B: %d\r\n", db_options.b_gain);
	



//    if(last_zoom_level == zoom_level_1){
//        zoom_level = zoom_level_2;
    // }
    // else if(last_zoom_level == zoom_level_2){
    //     zoom_level = zoom_level_3;
    // }
    // else if(last_zoom_level == zoom_level_3){
    //     zoom_level = zoom_level_1;
    // }
    // last_zoom_level = zoom_level;
    

    STATE_set_current(COMPOSE);
    STATE_transition();
}

void STATE_Adjust_Exit(void) {
    PRINTF("ADJUST: Exiting adjust state\r\n");
    PRINTF("ADJUST: current_state: %d\r\n", zoom_level);
    PRINTF("ADJUST: current_state: %d\r\n", last_zoom_level);
}
