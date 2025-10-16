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
		.encoder_a = 0
};

encoder_state_t _encoder_a = 0;;
void STATE_Adjust_Enter(void) {
    PRINTF("ADJUST: Entering adjust state\r\n");

    _encoder_a = adjustMailbox.encoder_a;

    //    check the mailbox
    if(_encoder_a == CW){
    	db_options.r_gain+=10;
    }
    else if(_encoder_a == CCW){
    	db_options.r_gain-=10;
    }

    PRINTF("ADJUST: %d\r\n", db_options.r_gain);
	adjustMailbox.encoder_a = 0;

    

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
