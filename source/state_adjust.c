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

typedef enum {
	rgb = 0xA,
	exposure = 0xB,
 } control_state_t;


control_state_t whichControl = rgb; //set up in rgb control to start

void STATE_Adjust_Enter(void) {
    PRINTF("ADJUST: Entering adjust state\r\n");

	// check if multifunction toggle is requested and switch encoder control as necessary
	uint8_t toggle = adjustMailbox.multifunction_toggle;
	adjustMailbox.multifunction_toggle = 0; //clear 
	
	if(toggle){
		if(whichControl == rgb) whichControl = exposure;
		else if(whichControl == exposure) whichControl = rgb;
	}

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
		if (whichControl == rgb){
			db_options.r_gain+=10;
		}
		else if (whichControl == exposure){
			// dispatch exposure settings to camera
			ov5640_settings.exposure+=50;
			OV5640_SetExposure(ov5640_settings.exposure);
		}
	}
	if(_encoder_a == CCW){
		if (whichControl == rgb){
			db_options.r_gain-=10;
		}
		else if (whichControl == exposure){
			// dispatch exposure settings to camera
			ov5640_settings.exposure-=50;
			OV5640_SetExposure(ov5640_settings.exposure);
		}
	}

	if(_encoder_b == CW){
		if(whichControl == rgb){
			db_options.g_gain+=10;
		}
		else if(whichControl == exposure){
			ov5640_settings.gain+=1;
			OV5640_SetGain(ov5640_settings.gain);
		}
	}
	if(_encoder_b == CCW){
		if(whichControl == rgb){
			db_options.g_gain-=10;
		}
		else if(whichControl == exposure){
			ov5640_settings.gain-=1;
			OV5640_SetGain(ov5640_settings.gain);
		}
	}

	if(_encoder_c == CW){
		if(whichControl == rgb){
			db_options.b_gain+=10;
		}
		else if(whichControl == exposure){
			if(last_zoom_level == zoom_level_1) zoom_level = zoom_level_2;
			if(last_zoom_level == zoom_level_2) zoom_level = zoom_level_3;
			last_zoom_level = zoom_level;
		}
	}
	if(_encoder_c == CCW){
		if(whichControl == rgb){
			db_options.b_gain-=10;
		}
		else if(whichControl == exposure){
			if(last_zoom_level == zoom_level_3) zoom_level = zoom_level_2;
			if(last_zoom_level == zoom_level_2) zoom_level = zoom_level_1;
			last_zoom_level = zoom_level;
		}
	}


    PRINTF("ADJUST EXP: %d\r\n", ov5640_settings.exposure);
    // PRINTF("ADJUST WPT: %d\r\n", ov5640_settings.exposure_width);
    // PRINTF("ADJUST B: %d\r\n", db_options.b_gain);
	



    STATE_set_current(COMPOSE);
    STATE_transition();
}

void STATE_Adjust_Exit(void) {
    PRINTF("ADJUST: Exiting adjust state\r\n");
    PRINTF("ADJUST: current_state: %d\r\n", zoom_level);
    PRINTF("ADJUST: current_state: %d\r\n", last_zoom_level);
}
