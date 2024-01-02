/*
 * GPS.c
 *
 *  Created on: Apr 20, 2023
 *      Author: thean
 */

#include "gps.h"

extern GPS_Handle gps;

void parse_nmea(uint8_t *nmea_string) {
	char *line;
	char *saveptr;

	// Loop through each line in the input string
	line = strtok_r((char*) nmea_string, "\n", &saveptr);
	while (line != NULL) {
		// Pass the line to minmea_sentence_id with strict=false
		switch (minmea_sentence_id(line, false)) {
		case MINMEA_SENTENCE_RMC: {
			if (minmea_parse_rmc(&gps.rmc_frame, line)) {
			} else {
				debug_print("$xxRMC sentence is not parsed\n", sizeof("$xxRMC sentence is not parsed\n"), dbg = DBG);
			}
		}
			break;

		case MINMEA_SENTENCE_GGA: {
			if (minmea_parse_gga(&gps.gga_frame, line)) {
			} else {
				debug_print("$xxGGA sentence is not parsed\n", sizeof("$xxGGA sentence is not parsed\n"), dbg = DBG);
			}
		}
			break;

		case MINMEA_SENTENCE_GST: {
			if (minmea_parse_gst(&gps.gst_frame, line)) {
			} else {
				debug_print("$xxGST sentence is not parsed\n", sizeof("$xxGST sentence is not parsed\n"), dbg = DBG);
			}
		}
			break;

		case MINMEA_SENTENCE_GSV: {
			if (minmea_parse_gsv(&gps.gsv_frame, line)) {
			} else {
				debug_print("$xxGSV sentence is not parsed\n", sizeof("$xxGSV sentence is not parsed\n"), dbg = DBG);
			}
		}
			break;

		case MINMEA_SENTENCE_VTG: {
			if (minmea_parse_vtg(&gps.vtg_frame, line)) {
			} else {
				debug_print("$xxVTG sentence is not parsed\n", sizeof("$xxVTG sentence is not parsed\n"), dbg = DBG);
			}
		}
			break;

		case MINMEA_SENTENCE_ZDA: {
			if (minmea_parse_zda(&gps.zda_frame, line)) {
			} else {
				debug_print("$xxZDA sentence is not parsed\n", sizeof("$xxZDA sentence is not parsed\n"), dbg = DBG);
			}
		}
			break;

		case MINMEA_INVALID: {
			debug_print("$xxxxx sentence is not valid\n", sizeof("$xxxxx sentence is not valid\n"), dbg = DBG);
		}
			break;

		default: {
			debug_print("$xxxxx sentence is not parsed\n", sizeof("$xxxxx sentence is not parsed\n"), dbg = DBG);
		}
			break;
		}
		line = strtok_r(NULL, "\n", &saveptr);
	}
}

