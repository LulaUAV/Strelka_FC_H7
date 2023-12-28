/*
 * GPS.h
 *
 *  Created on: Apr 20, 2023
 *      Author: Angus McLennan
 *      Description: A high level access to the minmea.h library.
 *      This library implements the bulk code to decode NMEA strings
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdbool.h>
#include <string.h>
#include "minmea.h"
#include "debug.h"

#define GPS_Buff_Size 1024

typedef struct {
	uint8_t gps_buffer[GPS_Buff_Size];
	struct minmea_sentence_gbs gbs_frame;
	struct minmea_sentence_rmc rmc_frame;
	struct minmea_sentence_gga gga_frame;
	struct minmea_sentence_gll gll_frame;
	struct minmea_sentence_gst gst_frame;
	struct minmea_sentence_gsa gsa_frame;
	struct minmea_sentence_gsv gsv_frame;
	struct minmea_sentence_vtg vtg_frame;
	struct minmea_sentence_zda zda_frame;
	bool gps_good;
}GPS_Handle;

typedef struct {
	bool tracking_enabled;
	float chirp_frequency;
} GPS_Tracking_Handle;

void parse_nmea(uint8_t* nmea_string);

#endif /* INC_GPS_H_ */
