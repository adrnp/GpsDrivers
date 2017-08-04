/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sbf.h
 *
 * Septentrio binary format defition.  Designed for use with the Astrex-m2 UAV.
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 *
 */

#ifndef SBF_H_
#define SBF_H_

#include "gps_helper.h"
#include "../../definitions.h"

#define SBF_SYNC1 0x24
#define SBF_SYNC2 0x40

/* Message IDs */
#define SBF_MSG_PVT_GEO		4007
#define SBF_MSG_PVT_GEO_COV	5906
#define SBF_MSG_DOP			4001

/* Do Not Use Values */
#define DNU_FLOAT	-20000000000.0f
#define DNU_DOUBLE	-20000000000.0
#define DNU_UINT8	255
#define DNU_UINT16	65535
#define DNU_UINT32	4294967295


/* RX PVT message content details */
/*   Bitfield "valid" masks */
//#define UBX_RX_NAV_PVT_VALID_VALIDDATE		0x01	/**< validDate (Valid UTC Date) */


/*   Bitfield "flags" masks */
//#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK		0x01	/**< gnssFixOK (A valid fix (i.e within DOP & accuracy masks)) */


/*** septentrio protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* General: Header */
typedef struct {
	// true header has sync bytes here, not needed for this
	uint16_t	crc16;
	uint16_t	msg;
	uint16_t	length;
} sbf_header_t;

/* General: time */
typedef struct {
	uint32_t tow;	/**< GPS time of week [ms] */
	uint16_t week;	/**< GPS week number */
} sbf_time_t;

/* Rx PVT GEO */
typedef struct {
	sbf_header_t header;
	sbf_time_t time;
	uint8_t mode;			/**< bitfield for pvt solution mode */
	uint8_t error;			/**< pvt error code */
	double lat;				/**< latitude in [radians] */
	double lon;				/**< longitude in [radians] */
	double height;			/**< eight above elliposide in [m] */
	float undulation;		/**< geoid undulation [m] */
	float vn;				/**< north velocity [m/s] */
	float ve;				/**< east velocity [m/s] */
	float vu;				/**< up velocity [m/s] */
	float cog;				/**< course over ground [deg] */
	double rxClkBias;		/**< receiver clock bias [ms]  (+ means faster than system time) */
	float rxClkDrift;		/**< receiver clock drift [ppm] (+ means faster than system time) */
	uint8_t timeSystem;		/**< time system for which the offsets are provided */
	uint8_t datum;			/**< defines the datum in which the coordinates are expressed */
	uint8_t nSat;			/**< number of satellites used to compute PVT solution */
	uint8_t waas;			/**< bitfield defining which waas corrections applied */
	uint16_t referenceId;	/**< id of the reference used for a differnetial solution */
	uint16_t meanCorrAge;	/**< mean age of correction when DGPS or RTK */
	uint32_t signalInfo;	/**< bitfield of type of GNSS signals used */
	uint8_t alertFlag;		/**< bitfield for integrety info */
	uint8_t nBase;			/**< number of based stations used */
	uint16_t pppInfo;		/**< bitfield for PPP info */
	uint16_t latency;		/**< time between position fix and this message [ms] */
	uint16_t hAccuracy;		/**< 2D RMS horizontal accuracy [cm] */
	uint16_t vAccuracy;		/**< 2D RMS vertical accuracy [cm] */
	uint8_t misc;			/**< bitfield of misc data */
	// padding
	uint8_t padding[1];
} sbf_payload_rx_pvt_geo_t;

/* Rx PVT GEO COV */
typedef struct {
	sbf_header_t header;
	sbf_time_t time;
	uint8_t mode;	/**< bitfield for pvt solution mode */
	uint8_t error;	/**< pvt error code */
	float latlat;	/**< latitude varianace [m^2] */
	float lonlon;	/**< longitude varianace [m^2] */
	float hgthgt;	/**< height varianace [m^2] */
	float bb;		/**< clock bias varianace [m^2] */
	float latlon;
	float lathgt;
	float latb;
	float lonhgt;
	float lonb;
	float hgtb;
	// padding
	// not needed
} sbf_payload_rx_pvt_geo_cov_t;

/* Rx PVT-DOP */
typedef struct {
	sbf_header_t header;
	sbf_time_t time;
	uint8_t nSat;		/**< number of satellites in the solution */
	uint8_t reserved;
	uint16_t pdop;		/**< PDOP - 0 not available, otherwise this is PDOP*100 */
	uint16_t tdop;
	uint16_t hdop;
	uint16_t vdop;
	float hpl;			/**< horizontal protection level [m] */
	float vpl;			/**< vertical protection level [m] */
	// padding
	// not needed
} sbf_payload_rx_dop_t;



/* General message and payload buffer union */
typedef union {
	sbf_payload_rx_pvt_geo_t 		payload_rx_pvt_geo;
	sbf_payload_rx_pvt_geo_cov_t	payload_rx_pvt_geo_cov;
	sbf_payload_rx_dop_t			payload_rx_dop;
} sbf_buf_t;

#pragma pack(pop)
/*** END OF septentrio protocol binary message and payload definitions ***/

/* Decoder state */
typedef enum {
	SBF_DECODE_SYNC1 = 0,
	SBF_DECODE_SYNC2,
	SBF_DECODE_HEADER,
	SBF_DECODE_PAYLOAD
} sbf_decode_state_t;


class GPSDriverSBF : public GPSHelper
{
public:
	GPSDriverSBF(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
		     struct vehicle_gps_position_s *gps_position);
	virtual ~GPSDriverSBF();
	int receive(unsigned timeout);
	int configure(unsigned &baudrate, OutputMode output_mode);

private:

	/**
	 * Parse the binary SBF packet
	 */
	int parseChar(const uint8_t b);

	/**
	 * Start payload rx
	 */
	int payloadRxInit(void);

	/**
	 * Add payload rx byte
	 */
	int payloadRxAdd(const uint8_t b);

	/**
	 * Finish payload rx
	 */
	int payloadRxDone(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void decodeInit(void);

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void addByteToChecksum(const uint8_t b);

	struct vehicle_gps_position_s *_gps_position {nullptr};
	uint64_t		_last_timestamp_time{0};
	bool			_configured{false};
	bool			_got_pvt{false};
	sbf_decode_state_t	_decode_state{};
	uint16_t		_rx_msg{};
	uint16_t		_rx_payload_length{};
	uint16_t		_rx_payload_index{};
	uint16_t		_rx_checksum{0};
	gps_abstime		_disable_cmd_last{0};
	sbf_buf_t		_buf{};
	OutputMode		_output_mode{OutputMode::GPS};

	const Interface		_interface;
};

#endif /* SBF_H_ */
