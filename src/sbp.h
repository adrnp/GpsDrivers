/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file sbp.h
 *
 * Swift Binary Protocol definition. Following Swift Piksi SBP definitions (v1.2)
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 *
 */

#ifndef SBP_H_
#define SBP_H_

#include "gps_helper.h"
#include "../../definitions.h"

#define SBP_PREAMBLE	0x55

/* message identifiers */
#define SBP_MSG_GPS_TIME	0x0100
#define SBP_MSG_DOPS		0x0206
#define SBP_MSG_POS_LLH		0x0201
#define SBP_MSG_VEL_NED		0x0205
#define SBP_MSG_OBS			0x0043 //0x0049
#define SBP_MSG_TRACKING_STATE_DETAILED	0x0011


/* RX MSG_POS_LLH content details */
#define SBP_RX_MSG_POS_LLH_FLAGS_SPP		0x00
#define SBP_RX_MSG_POS_LLH_FLAGS_RTKFIXED	0x01
#define SBP_RX_MSG_POS_LLH_FLAGS_RTKFLOAT	0x02
#define SBP_RX_MSG_POS_LLH_FLAGS_HEIGHT		0x08
#define SBP_RX_MSG_POS_LLH_FLAGS_RAIM		0x16
#define SBP_RX_MSG_POS_LLH_FLAGS_REPAIR		0x32

/* RX MSG_TRACKING_STATE_DETAILED content details */
// TODO: add the other flag elements

/* miscellaneous flags */
#define SBP_RX_MSG_TRACKING_VALID_ACCEL			0x08
#define SBP_RX_MSG_TRACKING_VALID_PSEUDORANGE	0x16

/* settings */
#define SBP_BAUDRATE 115200

/** the structures of the binary packets */
#pragma pack(push, 1)


/* General: Header */
typedef struct {
	uint16_t	msg;
	uint16_t	sender;
	uint8_t		length;
} sbp_header_t;



/* General: CRC16 Checksum */
typedef struct {
	uint8_t		high;
	uint8_t		low;
} sbp_checksum_t ;


/* Rx MSG_GPS_TIME */
typedef struct {
	uint16_t wn;	/**< GPS week number [weeks] */
	uint32_t tow;	/**< GPS time of week rounded to the nearest millisecond [ms] */
	int32_t ns;		/**< Nanosecond residual of millisecond-rounded TOW (ranges from -500000 to 500000) [ns] */
	uint8_t flags;	/**< Status flags (reserved) */
} sbp_payload_rx_gps_time_t;

/* Rx MSG_DOP */
typedef struct {
	uint32_t tow;	/**< GPS Time of Week [ms] */
	uint16_t gdop;	/**< Geometric Dilution of Precision [0.01] */
	uint16_t pdop;	/**< Position Dilution of Precision [0.01] */
	uint16_t tdop;	/**< Time Dilution of Precision [0.01] */
	uint16_t hdop;	/**< Horizontal Dilution of Precision [0.01] */
	uint16_t vdop;	/**< Vertical Dilution of Precision [0.01] */
} sbp_payload_rx_dops_t;

/* Rx MAG_POS_LLH */
typedef struct {
	uint32_t tow;			/**< GPS Time of Week [ms] */
	double lat;				/**< Latitude [deg] */
	double lon;				/**< Longitude [deg] */
	double height;			/**< Height [m] */
	uint16_t h_accuracy;	/**< Horizontal position accuracy estimate (not implemented). Defaults to 0. [mm] */
	uint16_t v_accuracy;	/**< Vertical position accuracy estimate (not implemented). Defaults to 0. [mm] */
	uint8_t n_sats;			/**< Number of satellites used in solution. */
	uint8_t flags;			/**< Status flags */
} sbp_payload_rx_pos_llh_t;

/* Rx MSG_VEL_NED */
typedef struct {
	uint32_t tow;			/**< GPS Time of Week [ms] */
	int32_t n;				/**< Velocity North coordinate [mm/s] */
	int32_t e;				/**< Velocity East coordinate [mm/s] */
	int32_t d;				/**< Velocity Down coordinate [mm/s] */
	uint16_t h_accuracy;	/**< Horizontal velocity accuracy estimate (not implemented). Defaults to 0. [mm/s] */
	uint16_t v_accuracy;	/**< Vertical velocity accuracy estimate (not implemented). Defaults to 0. [mm/s] */
	uint8_t n_sats;			/**< Number of satellites used in solution */
	uint8_t flags;			/**< Status flags (reserved) */
} sbp_payload_rx_vel_ned_t;

/* sbp gnss signal container */
typedef struct {
	uint16_t sat;		/**< Constellation-specific satellite identifier */
	uint8_t code;		/**< Signal constellation, band and code */
	uint8_t reserved;	/**< Reserved */
} sbp_gnss_signal_t;

/* sbp gps time container */
typedef struct {
	uint32_t tow;	/**< Milliseconds since start of GPS week [ms] */
	uint16_t wn;	/**< GPS week number [week] */
} sbp_gps_time_t;

/* sbp carrier phase measurement container */
typedef struct {
	int32_t i;	/**< Carrier phase whole cycles [cycles] */
	uint8_t f;	/**< Carrier phase fractional part [cycles / 256] */
} sbp_carrier_phase_t;

/* Rx MSG_OBS header (Part 1) */
typedef struct {
	sbp_gps_time_t t;	/**< GPS time of this observation */
	uint8_t n_obs;		/**< Total number of observations. First nibble is the size of the sequence (n), second nibble is the zero-indexed counter (ith packet of n) */
} sbp_payload_rx_obs_header_t;

/* Rx MSG_OBS contents (Part 2) */
typedef struct {
	uint32_t P;				/**< Pseudorange observation [cm] */
	sbp_carrier_phase_t L;	/**< Carrier phase observation with typical sign convention. [cycles] */
	uint8_t cn0;			/**< Carrier-to-Noise density [dB Hz * 4] */
	uint16_t lock;			/**< Lock indicator. This value changes whenever a satellite signal has lost and regained lock, indicating that the carrier phase ambiguity may have changed. */
	sbp_gnss_signal_t sid;	/**< GNSS signal identifier */
} sbp_payload_rx_obs_content_t;


/* Rx MSG_TRACKING_STATE_DETAILED */
typedef struct {
	uint64_t recv_time;			/**< Receiver clock time. [ns] */
	sbp_gps_time_t tot;			/**< Time of transmission of signal from satellite. TOW only valid when TOW status is decoded or propagated. WN only valid when week number valid flag is set. */
	uint32_t P;					/**< Pseudorange observation. Valid only when pseudorange valid flag is set. [2 cm] */
	uint16_t P_std;				/**< Pseudorange observation standard deviation. Valid only when pseudorange valid flag is set. [2 cm] */
	sbp_carrier_phase_t L;		/**< Carrier phase observation with typical sign convention. Valid only when PLL pessimistic lock is achieved. [cycles] */
	uint8_t cn0;				/**< Carrier-to-Noise density [dB Hz / 4] */
	uint16_t lock;				/**< Lock indicator. This value changes whenever a satellite signal has lost and regained lock, indicating that the carrier phase ambiguity may have changed. */
	sbp_gnss_signal_t sid;		/**< GNSS signal identifier. */
	int32_t doppler;			/**< Carrier Doppler frequency. [Hz / 16] */
	uint16_t doppler_std;		/**< Carrier Doppler frequency standard deviation. [Hz / 16] */
	uint32_t uptime;			/**< Number of seconds of continuous tracking. Specifies how much time signal is in continuous track. [s] */
	int16_t clock_offset;		/**< TCXO clock offset. Valid only when valid clock valid flag is set. [s / (2 ^ 20)] */
	int16_t clock_drift;		/**< TCXO clock drift. Valid only when valid clock valid flag is set. [(s / s) / (2 ^ 31)] */
	uint16_t corr_spacing;		/**< Early-Prompt (EP) and Prompt-Late (PL) correlators spacing. [ns] */
	int8_t acceleration;		/**< Acceleration. Valid only when acceleration valid flag is set. [g / 8] */
	uint8_t sync_flags;			/**< Synchronization status flags. */
	uint8_t tow_flags;			/**< TOW status flags. */
	uint8_t track_flags;		/**< Tracking loop status flags. */
	uint8_t nav_flags;			/**< Navigation data status flags. */
	uint8_t pset_flags;			/**< Parameters sets flags. */
	uint8_t misc_flags;			/**< Miscellaneous flags. */
} sbp_payload_rx_tracking_state_detailed_t;



/* General message and payload buffer union */
typedef union {
	sbp_header_t 					payload_header;
	sbp_payload_rx_gps_time_t		payload_rx_gps_time;
	sbp_payload_rx_dops_t			payload_rx_dops;
	sbp_payload_rx_pos_llh_t		payload_rx_pos_llh;
	sbp_payload_rx_vel_ned_t		payload_rx_vel_ned;
	sbp_payload_rx_obs_header_t		payload_rx_obs_header;
	sbp_payload_rx_obs_content_t	payload_rx_obs_content;
	sbp_payload_rx_tracking_state_detailed_t	payload_rc_tracking;
} sbp_buf_t;


#pragma pack(pop)


/* decoding state */
typedef enum {
	SBP_DECODE_PREAMBLE = 0,
	SBP_DECODE_HEADER,
	SBP_DECODE_PAYLOAD,
	SBP_DECODE_CHECKSUM1,
	SBP_DECODE_CHECKSUM2
} sbp_decode_state_t;

/* Rx message state */
typedef enum {
	SBP_RXMSG_IGNORE = 0,
	SBP_RXMSG_HANDLE,
	SBP_RXMSG_ERROR_LENGTH
} sbp_rxmsg_state_t;


class GPSDriverSBP : public GPSHelper
{
public:
	GPSDriverSBP(GPSCallbackPtr callback, void *callback_user,
			 struct vehicle_gps_position_s *gps_position,
		     struct gps_raw_measurements_s *raw_meas);
	GPSDriverSBP(GPSCallbackPtr callback, void *callback_user,
			 struct vehicle_gps_position_s *gps_position);
	virtual ~GPSDriverSBP();
	int receive(unsigned timeout);
	int configure(unsigned &baudrate, OutputMode output_mode);

private:

	/**
	 * Parse the binary SBP packet
	 */
	int parseChar(const uint8_t b);

	/**
	 * Start payload rx
	 */
	int payloadRxInit();

	/**
	 * Add payload rx byte
	 */
	int payloadRxAdd(const uint8_t b);
	int payloadRxAddObs(const uint8_t c);

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
	void addByteToChecksum(const uint8_t);

	/* outputs */
	struct vehicle_gps_position_s *_gps_position;
	struct gps_raw_measurements_s *_raw_meas;

	/* states */
	bool					_got_posllh;
	bool					_got_velned;
	bool					_got_obs;
	sbp_decode_state_t		_decode_state;
	sbp_rxmsg_state_t		_rx_state;

	/* indices */
	uint16_t		_rx_payload_index;
	uint8_t			_rx_obs_frame_index;
	uint8_t			_rx_obs_frame_count;

	/* lengths */
	uint8_t		_rx_payload_length;

	/* payload information */
	uint16_t		_rx_msg;

	/* buffers */
	sbp_buf_t		_buf;

	/* checksum */
	uint16_t	_rx_checksum;

};

#endif /* SBP_H_ */