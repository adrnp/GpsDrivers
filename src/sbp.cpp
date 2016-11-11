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
 * @file sbp.cpp
 *
 * Swift Binary Protocol (SBP) implementation. 
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 *
 * @see Swift Binary Protocol documentation https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf
 */


#include <cstdlib>	/* standard library (std namespace) */
#include <unistd.h>	/* unix standard library (e.g. read call) */
#include <iostream>	/* output and printing library */

#include <cstdint>	/* for more datatypes */
#include <string.h>	/* use of strings and string related functions */
#include <math.h>	/* for pow and other math functions */

#include <iomanip>	/* std::setfill and setw functions */

#include <bitset>	/* std::bitset<#> for bit representing an int */

#include "sbp.h"


#define SBP_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls


GPSDriverSBP::GPSDriverSBP(GPSCallbackPtr callback, void *callback_user,
				struct vehicle_gps_position_s *gps_position,
				struct gps_raw_measurements_s *raw_meas) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position),
	_raw_meas(raw_meas),
	_got_posllh(false),
	_got_velned(false),
	_got_obs(false)
{
	decodeInit();
}

GPSDriverSBP::GPSDriverSBP(GPSCallbackPtr callback, void *callback_user,
				struct vehicle_gps_position_s *gps_position) :
	GPSDriverSBP(callback, callback_user, gps_position, nullptr)
{
}

GPSDriverSBP::~GPSDriverSBP()
{
}

int
GPSDriverSBP::configure(unsigned &baudrate, OutputMode output_mode)
{
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("SBP: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	/* set baudrate first */
	if (GPSHelper::setBaudrate(SBP_BAUDRATE) != 0) {
		return -1;
	}
	baudrate = SBP_BAUDRATE;

	/* no configuration messages to send to the receiver */

	return 0;
}

int
GPSDriverSBP::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	int handled = 0;

	while (true) {

		/* read in next bytes. */
		int ret = read(buf, sizeof(buf), timeout);

		if (ret < 0) {
			/* something went wrong when polling or reading */
			GPS_WARN("sbp poll_or_read err");
			return -1;

		} else {
			//UBX_DEBUG("read %d bytes", ret);

			/* pass received bytes to the packet decoder */
			for (int i = 0; i < ret; i++) {
				handled |= parseChar(buf[i]);
				//UBX_DEBUG("parsed %d: 0x%x", i, buf[i]);
			}

			if (_got_posllh && _got_velned) {
				_got_posllh = false;
				_got_velned = false;
				return handled;
			}

		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			GPS_WARN("timed out, returning");
			return -1;
		}
	}
}

void
GPSDriverSBP::decodeInit()
{
	//std::cout << "initializing decode\n";

	/* set state to wait for sync */
	_decode_state = SBP_DECODE_SYNC;

	/* reinit the payload starting at the header */
	_rx_payload_index = 0;
	_rx_payload_length = sizeof(sbp_header_t);  // we handle the header with the buffer, so need this here

	/* reinit the CRC-CCITT XMODEN checksum for the payload */
	_rx_checksum = 0x0000;

	/* reset the obs payload stuff */
	_rx_obs_frame_index = 0;
	_rx_obs_frame_count = 0;
}

int  // 0 = decoding, 1 = message handled, 2 = sat info message handled, 4 = raw measurement message handled
GPSDriverSBP::parseChar(const uint8_t c)
{

	int ret = 0;  // default to still decoding

	switch(_decode_state) {
	
	/* Expecting Sync message */
	case SBP_DECODE_PREAMBLE:

		if (c == SBP_PREAMBLE) {
			_decode_state = SBP_DECODE_HEADER;
		}
		break;

	/* Expecting Header */
	case SBP_DECODE_HEADER:

		addByteToChecksum(c);	// add to checksum
		ret = payloadRxAdd(c);  // add to buffer

		if (ret > 0) {

			// DEBUG
			//std::cout << "\n\tmessage id: " << std::hex << std::setfill('0') << std::setw(4) << (int) _buf.payload_header.msg << std::endl;
			//std::cout << "\tsender: " << std::hex << std::setfill('0') << std::setw(4) << (int) _buf.payload_header.sender << std::endl;
			//std::cout << "\tpayload length: " << std::hex << std::setfill('0') << std::setw(2) << (int) _buf.payload_header.length << "  " << std::dec << (int) _buf.payload_header.length << std::endl;

			// extract the message id and the payload length, still not guaranteed as checksum needs to be checked
			_rx_msg = _buf.payload_header.msg;
			_rx_payload_length = _buf.payload_header.length;
			
			if (payloadRxInit() == 0) {
				// move to the header checksum decode state
				_decode_state = SBP_DECODE_PAYLOAD;

			} else {
				// not handling the payload
				decodeInit();
			}
			
		}

		ret = 0;
		break;

	/* Expecting Payload */
	case SBP_DECODE_PAYLOAD:

		addByteToChecksum(c);	// add to checksum

		// DEBUG - show incoming byte
		//std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) c << " ";

		/* obs message needs to be handled differently */
		switch (_rx_msg) {
		case SBP_MSG_OBS:
			ret = payloadRxAddObs(c);
			break;

		default:
			ret = payloadRxAdd(c);	// add a nav (ext and normal) solution byte
			break;
		}

		// check if we have completed the payload
		if (ret > 0) {

			// DEBUG
			//std::cout << "\n\tpayload completed, read " << std::dec << _rx_payload_index << " bytes\n";

			_decode_state = SBP_DECODE_CHECKSUM1;
		}

		ret = 0;
		break;

	/* Expecting payload checksum */
	case SBP_DECODE_CHECKSUM1:

		// DEBUG - show incoming byte
		//std::cout << "\nlow: " << std::hex << std::setfill('0') << std::setw(2) << (int) c << " " << (int) (_rx_checksum & 0xFF);
	
		// payloadRxAddChecksum(c, 0);

		if (c == (_rx_checksum & 0xFF)) {
			_decode_state = SBP_DECODE_CHECKSUM2;			
		} else {
			// DEBUG
			//std::cout << "\n first byte error" << std::endl;
			decodeInit();
		}

		

		// get the next checksum byte
		//decodeInit();
		//_decode_state = SBP_DECODE_SYNC;
		
		break;
	
	/* Expecting payload checksum (2nd byte) */
	case SBP_DECODE_CHECKSUM2:
	
		// payloadRxAddChecksum(c, 1);
		
		if (c == ((_rx_checksum >> 8) & 0xFF)) {
			ret = payloadRxDone();	
			
		} else {
			// DEBUG
			//td::cout << "\n second byte error" << std::endl;
		}

		/*
		// DEBUG - show incoming byte
		std::cout << "\n\tchecksum 2: " << std::hex << std::setfill('0') << std::setw(2) << (int) c << " ";
		std::cout << "\n\tcomparison: " << std::hex << std::setfill('0') << std::setw(4) << (int) _payload_rx_checksum << " " << (int) _rx_checksum << std::endl;

		if (_payload_rx_checksum == _rx_checksum) {

			std::cout << "\nlow: " << std::hex << std::setfill('0') << std::setw(2) << (int) (_payload_rx_checksum & 0xFF) << " " << (int) (_rx_checksum & 0xFF);
			std::cout << "\nhigh: " << std::hex << std::setfill('0') << std::setw(2) << (int) ((_payload_rx_checksum >> 8) & 0xFF) << " " << (int) ((_rx_checksum >> 8) & 0xFF);

			ret = payloadRxDone();	
		}
		*/

		decodeInit();
		break;
	
	}

	return ret;
}


/**
 * Start payload rx
 */
int	// -1 = abort, 0 = continue
GPSDriverSBP::payloadRxInit()
{
	int ret = 0;

	// reset the payload index - since we use the buffer for the header
	_rx_payload_index = 0;

	_rx_state = SBP_RXMSG_HANDLE;	// handle by default

	switch (_rx_msg) {
	
	case SBP_MSG_POS_LLH:
		if (_rx_payload_length != sizeof(sbp_payload_rx_pos_llh_t)) {
			_rx_state = SBP_RXMSG_ERROR_LENGTH;
		}
		break;

	case SBP_MSG_VEL_NED:
		if (_rx_payload_length != sizeof(sbp_payload_rx_vel_ned_t)) {
			_rx_state = SBP_RXMSG_ERROR_LENGTH;
		}
		break;

	case SBP_MSG_DOPS:
		if (_rx_payload_length != sizeof(sbp_payload_rx_dops_t)) {
			_rx_state = SBP_RXMSG_ERROR_LENGTH;
		}
		break;

	case SBP_MSG_GPS_TIME:
		if (_rx_payload_length != sizeof(sbp_payload_rx_gps_time_t)) {
			_rx_state = SBP_RXMSG_ERROR_LENGTH;
		}
		break;

	case SBP_MSG_OBS:
		if (_raw_meas == nullptr) {
			_rx_state = SBP_RXMSG_IGNORE;        // disable if raw measurement not requested

		} else {
			memset(_raw_meas, 0, sizeof(*_raw_meas));        // initialize raw measurement info
		}
		break;

	default:
		_rx_state = SBP_RXMSG_IGNORE;	// disable all other messages
		break;
	}

	switch (_rx_state) {
	case SBP_RXMSG_HANDLE:	// handle message
	case SBP_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case SBP_RXMSG_ERROR_LENGTH:	// error: invalid length
		//UBX_WARN("ubx msg 0x%04x invalid len %u", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		//UBX_WARN("ubx internal err1");
		ret = -1;	// return error, abort handling this message
		break;
	}

	return ret;
}


int  // -1 = error, 0 = continue, 1 = completed
GPSDriverSBP::payloadRxAdd(const uint8_t c)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	p_buf[_rx_payload_index] = c;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;  // paylaod completed
	}

	return ret;
}


int
GPSDriverSBP::payloadRxAddObs(const uint8_t c)
{
	// DEBUG
	//std::cout << std::hex << std::setfill('0') << std::setw(2) << (int) c << " ";

	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(sbp_payload_rx_obs_header_t)) {
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = c;

	} else {
		if (_rx_payload_index == sizeof(sbp_payload_rx_obs_header_t)) {
			// Part 1 complete: decode Part 1 buffer
			uint8_t nobs = _buf.payload_rx_obs_header.n_obs;
			_rx_obs_frame_index = (nobs & 0x0F);
			_rx_obs_frame_count = ((nobs >> 4) & 0x0F);

			_raw_meas->tow = _buf.payload_rx_obs_header.t.tow;
			_raw_meas->wn = _buf.payload_rx_obs_header.t.wn;
			_raw_meas->nobs = 0;

			/*
			std::cout << "\ncount: " << std::hex << std::setfill('0') << std::setw(2) << (int) nobs << std::endl;
			std::cout << "counta: " << (int) _rx_obs_frame_index << std::endl;
			std::cout << "countb: " << (int) _rx_obs_frame_count << std::endl;
			*/
		}

		if (_rx_payload_index < _rx_payload_length) {  // TODO: may need to limit the number of recorded obs
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(sbp_payload_rx_obs_header_t)) % sizeof(sbp_payload_rx_obs_content_t);
			p_buf[buf_index] = c;

			if (buf_index == sizeof(sbp_payload_rx_obs_content_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = 5*_rx_obs_frame_index + (_rx_payload_index - sizeof(sbp_payload_rx_obs_header_t)) / sizeof(sbp_payload_rx_obs_content_t);
				std::cout << "\t(" << (int) sat_index << ") sat identifier: " << (int) _buf.payload_rx_obs_content.sid.sat << " " << (int) _buf.payload_rx_obs_content.sid.code << std::endl;

				_raw_meas->psuedorange[sat_index] = _buf.payload_rx_obs_content.P;
				_raw_meas->carrier_i[sat_index] = _buf.payload_rx_obs_content.L.i;
				_raw_meas->carrier_f[sat_index] = _buf.payload_rx_obs_content.L.f;
				_raw_meas->cn0[sat_index] = _buf.payload_rx_obs_content.cn0;
				_raw_meas->lock[sat_index] = _buf.payload_rx_obs_content.lock;
				_raw_meas->prn[sat_index] = _buf.payload_rx_obs_content.sid.sat;

				// mark as having made one more observation
				_raw_meas->nobs++;
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		if (_rx_obs_frame_count == (_rx_obs_frame_index + 1)) {
			_got_obs = true;

			// DEBUG
			std::cout << "\tgot all observations\n";
		}
		ret = 1;	// payload received completely
	}

	return ret;
}


int  // 0 = no message handled, 1 = message handled, 2 = "sat" info message handled
GPSDriverSBP::payloadRxDone(void)
{
	int ret = 0;


	switch (_rx_msg) {

		case SBP_MSG_GPS_TIME:
			//std::cout << "\nparsed time message\n";
			break;

		case SBP_MSG_DOPS:
			//std::cout << "\nparsed DOPS message\n";

			_gps_position->hdop = _buf.payload_rx_dops.hdop * 0.01f;	// from cm to m
			_gps_position->vdop = _buf.payload_rx_dops.vdop * 0.01f;	// from cm to m

			ret = 1;
			break;

		case SBP_MSG_POS_LLH:
			// DEBUG
			/*
			std::cout << "\nparsed LLH message:\n";
			std::cout << "\ttow: " << std::dec << (long)_buf.payload_rx_pos_llh.tow << std::endl;
			std::cout << "\tlatitude: " << std::dec << (double)_buf.payload_rx_pos_llh.lat << std::endl;
			std::cout << "\tlongitude: " << std::dec << (double)_buf.payload_rx_pos_llh.lon << std::endl;
			std::cout << "\theight: " << std::dec << (double)_buf.payload_rx_pos_llh.height << std::endl;
			std::cout << "\tn sats: " << std::dec << (int) _buf.payload_rx_pos_llh.n_sats << std::endl;
			*/

			_gps_position->lat	= (int32_t) _buf.payload_rx_pos_llh.lat * 1.0e7; // TODO: need to put into lat 1E-7
			_gps_position->lon	= (int32_t) _buf.payload_rx_pos_llh.lon * 1.0e7;
			_gps_position->alt	= (int32_t) _buf.payload_rx_pos_llh.height * 1.0e3;
			_gps_position->alt_ellipsoid = (int32_t) _buf.payload_rx_pos_llh.height * 1.0e3;  // TODO: figure out how to make this hieght correct

			_gps_position->fix_type = 3;
			//_gps_position->s_variance_m_s	= (float)_buf.payload_rx_nav_sol.sAcc * 1e-2f;	// from cm to m
			_gps_position->satellites_used = _buf.payload_rx_pos_llh.n_sats;

			_gps_position->timestamp = gps_absolute_time();

			_got_posllh = true;

			ret = 1;
			break;

		case SBP_MSG_VEL_NED:
			//std::cout << "\nparsed vel ned message\n";

			_gps_position->vel_m_s		= 0; // TODO: use the velcity measurements
			_gps_position->vel_n_m_s	= (float)_buf.payload_rx_vel_ned.n * 1e-3f; /* NED NORTH velocity */
			_gps_position->vel_e_m_s	= (float)_buf.payload_rx_vel_ned.e * 1e-3f; /* NED EAST velocity */
			_gps_position->vel_d_m_s	= (float)_buf.payload_rx_vel_ned.d * 1e-3f; /* NED DOWN velocity */
			//_gps_position->cog_rad		= (float)_buf.payload_rx_vel_ned.heading * M_DEG_TO_RAD_F * 1e-5f;  // TODO: need to get this one
			//_gps_position->c_variance_rad	= (float)_buf.payload_rx_vel_ned.cAcc * M_DEG_TO_RAD_F * 1e-5f;
			_gps_position->vel_ned_valid	= true;

			_got_velned = true;

			ret = 1;
			break;

		case SBP_MSG_OBS:

			_raw_meas->timestamp = gps_absolute_time();

			// if all obs received, mark as raw meas being handled
			if (_got_obs) {
				_got_obs = false;
				ret = 4;
			}

			break;

		case SBP_MSG_TRACKING_STATE_DETAILED:
			std::cout << "\nparsed time message\n";
			break;

		default:
			break;
	}


	return ret;
}

void
GPSDriverSBP::addByteToChecksum(const uint8_t c) {

	// CRC-CCITT XMODEM implementation
	_rx_checksum ^= c << 8;
    for (int i = 0; i < 8; i++) {
        _rx_checksum = _rx_checksum & 0x8000 ? (_rx_checksum << 1) ^ 0x1021 : _rx_checksum << 1;
    }
}