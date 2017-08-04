/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file sbf.cpp
 *
 * Septentrio protocol implementation.
 * Specifically for the Astrex-M2 UAV receiver
 *
 * @author Adrien Perkins <adrienp@stanford.edu>
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctime>

#include "sbf.h"

/**** Trace macros, disable for production builds */
#define SBF_TRACE_PARSER(...)	{/*GPS_INFO(__VA_ARGS__);*/}	/* decoding progress in parse_char() */
#define SBF_TRACE_RXMSG(...)		{/*GPS_INFO(__VA_ARGS__);*/}	/* Rx msgs in payload_rx_done() */
#define SBF_TRACE_SVINFO(...)	{/*GPS_INFO(__VA_ARGS__);*/}	/* NAV-SVINFO processing (debug use only, will cause rx buffer overflows) */

/**** Warning macros, disable to save memory */
#define SBF_WARN(...)		{GPS_WARN(__VA_ARGS__);}
#define SBF_DEBUG(...)		{/*GPS_WARN(__VA_ARGS__);*/}

GPSDriverSBF::GPSDriverSBF(GPSCallbackPtr callback, void *callback_user,
			   struct vehicle_gps_position_s *gps_position)
	: GPSHelper(callback, callback_user)
	, _gps_position(gps_position)
{
	decodeInit();
}

GPSDriverSBF::~GPSDriverSBF() {}

int
GPSDriverSBF::configure(unsigned &baudrate, OutputMode output_mode)
{
	_configured = false;
	if (output_mode != OutputMode::GPS) {
		SBF_WARN("SBF: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	/* set baudrate first */
	if (GPSHelper::setBaudrate(115200) != 0) {
		return -1;
	}

	baudrate = 115200;

	_configured = true;
	return 0;
}


int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverSBF::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	int handled = 0;

	while (true) {

		// read in data
		int ret = read(buf, sizeof(buf), timeout);

		if (ret < 0) {
			/* something went wrong when polling or reading */
			SBF_WARN("sbf poll_or_read err");
			return -1;

		} else if (ret == 0) {
			/* return success if ready */
			if (_got_pvt) {
				_got_pvt = false;
				return handled;
			}

		} else {
			//SBF_DEBUG("read %d bytes", ret);

			/* pass received bytes to the packet decoder */
			for (int i = 0; i < ret; i++) {
				handled |= parseChar(buf[i]);
				//SBF_DEBUG("parsed %d: 0x%x", i, buf[i]);
			}

			// if we got a position solution, send it
			if (_got_pvt) {
				_got_pvt = false;
				return handled;
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			SBF_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
GPSDriverSBF::parseChar(const uint8_t b)
{
	int ret = 0;

	switch (_decode_state) {

	/* Expecting Sync1 */
	case SBF_DECODE_SYNC1:
		if (b == SBF_SYNC1) {	// Sync1 found --> expecting Sync2
			SBF_TRACE_PARSER("A");
			_decode_state = SBF_DECODE_SYNC2;

		}
		break;

	/* Expecting Sync2 */
	case SBF_DECODE_SYNC2:
		if (b == SBF_SYNC2) {	// Sync2 found --> expecting rest of header
			SBF_TRACE_PARSER("B");
			_decode_state = SBF_DECODE_HEADER;

			// initialize that we are going to parse the header
			_rx_payload_length = sizeof(sbf_header_t);
			_rx_payload_index = 0;

		} else {		// Sync1 not followed by Sync2: reset parser
			decodeInit();
		}

		break;

	/* Expecting rest of header */
	case SBF_DECODE_HEADER:
		SBF_TRACE_PARSER("-");

		// add the header info to the payload buffer
		ret = payloadRxAdd(b);

		// don't want to add the first 2 bytes to the checksum
		if (_rx_payload_index > 2) {
			addByteToChecksum(b);
		}

		// header is finished, need to decode length and message
		if (ret > 0) {
			_rx_msg = (0x1FFF & _buf.payload_rx_pvt_geo.header.msg);
			_rx_payload_length += _buf.payload_rx_pvt_geo.header.length - 8; // length includes 8 byte header

			// depending on the message type, handle it or not
			switch (_rx_msg) {
				case SBF_MSG_PVT_GEO:
				case SBF_MSG_DOP:
					_decode_state = SBF_DECODE_PAYLOAD;
					break;

				default:
					decodeInit();
					break;
			}
		}

		ret = 0;
		break;

	/* Expecting payload */
	case SBF_DECODE_PAYLOAD:
		SBF_TRACE_PARSER(".");
		addByteToChecksum(b);

		// no sub packets to decode with any of these payloads
		// therefore, regardless of message, can simply call this
		ret = payloadRxAdd(b);		// add a payload byte

		if (ret < 0) {
			// payload not handled, discard message
			decodeInit();
			ret = 0;

		} else if (ret > 0) {
			// check the checksum, and based on that, handle the message
			if (_rx_checksum == _buf.payload_rx_pvt_geo.header.crc16) {
				ret = payloadRxDone();
			} else {
				ret = 0;
			}
			
			// at this point we are done, but may not have gotten a PVT message so reinit the decoding
			decodeInit();
		} else {
			// expecting more payload, stay in state SBF_DECODE_PAYLOAD
			ret = 0;
		}
		break;

	default:
		break;
	}

	return ret;
}

/**
 * Add payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverSBF::payloadRxAdd(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	p_buf[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Finish payload rx
 */
int	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverSBF::payloadRxDone()
{
	int ret = 0;

	// handle message
	switch (_rx_msg) {

	case SBF_MSG_PVT_GEO:
		SBF_TRACE_RXMSG("Rx PVT GEO");

		// check to make sure have a position
		if ((_buf.payload_rx_pvt_geo.mode & 0xF) > 0 && (_buf.payload_rx_pvt_geo.error == 0)) {
			_gps_position->fix_type = 3;
		} else {
			_gps_position->fix_type = 0;
			_gps_position->vel_ned_valid = false;
		}

		// TODO: need to decide what to do if we don't have a fix
		// do we return early, or go through the rest and return positive


		_gps_position->satellites_used	= _buf.payload_rx_pvt_geo.nSat;

		_gps_position->lat		= (int32_t)(_buf.payload_rx_pvt_geo.lat*M_RAD_TO_DEG*1e7);
		_gps_position->lon		= (int32_t)(_buf.payload_rx_pvt_geo.lon*M_RAD_TO_DEG*1e7);
		_gps_position->alt		= (int32_t)((_buf.payload_rx_pvt_geo.height - (double)_buf.payload_rx_pvt_geo.undulation)*1e3);
		_gps_position->alt_ellipsoid	= (int32_t)(_buf.payload_rx_pvt_geo.height*1e3);

		_gps_position->eph		= _buf.payload_rx_pvt_geo.hAccuracy * 1e2f;
		_gps_position->epv		= _buf.payload_rx_pvt_geo.vAccuracy * 1e2f;
		
		if (_buf.payload_rx_pvt_geo.vn > DNU_FLOAT) {
			//_gps_position->s_variance_m_s	= // TODO: get the variance for the velocity
			_gps_position->vel_n_m_s	= _buf.payload_rx_pvt_geo.vn;
			_gps_position->vel_e_m_s	= _buf.payload_rx_pvt_geo.ve;
			_gps_position->vel_d_m_s	= -_buf.payload_rx_pvt_geo.vu;
			_gps_position->vel_m_s = sqrt(_gps_position->vel_n_m_s*_gps_position->vel_n_m_s + _gps_position->vel_e_m_s*_gps_position->vel_e_m_s  + _gps_position->vel_d_m_s*_gps_position->vel_d_m_s );

			_gps_position->vel_ned_valid = true;
		}


		if (_buf.payload_rx_pvt_geo.cog > DNU_FLOAT) {
			_gps_position->cog_rad = _buf.payload_rx_pvt_geo.cog * M_DEG_TO_RAD_F;
			//_gps_position->c_variance_rad = // TODO: get cog variance
		}

		// if the time is valid, get the time info
		if (_buf.payload_rx_pvt_geo.time.tow != DNU_UINT32 && _buf.payload_rx_pvt_geo.time.week != DNU_UINT16) {
			// convert from GPS TOW and Week to unix timestamp
			
			// get the GPS datum
			struct tm timeinfo;
			memset(&timeinfo, 0, sizeof(timeinfo));
			timeinfo.tm_year	= 1980 - 1900;
			timeinfo.tm_mon		= 0;
			timeinfo.tm_mday	= 6;
			timeinfo.tm_hour	= 0;
			timeinfo.tm_min		= 0;
			timeinfo.tm_sec		= 0;

			// calculate seconds and nanoseconds from the tow
			int seconds = (int) ((float)_buf.payload_rx_pvt_geo.time.tow * 0.001f);
			long milliseconds = (long) (_buf.payload_rx_pvt_geo.time.tow - (uint32_t)seconds * 1000);

			// add to the GPS datum the number of weeks and seconds
			timeinfo.tm_mday += _buf.payload_rx_pvt_geo.time.week * 7;
			timeinfo.tm_sec += seconds;

#ifndef NO_MKTIME
			time_t epoch = mktime(&timeinfo);

			if (epoch > GPS_EPOCH_SECS) {
				// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
				// and control its drift. Since we rely on the HRT for our monotonic
				// clock, updating it from time to time is safe.

				timespec ts;
				memset(&ts, 0, sizeof(ts));
				ts.tv_sec = epoch;
				ts.tv_nsec = milliseconds*1000000;

				setClock(ts);

				_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
				_gps_position->time_utc_usec += milliseconds*1000;

			} else {
				_gps_position->time_utc_usec = 0;
			}

#else
			_gps_position->time_utc_usec = 0;
#endif
		}

		_gps_position->timestamp = gps_absolute_time();
		_last_timestamp_time = _gps_position->timestamp;

		_rate_count_vel++;
		_rate_count_lat_lon++;

		_got_pvt = true;

		ret = 1;
		break;


	case SBF_MSG_DOP:
		SBF_TRACE_RXMSG("Rx DOP");

		if (_buf.payload_rx_dop.hdop != 0) {
			_gps_position->hdop		= (float)_buf.payload_rx_dop.hdop * 0.01f;
		}

		if (_buf.payload_rx_dop.vdop != 0) {
			_gps_position->vdop		= (float)_buf.payload_rx_dop.vdop * 0.01f;
		}

		ret = 1;
		break;

	default:
		break;
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}

void
GPSDriverSBF::decodeInit()
{
	_decode_state = SBF_DECODE_SYNC1;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
	_rx_checksum = 0;
}

void
GPSDriverSBF::addByteToChecksum(const uint8_t b)
{
	// CRC-CCITT XMODEM implementation
	_rx_checksum ^= b << 8;
    for (int i = 0; i < 8; i++) {
        _rx_checksum = _rx_checksum & 0x8000 ? (_rx_checksum << 1) ^ 0x1021 : _rx_checksum << 1;
    }
}
