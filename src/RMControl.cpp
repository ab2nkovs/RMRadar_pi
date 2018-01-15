/******************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  Navico BR24 Radar Plugin
 * Author:   David Register
 *           Dave Cowell
 *           Kees Verruijt
 *           Hakan Svensson
 *           Douwe Fokkema
 *           Sean D'Epagnier
 *	     RM Guy
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register              bdbcat@yahoo.com *
 *   Copyright (C) 2012-2013 by Dave Cowell                                *
 *   Copyright (C) 2012-2016 by Kees Verruijt         canboat@verruijt.net *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 */

#include "RMControl.h"
#ifndef _WIN32
#include <netinet/in.h>
#include <arpa/inet.h>
#endif
PLUGIN_BEGIN_NAMESPACE

#define SEATALK_HS_ANNOUNCE_GROUP	"224.0.0.1"
#define SEATALK_HS_ANNOUNCE_PORT	5800

/*
 * This file not only contains the radar receive threads, it is also
 * the only unit that understands what the radar returned data looks like.
 * The rest of the plugin uses a (slightly) abstract definition of the radar.
 */

CRMControl::CRMControl(br24radar_pi *pi, RadarInfo *ri)
	: wxThread(wxTHREAD_JOINABLE)
	, m_pi(pi)
	, m_ri(ri)
	, m_quit(false)
	, m_haveRadar(false)
	, m_mcast_addr(0)
	, m_range_meters(0)
	, m_updated_range(false)
	, m_radar_status(0)
	, m_dataSocket(INVALID_SOCKET)
	, m_new_ip_addr(false)
	, m_interface_array(0)
	, m_interface(0)
	, m_next_spoke(-1)
	, m_next_rotation(0)
{
	Create(1024 * 1024);  // Stack size, be liberal

	if (m_pi->m_settings.verbose >= 2) 
	{
		wxLogMessage(wxT("RMRadar_pi: CRMControl ctor"));
	}

	fprintf(stderr, "CRMControl::CRMControl\n");

	// Set constant min/max control values
	m_stc.SetMin(0);
	m_tuneFine.SetMin(0); m_tuneFine.SetMax(255);
	m_tuneCoarse.SetMin(0); m_tuneCoarse.SetMax(255);
	m_bearingOffset.SetMin(-1800); m_bearingOffset.SetMax(1795);
	m_autoSea.SetMin(0); m_autoSea.SetMax(3);
	m_interferenceRejection.SetMin(0); m_interferenceRejection.SetMax(2);
	m_targetBoost.SetMin(0); m_targetBoost.SetMax(2);
	m_displayTiming.SetMin(0); m_displayTiming.SetMax(255);
	m_mbsEnabled.SetMin(0); m_mbsEnabled.SetMax(1);
	m_stcCurve.SetMin(1); m_stcCurve.SetMax(8);
}

CRMControl::~CRMControl() 
{ 
	wxLogMessage(wxT("RMRadar_pi: %s thread is stopping"), m_ri->m_name.c_str());
}

void CRMControl::OnExit()
{
}

void CRMControl::logBinaryData(const wxString &what, const UINT8 *data, int size) 
{
	wxString explain;
	int i = 0;

	explain.Alloc(size * 3 + 50);
	explain += wxT("RMRadar_pi: ") + m_ri->m_name + wxT(" ");
	explain += what;
	explain += wxString::Format(wxT(" %d bytes: "), size);
	for (i = 0; i < size; i++) 
	{
		explain += wxString::Format(wxT(" %02X"), data[i]);
	}
	wxLogMessage(explain);
}

void CRMControl::ProcessFrame(const UINT8 *data, int len) 
{
	wxLongLong nowMillis = wxGetLocalTimeMillis();
	time_t now = time(0);

	if (m_quit || !m_pi->m_initialized) 
	{
		return;
	}

	m_ri->m_radar_timeout = now + WATCHDOG_TIMEOUT;

	int spoke = 0;
	m_ri->m_statistics.packets++;

	if(len >= 4)
	{
		uint32_t msgId = 0;
		memcpy(&msgId, data, sizeof(msgId));
		switch(msgId)
		{
		case 0x00010001:
			ProcessFeedback(data, len);
			break;
		case 0x00010002:
			ProcessPresetFeedback(data, len);
			break;
		case 0x00010003:
			ProcessScanData(data, len);
			m_ri->m_data_timeout = now + DATA_TIMEOUT;
			break;
		case 0x00010005:
			ProcessCurveFeedback(data, len);
			break;
		case 0x00010006:
		case 0x00010007:
		case 0x00010008:
		case 0x00010009:
		case 0x00018942:
			break;	
		default:
			// fprintf(stderr, "Unknown message ID %08X.\n", (int)msgId);
			break;
		}
	}
}

/*
 * Called once a second. Emulate a radar return that is
 * at the current desired auto_range.
 * Speed is 24 images per minute, e.g. 1/2.5 of a full
 * image.
 */

#define MILLIS_PER_SELECT 250
#define SECONDS_SELECT(x) ((x)*MILLISECONDS_PER_SECOND / MILLIS_PER_SELECT)

void CRMControl::EmulateFakeBuffer(void) {
  time_t now = time(0);
  UINT8 data[RETURNS_PER_LINE];

  m_ri->m_radar_timeout = now + WATCHDOG_TIMEOUT;

  if (m_ri->m_state.value != RADAR_TRANSMIT) {
    if (m_ri->m_state.value == RADAR_OFF) {
      m_ri->m_state.value = RADAR_STANDBY;
    }
    return;
  }

  m_ri->m_statistics.packets++;
  m_ri->m_data_timeout = now + WATCHDOG_TIMEOUT;

  m_next_rotation = (m_next_rotation + 1) % SPOKES;

  int scanlines_in_packet = SPOKES * 24 / 60 * MILLIS_PER_SELECT / MILLISECONDS_PER_SECOND;
  int range_meters = 2308;
  int display_range_meters = 3000;
  int spots = 0;
  m_ri->m_radar_type = RT_4G;  // Fake for emulator
  m_pi->m_pMessageBox->SetRadarType(RT_4G);
  m_ri->m_range.Update(display_range_meters);

  for (int scanline = 0; scanline < scanlines_in_packet; scanline++) {
    int angle_raw = m_next_spoke;
    m_next_spoke = (m_next_spoke + 1) % SPOKES;
    m_ri->m_statistics.spokes++;

    // Invent a pattern. Outermost ring, then a square pattern
    for (size_t range = 0; range < sizeof(data); range++) {
      size_t bit = range >> 7;
      // use bit 'bit' of angle_raw
      UINT8 colour = (((angle_raw + m_next_rotation) >> 5) & (2 << bit)) > 0 ? (range / 2) : 0;
      if (range > sizeof(data) - 10) {
        colour = ((angle_raw + m_next_rotation) % SPOKES) <= 8 ? 255 : 0;
      }
      data[range] = colour;
      if (colour > 0) {
        spots++;
      }
    }

    int hdt_raw = SCALE_DEGREES_TO_RAW(m_pi->m_hdt);
    int bearing_raw = angle_raw + hdt_raw;
    bearing_raw += SCALE_DEGREES_TO_RAW(270);  // Compensate openGL rotation compared to North UP

    SpokeBearing a = MOD_ROTATION2048(angle_raw / 2);    // divide by 2 to map on 2048 scanlines
    SpokeBearing b = MOD_ROTATION2048(bearing_raw / 2);  // divide by 2 to map on 2048 scanlines

    m_ri->ProcessRadarSpoke(a, b, data, sizeof(data), range_meters);
  }

  LOG_VERBOSE(wxT("BR24radar_pi: emulating %d spokes at range %d with %d spots"), scanlines_in_packet, range_meters, spots);
}

SOCKET CRMControl::PickNextEthernetCard()
{
	SOCKET socket = INVALID_SOCKET;

	// Pick the next ethernet card
	// If set, we used this one last time. Go to the next card.
	if (m_interface)
	{
		m_interface = m_interface->ifa_next;
	}
	// Loop until card with a valid IPv4 address
	while (m_interface && !VALID_IPV4_ADDRESS(m_interface))
	{
		m_interface = m_interface->ifa_next;
	}

	if (!m_interface)
	{
		if (m_interface_array) 
		{
			freeifaddrs(m_interface_array);
			m_interface_array = 0;
		}
		if (!getifaddrs(&m_interface_array))
		{
			m_interface = m_interface_array;
		}
		// Loop until card with a valid IPv4 address
		while (m_interface && !VALID_IPV4_ADDRESS(m_interface))
		{
			m_interface = m_interface->ifa_next;
		}
	}

	if(VALID_IPV4_ADDRESS(m_interface)) 
	{
		wxString error;
		socket = startUDPMulticastReceiveSocket((struct sockaddr_in *)m_interface->ifa_addr, SEATALK_HS_ANNOUNCE_PORT,
				                    SEATALK_HS_ANNOUNCE_GROUP, error);
		if(socket != INVALID_SOCKET)
		{
			wxString addr;
			UINT8 *a = (UINT8 *)&((struct sockaddr_in *)m_interface->ifa_addr)->sin_addr;  // sin_addr is in network layout
			addr.Printf(wxT("%u.%u.%u.%u"), a[0], a[1], a[2], a[3]);
			if (m_pi->m_settings.verbose >= 1)
			{
				wxLogMessage(wxT("RMRadar_pi: Listening for %s reports on %s"), m_ri->m_name.c_str(), addr.c_str());
			}
			m_pi->m_pMessageBox->SetMcastIPAddress(addr);
		} 
		else 
		{
			wxLogMessage(wxT("RMRadar_pi: Unable to listen to socket: %s"), error.c_str());
		}
	}

	return socket;
}

SOCKET CRMControl::GetNewDataSocket()
{
	SOCKET socket;
	wxString error;

	if (!m_mcast_addr)
	{
		return INVALID_SOCKET;
	}

	char *mcast_ip = inet_ntoa(m_radar_mcast.sin_addr);
	// fprintf(stderr, "RMRadar_pi: creating socket for %s:%d", mcast_ip, ntohs(m_radar_mcast.sin_port));
	socket = startUDPMulticastReceiveSocket(m_mcast_addr, ntohs(m_radar_mcast.sin_port), mcast_ip, error);
	if (socket != INVALID_SOCKET)
	{
		wxString addr;
		UINT8 *a = (UINT8 *)&m_mcast_addr->sin_addr;  // sin_addr is in network layout
		addr.Printf(wxT("%u.%u.%u.%u"), a[0], a[1], a[2], a[3]);
		if (m_pi->m_settings.verbose)
		{
		wxLogMessage(wxT("RMRadar_pi: %s listening for data on %s"), m_ri->m_name.c_str(), addr.c_str());
		}
		// fprintf(stderr, " on %d.%d.%d.%d.\n", a[0], a[1], a[2], a[3]);
	} 
	else 
	{
		wxLogMessage(wxT("RMRadar_pi: Unable to listen to socket: %s"), error.c_str());
	}
	return socket;
}

void *CRMControl::Entry(void)
{
	int r = 0;
	int no_data_timeout = 0;

	union {
		sockaddr_storage addr;
		sockaddr_in ipv4;
	} rx_addr;

	socklen_t rx_len;
	UINT8 *a = (UINT8 *)&rx_addr.ipv4.sin_addr;  // sin_addr is in network layout

	UINT8 data[2048/*sizeof(radar_frame_pkt)*/];
	m_interface_array = 0;
	m_interface = 0;
	static struct sockaddr_in mcastFoundAddr;
	static struct sockaddr_in radarFoundAddr;

	// SOCKET dataSocket = INVALID_SOCKET;
	// SOCKET commandSocket = INVALID_SOCKET;
	SOCKET reportSocket = INVALID_SOCKET;

	if (m_pi->m_settings.verbose) {
		wxLogMessage(wxT("RMRadar_pi: CRMControl thread %s starting"), m_ri->m_name.c_str());
	}

	fprintf(stderr, "RMRadar_pi: CRMControl thread starting\n");

	socketReady(INVALID_SOCKET, 1000);  // sleep for 1s so that other stuff is set up (fixes Windows core on startup)

	while (!m_quit)
	{
		if (m_pi->m_settings.emulator_on) 
		{
			socketReady(INVALID_SOCKET, 1000);  // sleep for 1s
			EmulateFakeBuffer();
			continue;
		}

		if (reportSocket == INVALID_SOCKET) 
		{
			reportSocket = PickNextEthernetCard();
			if (reportSocket != INVALID_SOCKET)
			{
				no_data_timeout = -10;
			}
		} 

		struct timeval tv = {(long)1, (long)0};

		fd_set fdin;
		FD_ZERO(&fdin);

		int maxFd = INVALID_SOCKET;
		if (reportSocket != INVALID_SOCKET)
		{
			FD_SET(reportSocket, &fdin);
			maxFd = MAX(reportSocket, maxFd);
		}
		if (m_dataSocket != INVALID_SOCKET)
		{
			FD_SET(m_dataSocket, &fdin);
			maxFd = MAX(m_dataSocket, maxFd);
		}

		r = select(maxFd + 1, &fdin, 0, 0, &tv);

		if (m_quit) 
		{
			break;
		}

		time_t now = time(0);
		if(m_haveRadar && m_pi->m_settings.enable_transmit)
		{
			if(now >= m_lastKeepalive1s)
			{
				Send1sKeepalive();
				m_lastKeepalive1s = now + 1;
			}
			if(now >= m_lastKeepalive5s)
			{
				Send5sKeepalive();
				m_lastKeepalive5s = now + 5;
			}
		}

		if (r > 0) 
		{
			if (m_dataSocket != INVALID_SOCKET && FD_ISSET(m_dataSocket, &fdin)) 
			{
				rx_len = sizeof(rx_addr);
				r = recvfrom(m_dataSocket, (char *)data, sizeof(data), 0, (struct sockaddr *)&rx_addr, &rx_len);
				if (r > 0) 
				{
					ProcessFrame(data, r);
					no_data_timeout = -15;
				} 
				else 
				{
					closesocket(m_dataSocket);
					m_dataSocket = INVALID_SOCKET;
					wxLogMessage(wxT("RMRadar_pi: %s at %u.%u.%u.%u illegal frame"), m_ri->m_name.c_str(), a[0], a[1], a[2], a[3]);
				}
			}

			if (reportSocket != INVALID_SOCKET && FD_ISSET(reportSocket, &fdin))
			{
				rx_len = sizeof(rx_addr);
				r = recvfrom(reportSocket, (char *)data, sizeof(data), 0, (struct sockaddr *)&rx_addr, &rx_len);
				if (r > 0)
				{
					if (ProcessReport(data, r))
					{
						memcpy(&mcastFoundAddr, m_interface->ifa_addr, sizeof(mcastFoundAddr));
						m_mcast_addr = &mcastFoundAddr;
						memcpy(&radarFoundAddr, &rx_addr, sizeof(radarFoundAddr));
						// m_radar_addr = &radarFoundAddr;
						wxString addr;
						addr.Printf(wxT("%u.%u.%u.%u"), a[0], a[1], a[2], a[3]);
						m_pi->m_pMessageBox->SetRadarIPAddress(addr);
						if (m_ri->m_state.value == RADAR_OFF)
						{
							if (m_pi->m_settings.verbose)
							{
								wxLogMessage(wxT("RMRadar_pi: %s detected at %s"), m_ri->m_name.c_str(), addr.c_str());
							}
							m_ri->m_state.Update(RADAR_STANDBY);
						}
						m_ri->m_radar_timeout = time(0) + WATCHDOG_TIMEOUT;
						no_data_timeout++; // Make sure we do get some data

						if (m_dataSocket == INVALID_SOCKET)
						{
							m_dataSocket = GetNewDataSocket();
							m_lastKeepalive1s = time(0) + 1;
							m_lastKeepalive5s = m_lastKeepalive1s + 4;
							SendInitMessages();
						}
					}
				} 
				else 
				{
					wxLogMessage(wxT("RMRadar_pi: %s at %u.%u.%u.%u illegal report"), m_ri->m_name.c_str(), a[0], a[1], a[2], a[3]);
					closesocket(reportSocket);
					reportSocket = INVALID_SOCKET;
				}
			}
		} 
		else 
		{
			no_data_timeout++;
		}

		if (no_data_timeout >= 2)
		{
			no_data_timeout = 0;
			if (m_dataSocket != INVALID_SOCKET)
			{
				closesocket(m_dataSocket);
				m_dataSocket = INVALID_SOCKET;
			}
			if (reportSocket != INVALID_SOCKET) 
			{
				closesocket(reportSocket);
				reportSocket = INVALID_SOCKET;
				m_ri->m_state.Update(RADAR_OFF);
				m_mcast_addr = 0;
				// m_radar_addr = 0;
				m_haveRadar = false;
			}
		}
	}

	if (m_dataSocket != INVALID_SOCKET)
	{
		closesocket(m_dataSocket);
	}
	if (reportSocket != INVALID_SOCKET)
	{
		closesocket(reportSocket);
	}

	if (m_interface_array)
	{
		freeifaddrs(m_interface_array);
	}
	return 0;
}

//
// The following is the received radar state. It sends this regularly
// but especially after something sends it a state change.
//
#pragma pack(push, 1)

struct SRadarFeedback {
	uint32_t type;	// 0x010001
	uint32_t range_values[11];
	uint32_t something_1[33]; 
	uint8_t status;		// 2 - warmup, 1 - transmit, 0 - standby, 6 - shutting down (warmup time - countdown), 3 - shutdown
	uint8_t something_2[3];
	uint8_t warmup_time;
	uint8_t signal_strength;	// number of bars
	uint8_t something_3[7];
	uint8_t range_id;
	uint8_t something_4[2];
	uint8_t auto_gain;
	uint8_t something_5[3];
	uint32_t gain;
	uint8_t auto_sea; // 0 - disabled; 1 - harbour, 2 - offshore, 3 - coastal 
	uint8_t something_6[3];
	uint8_t sea_value;
	uint8_t rain_enabled;
	uint8_t something_7[3];
	uint8_t rain_value;
	uint8_t ftc_enabled;
	uint8_t something_8[3];
	uint8_t ftc_value;
	uint8_t auto_tune;
	uint8_t something_9[3];
	uint8_t tune;
	int16_t bearing_offset;	// degrees * 10; left - negative, right - positive
	uint8_t interference_rejection;
	uint8_t something_10[3];
	uint8_t target_expansion;
	uint8_t something_11[13];
	uint8_t mbs_enabled;	// Main Bang Suppression enabled if 1
};

struct SRadarPresetFeedback {
	uint32_t type;	// 0x010002
	uint8_t something_1[213]; // 221 - magnetron current; 233, 234 - rotation time ms (251 total)
	uint16_t magnetron_hours;
	uint8_t something_2[6];
	uint8_t magnetron_current;
	uint8_t something_3[11];
	uint16_t rotation_time;
	uint8_t something_4[13];
	uint8_t stc_preset_max;
	uint8_t something_5[2];
	uint8_t coarse_tune_arr[3];
	uint8_t fine_tune_arr[3]; // 0, 1, 2 - fine tune value for SP, MP, LP
	uint8_t something_6[6];	  
	uint8_t display_timing_value;
	uint8_t something_7[12];
	uint8_t stc_preset_value;
	uint8_t something_8[12];
	uint8_t min_gain;
	uint8_t max_gain;
	uint8_t min_sea;
	uint8_t max_sea;
	uint8_t min_rain;
	uint8_t max_rain;
	uint8_t min_ftc;
	uint8_t max_ftc;
	uint8_t gain_value;
	uint8_t sea_value;
	uint8_t fine_tune_value;
	uint8_t coarse_tune_value;
	uint8_t signal_strength_value;
	uint8_t something_9[2];
};

struct SCurveFeedback {
	uint32_t type;	// 0x010005
	uint8_t curve_value;
};


#pragma pack(pop)

static uint8_t radar_signature_id[4] = { 1, 0, 0, 0 };
static char *radar_signature = (char *)"Ethernet Dome";
struct SRMRadarFunc {
	uint32_t type;
	uint32_t dev_id;
	uint32_t func_id;	// 1
	uint32_t something_1;
	uint32_t something_2;
	uint32_t mcast_ip;
	uint32_t mcast_port;
	uint32_t radar_ip;
	uint32_t radar_port;
};	

bool CRMControl::ProcessReport(const UINT8 *report, int len)
{
	if (m_pi->m_settings.verbose >= 3)
	{
		logBinaryData(wxT("ProcessReport"), report, len);
	}
	if(len == sizeof(SRMRadarFunc))
	{
		SRMRadarFunc *rRec = (SRMRadarFunc *)report;
		if(rRec->func_id == 1)
		{
			if(!m_haveRadar)
			{
				m_radar_mcast.sin_family = AF_INET;
				m_radar_mcast.sin_addr.s_addr = ntohl(rRec->mcast_ip);
				m_radar_mcast.sin_port = htons(rRec->mcast_port);
				m_radar_addr.sin_family = AF_INET;
				m_radar_addr.sin_addr.s_addr = ntohl(rRec->radar_ip);
				m_radar_addr.sin_port = ntohs(rRec->radar_port);
				m_haveRadar = true;

				if(m_pi->m_settings.enable_transmit)
				{
					fprintf(stderr, "Sending initial messages to %d.%d.%d.%d:%d.\n", 
						(rRec->radar_ip >> 24) & 0xff, (rRec->radar_ip >> 16) & 0xff,
						(rRec->radar_ip >> 8) & 0xff, rRec->radar_ip & 0xff, rRec->radar_port);
				}
				else
				{
					fprintf(stderr, "Transmit not enabled.\n");
				}
			}
			return true;
		}
	}

	if (m_pi->m_settings.verbose >= 2) {
		logBinaryData(wxT("received unknown message"), report, len);
	}
	return false;
}

//static int radar_ranges[] = { 1852 / 8, 1852 / 4, 1852 / 2, 1852 * 3 / 4, 1852 * 3 / 2, 1852 * 3, 1852 * 6, 1852 * 12, 1852 * 24 };
static int radar_ranges[] = { 1852 / 4, 1852 / 2, 1852, 1852 * 3 / 2, 1852 * 3, 1852 * 6, 1852 * 12, 1852 * 24, 1852 * 48, 1852 * 96, 1852 * 144 };
static int current_ranges[11] = { 125, 250, 500, 750, 1500, 3000, 6000, 12000, 24000, 48000, 72000 };

bool CRMControl::SetRangeMeters(int meters)
{
	for(int i = 0; i < sizeof(radar_ranges) / sizeof(radar_ranges[0]); i++)
	{
		if(meters <= radar_ranges[i])
		{
			SetRange(i);
			return true;
		}
	}
	SetRange(sizeof(radar_ranges) / sizeof(radar_ranges[0]) - 1);
	return false;
}

void CRMControl::ProcessFeedback(const UINT8 *data, int len)
{
	if(len == sizeof(SRadarFeedback))
	{
		SRadarFeedback *fbPtr = (SRadarFeedback *)data;
		if(fbPtr->type == 0x010001)
		{
			switch(fbPtr->status)
			{
			case 0:
				wxLogMessage(wxT("RMRadar_pi: %s received transmit off from %s"), m_ri->m_name.c_str(), "--"/*addr.c_str()*/);
				m_ri->m_state.Update(RADAR_STANDBY);
				break;	
			case 1:
				wxLogMessage(wxT("RMRadar_pi: %s received transmit on from %s"), m_ri->m_name.c_str(), "--"/*addr.c_str()*/);
				m_ri->m_state.Update(RADAR_TRANSMIT);
				break;
			case 2:	// Warmup
				wxLogMessage(wxT("RMRadar_pi: %s radar is warming up %s"), m_ri->m_name.c_str(), "--"/*addr.c_str()*/);
				m_ri->m_state.Update(RADAR_WAKING_UP);
				break;
			case 3:	// Off
				wxLogMessage(wxT("RMRadar_pi: %s radar is off %s"), m_ri->m_name.c_str(), "--"/*addr.c_str()*/);
				m_ri->m_state.Update(RADAR_OFF);
				break;
			default:
				m_ri->m_state.value = RADAR_STANDBY;
				break;	
			}
			if(fbPtr->range_values[0] != current_ranges[0]) // Units must have changed
			{
				for(int i = 0; i < sizeof(current_ranges) / sizeof(current_ranges[0]); i++)
				{
					current_ranges[i] = fbPtr->range_values[i];
					radar_ranges[i] = 1852 * fbPtr->range_values[i] / 500;

					fprintf(stderr, "%d (%d)\n", current_ranges[i], radar_ranges[i]);
				}
			}
			if(radar_ranges[fbPtr->range_id] != m_range_meters) {
				if (m_pi->m_settings.verbose >= 1)
				{
					  wxLogMessage(wxT("RMRadar_pi: %s now scanning with range %d meters (was %d meters)"), m_ri->m_name.c_str(),
						radar_ranges[fbPtr->range_id],
						m_range_meters);
				}
				m_range_meters = radar_ranges[fbPtr->range_id];
				m_updated_range = true;
				m_ri->m_range.Update(m_range_meters / 2); // RM MFD shows half of what is received
			}

			m_gain.Set(fbPtr->gain);
			m_gain.SetActive(fbPtr->auto_gain == 0);
			m_sea.Set(fbPtr->sea_value);
			m_sea.SetActive(fbPtr->auto_sea == 0);
			m_autoSea.Set(fbPtr->auto_sea);
			m_autoSea.SetActive(fbPtr->auto_sea != 0);
				
			m_rain.SetActive(fbPtr->rain_enabled == 1);
			m_rain.Set(fbPtr->rain_value);

			m_targetBoost.Set(fbPtr->target_expansion);
			m_interferenceRejection.Set(fbPtr->interference_rejection);

			int ba = (int)fbPtr->bearing_offset;
			m_bearingOffset.Set(ba);

			m_tuneFine.SetActive(fbPtr->auto_tune == 0);
			m_tuneFine.Set(fbPtr->tune);
			m_tuneCoarse.SetActive(fbPtr->auto_tune == 0);

			m_mbsEnabled.Set(fbPtr->mbs_enabled);

			m_miscInfo.m_warmupTime = fbPtr->warmup_time;
			m_miscInfo.m_signalStrength = fbPtr->signal_strength;

			m_ftc.SetActive(fbPtr->ftc_enabled == 1);
			m_ftc.Set(fbPtr->ftc_value);

  			if(m_ri->m_control_dialog)
			{
				wxCommandEvent event(wxEVT_COMMAND_TEXT_UPDATED, ID_CONTROL_DIALOG_REFRESH);
				m_ri->m_control_dialog->GetEventHandler()->AddPendingEvent(event);
			}
		}
	}
}

void CRMControl::ProcessPresetFeedback(const UINT8 *data, int len)
{
	if(len == sizeof(SRadarPresetFeedback))
	{
		SRadarPresetFeedback *fbPtr = (SRadarPresetFeedback *)data;

		m_tuneCoarse.Set(fbPtr->coarse_tune_value);
		m_stc.Set(fbPtr->stc_preset_value);
		m_displayTiming.Set(fbPtr->display_timing_value);
		m_stc.SetMax(fbPtr->stc_preset_max);
		m_gain.SetMin(fbPtr->min_gain); m_gain.SetMax(fbPtr->max_gain);
		m_sea.SetMin(fbPtr->min_sea); m_sea.SetMax(fbPtr->max_sea);
		m_rain.SetMin(fbPtr->min_rain); m_rain.SetMax(fbPtr->max_rain);
		m_ftc.SetMin(fbPtr->min_ftc); m_ftc.SetMax(fbPtr->max_ftc);

		m_miscInfo.m_signalStrength = fbPtr->signal_strength_value;
		m_miscInfo.m_magnetronCurrent = fbPtr->magnetron_current;
		m_miscInfo.m_magnetronHours = fbPtr->magnetron_hours;
		m_miscInfo.m_rotationPeriod = fbPtr->rotation_time;

	}
}

void CRMControl::ProcessCurveFeedback(const UINT8 *data, int len)
{
	if(len == sizeof(SCurveFeedback))
	{
		SCurveFeedback *fbPtr = (SCurveFeedback *)data;
		switch(fbPtr->curve_value)
		{
		case 0:
			m_stcCurve.Set(1);
			break;
		case 1:
			m_stcCurve.Set(2);
			break;
		case 2:
			m_stcCurve.Set(3);
			break;
		case 4:
			m_stcCurve.Set(4);
			break;
		case 6:
			m_stcCurve.Set(5);
			break;
		case 8:
			m_stcCurve.Set(6);
			break;
		case 10:
			m_stcCurve.Set(7);
			break;
		case 13:
			m_stcCurve.Set(8);
			break;
		default:
			fprintf(stderr, "ProcessCurveFeedback: unknown curve value %d.\n", (int)fbPtr->curve_value);
		}
	}
	else
	{
		fprintf(stderr, "ProcessCurveFeedback: got %d bytes, expected %d.\n", len, (int)sizeof(SCurveFeedback));
	}
}

// Radar data

struct CRMPacketHeader {
    uint32_t type;		// 0x00010003
    uint32_t zero_1;
    uint32_t something_1;	// 0x0000001c
    uint32_t nspokes;		// 0x00000008 - usually but changes
    uint32_t spoke_count;	// 0x00000000 in regular, counting in HD
    uint32_t zero_3;
    uint32_t something_3;	// 0x00000001
    uint32_t something_4;	// 0x00000000 or 0xffffffff in regular, 0x400 in HD
};

struct CRMRecordHeader {
    uint32_t type;
    uint32_t length;
    // ...
};

struct CRMScanHeader {
    uint32_t type;		// 0x00000001
    uint32_t length;		// 0x00000028
    uint32_t azimuth;
    uint32_t something_2;	// 0x00000001 - 0x03 - HD
    uint32_t something_3;	// 0x00000002
    uint32_t something_4;	// 0x00000001 - 0x03 - HD
    uint32_t something_5;	// 0x00000001 - 0x00 - HD
    uint32_t something_6;	// 0x000001f4 - 0x00 - HD
    uint32_t zero_1;
    uint32_t something_7;	// 0x00000001
};

struct CRMOptHeader {		// No idea what is in there
    uint32_t type;		// 0x00000002
    uint32_t length;		// 0x0000001c
    uint32_t zero_2[5];
};

struct CRMScanData {
    uint32_t type;		// 0x00000003
    uint32_t length;
    uint32_t data_len;
    // unsigned char data[rec_len - 8];
};

void CRMControl::ProcessScanData(const UINT8 *data, int len)
{
	if(len > sizeof(CRMPacketHeader) + sizeof(CRMScanHeader))
	{
		CRMPacketHeader *pHeader = (CRMPacketHeader *)data;
		if(pHeader->type != 0x00010003 || pHeader->something_1 != 0x0000001c || 
			pHeader->something_3 != 0x0000001)
		{
			fprintf(stderr, "ProcessScanData::Packet header mismatch %x, %x, %x, %x.\n", pHeader->type, pHeader->something_1, 
				pHeader->nspokes, pHeader->something_3);
			return;
		}

		m_ri->m_state.Update(RADAR_TRANSMIT);

		if(pHeader->something_4 == 0x400)
		{
			if(m_ri->m_radar_type != RT_4G)
			{
				m_ri->m_radar_type = RT_4G;
				m_pi->m_pMessageBox->SetRadarType(RT_4G);
			}
		}
		else
		{
			if(m_ri->m_radar_type != RT_BR24)
			{
				m_ri->m_radar_type = RT_BR24;
				m_pi->m_pMessageBox->SetRadarType(RT_BR24);
			}
		}
	
		wxLongLong nowMillis = wxGetLocalTimeMillis();
		int headerIdx = 0;
		int nextOffset = sizeof(CRMPacketHeader);


		while(nextOffset < len)
		{
			CRMScanHeader *sHeader = (CRMScanHeader *)(data + nextOffset);
			if(sHeader->type != 0x00000001 || sHeader->length != 0x00000028)
			{
				fprintf(stderr, "ProcessScanData::Scan header #%d (%d) - %x, %x.\n", headerIdx, nextOffset, sHeader->type, sHeader->length);
				break;
			}
		
			if(sHeader->something_2 != 0x00000001 || sHeader->something_3 != 0x00000002 || sHeader->something_4 != 0x00000001 ||
    				sHeader->something_5 != 0x00000001 || sHeader->something_6 != 0x000001f4 || sHeader->something_7 != 0x00000001)
			{
				if(sHeader->something_2 != 3 || sHeader->something_3 != 2 || sHeader->something_4 != 3 ||
					sHeader->something_5 != 0 || sHeader->something_6 != 0 || sHeader->something_7 != 1)
				{
					fprintf(stderr, "ProcessScanData::Scan header #%d part 2 check failed.\n", headerIdx);
					break;
				}
				else if(m_ri->m_radar_type != RT_4G)
				{
					m_ri->m_radar_type = RT_4G;
					m_pi->m_pMessageBox->SetRadarType(RT_4G);
					fprintf(stderr, "ProcessScanData::Scan header #%d HD second header with regular first.\n", headerIdx);
				}				
				
			}
			else if(m_ri->m_radar_type != RT_BR24)
			{
				m_ri->m_radar_type = RT_BR24;
				m_pi->m_pMessageBox->SetRadarType(RT_BR24);
				fprintf(stderr, "ProcessScanData::Scan header #%d regular second header with HD first.\n", headerIdx);
			}

			nextOffset += sizeof(CRMScanHeader);

			CRMRecordHeader *nHeader = (CRMRecordHeader *)(data + nextOffset);
			if(nHeader->type == 0x00000002)
			{
				if(nHeader->length != 0x0000001c)
				{
					// fprintf(stderr, "ProcessScanData::Opt header #%d part 2 check failed.\n", headerIdx);
				}
				nextOffset += nHeader->length;
			}

			CRMScanData *pSData = (CRMScanData *)(data + nextOffset);
			
			if((pSData->type & 0x7fffffff) != 0x00000003 || pSData->length < pSData->data_len + 8)
			{
				fprintf(stderr, "ProcessScanData::Scan data header #%d check failed %x, %d, %d.\n", headerIdx, 
					pSData->type, pSData->length, pSData->data_len);
				break;
			}

			UINT8 unpacked_data[1024], *dataPtr = 0;
			if(m_ri->m_radar_type == RT_BR24)
			{
				uint8_t *dData = (uint8_t *)unpacked_data;
				uint8_t *sData = (uint8_t *)data + nextOffset + sizeof(CRMScanData);

				int iS = 0;
				int iD = 0;
				while(iS < pSData->data_len)
				{
					if(*sData != 0x5c)
					{
						*dData++ = (((*sData) & 0x0f) << 4) + 0x0f;
						*dData++ = ((*sData) & 0xf0) + 0x0f;
						sData++;
						iS++; iD += 2;
					}
					else
					{					
						uint8_t nFill = sData[1];
						uint8_t cFill = sData[2];
					
						for(int i = 0; i < nFill; i++)
						{
							*dData++ = ((cFill & 0x0f) << 4) + 0x0f;
							*dData++ = (cFill & 0xf0) + 0x0f;
						}
						sData += 3;
						iS += 3;
						iD += nFill * 2;
					}
	  
				}
				if(iD != 512)
				{
					while(iS < pSData->length - 8 && iD < 512)
					{
						*dData++ = ((*sData) & 0x0f) << 4;
						*dData++ = (*sData) & 0xf0;
						sData++;
						iS++; iD += 2;
					}
				}
				if(iD != 512)
				{
					// fprintf(stderr, "ProcessScanData::Packet %d line %d (%d/%x) not complete %d.\n", packetIdx, headerIdx,
					// 	scan_idx, scan_idx, iD);
				}
				dataPtr = unpacked_data;
			}
			else if(m_ri->m_radar_type == RT_4G)
			{
				if(pSData->data_len != RETURNS_PER_LINE * 2)
				{
					m_ri->m_statistics.broken_spokes++;
					fprintf(stderr, "ProcessScanData data len %d should be %d.\n", pSData->data_len, RETURNS_PER_LINE);
					break;
				}
				if(m_range_meters == 0) m_range_meters = 1852 / 4; // !!!TEMP delete!!!
				dataPtr = (UINT8 *)data + nextOffset + sizeof(CRMScanData);
			}
			else
			{
				fprintf(stderr, "ProcessScanData::Packet radar type is not set somehow.\n");
				break;
			}

			nextOffset += pSData->length;
			m_ri->m_statistics.spokes++;
			unsigned int spoke = sHeader->azimuth;
			if (m_next_spoke >= 0 && spoke != m_next_spoke) {
				if (spoke > m_next_spoke) {
					m_ri->m_statistics.missing_spokes += spoke - m_next_spoke;
				} else {
					m_ri->m_statistics.missing_spokes += SPOKES + spoke - m_next_spoke;
				}
			}
			m_next_spoke = (spoke + 1) % 2048;

			if((pSData->type & 0x80000000) != 0 && nextOffset < len)
			{
				// fprintf(stderr, "ProcessScanData::Last record %d (%d) in packet %d but still data to go %d:%d.\n",
				// 	headerIdx, scan_idx, packetIdx, nextOffset, len); 
			}
				
			headerIdx++;
			
			m_pi->SetRadarHeading();
			int hdt_raw = SCALE_DEGREES_TO_RAW(m_pi->m_hdt + m_ri->m_viewpoint_rotation);

			int angle_raw = spoke * 2 + SCALE_DEGREES_TO_RAW(180);  // Compensate openGL rotation compared to North UP
			int bearing_raw = angle_raw + hdt_raw;

			SpokeBearing a = MOD_ROTATION2048(angle_raw / 2);    // divide by 2 to map on 2048 scanlines
			SpokeBearing b = MOD_ROTATION2048(bearing_raw / 2);  // divide by 2 to map on 2048 scanlines

			m_ri->ProcessRadarSpoke(a, b, dataPtr, RETURNS_PER_LINE, m_range_meters);
		}
	}
}

static uint8_t rd_msg_1s[] = {
	0x00, 0x80, 0x01, 0x00, 0x52, 0x41, 0x44, 0x41, 0x52, 0x00, 0x00, 0x00
};

void CRMControl::Send1sKeepalive()
{
	sendto(m_dataSocket, (const char *)rd_msg_1s, sizeof(rd_msg_1s), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}


static uint8_t rd_msg_5s[] = {
	0x03, 0x89, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x68, 0x01, 0x00, 0x00, 0x9e, 0x03, 0x00, 0x00, 0xb4, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

void CRMControl::Send5sKeepalive()
{
	sendto(m_dataSocket, (const char *)rd_msg_5s, sizeof(rd_msg_5s), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

static uint8_t rd_msg_once[] = {
	0x02, 0x81, 0x01, 0x00, 0x7d, 0x00, 0x00, 0x00, 0xfa, 0x00, 0x00, 0x00, 0xf4, 0x01, 0x00, 0x00,
	0xee, 0x02, 0x00, 0x00, 0xdc, 0x05, 0x00, 0x00, 0xb8, 0x0b, 0x00, 0x00, 0x70, 0x17, 0x00, 0x00,
	0xe0, 0x2e, 0x00, 0x00, 0xc0, 0x5d, 0x00, 0x00, 0x80, 0xbb, 0x00, 0x00, 0x40, 0x19, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static uint8_t wakeup_msg[] = "ABCDEFGHIJKLMNOP";

void CRMControl::WakeupRadar()
{
	if(!m_pi->m_settings.enable_transmit) return;

	int outSD = socket(AF_INET, SOCK_DGRAM, 0);
	if(outSD < 0)
	{
		perror("Unable to create a socket");
		return;
	}

	struct sockaddr_in groupSock;
	memset((char *)&groupSock, 0, sizeof(groupSock));
	groupSock.sin_family = AF_INET;
	groupSock.sin_addr.s_addr = inet_addr("224.0.0.1");
	groupSock.sin_port = htons(5800);

	for(int i = 0; i < 10; i++)
	{
		int res = sendto(outSD, (const char *)wakeup_msg, 16, 0, (struct sockaddr*)&groupSock, sizeof(groupSock));
		if(res < 0)
		{
			perror("send_revive, dying 1");
		}
		Sleep(10);
	}

	close(outSD);
}

void CRMControl::SendInitMessages()
{
	Send1sKeepalive();
	Send5sKeepalive();
	int rv = sendto(m_dataSocket, (const char *)rd_msg_once, sizeof(rd_msg_once), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
	perror("SendInitMessages");
}

static uint8_t rd_msg_tx_control[] = {
	0x01, 0x80, 0x01, 0x00,
	0x00, // Control value at offset 4 : 0 - off, 1 - on
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_set_range[] = {
	0x01, 0x81, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x01,	// Range at offset 8 (0 - 1/8, 1 - 1/4, 2 - 1/2, 3 - 3/4, 4 - 1, 5 - 1.5, 6 - 3...)
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_mbs_control[] = {
	0x01, 0x82, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, // MBS Enable (1) at offset 16
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t rd_msg_set_display_timing[] = {
	0x02, 0x82, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x6d, // Display timing value at offset 8
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_set_stc_preset[] = {
	0x03, 0x82, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x74, // STC preset value at offset 8
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_tune_coarse[] = {
	0x04, 0x82, 0x01, 0x00, 
	0x00, // Coarse tune at offset 4
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_bearing_offset[] = {
	0x07, 0x82, 0x01, 0x00, 
	0x14, 0x00, 0x00, 0x00
};

static uint8_t rd_msg_set_sea[] = {
	0x02, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 
	0x00, // Sea value at offset 20
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_sea_auto[] = {
	0x02, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, // Sea auto value at offset 16
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t rd_msg_set_gain[] = {
	0x01, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, // Gain value at offset 20
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_set_gain_auto[] = {
	0x01, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, // Gain auto - 1, manual - 0 at offset 16
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t rd_msg_rain_on[] = {
	0x03, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, // Rain on at offset 16 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t rd_msg_rain_set[] = {
	0x03, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 
	0x33, // Rain value at offset 20 
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_ftc_on[] = {
	0x04, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, // FTC on at offset 16
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t rd_msg_ftc_set[] = {
	0x04, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 
	0x1a, // FTC value at offset 20
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_tune_auto[] = {
	0x05, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x01, // Enable at offset 12
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t rd_msg_tune_fine[] = {
	0x05, 0x83, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, // Tune value at offset 16
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_target_expansion[] = {
	0x06, 0x83, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x01,	// Expansion value 0 - disabled, 1 - low, 2 - high 
	0x00, 0x00, 0x00
};

static uint8_t rd_msg_interference_rejection[] = {
	0x07, 0x83, 0x01, 0x00, 
	0x01,	// Interference rejection at offset 4, 0 - off, 1 - normal, 2 - high 
	0x00, 0x00, 0x00
};

static uint8_t curve_values[] = { 0, 1, 2, 4, 6, 8, 10, 13 };
static uint8_t rd_msg_curve_select[] = {
	0x0a, 0x83, 0x01, 0x00, 
	0x01	// Curve value at offset 4
};

void CRMControl::SetGain(uint8_t value)
{
	rd_msg_set_gain[20] = value;
	sendto(m_dataSocket, (const char *)rd_msg_set_gain, sizeof(rd_msg_set_gain), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetAutoGain(bool enable)
{
	rd_msg_set_gain_auto[16] = enable ? 1 : 0;
	sendto(m_dataSocket, (const char *)rd_msg_set_gain_auto, sizeof(rd_msg_set_gain_auto), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetTune(uint8_t value)
{
	rd_msg_tune_fine[16] = value;
	sendto(m_dataSocket, (const char *)rd_msg_tune_fine, sizeof(rd_msg_tune_fine), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetAutoTune(bool enable)
{
	rd_msg_tune_auto[12] = enable ? 1 : 0;
	sendto(m_dataSocket, (const char *)rd_msg_tune_auto, sizeof(rd_msg_tune_auto), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetCoarseTune(uint8_t value)
{
	rd_msg_tune_coarse[4] = value;
	sendto(m_dataSocket, (const char *)rd_msg_tune_coarse, sizeof(rd_msg_tune_coarse), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::EnableTX(bool enabled)
{
	if(m_haveRadar && m_pi->m_settings.enable_transmit)
	{
		rd_msg_tx_control[4] = enabled ? 1 : 0;
		sendto(m_dataSocket, (const char *)rd_msg_tx_control, sizeof(rd_msg_tx_control), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
	}
}

void CRMControl::TurnOff()
{
	if(m_haveRadar && m_pi->m_settings.enable_transmit)
	{
		rd_msg_tx_control[4] = 3;
		sendto(m_dataSocket, (const char *)rd_msg_tx_control, sizeof(rd_msg_tx_control), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
	}
}

void CRMControl::SetRange(uint8_t range_idx)
{
	rd_msg_set_range[8] = range_idx;
	sendto(m_dataSocket, (const char *)rd_msg_set_range, sizeof(rd_msg_set_range), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetSTCPreset(uint8_t value)
{
	rd_msg_set_stc_preset[8] = value;
	sendto(m_dataSocket, (const char *)rd_msg_set_stc_preset, sizeof(rd_msg_set_stc_preset), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetFTC(uint8_t value)
{
	rd_msg_ftc_set[20] = value;
	sendto(m_dataSocket, (const char *)rd_msg_ftc_set, sizeof(rd_msg_ftc_set), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetFTCEnabled(bool enable)
{
	rd_msg_ftc_on[16] = enable ? 1 : 0;
	sendto(m_dataSocket, (const char *)rd_msg_ftc_on, sizeof(rd_msg_ftc_on), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetRain(uint8_t value)
{
	rd_msg_rain_set[20] = value;
	sendto(m_dataSocket, (const char *)rd_msg_rain_set, sizeof(rd_msg_rain_set), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetRainEnabled(bool enable)
{
	rd_msg_rain_on[16] = enable ? 1 : 0;
	sendto(m_dataSocket, (const char *)rd_msg_rain_on, sizeof(rd_msg_rain_on), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetSea(uint8_t value)
{
	rd_msg_set_sea[20] = value;
	sendto(m_dataSocket, (const char *)rd_msg_set_sea, sizeof(rd_msg_set_sea), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetAutoSea(uint8_t value)
{
	rd_msg_sea_auto[16] = value;
	sendto(m_dataSocket, (const char *)rd_msg_sea_auto, sizeof(rd_msg_sea_auto), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetDisplayTiming(uint8_t value)
{
	rd_msg_set_display_timing[8] = value;
	sendto(m_dataSocket, (const char *)rd_msg_set_display_timing, sizeof(rd_msg_set_display_timing), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetBearingOffset(int32_t value)
{
	rd_msg_bearing_offset[4] = value & 0xff;
	rd_msg_bearing_offset[5] = (value >> 8) & 0xff;
	rd_msg_bearing_offset[6] = (value >> 16) & 0xff;
	rd_msg_bearing_offset[7] = (value >> 24) & 0xff;
	sendto(m_dataSocket, (const char *)rd_msg_bearing_offset, sizeof(rd_msg_bearing_offset), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::SetSeaClutterCurve(uint8_t id)
{
	rd_msg_curve_select[4] = curve_values[id - 1];
	sendto(m_dataSocket, (const char *)rd_msg_curve_select, sizeof(rd_msg_curve_select), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

void CRMControl::EnableMBS(bool enable)
{
	rd_msg_mbs_control[16] = enable ? 1 : 0;
	sendto(m_dataSocket, (const char *)rd_msg_mbs_control, sizeof(rd_msg_mbs_control), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
}

bool CRMControl::SetInterferenceRejection(uint8_t value)
{
	if(value >= 0 && value <= 2)
	{
		rd_msg_interference_rejection[4] = value;
		sendto(m_dataSocket, (const char *)rd_msg_interference_rejection, sizeof(rd_msg_interference_rejection), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
	}
	else return false;
}

bool CRMControl::SetTargetExpansion(uint8_t value)
{
	if(value >= 0 && value <= 2)
	{
		rd_msg_target_expansion[8] = value;
		sendto(m_dataSocket, (const char *)rd_msg_target_expansion, sizeof(rd_msg_target_expansion), 0, (struct sockaddr*)&m_radar_addr, sizeof(m_radar_addr));
	}
	else return false;
}

bool CRMControl::SetControlValue(ControlType controlType, int value)
{
	if(!m_haveRadar || !m_pi->m_settings.enable_transmit) return false;

	switch(controlType)
	{
	case CT_GAIN:
		SetGain(value);
		break;
	case CT_SEA:
		SetSea(value);
		break;
	case CT_SEA_AUTO:
		SetAutoSea(value);
		break;
	case CT_RAIN:
		SetRain(value);
		break;
	case CT_FTC:
		SetFTC(value);
		break;
	case CT_INTERFERENCE_REJECTION:
		SetInterferenceRejection(value);
		break;
	case CT_TARGET_BOOST:
		SetTargetExpansion(value);
		break;
	case CT_BEARING_ALIGNMENT:
		SetBearingOffset(value);
		break;
	case CT_STC:
		SetSTCPreset(value);
		break;
	case CT_TUNE_FINE:
		SetTune(value);
		break;
	case CT_TUNE_COARSE:
		SetCoarseTune(value);
		break;
	case CT_MBS_ENABLED:
		EnableMBS(value == 1);
		break;
	case CT_DISPLAY_TIMING:
		SetDisplayTiming(value);
		break;
	case CT_STC_CURVE:
		SetSeaClutterCurve(value);
		break;
	default:
		return false;
	}
	return true;
	
}

const CControlItem & CRMControl::GetControlValue(ControlType controlType) const
{
	switch(controlType)
	{
	case CT_GAIN:
		return m_gain;
	case CT_SEA:
		return m_sea;
	case CT_SEA_AUTO:
		return m_autoSea;
	case CT_RAIN:
		return m_rain;
	case CT_FTC:
		return m_ftc;
	case CT_INTERFERENCE_REJECTION:
		return m_interferenceRejection;
	case CT_TARGET_BOOST:
		return m_targetBoost;
	case CT_BEARING_ALIGNMENT:
		return m_bearingOffset;
	case CT_STC:
		return m_stc;
	case CT_TUNE_FINE:
		return m_tuneFine;
	case CT_TUNE_COARSE:
		return m_tuneCoarse;
	case CT_MBS_ENABLED:
		return m_mbsEnabled;
	case CT_DISPLAY_TIMING:
		return m_displayTiming;
	case CT_STC_CURVE:
		return m_stcCurve;
	default:
		throw invalid_control();
	}
}

bool CRMControl::ChangeControlValue(ControlType controlType, int change)
{
	const CControlItem & item = GetControlValue(controlType);
	try
	{
		// fprintf(stderr, "Changing item %d by %d.\n", controlType, change);
		if(!item.IsSet()) return false;
		int newValue = item.Get() + change;
		// fprintf(stderr, "New value %d - ", newValue);
		if(item.Min().IsSet() && item.Max().IsSet())
		{
			if(newValue < item.Min().Get()) newValue = item.Min().Get();
			else if(newValue > item.Max().Get()) newValue = item.Max().Get();

			// fprintf(stderr, "%d.\n", newValue);

			if(!item.IsActive()) ToggleAuto(controlType);
			return SetControlValue(controlType, newValue);
		}
		else
		{
			// fprintf(stderr, "Did not pass check. ");
			if(item.Min().IsSet()) fprintf(stderr, "Min: %d ", item.Min().Get());
			if(item.Max().IsSet()) fprintf(stderr, "Max: %d ", item.Max().Get());
			// fprintf(stderr, "\n");
		}
	}
	catch(invalid_control &e)
	{
		fprintf(stderr, "CRMControl::ChangeControlValue invalid control %d.\n", controlType);
	}
	catch(value_not_set &e)
	{
		fprintf(stderr, "CRMControl::ChangeControlValue value not (yet) set for control %d.\n", controlType);
	}
	return false;
}

bool CRMControl::ToggleAuto(ControlType controlType)
{
	try
	{
		const CControlItem & item = GetControlValue(controlType);
		switch(controlType)
		{
		case CT_GAIN:
		case CT_RAIN:
		case CT_SEA:
		case CT_SEA_AUTO:
		case CT_FTC:
		case CT_TUNE_FINE:
		case CT_TUNE_COARSE:
			if(!item.IsSet()) return false;
			break;
		default:
			return false;
		}

		switch(controlType)
		{
		case CT_GAIN:
			SetAutoGain(item.IsActive());	// IsActive means not auto
			break;
		case CT_RAIN:
			SetRainEnabled(!item.IsActive()); // IsActive means enabled
			break;
		case CT_SEA:
			if(item.IsActive())		// IsActive means not auto
			{
				SetAutoSea(1);
			}
			else
			{
				SetAutoSea(0);
			}
			break;
		case CT_SEA_AUTO:
			SetAutoSea(item.IsActive() ? 0 : 1);
			break;
		case CT_FTC:
			SetFTCEnabled(!item.IsActive()); // IsActive means enabled
			break;
		case CT_TUNE_FINE:
		case CT_TUNE_COARSE:
			SetAutoTune(item.IsActive());	// IsActive means not auto
			break;
		default:
			return false;
		}
	}
	catch(invalid_control &e)
	{
		fprintf(stderr, "CRMControl::ChangeControlValue invalid control %d.\n", controlType);
		return false;
	}
	return true;
}

PLUGIN_END_NAMESPACE
