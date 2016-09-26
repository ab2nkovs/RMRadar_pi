/******************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  Navico BR24 Radar Plugin
 * Author:   David Register
 *           Dave Cowell
 *           Kees Verruijt
 *           Douwe Fokkema
 *           Sean D'Epagnier
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

#ifndef _RM_CONTROL_H_
#define _RM_CONTROL_H_

#include <exception>
#include "pi_common.h"
#include "socketutil.h"
#include "RadarInfo.h"

PLUGIN_BEGIN_NAMESPACE

struct value_not_set : public std::exception {
	const char * what () const throw ()
	{
		return "Value not set";
	}
};

struct invalid_control : public std::exception {
	const char * what () const throw ()
	{
		return "Invalid control";
	}
};
	

class CValue {
	bool m_set;
	int m_value;
    public:
	CValue(int value = 0, bool set = false) : m_value(value), m_set(set) { };
	int Get() const { if(!m_set) throw value_not_set(); return m_value; }
	bool IsSet() const { return m_set; }
	void Set(int value) { m_value = value; m_set = true; }
};

class CControlItem {
	CValue m_value;
	CValue m_min, m_max;
	bool m_active; // True if Auto or Off
    public:
	CControlItem(int value = 0, int min = 0, int max = 0, bool active = true, bool set = false) 
		: m_value(value, set), m_min(min, set), m_max(max, set), m_active(active) { };
	int Get() const { return m_value.Get(); }
	bool IsSet() const { return m_value.IsSet(); }
	bool Check(int value) const { return value <= m_max.Get() && value >= m_min.Get(); }
	void Set(int value) { m_value.Set(value); }
	void SetMin(int min) { m_min.Set(min); }
	void SetMax(int max) { m_max.Set(max); }
	bool IsActive() const { if(!m_value.IsSet()) throw value_not_set(); return m_active; }
	bool SetActive(bool active) { m_active = active; }
	const CValue & Min() const { return m_min; }
	const CValue & Max() const { return m_max; }
};

struct SMiscRadarInfo {
	int m_warmupTime;
	int m_signalStrength;
	int m_magnetronCurrent;
	int m_magnetronHours;
	int m_rotationPeriod;
};

class CRMControl : public wxThread {
	SMiscRadarInfo m_miscInfo;

    public:
	CRMControl(br24radar_pi *pi, RadarInfo *ri);
	~CRMControl(void);

	void *Entry(void);
	void OnExit(void);
	void Shutdown(void) { m_quit = true; }

	sockaddr_in *m_mcast_addr;
	sockaddr_in m_radar_addr;
	sockaddr_in m_radar_mcast;
	bool m_haveRadar;
	SOCKET m_dataSocket;
	bool m_new_ip_addr;

	int m_range_meters;    // Last received range in meters
	bool m_updated_range;  // m_range_meters has changed

	bool SetControlValue(ControlType controlType, int value);

	const CControlItem & GetControlValue(ControlType controlType) const;

	bool ChangeControlValue(ControlType controlType, int change);
	bool ToggleAuto(ControlType controlType);
	
	void EnableTX(bool enabled);
	void TurnOff();
	void WakeupRadar();
	bool SetRangeMeters(int meters);

	const SMiscRadarInfo & GetMiscInfo() const { return m_miscInfo; }


    private:
	void logBinaryData(const wxString &what, const UINT8 *data, int size);

	void ProcessFrame(const UINT8 *data, int len);
	bool ProcessReport(const UINT8 *data, int len);

	void ProcessScanData(const UINT8 *data, int len);
	void ProcessFeedback(const UINT8 *data, int len);
	void ProcessPresetFeedback(const UINT8 *data, int len);
	void ProcessCurveFeedback(const UINT8 *data, int len);

	void SetRange(uint8_t range_idx);
	void SetGain(uint8_t value);
	void SetAutoGain(bool enable);
	void SetTune(uint8_t value);
	void SetAutoTune(bool enable);
	void SetCoarseTune(uint8_t value);
	void SetSTCPreset(uint8_t value);
	void SetFTC(uint8_t value);
	void SetFTCEnabled(bool enable);
	void SetSea(uint8_t value);
	void SetAutoSea(uint8_t value);
	void SetRain(uint8_t value);
	void SetRainEnabled(bool enable);
	void SetDisplayTiming(uint8_t value);
	void SetBearingOffset(int32_t value);
	void SetSeaClutterCurve(uint8_t id);
	void EnableMBS(bool enable);
	bool SetTargetExpansion(uint8_t value);
	bool SetInterferenceRejection(uint8_t value);

	void Send1sKeepalive();
	void Send5sKeepalive();
	void SendInitMessages();

	void EmulateFakeBuffer(void);
	int m_next_spoke;     // emulator next spoke
	int m_next_rotation;  // slowly rotate emulator

	SOCKET PickNextEthernetCard();
	SOCKET GetNewDataSocket();
	// SOCKET GetNewCommandSocket();

	br24radar_pi *m_pi;
//	wxString m_ip;
	RadarInfo *m_ri;  // All transfer of data passes back through this.
	volatile bool m_quit;

	struct ifaddrs *m_interface_array;
	struct ifaddrs *m_interface;

	char m_radar_status;

	time_t m_lastKeepalive1s;
	time_t m_lastKeepalive5s;

	CControlItem m_gain;
	CControlItem m_stc;
	CControlItem m_rain;
	CControlItem m_sea;
	CControlItem m_autoSea;
	CControlItem m_ftc;
	CControlItem m_interferenceRejection;
	CControlItem m_targetBoost;
	CControlItem m_bearingOffset;
	CControlItem m_tuneFine;
	CControlItem m_tuneCoarse;
	CControlItem m_displayTiming;
	CControlItem m_stcCurve;
	CControlItem m_mbsEnabled;
};

PLUGIN_END_NAMESPACE

#endif /* _RM_CONTROL_H_ */
