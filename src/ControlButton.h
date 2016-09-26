#ifndef __CCONTROL_BUTTON_H__
#define __CCONTROL_BUTTON_H__

#include <map>
#include "br24radar_pi.h"

PLUGIN_BEGIN_NAMESPACE

class CControlButton : public wxButton {
    protected:
	static wxString m_noAuto;
	br24ControlsDialog *m_parent;
	ControlType m_controlType;
	wxString m_controlName;
	const wxString *m_autoLabel;
	bool m_hasAuto;
	const wxString *m_valueLabels;
    public:
	static wxString m_offOn[2];
	static wxString m_autoOff[2];
	CControlButton(br24ControlsDialog* parent, wxWindowID id, const wxString& label, ControlType ct, const wxSize &buttonSize,
			bool hasAuto = false, const wxString * autoLabel = m_autoOff, const wxString * labels = 0);
	wxString GetLabel() const;
	virtual bool HasAuto() const { return m_hasAuto; }
	virtual ControlType GetControlType() const { return m_controlType; }
	virtual const wxString & GetAutoLabel() const;
};

class CControlButtonSea : public CControlButton {
	ControlType m_autoType;
    public:
	CControlButtonSea(br24ControlsDialog* parent, wxWindowID id, const wxString& label, ControlType ct, const wxSize &buttonSize, ControlType autoCT, const wxString * labels);
	wxString GetLabel() const;
	ControlType GetControlType() const;
};

typedef std::map<ControlType, CControlButton &> CButtonMap;

struct SButtonSeq {
	wxWindowID id;
	wxString label;
	ControlType ctrl;
	bool hasAuto;
	const wxString * autoLabel;
	const wxString * labels;
};

PLUGIN_END_NAMESPACE

#endif

