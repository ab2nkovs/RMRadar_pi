#include "br24radar_pi.h"
#include "ControlButton.h"

PLUGIN_BEGIN_NAMESPACE

//// CControlButton
////

wxString CControlButton::m_offOn[2] = { _("Off"), _("On") }; 
wxString CControlButton::m_autoOff[2] = { _("Auto"), _("Disable Auto") };
wxString CControlButton::m_noAuto = _("-----");

CControlButton::CControlButton(br24ControlsDialog *parent, wxWindowID id, const wxString& label, ControlType ct, 
				const wxSize &buttonSize, bool hasAuto, const wxString * autoLabel, const wxString * labels)
	: wxButton(parent, id, label + wxT("\n"), wxDefaultPosition, buttonSize, 0, wxDefaultValidator, label)
	, m_parent(parent)
	, m_controlName(label)
	, m_controlType(ct)
	, m_hasAuto(hasAuto)
	, m_autoLabel(autoLabel)
	, m_valueLabels(labels)
{
}

wxString CControlButton::GetLabel() const
{
	// fprintf(stderr, "GetLabel() %d.\n", m_controlType);

	wxString label;
	if(m_parent->m_ri->m_radarControl != 0)
	{
		const CControlItem & item = m_parent->m_ri->m_radarControl->GetControlValue(m_controlType);
		if(item.IsSet())
		{
			if(m_hasAuto && !item.IsActive())
			{
				label.Printf(wxT("%s\n%s"), m_controlName.c_str(), m_autoLabel[0].c_str());
			}
			else if(m_valueLabels != 0)
			{
				label.Printf(wxT("%s\n%s"), m_controlName.c_str(), m_valueLabels[item.Get()].c_str());
			}
			else
			{
				label.Printf(wxT("%s\n%d"), m_controlName.c_str(), item.Get());
			}
		}
		else
		{
			label.Printf(wxT("%s\n---"), m_controlName.c_str());
		}
	}
	else
	{
		label.Printf(wxT("%s\nXXX"), m_controlName.c_str());
	}
	
	return label;	
}

const wxString & CControlButton::GetAutoLabel() const
{
	if(m_parent->m_ri->m_radarControl != 0)
	{
		const CControlItem & item = m_parent->m_ri->m_radarControl->GetControlValue(m_controlType);
		if(item.IsSet())
		{
			if(m_hasAuto)
			{
				if(item.IsActive())
				{
					return m_autoLabel[0];
				}
				else
				{
					return m_autoLabel[1];
				}
			}
		}
	}
	return m_noAuto;
}

//// CControlButtonSea
////

CControlButtonSea::CControlButtonSea(br24ControlsDialog* parent, wxWindowID id, const wxString& label, ControlType ct, 
			const wxSize &buttonSize, ControlType autoCT, const wxString * labels)
	: CControlButton(parent, id, label, ct, buttonSize, true, m_autoOff, labels)
	, m_autoType(autoCT)
{
}

wxString CControlButtonSea::GetLabel() const
{
	// fprintf(stderr, "Sea::GetLabel() %d.\n", m_controlType);

	wxString label;
	if(m_parent->m_ri->m_radarControl != 0)
	{
		const CControlItem & item = m_parent->m_ri->m_radarControl->GetControlValue(m_controlType);
		if(item.IsSet())
		{
			if(m_hasAuto && !item.IsActive())
			{
				const CControlItem & autoItem = m_parent->m_ri->m_radarControl->GetControlValue(m_autoType);
				if(autoItem.IsSet())
				{
					label.Printf(wxT("%s\n%s"), m_controlName.c_str(), m_valueLabels[autoItem.Get()].c_str());
				}
				else
				{
					label.Printf(wxT("%s\n+++"), m_controlName.c_str());
				}
			}
			else
			{
				label.Printf(wxT("%s\n%d"), m_controlName.c_str(), item.Get());
			}
		}
		else
		{
			label.Printf(wxT("%s\n---"), m_controlName.c_str());
		}
	}
	else
	{
		label.Printf(wxT("%s\nXXX"), m_controlName.c_str());
	}
	
	return label;	
}

ControlType CControlButtonSea::GetControlType() const
{
	if(m_parent->m_ri->m_radarControl != 0)
	{
		const CControlItem & item = m_parent->m_ri->m_radarControl->GetControlValue(m_controlType);
		if(item.IsSet() && m_hasAuto && !item.IsActive())
		{
			return m_autoType;
		}
	}
	return CControlButton::GetControlType();
}

PLUGIN_END_NAMESPACE
