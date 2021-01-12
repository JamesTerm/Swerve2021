#include <frc/Joystick.h>
#include "wpi_Joystick.h"

namespace Module {
    namespace Input {

class WPI_Joystick
{
private:
    //We shouldn't need anymore than 4, currently wpi supports up to 6
    static constexpr size_t k_NoPorts = 4;
    std::shared_ptr<frc::Joystick> m_stick[k_NoPorts];
    using JoystickInfo = wpi_Joystick::JoystickInfo;
    using JoyState = wpi_Joystick::JoyState;
    JoystickInfo m_JoyInfo[k_NoPorts];
public:
    WPI_Joystick()
    {
        for (size_t i = 0; i < k_NoPorts; i++)
            m_stick[i] = std::make_shared<frc::Joystick>(i);
    }
    size_t GetNoJoysticksFound()
    {
        //for now we assume we have them all, as the dead one's are always zero'd
        return 4;
    }
    const JoystickInfo& GetJoyInfo(size_t nr)
    {
        assert(nr < k_NoPorts);
        //Since this is not called often, we populate it on demand
        const size_t i = nr;
        memset(&m_JoyInfo[i], 0, sizeof(JoystickInfo));
        {
            //populate some of the capabilities that we have access to
            int AxisCount = m_stick[i]->GetAxisCount();
            unsigned long JoyCapFlags = 0;
            switch (AxisCount)
            {
            case 1:
                JoyCapFlags = JoystickInfo::fX_Axis;
                break;
            case 2:
                JoyCapFlags = JoystickInfo::fX_Axis | JoystickInfo::fY_Axis;
                break;
            case 3:
                JoyCapFlags = JoystickInfo::fX_Axis | JoystickInfo::fY_Axis | JoystickInfo::fZ_Axis;
                break;
            case 4:
                JoyCapFlags =
                    JoystickInfo::fX_Axis | JoystickInfo::fY_Axis | JoystickInfo::fZ_Axis |
                    JoystickInfo::fX_Rot;
                break;
            case 5:
                JoyCapFlags =
                    JoystickInfo::fX_Axis | JoystickInfo::fY_Axis | JoystickInfo::fZ_Axis |
                    JoystickInfo::fX_Rot | JoystickInfo::fY_Rot;
                break;
            case 6:
                JoyCapFlags =
                    JoystickInfo::fX_Axis | JoystickInfo::fY_Axis | JoystickInfo::fZ_Axis |
                    JoystickInfo::fX_Rot | JoystickInfo::fY_Rot | JoystickInfo::fZ_Rot;
                break;
            default:
                JoyCapFlags = 0;
            }
            m_JoyInfo[i].JoyCapFlags = JoyCapFlags;
            m_JoyInfo[i].bPresent = m_stick[i]->IsConnected();
            m_JoyInfo[i].nPOVCount = m_stick[i]->GetPOVCount();
            m_JoyInfo[i].nButtonCount=m_stick[i]->GetButtonCount();
        }
        return m_JoyInfo[nr];
    }
    bool read_joystick(size_t nr, JoyState& Info)
    {
        memset(&Info, 0, sizeof(JoyState));
        bool ret = false;
        Info.JoystickNumber = nr;
        if (m_stick[nr]->IsConnected())
        {
            //first read axis
            const size_t axis_count = (size_t)m_stick[nr]->GetAxisCount();
            for (size_t i = 0; i < axis_count; i++)
            {
                Info.Axis.AsArray[i] = m_stick[nr]->GetRawAxis(i);
            }
            const size_t button_count = (size_t)m_stick[nr]->GetButtonCount();
            Info.ButtonBank[0]=0;
            for (size_t i = 0; i < button_count; i++)
            {
                //index begins at 1, displays warning if starting from 0
                Info.ButtonBank[0]|=m_stick[nr]->GetRawButton(i+1)?1<<i:0;
            }
            //TODO POV
            ret = true;
        }
        return ret;
    }
};

#pragma region _wrapper methods_
void wpi_Joystick::Init()
{
    m_joystick_internal = std::make_shared<WPI_Joystick>();
}
size_t wpi_Joystick::GetNoJoysticksFound()
{
    return m_joystick_internal->GetNoJoysticksFound();
}
const wpi_Joystick::JoystickInfo& wpi_Joystick::GetJoyInfo(size_t nr)
{
    return m_joystick_internal->GetJoyInfo(nr);
}
bool wpi_Joystick::read_joystick(size_t nr, JoyState& Info)
{
    return m_joystick_internal->read_joystick(nr, Info);
}

#pragma endregion

    }
}