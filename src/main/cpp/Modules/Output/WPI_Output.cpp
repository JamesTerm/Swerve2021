#include <frc/WPILib.h>
#include "WPI_Output.h"

namespace Module
{
	namespace Output
	{

//Straight example to show how it works
class SimplePWM_Example
{
private:
    Robot::SwerveVelocities m_PhysicalOdometry;
public:
    void Init(const Framework::Base::asset_manager *props=nullptr)
    {

    }
    void TimeSlice(double dTime_s)
    {

    }
    void SimulatorTimeSlice(double dTime_s) 
    {

    }
    void SetSimOdometry(std::function<Robot::SwerveVelocities ()> callback)
    {

    }
	void SetVoltageCallback(std::function<Robot::SwerveVelocities ()> callback)
    {

    }
	const Robot::SwerveVelocities &GetCurrentVelocities() const
    {
        return m_PhysicalOdometry;
    }
};

class WPI_Output_Internal
{
private:
    SimplePWM_Example m_Implementation;
public:
    void Init(const Framework::Base::asset_manager *props=nullptr)
    {
        m_Implementation.Init(props);
    }
    void TimeSlice(double dTime_s)
    {
        m_Implementation.TimeSlice(dTime_s);
    }
    void SimulatorTimeSlice(double dTime_s) 
    {
        m_Implementation.SimulatorTimeSlice(dTime_s);
    }
    void SetSimOdometry(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_Implementation.SetSimOdometry(callback);
    }
	void SetVoltageCallback(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_Implementation.SetVoltageCallback(callback);
    }
	const Robot::SwerveVelocities &GetCurrentVelocities() const
    {
        return m_Implementation.GetCurrentVelocities();
    }
};

#pragma region _Wrapper methods_
WPI_Output::WPI_Output()
{
    m_WPI=std::make_shared<WPI_Output_Internal>();
}
void WPI_Output::Init(const Framework::Base::asset_manager *props)
{
    m_WPI->Init(props);
}
void WPI_Output::TimeSlice(double dTime_s)
{
    m_WPI->TimeSlice(dTime_s);
}
void WPI_Output::SimulatorTimeSlice(double dTime_s) 
{
    m_WPI->SimulatorTimeSlice(dTime_s);
}
void WPI_Output::SetSimOdometry(std::function<Robot::SwerveVelocities ()> callback)
{
    m_WPI->SetSimOdometry(callback);
}
void WPI_Output::SetVoltageCallback(std::function<Robot::SwerveVelocities ()> callback)
{
    m_WPI->SetVoltageCallback(callback);
}
const Robot::SwerveVelocities &WPI_Output::GetCurrentVelocities() const
{
    return m_WPI->GetCurrentVelocities();
}
#pragma endregion

    }
}