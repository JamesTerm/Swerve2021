#include <ctre/Phoenix.h>
#include "VendorSpeedControllers.h"

using namespace frc;

namespace Module
{
	namespace Output
	{

class TalonSRXItem
{
private:
    //just like with SparkMax, we can't clean up... I suspect there may be something internal doing this
	WPI_TalonSRX *m_talon=nullptr;
    //std::shared_ptr<WPI_TalonSRX> m_talon;
	int m_channel;
	bool m_reversed;
	bool m_encoderEnabled;

public:
	TalonSRXItem(int _channel, std::string _name, bool _reversed, bool enableEncoder)
	{
	m_channel = _channel;
	m_reversed = _reversed;
	m_talon = new WPI_TalonSRX(m_channel);
    //m_talon = std::make_shared<WPI_TalonSRX>(m_channel);
	m_encoderEnabled = enableEncoder;
	if(enableEncoder)
		m_talon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	}
	int GetQuadraturePosition()
	{
        	return m_encoderEnabled ? m_talon->GetSensorCollection().GetQuadraturePosition() : -1;
	}
	void SetQuadraturePosition(int val)
	{
        m_talon->GetSensorCollection().SetQuadraturePosition(val, 0);
	}
	double Get() const
	{
        if (m_reversed)
            return m_talon->GetMotorOutputPercent() * -1;
        return m_talon->GetMotorOutputPercent();
	}
	void Set(double val)
	{
        //val = CalculateVal(val);
        //Log::General(name+" : "  + to_string(val));
        if (val<0 || val>0)
        {
            if(m_reversed) 
                m_talon->Set(ControlMode::PercentOutput, -val);
            else 
                m_talon->Set(ControlMode::PercentOutput, val);
        }
	}
	void Stop()
	{
        TalonSRXItem::Set(0);
	}
};

#pragma region _Wrapper methods_
TalonSRX_Controller::TalonSRX_Controller(int channel, std::string name, bool reversed, bool enableEncoder)
{
	m_controller = std::make_shared<TalonSRXItem>(channel, name, reversed,enableEncoder);
}
TalonSRX_Controller::TalonSRX_Controller(int _channel, bool enableEncoder)
{
	//provide a default name
	std::string _name = "TalonSRX_";
	_name += _channel;
	m_controller = std::make_shared<TalonSRXItem>(_channel, _name, false, enableEncoder);
}

void TalonSRX_Controller::SetDistancePerPulse(double distancePerPulse)
{

}
double TalonSRX_Controller::GetEncoderPosition() const
{
	return m_controller->GetQuadraturePosition();
}
double TalonSRX_Controller::GetEncoderVelocity() const
{
    return 0.0;  //TODO
}

double TalonSRX_Controller::Get() const
{
	return m_controller->Get();
}
void TalonSRX_Controller::Set(double val)
{
	m_controller->Set(val);
}
void TalonSRX_Controller::Stop()
{
	m_controller->Stop();
}

#pragma endregion
}}