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
    double m_DPP=1.0;
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
    void SetDistancePerPulse(double distancePerPulse)
    {
        m_DPP=distancePerPulse;
    }
	double GetQuadraturePosition()
	{
        //TODO until I test on a real motor I will leave the method as it was
        //Since talon doesn't have DPP methods, we manage it at this level
       	return m_encoderEnabled ? ((double)m_talon->GetSensorCollection().GetQuadraturePosition())*m_DPP : -1.0;
        //From https://www.chiefdelphi.com/t/ctre-encoder-cant-zero-reset-encoder-values/162269/8
        //found that in simulation this works more reliable than GetQuadraturePosition() in that it updates more frequently
        //This may be true for real robot too, unless the issue is the talon's ability to get updates from simulation
        //return   m_talon->GetSelectedSensorPosition()*m_DPP;
	}
	void SetQuadraturePosition(int val)
	{
        m_talon->GetSensorCollection().SetQuadraturePosition(val, 0);
	}
   	void sim_SetQuadratureRawPosition(double new_pos)
    {
        //we factor our own inverse DPP
        m_talon->GetSimCollection().SetQuadratureRawPosition((int)(new_pos*(1.0/m_DPP)));    
    }
    void sim_SetQuadratureVelocity(double newRate_s)
    {
        //in native units per 100ms
        m_talon->GetSimCollection().SetQuadratureVelocity((int)(newRate_s*10.0));
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
    m_controller->SetDistancePerPulse(distancePerPulse);
}
double TalonSRX_Controller::GetEncoderPosition() const
{
	return m_controller->GetQuadraturePosition();
}
double TalonSRX_Controller::GetEncoderVelocity() const
{
    return 0.0;  //TODO
}
void TalonSRX_Controller::sim_SetQuadratureRawPosition(double new_pos)
{
    m_controller->sim_SetQuadratureRawPosition(new_pos);
}
void TalonSRX_Controller::sim_SetQuadratureVelocity(double newRate_s)
{
    m_controller->sim_SetQuadratureVelocity(newRate_s);
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