#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include "VendorSpeedControllers.h"

using namespace frc;
using namespace rev;
namespace Module
{
	namespace Output
	{

//TODO Move this to WPI_Output.cpp, unless there is a reason to discern motor controllers of the same group
class ControllerVoltageManager //: public OutputComponent
{
    //Formally known as Motor, changed name to better describe its purpose
private:
    int m_PDBPort = 0;
    double m_TimeTimedOut = 0;
    double m_LowerAmount = 0;
    double m_RegenRate = 0.01;
    double m_PersonalLowerRate = 0;
    //since we have simulation, we no longer need this at this level
    //Note: We probably should keep all SmartDashboard calls in one place, back in 2014 there was a catastrophic bug in SmartDashboard that caused robots
    //to crash, it was intermittent and it injured Jeremy... big teams like 118 had to remove all this code to get back reliability.  Now, while this
    //probably may never happen again, the idea of having UI work being called in a place that needs to be rock solid still hold true, where it should
    //not happen here.
    //std::shared_ptr<NetworkTable> MotorTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
    static double ABSVal(double val)
    {
        return (val < 0 ? val * -1 : val);
    }
    static int SignVal(double val)
    {
        return (val >= 0 ? 1 : -1);
    }
public:
    // Motor() : OutputComponent(){}
    // Motor(string name) : OutputComponent(name){  }
    //The methods below on formatting have changed to be written on their own lines for 2 reasons:
    // 1.  Ability to trace into code with breakpoints
    // 2.  When collapsed (in visual studio especially) it is easier to read, and see what has been implemented
    void SetPDBChannel(int val) 
    { 
        m_PDBPort = val; 
    }
    int GetPDBChannel() const
    {
        return m_PDBPort;
    }
    void Setm_RegenRate(double Rate) 
    {
        m_RegenRate = Rate;
    }
    void SetLowerRate(double Rate) 
    {
        m_PersonalLowerRate = Rate;
    }
    void SetTimeOut(double Time, double m_LowerAmount) 
    {
        m_TimeTimedOut = Time; 
        m_LowerAmount += (m_PersonalLowerRate == 0 ? m_LowerAmount : m_PersonalLowerRate);
    }
    double CalculateVal(double val)
    {
        double ReturnVal = val;
        if (val != 0)
        {
            if (m_TimeTimedOut > 0)
            {
                m_TimeTimedOut -= 0.1;
            }
            else
            {
                if (m_LowerAmount > 0)
                    m_LowerAmount -= m_RegenRate;
                if (m_LowerAmount < 0)
                    m_LowerAmount = 0;
            }

            double NewVal = ABSVal(val) - ABSVal(m_LowerAmount);
            if (NewVal < 0)
            {
                NewVal = 0;
            }
            ReturnVal = NewVal * SignVal(val);
        }
        else
        {
            m_TimeTimedOut = 0;
            m_LowerAmount = 0;
        }
        //MotorTable->PutNumber(name, ReturnVal);
        //Log::General(name + " : Power->" + to_string(ReturnVal));
        return ReturnVal;
    }
};

class SparkMaxItem
{
private:
	std::shared_ptr<CANSparkMax> m_Max;
	int m_channel;
	bool m_reversed;
	std::string m_Name;
	double m_Offset;
    ControllerVoltageManager m_CVM;  //Avoid inhertance if at all possible!
public:
	SparkMaxItem(int _channel, std::string _name, bool _reversed)
    {
        m_channel = _channel;
        m_reversed = _reversed;
        m_Max = std::make_shared<CANSparkMax>(m_channel, rev::CANSparkMax::MotorType::kBrushless);
        m_Max->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_Name = _name;
        m_Offset = 0;
    }
    ~SparkMaxItem()
    {
        m_Max.reset();
    }
	double GetEncoderValue() const
    {
        return m_Max->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 24).GetPosition() - m_Offset;
    }
	virtual double Get() const
    {
        return m_Max->Get();
    }
	int GetPolarity() const
    {
        return (m_reversed ? -1 : 1);
    }
	void Reset() 
    {
        m_Offset = m_Max->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 24).GetPosition();
    }
	virtual void Set(double val) 
    {
        //reserved... if the methods from CVM are needed
        //val = m_CVM.CalculateVal(val);
        // Log::General(Name+" : "  + to_string(val));
        //Note: The InUse() has been stripped out, because there should never be any reason to run this code
        //in a multi-threaded environment, for a single threaded environment there should never be a reason to have another
        //caller need this class, this design has autonomous and teleop to use the same code path, which makes it possible
        //for hybrid teleop, which is ideal for faster cycles.
        if (val < 0 || val>0)
        {
            if (m_reversed) m_Max->Set(-val);
            else m_Max->Set(val);
        }
    }
	//virtual void Set(DoubleSolenoid::Value value) ;
	virtual void Stop() 
    {
        m_Max->StopMotor();
    }
	std::string GetName() const
    {
        return m_Name;
    }
	//virtual void DefaultSet() ;
	//virtual ~SparkMaxItem();
    //The caller does not need to access this
	// CANSparkMax *AsSparkMax() 
    // { 
    //     return m_Max;
    // }
    void ResetEncoderValue() 
    {
        m_Max->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 24).SetPosition(0);
    }
};

#pragma region _wrapper methods_
SparkMaxController::SparkMaxController(int _channel, std::string _name, bool _reversed)
{
    m_controller = std::make_shared<SparkMaxItem>(_channel, _name, _reversed);
}
SparkMaxController::SparkMaxController(int _channel)
{
    //provide a default name
    std::string _name="SparkMax_";
    _name += _channel;
    m_controller = std::make_shared<SparkMaxItem>(_channel, _name, false);
}
double SparkMaxController::GetEncoderValue() const
{
    return m_controller->GetEncoderValue();
}
double SparkMaxController::Get() const
{
    return m_controller->Get();
}
int SparkMaxController::GetPolarity() const
{
    return m_controller->GetPolarity();
}
void SparkMaxController::Reset()
{
    m_controller->Reset();
}
void SparkMaxController::Set(double val)
{
    m_controller->Set(val);
}
void SparkMaxController::Stop()
{
    m_controller->Stop();
}
std::string SparkMaxController::GetName() const
{
    return m_controller->GetName();
}
void SparkMaxController::ResetEncoderValue()
{
    m_controller->ResetEncoderValue();
}

#pragma endregion

}}
