//Avoid the nasty compiler warning by only including what we need
#include <frc/RobotBase.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>
#include <wpi/math>
#include "WPI_Output.h"
//reserved
//#include "../../Properties/RegistryV1.h"

namespace Module
{
	namespace Output
	{

//Straight example to show how it works
class SimplePWM_Example
{
private:
    Robot::SwerveVelocities m_PhysicalOdometry;
    std::function<Robot::SwerveVelocities ()> m_VoltageCallback;
    std::function<Robot::SwerveVelocities ()> m_OurSimCallback;
    struct WheelModules
    {
        enum SectionOrder
        {
            eFrontLeft,
            eFrontRight,
            eRearLeft,
            eRearRight
        };
        //we'll keep the same coding conventions as the example
        struct WheelModule
        {
            //keep as pointers to assign during init
            std::shared_ptr<frc::PWMVictorSPX> m_drive_motor;
            std::shared_ptr<frc::PWMVictorSPX> m_swivel_motor;
            std::shared_ptr<frc::Encoder> m_driveEncoder;  //Note... need two channnels per encoder
            std::shared_ptr<frc::Encoder> m_turningEncoder;
            //These are not instanciated in the real robot
            std::shared_ptr<frc::sim::EncoderSim> m_driveEncoder_sim=nullptr;
            std::shared_ptr<frc::sim::EncoderSim> m_turningEncoder_sim=nullptr;
            size_t m_ThisSectionIndex;  //see section order (mostly used for diagnostics)
            void Init(size_t index,const Framework::Base::asset_manager *props=nullptr)
            {
                m_ThisSectionIndex=index;
                using namespace frc;
                //Note here we can use the asset manager to switch motor assignments
                //the index is always the section order, but the motor to use can
                //be the property that section order represents
                m_drive_motor=std::make_shared<PWMVictorSPX>(index);
                m_swivel_motor=std::make_shared<PWMVictorSPX>(index+4);
                m_driveEncoder=std::make_shared<Encoder>(index*2,index*2+1);
                m_turningEncoder=std::make_shared<Encoder>((index+4)*2,(index+4)*2+1);

                //TODO pull these from properties these are defaults from demo
                const double kWheelRadius= 0.0508;  //2 inches
                const int kEncoderResolution = 4096;

                // Set the distance per pulse for the drive encoder. We can simply use the
                // distance traveled for one rotation of the wheel divided by the encoder
                // resolution.
                m_driveEncoder->SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);

                // Set the distance (in this case, angle) per pulse for the turning encoder.
                // This is the the angle through an entire rotation (2 * wpi::math::pi)
                // divided by the encoder resolution.
                m_turningEncoder->SetDistancePerPulse(2 * wpi::math::pi / kEncoderResolution);
                //Only instantiate if we are in a simulation
                if (RobotBase::IsSimulation())
                {
                    m_driveEncoder_sim=std::make_shared<sim::EncoderSim>(*m_driveEncoder);
                    m_turningEncoder_sim=std::make_shared<sim::EncoderSim>(*m_turningEncoder);
                }
            }
            void TimeSlice(double dTime_s, double drive_voltage, double swivel_voltage, Robot::SwerveVelocities &physicalOdometry)
            {
                m_drive_motor->Set(drive_voltage);
                m_swivel_motor->Set(swivel_voltage);
                //now to update our odometry
                //for now this is an exact read, but will need to be translated from a real encoder
                //If the WPI simulation solves this we can simulate that as well; otherwise we can
                //add the conversions ourself.
                physicalOdometry.Velocity.AsArray[m_ThisSectionIndex]=m_driveEncoder->GetRate();
                physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=m_turningEncoder->GetDistance();
            }
            void SimulatorTimeSlice(double dTime_s, double drive_velocity, double swivel_distance) 
            {
                //This will have late binding, so we check for null
                if (m_driveEncoder_sim)
                {
                  m_driveEncoder_sim->SetRate(drive_velocity);
                  //assert turning sim is set
                  m_turningEncoder_sim->SetDistance(swivel_distance);
                }
            }
        };

        WheelModule Module[4];
        SimplePWM_Example *m_pParent;
        WheelModules(SimplePWM_Example *parent) : m_pParent(parent)
        {}
        void Init(const Framework::Base::asset_manager *props=nullptr)
        {
            for (size_t i=0;i<4;i++)
                Module[i].Init(i,props);
        }
        void TimeSlice(double dTime_s)
        {
            //call each module with updated voltages
            for (size_t i=0;i<4;i++)
                Module[i].TimeSlice(dTime_s,
                m_pParent->m_VoltageCallback().Velocity.AsArray[i],
                m_pParent->m_VoltageCallback().Velocity.AsArray[i+4],
                m_pParent->m_PhysicalOdometry);
        }
        void SimulatorTimeSlice(double dTime_s) 
        {
            for (size_t i=0;i<4;i++)
                Module[i].SimulatorTimeSlice(dTime_s,
                m_pParent->m_OurSimCallback().Velocity.AsArray[i],
                m_pParent->m_OurSimCallback().Velocity.AsArray[i+4]);
        }
    };
    WheelModules m_WheelModule=this;
public:
    void Init(const Framework::Base::asset_manager *props=nullptr)
    {
        m_WheelModule.Init(props);
    }
    void TimeSlice(double dTime_s)
    {
        m_WheelModule.TimeSlice(dTime_s);
    }
    void SimulatorTimeSlice(double dTime_s) 
    {
        m_WheelModule.SimulatorTimeSlice(dTime_s);
    }
    void SetSimOdometry(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_OurSimCallback=callback;
    }
	void SetVoltageCallback(std::function<Robot::SwerveVelocities ()> callback)
    {
        m_VoltageCallback=callback;
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