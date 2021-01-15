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

//The implementation here is direct and works with the simulation
class WheelModule_Interface
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
            std::shared_ptr<frc::Encoder> m_driveEncoder;  //Note... need two channels per encoder
            std::shared_ptr<frc::Encoder> m_turningEncoder;
            //These are not instantiated in the real robot
            std::shared_ptr<frc::sim::EncoderSim> m_driveEncoder_sim=nullptr;
            std::shared_ptr<frc::sim::EncoderSim> m_turningEncoder_sim=nullptr;
            size_t m_ThisSectionIndex;  //see section order (mostly used for diagnostics)

            class EncoderTranslation_Direct
            {
                #pragma region _Description_
                //This class helps separate the conversion necessary between the encoder readings we get and
                //the readings we need for the odometry, this class sets up the environment directly so that
                //the simulation can use it as a base line, then we derive from it with the actual conversions
                //so both cases can be tested
                #pragma endregion
            protected:
                const Framework::Base::asset_manager *m_props;
                __inline double GetDistancePerPulse_Default()
                {
                    //keeping this property free for this class
                    //Distance per pulse has to be set properly for real encoder to give radians, the encoder resolution
                    //represents how many pulses per full turn, or in angles it depends on how the vendor publishes it
                    //WPI's example assumes it is for a full turn where 2 pi converts to radians
                    const double kEncoderResolution = 4096.0;
                    return Pi2 / kEncoderResolution;
                }
            public:
                void Init(const Framework::Base::asset_manager *props)
                {
                    m_props=props;
                }
                virtual double Drive_GetDistancePerPulse(size_t wheelSection)
                {
                    return GetDistancePerPulse_Default();
                }
                virtual double Swivel_GetDistancePerPulse(size_t wheelSection)
                {
                    return GetDistancePerPulse_Default();
                }
                virtual double Drive_ReadEncoderToLinearVelocity(double encoderReading,size_t wheelSection)
                {
                    //Linear Velocity is in meters per second, encoder reading is in radians
                    return encoderReading;
                }
                virtual double Drive_SimLinearVelocityToEncoderWrite(double linearVelocity,size_t wheelSection)
                {
                    //For simulation only and is the inverse computation of the read
                    return linearVelocity;
                }
                virtual double Swivel_ReadEncoderToPosition(double encoderReading,size_t wheelSection)
                {
                    //Both encoder reading and position are in radians, position is normalized from -pi to pi
                    //it can work within reason from -2pi to 2pi, but should not exceed this
                    //encoder reading has no normalization, but shouldn't be to far off, it starts with zero and can be subjected to
                    //multiple turns in any direction, typically this shouldn't be more than 5 turns so a while loop on the normalizaion
                    //is fine but to be safe I'd set a limit and trigger a drive disable if it is exceeded, if this fails driving on
                    //a faulty turned wheel can damage robot, so this would be the correct course of action.
                    //The encoder's radians can have the gear reduction in it, but it may be easier to keep like the example, so that 
                    //encoder can be tested by hand, and apply gear reduction here.
                    return encoderReading;
                }
                virtual double Swivel_SimPositionToEncoderWrite(double position,size_t wheelSection)
                {
                    return position;
                }
            };
            class EncoderTranslation : public EncoderTranslation_Direct
            {
            public:
                virtual double Drive_GetDistancePerPulse(size_t wheelSection)
                {
                    //TODO
                    return GetDistancePerPulse_Default();
                }
                virtual double Swivel_GetDistancePerPulse(size_t wheelSection)
                {
                    //TODO
                    return GetDistancePerPulse_Default();
                }

                virtual double Drive_ReadEncoderToLinearVelocity(double encoderReading,size_t wheelSection)
                {
                    //Since we can assume radians, we just need to factor in the gear reduction and wheel radius (in meters)
                    //TODO
                    return encoderReading;
                }
                virtual double Drive_SimLinearVelocityToEncoderWrite(double linearVelocity,size_t wheelSection)
                {
                    //Factor in the reciprocal radius (1/radius is like divide) and reciprocal gear reduction
                    //Note: Even though compilers are smart these days, to multiply makes it easy to ensure no
                    //division of zero
                    //TODO
                    return linearVelocity;
                }
                virtual double Swivel_ReadEncoderToPosition(double encoderReading,size_t wheelSection)
                {
                    //Factor in the gear reduction and normalize, for now I'll just assert() the limit
                    //but we should really push a smartdashboard checkbox to disable drive
                    //TODO
                    return encoderReading;
                }
                virtual double Swivel_SimPositionToEncoderWrite(double position,size_t wheelSection)
                {
                    //Factor inverse gear reduction, we are normalized so we needn't worry about this
                    //TODO
                    return position;
                }
            };

            EncoderTranslation m_Converter;
            void Init(size_t index,const Framework::Base::asset_manager *props=nullptr)
            {
                m_ThisSectionIndex=index;
                m_Converter.Init(props); //This must happen before getting the distance per pulse below
                using namespace frc;
                //Note here we can use the asset manager to switch motor assignments
                //the index is always the section order, but the motor to use can
                //be the property that section order represents
                m_drive_motor=std::make_shared<PWMVictorSPX>(index);
                m_swivel_motor=std::make_shared<PWMVictorSPX>(index+4);
                m_driveEncoder=std::make_shared<Encoder>(index*2,index*2+1);
                m_turningEncoder=std::make_shared<Encoder>((index+4)*2,(index+4)*2+1);

                #pragma region _WPI example_
                //Note: This is the example these are pulled from properties
                //const double kWheelRadius= 0.0508;  //2 inches
                //const int kEncoderResolution = 4096;

                //Example code---
                // Set the distance per pulse for the drive encoder. We can simply use the
                // distance traveled for one rotation of the wheel divided by the encoder
                // resolution.
                //m_driveEncoder->SetDistancePerPulse(2 * wpi::math::pi * kWheelRadius / kEncoderResolution);
                //---------------------------------------------------------------------------
                //Let's not factor in the wheel radius here because we should not be dependant on robot
                //properties at this level, aside from this the example didn't factor in the gear reduction
                //We can factor both later down, just a simple unit of radians will suffice

                // Set the distance (in this case, angle) per pulse for the turning encoder.
                // This is the angle through an entire rotation (2 * wpi::math::pi)
                // divided by the encoder resolution.
                //m_turningEncoder->SetDistancePerPulse(2 * wpi::math::pi / kEncoderResolution);
                #pragma endregion

                m_driveEncoder->SetDistancePerPulse(m_Converter.Drive_GetDistancePerPulse(m_ThisSectionIndex));
                m_turningEncoder->SetDistancePerPulse(m_Converter.Swivel_GetDistancePerPulse(m_ThisSectionIndex));

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
                physicalOdometry.Velocity.AsArray[m_ThisSectionIndex]=
                    m_Converter.Drive_ReadEncoderToLinearVelocity(m_driveEncoder->GetRate(),m_ThisSectionIndex);
                physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=
                    m_Converter.Swivel_ReadEncoderToPosition(m_turningEncoder->GetDistance(),m_ThisSectionIndex);
            }
            void SimulatorTimeSlice(double dTime_s, double drive_velocity, double swivel_distance) 
            {
                //This will have late binding, so we check for null
                if (m_driveEncoder_sim)
                {
                  m_driveEncoder_sim->SetRate(m_Converter.Drive_SimLinearVelocityToEncoderWrite(drive_velocity,m_ThisSectionIndex));
                  //assert turning sim is set
                  m_turningEncoder_sim->SetDistance(m_Converter.Swivel_SimPositionToEncoderWrite(swivel_distance,m_ThisSectionIndex));
                }
            }
        };

        WheelModule Module[4];
        WheelModule_Interface *m_pParent;
        WheelModules(WheelModule_Interface *parent) : m_pParent(parent)
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
    WheelModule_Interface m_Implementation;
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