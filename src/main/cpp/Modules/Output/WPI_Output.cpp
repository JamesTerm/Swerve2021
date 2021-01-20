//Avoid the nasty compiler warning by only including what we need
#include <frc/RobotBase.h>
#include <frc/PWMVictorSPX.h>
#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <hal/SimDevice.h>
#include <wpi/math>
#include "WPI_Output.h"
#include "VendorSpeedControllers.h"
//use properties for encoder reading conversions
#include "../../Properties/RegistryV1.h"

//Keep this disabled... only enable locally for testing
//#include <frc/smartdashboard/SmartDashboard.h>

namespace Module
{
	namespace Output
	{

inline double NormalizeRotation2(double Rotation)
{
    //we should really push a SmartDashboard check-box to disable drive
    assert(fabs(Rotation) < Pi2 * 20); //should be less than 20 turns!  If it's greater something is terribly wrong!
    //const double Pi2 = M_PI * 2.0;
    //Normalize the rotation
    while (Rotation > M_PI)
        Rotation -= Pi2;
    while (Rotation < -M_PI)
        Rotation += Pi2;
    return Rotation;
}

//The implementation here is direct and works with the simulation
class WheelModule_Interface
{
private:
    Robot::SwerveVelocities m_PhysicalOdometry;
    std::function<Robot::SwerveVelocities ()> m_VoltageCallback;
    std::function<Robot::SwerveVelocities ()> m_OurSimCallback;
    class WheelModules
    {
    private:
        enum SectionOrder
        {
            eFrontLeft,
            eFrontRight,
            eRearLeft,
            eRearRight
        };
        //we'll keep the same coding conventions as the example
        class WheelModule
        {
        private:
            std::shared_ptr<SparkMaxController> m_drive_motor=nullptr;            
            std::shared_ptr<TalonSRX_Controller> m_swivel_motor=nullptr;
            std::shared_ptr<frc::Encoder> m_driveEncoder=nullptr;  //Note... need two channels per encoder
            std::shared_ptr<frc::Encoder> m_turningEncoder=nullptr;
            //These are not instantiated in the real robot
            std::shared_ptr<frc::sim::EncoderSim> m_driveEncoder_sim=nullptr;
            std::shared_ptr<frc::sim::EncoderSim> m_turningEncoder_sim=nullptr;
            size_t m_ThisSectionIndex;  //see section order (mostly used for diagnostics)

            #pragma region _EncoderTranslation_
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
                __inline double GetDistancePerPulse_Default() const
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
                virtual double Drive_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse_Default();
                }
                virtual double Swivel_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse_Default();
                }
                virtual double Drive_ReadEncoderToLinearVelocity(double encoderReading,size_t wheelSection) const
                {
                    //Linear Velocity is in meters per second, encoder reading is in radians
                    return encoderReading;
                }
                virtual double Drive_SimLinearVelocityToEncoderWrite(double linearVelocity,size_t wheelSection) const
                {
                    //For simulation only and is the inverse computation of the read
                    return linearVelocity;
                }
                virtual double Swivel_ReadEncoderToPosition(double encoderReading,size_t wheelSection) const
                {
                    //Both encoder reading and position are in radians, position is normalized from -pi to pi
                    //it can work within reason from -2pi to 2pi, but should not exceed this
                    //encoder reading has no normalization, but shouldn't be to far off, it starts with zero and can be subjected to
                    //multiple turns in any direction, typically this shouldn't be more than 5 turns so a while loop on the normalization
                    //is fine but to be safe I'd set a limit and trigger a drive disable if it is exceeded, if this fails driving on
                    //a faulty turned wheel can damage robot, so this would be the correct course of action.
                    //The encoder's radians can have the gear reduction in it, but it may be easier to keep like the example, so that 
                    //encoder can be tested by hand, and apply gear reduction here.
                    return encoderReading;
                }
                virtual double Swivel_SimPositionToEncoderWrite(double position,size_t wheelSection) const
                {
                    return position;
                }
            };
            class EncoderTranslation : public EncoderTranslation_Direct
            {
            protected:
                #define GetPropertyName(x)\
                    using namespace properties::registry_v1;\
                    std::string Name = GetPrefix(wheelSection, IsSwivel);\
                    std::string CommonName = GetCommonPrefix(IsSwivel); \
                    Name += #x, CommonName += #x;

                const char* GetPrefix(size_t index, bool IsSwivel) const
                {
                    using namespace properties::registry_v1;
                    //form our prefix, it will use the naming convention in Vehicle Drive.h
                    const char* const prefix_table[2][4] =
                    {
                        {csz_sFL_,csz_sFR_,csz_sRL_,csz_sRR_},
                        {csz_aFL_,csz_aFR_,csz_aRL_,csz_aRR_}
                    };
                    assert(index < 4);
                    const char* const prefix = prefix_table[IsSwivel ? 1 : 0][index];
                    return prefix;
                }
                const char* GetCommonPrefix(bool IsSwivel) const
                {
                    using namespace properties::registry_v1;
                    const char* prefix = IsSwivel ? csz_CommonSwivel_ : csz_CommonDrive_;
                    return prefix;
                }
                double GetDistancePerPulse(size_t wheelSection, bool IsSwivel) const
                {
                    const double kEncoderResolution = 4096.0 / 4;
                    GetPropertyName(Rotary_EncoderPulsesPerRevolution);
                    //This has the common nested as the default, so basically the individual property will return if present
                    //then the common property, then finally the default constant
                    const double enc_resolution= m_props->get_number(Name.c_str(), m_props->get_number(CommonName.c_str(),kEncoderResolution));
                    return Pi2 / kEncoderResolution;
                }
                double GetGearReduction(size_t wheelSection,bool IsSwivel) const
                {
                    GetPropertyName(Rotary_EncoderToRS_Ratio);
                    //Gear reduction driving over driven (e.g. driven is the bigger gear)
                    return m_props->get_number(Name.c_str(), m_props->get_number(CommonName.c_str(), 1.0));
                }
                double GetWheelRadius() const
                {
                    using namespace properties::registry_v1;
                    return Inches2Meters(m_props->get_number(csz_Drive_WheelDiameter_in,4.0)) * 0.5;
                }
            public:
                bool GetIsReversed(size_t wheelSection, bool IsSwivel) const
                {
                    // returns -1 if reversed
                    GetPropertyName(Rotary_EncoderToRS_Ratio);
                    //Gear reduction driving over driven (e.g. driven is the bigger gear)
                    return m_props->get_bool(Name.c_str(), m_props->get_bool(CommonName.c_str(), false));
                }
                virtual double Drive_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse(wheelSection,false);
                }
                virtual double Swivel_GetDistancePerPulse(size_t wheelSection) const
                {
                    return GetDistancePerPulse(wheelSection, true);
                }
                //TODO: we may want to add Kalman filter and averager for the encoder position, but for now I'll leave it
                //as it probably will not be too noisy given its reduction (unlike a potentiometer)
                virtual double Drive_ReadEncoderToLinearVelocity(double encoderReading,size_t wheelSection) const
                {
                    //Since we can assume radians, we just need to factor in the gear reduction and wheel radius (in meters)
                    return encoderReading * GetGearReduction(wheelSection,false) * GetWheelRadius();
                }
                virtual double Drive_SimLinearVelocityToEncoderWrite(double linearVelocity,size_t wheelSection) const
                {
                    //Factor in the reciprocal radius (1/radius is like divide) and reciprocal gear reduction
                    //Note: Even though compilers are smart these days, to multiply makes it easy to ensure no
                    //division of zero
                    return linearVelocity * (1.0/GetGearReduction(wheelSection, false)) * (1.0/GetWheelRadius());
                }
                virtual double Swivel_ReadEncoderToPosition(double encoderReading,size_t wheelSection) const
                {
                    //Factor in the gear reduction and normalize, for now I'll just assert() the limit
                    return  NormalizeRotation2(encoderReading * GetGearReduction(wheelSection,true));
                }
                virtual double Swivel_SimPositionToEncoderWrite(double position,size_t wheelSection) const
                {
                    //Factor inverse gear reduction, we are normalized so we needn't worry about this
                    return position * (1.0 / GetGearReduction(wheelSection, true));
                }
            };
            #pragma endregion

            EncoderTranslation m_Converter;
            #pragma region _Simulation Variables_
            //keep as pointers to assign during init, because of this we can have both simulation and actual controllers
            //same for encoders, this way we can somewhat test the actual controllers in simulation, and for the actual
            //robot they just remain as null pointers that are never used.

            //This is a fall back as I'll try to use SparkMax in simulation
            std::shared_ptr<frc::PWMVictorSPX> m_sim_drive_motor=nullptr;
            std::shared_ptr<frc::PWMVictorSPX> m_sim_swivel_motor=nullptr;
            std::shared_ptr<frc::sim::SimDeviceSim> m_SparkMaxSimDevice;
            //Testing, for simulation this can bypass the swivel encoders
            double m_TestSwivelPos=0.0;
            #pragma endregion

            bool UseFallbackSim() const
            {
                //override to test actual controllers in the simulation (will need to be open loop, unless we can get vendors to work properly)
                //ultimately, the vendors code should simulate properly, I will keep checking for updates.
                #if 0
                    return frc::RobotBase::IsSimulation();
                #else
                    return false;
                #endif
            }
        public:
            void Init(size_t index,const Framework::Base::asset_manager *props=nullptr)
            {
                m_ThisSectionIndex=index;
                m_Converter.Init(props); //This must happen before getting the distance per pulse below
                using namespace frc;
                //Note here we can use the asset manager to switch motor assignments
                //the index is always the section order, but the motor to use can
                //be the property that section order represents
                if (UseFallbackSim())
                {
                    m_sim_drive_motor=std::make_shared<PWMVictorSPX>(index);
                    m_sim_swivel_motor=std::make_shared<PWMVictorSPX>(index+4);
                }
                else
                {
                    m_drive_motor=std::make_shared<SparkMaxController>(index);
                    m_swivel_motor=std::make_shared<TalonSRX_Controller>(index+4,true);
                }

                if (UseFallbackSim())
                {
                    m_driveEncoder=std::make_shared<Encoder>(index*2,index*2+1);
                    m_turningEncoder=std::make_shared<Encoder>((index+4)*2,(index+4)*2+1);
                }

                //Grab our distance per pulse
                if (UseFallbackSim())
                {
                    m_driveEncoder->SetDistancePerPulse(m_Converter.Drive_GetDistancePerPulse(m_ThisSectionIndex));
                    m_turningEncoder->SetDistancePerPulse(m_Converter.Swivel_GetDistancePerPulse(m_ThisSectionIndex));
                }
                else
                {
                    m_drive_motor->SetDistancePerPulse(m_Converter.Drive_GetDistancePerPulse(m_ThisSectionIndex));
                    m_swivel_motor->SetDistancePerPulse(m_Converter.Swivel_GetDistancePerPulse(m_ThisSectionIndex));
                }
                //Grab if we need to reverse direction (broken up to step through code)
                //TODO provide reverse direction for SparkMax and TalonSRX
                if (UseFallbackSim())
                {
                    bool IsReversed=m_Converter.GetIsReversed(m_ThisSectionIndex,false);
                    m_driveEncoder->SetReverseDirection(IsReversed);
                    IsReversed=m_Converter.GetIsReversed(m_ThisSectionIndex,true);
                    m_turningEncoder->SetReverseDirection(IsReversed);
                }

                //Only instantiate if we are in a simulation
                if (RobotBase::IsSimulation())
                {
                    if (UseFallbackSim())
                    {
                        m_driveEncoder_sim=std::make_shared<sim::EncoderSim>(*m_driveEncoder);
                        m_turningEncoder_sim=std::make_shared<sim::EncoderSim>(*m_turningEncoder);
                    }
                    std::string deviceKey = "SPARK MAX [" ;
                    char buffer[4];
                    itoa(m_ThisSectionIndex,buffer,10);
                    deviceKey += buffer;
                    deviceKey += "]";
                    m_SparkMaxSimDevice = std::make_shared<sim::SimDeviceSim>(deviceKey.c_str());
                }
            }
            void TimeSlice(double dTime_s, double drive_voltage, double swivel_voltage, Robot::SwerveVelocities &physicalOdometry)
            {
                if (UseFallbackSim())
                {
                    m_sim_drive_motor->Set(drive_voltage);
                    m_sim_swivel_motor->Set(swivel_voltage);
                }
                else
                {
                    m_drive_motor->Set(drive_voltage);
                    if (frc::RobotBase::IsSimulation())
                    {
                        hal::SimDouble outputProp = m_SparkMaxSimDevice->GetDouble("Applied Output");
                        outputProp.Set(drive_voltage);
                    }
                    m_swivel_motor->Set(swivel_voltage);
                }
                
                //now to update our odometry
                if (UseFallbackSim())
                {
                    physicalOdometry.Velocity.AsArray[m_ThisSectionIndex]=
                        m_Converter.Drive_ReadEncoderToLinearVelocity(m_driveEncoder->GetRate(),m_ThisSectionIndex);
                    physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=
                        m_Converter.Swivel_ReadEncoderToPosition(m_turningEncoder->GetDistance(),m_ThisSectionIndex);
                }
                else
                {
                    physicalOdometry.Velocity.AsArray[m_ThisSectionIndex]=
                        m_Converter.Drive_ReadEncoderToLinearVelocity(m_drive_motor->GetEncoderVelocity(),m_ThisSectionIndex);

                    // if (m_ThisSectionIndex==0)
                    // {
                    //     const double encoderPos_raw=m_swivel_motor->GetEncoderPosition();
                    //     const double encoderPos=m_Converter.Swivel_ReadEncoderToPosition(m_swivel_motor->GetEncoderPosition(),m_ThisSectionIndex);
                    //     int x=9;
                    //     printf("-r-%.2f--",encoderPos_raw);
                    // }
                    //TODO There is a serious update issue with Talon's method to get position, so far it is not yet known if this is a simulation
                    //update problem or a real motor update problem, for now we'll use a fall-back bypass, but I'll need to know if the actual motor
                    //can update at least every 10ms.  If not then I'll need to consider averaging either the position or rotary's encoder velocity
                    if (frc::RobotBase::IsReal())
                    {
                        physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=
                            m_Converter.Swivel_ReadEncoderToPosition(m_swivel_motor->GetEncoderPosition(),m_ThisSectionIndex);
                    }
                    else
                        physicalOdometry.Velocity.AsArray[m_ThisSectionIndex+4]=m_TestSwivelPos;
                }
            }
            void SimulatorTimeSlice(double dTime_s, double drive_velocity, double swivel_distance) 
            {
                //Note:  this slice does not start until init is complete (solved higher level)
                double driveRate = m_Converter.Drive_SimLinearVelocityToEncoderWrite(drive_velocity, m_ThisSectionIndex);
                double swivelPos=m_Converter.Swivel_SimPositionToEncoderWrite(swivel_distance, m_ThisSectionIndex);
                if (UseFallbackSim())
                {
                    m_driveEncoder_sim->SetRate(driveRate);
                    m_turningEncoder_sim->SetDistance(swivelPos);
                }
                else
                {
                    m_SparkMaxSimDevice->GetDouble("Velocity").Set(driveRate);
                    m_swivel_motor->sim_SetQuadratureRawPosition(swivelPos);
                    m_TestSwivelPos=swivel_distance;
                   //frc::SmartDashboard::PutNumber("Test",m_drive_motor->GetEncoderVelocity());
                    #if 0
                    if (m_ThisSectionIndex==0)
                    {
                        printf("-s-%.2f--",swivelPos);
                        frc::SmartDashboard::PutNumber("Test",
                        m_Converter.Swivel_ReadEncoderToPosition(m_swivel_motor->GetEncoderPosition(),m_ThisSectionIndex));
                        frc::SmartDashboard::PutNumber("Test2",swivel_distance);
                    }
                    #endif
                }
            }
        };

        WheelModule Module[4];
        WheelModule_Interface *m_pParent;
        bool m_StartSimulation = false;
    public:
        WheelModules(WheelModule_Interface *parent) : m_pParent(parent)
        {}
        void Init(const Framework::Base::asset_manager *props=nullptr)
        {
            for (size_t i=0;i<4;i++)
                Module[i].Init(i,props);
            m_StartSimulation = true;
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
            if (!m_StartSimulation) return;
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