
#include <frc/smartdashboard/SmartDashboard.h>
#include "Base/Vec2d.h"
#include "Base/Misc.h"
#include "RobotAssembly.h"
#include "Modules/Input/wpi_Joystick_Controller/wpi_Joystick.h"

#pragma region _description_
//So in here I'll keep various assmeblies to use, these are in order of how they add on and get more developed
//the same is true for the include files as they are mostly independent and can be added in the order of 
//development.  From a birds eye view, we start with just teleop and just the joystick, and then integrate into 
//kinematics the output starts with the SmartDashboard, and some viewer for it on simulations, and finally
//gets routed to WPI itself where the voltage gets routed out, and the encoder and other sensors get read into
//the odometry.  Use the regions to help view each section of interest.
#pragma endregion

#pragma region _Test Joystick_
class TestJoystick
{
private:
    Module::Input::wpi_Joystick m_input;
public:
    void Init()
    {
        m_input.Init();
    }
    void TimeSlice(double dTime_s)
    {
        using namespace::Module::Input;
        wpi_Joystick::JoyState joystate;
        m_input.read_joystick(0,joystate);
        frc::SmartDashboard::PutNumber("Axis0",joystate.Axis.AsArray[0]);
    }
};
#pragma endregion

#include "Modules/Robot/DriveKinematics/Vehicle_Drive.h"
#pragma region _Test03_Swerve_Kinematics_with_Joystick_

class Test03_Swerve_Kinematics_with_Joystick
{
private:
	Module::Input::wpi_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Robot::Swerve_Drive m_robot;
	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
public:
	void Init()
	{
		m_joystick.Init();

		using namespace Module::Robot;
		//setup some robot properties
		using properties = Swerve_Drive::properties;
		//We'll make a square robot for ease to interpret angles
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		properties props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		//Set the properties... this should be a one-time setup operation
		m_robot.SetProperties(props);

		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;  
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
	}
	void TimeSlice(double dTime_s)
	{
		using namespace Module::Input;
		using JoystickInfo = wpi_Joystick::JoystickInfo;
        using namespace frc;
   		const size_t JoyNum = 0;
        wpi_Joystick::JoyState joyinfo;

        if (m_joystick.read_joystick(JoyNum, joyinfo))
        {
            //In this test we will not have magnitude clipping, but I may change that later
            //Get an input from the controllers to feed in... we'll hard code the x and y axis
            m_robot.UpdateVelocities(Feet2Meters(m_maxspeed*joyinfo.Axis.Named.lY), Feet2Meters(m_maxspeed*joyinfo.Axis.Named.lX), joyinfo.Axis.Named.lZ * m_max_heading_rad);

            //I've added SmartLayout_Swerve1.xml in the design folder to test
            //Use smart dashboard to see progress bar representation (gets a better idea of the clipping)
            //Roll the joystick around each direction when doing this... to confirm it's correct
            //set progress bar to 12 to -12 on the range in its properties
            SmartDashboard::PutNumber("Wheel_fl_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(0)));
            SmartDashboard::PutNumber("Wheel_fr_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(1)));
            SmartDashboard::PutNumber("Wheel_rl_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(2)));
            SmartDashboard::PutNumber("Wheel_rr_Velocity", Meters2Feet(m_robot.GetIntendedVelocitiesFromIndex(3)));
            //For the angles either show raw or use simple dial using 180 to -180 with a 45 tick interval
            //its not perfect, but it gives a good enough direction to tell (especially when going down)
            SmartDashboard::PutNumber("swivel_fl_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(0)));
            SmartDashboard::PutNumber("swivel_fr_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(1)));
            SmartDashboard::PutNumber("swivel_rl_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(2)));
            SmartDashboard::PutNumber("swivel_rr_Raw", RAD_2_DEG(m_robot.GetSwerveVelocitiesFromIndex(3)));
        }
	}
};
#pragma endregion

//We just pick what we test or use from here m_teleop declaration
class TeleOp_Internal
{
private:
//TestJoystick m_teleop;
Test03_Swerve_Kinematics_with_Joystick m_teleop;
public:
  void Init()
  {
    m_teleop.Init();
  }
  void TimeSlice(double dTime_s)
  {
      m_teleop.TimeSlice(dTime_s);
  }
};

#pragma region _wrapper_methods_
TeleOp::TeleOp()
{
    m_tester=std::make_shared<TeleOp_Internal>();
}
void TeleOp::Init()
{
    m_tester->Init();
}
void TeleOp::TimeSlice()
{
    //TODO put our time delta here
    m_tester->TimeSlice(0.010);
}

#pragma endregion