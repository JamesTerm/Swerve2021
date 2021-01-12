
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

//1. Just joystick
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

//2. kinematics and joystick
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
		//using JoystickInfo = wpi_Joystick::JoystickInfo;
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

//3. Add simple form of motion control (does not handle change of direction or centrepetal forces)
#include "Modules/Robot/Entity2D/Entity2D.h"
#include "Modules/Robot/MotionControl2D_simple/MotionControl2D.h"
#pragma region _Test_Swerve_Entity_Joystick Tele V1_

class Test_Swerve_Entity_Joystick
{
private:
	#pragma region _member variables_
	Module::Localization::Entity2D m_Entity;
	Module::Robot::Simple::MotionControl2D m_MotionControl2D;
	bool m_Done = false;
	Module::Input::wpi_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Robot::Swerve_Drive m_robot;
	Module::Robot::Inv_Swerve_Drive m_Entity_Input;

	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
	#pragma endregion

	void UpdateVariables()
	{
        using namespace frc;
		Module::Localization::Entity2D &entity = m_Entity;
		using namespace Module::Localization;
		Entity2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
		Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
		double magnitude = velocity_normalized.normalize();
		//Entity variables-------------------------------------------
		SmartDashboard::PutNumber("Velocity", Meters2Feet(magnitude));
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y", Meters2Feet(position.y));
		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
		SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		//SmartDashboard::PutNumber("setpoint_angle", RAD_2_DEG(entity.Get_IntendedOrientation()));
		//kinematic variables-------------------------------------------
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

	void GetInputSlice()
	{
		using namespace Module::Input;
		//using JoystickInfo = wpi_Joystick::JoystickInfo;
		size_t JoyNum = 0;

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			//printf("Button: 2=exit, x axis=strafe, y axis=forward/reverse, z axis turn \n");
			wpi_Joystick::JoyState joyinfo;
			memset(&joyinfo, 0, sizeof(wpi_Joystick::JoyState));
			if (m_joystick.read_joystick(JoyNum, joyinfo))
			{
				//Get an input from the controllers to feed in... we'll hard code the x and y axis
				m_robot.UpdateVelocities(Feet2Meters(m_maxspeed*joyinfo.Axis.Named.lY*-1.0), Feet2Meters(m_maxspeed*joyinfo.Axis.Named.lX), joyinfo.Axis.Named.lZ * m_max_heading_rad);
				//because of properties to factor we need to interpret the actual velocities resolved from the kinematics by inverse kinematics
				m_Entity_Input.InterpolateVelocities(m_robot.GetIntendedVelocities());

				//Here is how is we can use or not use motion control, some interface... the motion control is already linked to entity
				//so this makes it easy to use

				#if 0
				//Now we can update the entity with this inverse kinematic input
				m_Entity.SetAngularVelocity(m_Entity_Input.GetAngularVelocity());
				m_Entity.SetLinearVelocity_local(m_Entity_Input.GetLocalVelocityY(), m_Entity_Input.GetLocalVelocityX());
				#else
				//Now we can update the entity with this inverse kinematic input
				m_MotionControl2D.SetAngularVelocity(m_Entity_Input.GetAngularVelocity());
				m_MotionControl2D.SetLinearVelocity_local(m_Entity_Input.GetLocalVelocityY(), m_Entity_Input.GetLocalVelocityX());
				#endif

				//This comes in handy for testing
				if (joyinfo.ButtonBank[0] == 1)
					Reset();
			}
		}
	}

public:
	void TimeSlice(double dTime_s)
	{
        //Grab kinematic velocities from controller
        GetInputSlice();
        const double time_delta = 0.010;
        //Update the predicted motion for this time slice
        m_MotionControl2D.TimeSlice(time_delta);
        m_Entity.TimeSlice(time_delta);
        UpdateVariables();
	}


	void Reset()
	{
		m_Entity.Reset();
	}

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
		//redundant but reserved to be different
		Inv_Swerve_Drive::properties inv_props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		m_MotionControl2D.Initialize();
		m_Entity_Input.SetProperties(inv_props);
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
		//Note: We'll skip properties for motion control since we have good defaults
		#pragma region _optional linking of entity to motion control_
		//Now to link up the callbacks for motion control:  Note we can link them up even if we are not using it
		using Vector2D = Module::Robot::Simple::MotionControl2D::Vector2D;
		m_MotionControl2D.Set_UpdateGlobalVelocity([&](const Vector2D &new_velocity)
			{	m_Entity.SetLinearVelocity_global(new_velocity.y, new_velocity.x);
			});
		m_MotionControl2D.Set_UpdateHeadingVelocity([&](double new_velocity)
		{	m_Entity.SetAngularVelocity(new_velocity);
			});
		m_MotionControl2D.Set_GetCurrentPosition([&]() -> Vector2D
		{	
			//This is a bit annoying, but for the sake of keeping modules independent (not dependent on vector objects)
			//its worth the hassle
			Vector2D ret = { m_Entity.GetCurrentPosition().x, m_Entity.GetCurrentPosition().y };
			return ret;
		});
		m_MotionControl2D.Set_GetCurrentHeading([&]() -> double
		{
			return m_Entity.GetCurrentHeading();
		});

		#pragma endregion
		Reset();  //for entity variables
	}
};
#pragma endregion

//4. Full physics motion profiling, last version assembly before using swerve robot assembly
#include "Modules/Input/JoystickConverter.h"
#include "Modules/Robot/MotionControl2D_physics/MotionControl2D.h"
#pragma region _Test_Swerve_Viewer Tele V2_

class Test_Swerve_Viewer
{
private:
	#pragma region _member variables_
	Module::Localization::Entity2D m_Entity;
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	#ifdef __UseSimpleMotionControl__
	Module::Robot::Simple::MotionControl2D m_MotionControl2D;
	#else
	Module::Robot::Physics::MotionControl2D m_MotionControl2D;
	#endif
	Module::Input::wpi_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Input::Analog_EventEntry m_joystick_options;  //for now a simple one-stop option for all
	Module::Robot::Swerve_Drive m_robot;  //keep track of our intended velocities
	Module::Robot::Swerve_Drive m_robot_display; //For our display we'll interpret the velocities computed 
	Module::Robot::Inv_Swerve_Drive m_Entity_Input;

	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
	double m_dTime_s=0.016;  //cached so other callbacks can access
	#pragma endregion

	void UpdateVariables()
	{
        using namespace frc;
		Module::Localization::Entity2D &entity = m_Entity;
		using namespace Module::Localization;
		const Entity2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
		Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
		double magnitude = velocity_normalized.normalize();
		//Entity variables-------------------------------------------
		SmartDashboard::PutNumber("Velocity", Meters2Feet(magnitude));
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y", Meters2Feet(position.y));
		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
		SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
		//This is temporary and handy for now, will change once we get AI started
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		//To make this interesting, we keep the SmartDashboard to show the intended velocities...
		//SmartDashboard::PutNumber("setpoint_angle", RAD_2_DEG(entity.Get_IntendedOrientation()));
		//kinematic variables-------------------------------------------
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
		//but for our current state of the UI, use the actual velocities from the entity 
		//(which will be different as it accounts for mechanical resolve of momentum)
		Vec2D global_velocity= LocalToGlobal(entity.GetCurrentHeading(), Vec2D(linear_velocity.y,linear_velocity.x));
		m_robot_display.UpdateVelocities(global_velocity[0], global_velocity[1],entity.GetCurrentAngularVelocity());
	}

	void GetInputSlice()
	{
		using namespace Module::Input;
		//using JoystickInfo = wpi_Joystick::JoystickInfo;
		size_t JoyNum = 0;

        wpi_Joystick::JoyState joyinfo;
		//wpi_Joystick::JoyState joyinfo = {0}; //setup joy zero'd out
        memset(&joyinfo, 0, sizeof(wpi_Joystick::JoyState));

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		if (NoJoySticks)
		{
			//printf("Button: 2=exit, x axis=strafe, y axis=forward/reverse, z axis turn \n");
			m_joystick.read_joystick(JoyNum, joyinfo);
		}
		{
			//Get an input from the controllers to feed in... we'll hard code the x and y axis from both joy and keyboard
			//we simply combine them so they can work inter-changeably (e.g. keyboard for strafing, joy for turning)
			m_robot.UpdateVelocities(
				Feet2Meters(m_maxspeed*(AnalogConversion(joyinfo.Axis.Named.lY, m_joystick_options) )*-1.0),
				Feet2Meters(m_maxspeed*(AnalogConversion(joyinfo.Axis.Named.lX, m_joystick_options) )),
				(AnalogConversion(joyinfo.Axis.Named.lZ, m_joystick_options) ) * m_max_heading_rad);
			//because of properties to factor we need to interpret the actual velocities resolved from the kinematics by inverse kinematics
			m_Entity_Input.InterpolateVelocities(m_robot.GetIntendedVelocities());

			//Here is how is we can use or not use motion control, some interface... the motion control is already linked to entity
			//so this makes it easy to use

			#if 0
			//Now we can update the entity with this inverse kinematic input
			m_Entity.SetAngularVelocity(m_Entity_Input.GetAngularVelocity());
			m_Entity.SetLinearVelocity_local(m_Entity_Input.GetLocalVelocityY(), m_Entity_Input.GetLocalVelocityX());
			#else
			//Now we can update the entity with this inverse kinematic input
			m_MotionControl2D.SetAngularVelocity(m_Entity_Input.GetAngularVelocity());
			m_MotionControl2D.SetLinearVelocity_local(m_Entity_Input.GetLocalVelocityY(), m_Entity_Input.GetLocalVelocityX());
			#endif

			//This comes in handy for testing
			if (joyinfo.ButtonBank[0] == 1)
				Reset();
		}
		
	}

public:

	void TimeSlice(double dTime_s)
	{
		m_dTime_s = dTime_s;
		//Grab kinematic velocities from controller
		GetInputSlice();
		//Update the predicted motion for this time slice
		m_MotionControl2D.TimeSlice(dTime_s);
		m_Entity.TimeSlice(dTime_s);
		UpdateVariables();
	}

	void Reset()
	{
		m_Entity.Reset();
		m_MotionControl2D.Reset();
	}

	void Init()
	{
		m_joystick.Init();
		m_joystick_options = { 
			0.3,   //filter dead zone
			1.0,   //additional scale
			1.0,   // curve intensity
			false  //is flipped
			};
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
		m_robot_display.SetProperties(props);
		//redundant but reserved to be different
		Inv_Swerve_Drive::properties inv_props =
		{
		wheel_dimensions.length(),	//distance from diagonal for 4WD
		wheel_dimensions[0], //Length between wheels
		wheel_dimensions[1] //Width between wheels
		};
		m_MotionControl2D.Initialize();
		m_Entity_Input.SetProperties(inv_props);
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;
		//Note: We'll skip properties for motion control since we have good defaults
		#pragma region _optional linking of entity to motion control_
		//Now to link up the callbacks for motion control:  Note we can link them up even if we are not using it
		#ifdef __UseSimpleMotionControl__
		using Vector2D = Module::Robot::Simple::MotionControl2D::Vector2D;
		#else
		using Vector2D = Module::Robot::Physics::MotionControl2D::Vector2D;
		#endif
		m_MotionControl2D.Set_UpdateGlobalVelocity([&](const Vector2D &new_velocity)
			{	m_Entity.SetLinearVelocity_global(new_velocity.y, new_velocity.x);
			});
		m_MotionControl2D.Set_UpdateHeadingVelocity([&](double new_velocity)
		{	m_Entity.SetAngularVelocity(new_velocity);
			});
		m_MotionControl2D.Set_GetCurrentPosition([&]() -> Vector2D
		{	
			//This is a bit annoying, but for the sake of keeping modules independent (not dependent on vector objects)
			//its worth the hassle
			Vector2D ret = { m_Entity.GetCurrentPosition().x, m_Entity.GetCurrentPosition().y };
			return ret;
		});
		m_MotionControl2D.Set_GetCurrentHeading([&]() -> double
		{
			return m_Entity.GetCurrentHeading();
		});

		#pragma endregion
		Reset();  //for entity variables
	}
};
#pragma endregion

//We just pick what we test or use from here m_teleop declaration
class TeleOp_Internal
{
private:
//TestJoystick m_teleop;
//Test03_Swerve_Kinematics_with_Joystick m_teleop;
//Test_Swerve_Entity_Joystick m_teleop;
Test_Swerve_Viewer m_teleop;
double m_LastTime=0.0;

__inline double GetTime() 
{
    //TODO get the time from WPI so we can use the step effectively
		using std::chrono::duration;
		using std::chrono::duration_cast;
		using std::chrono::system_clock;

		return duration_cast<duration<double>>(system_clock::now().time_since_epoch())
			.count();
}
public:
  void Init()
  {
    m_teleop.Init();
  }
  void TimeSlice()
  {
        const double CurrentTime = GetTime();
        #if 1
        const double DeltaTime = CurrentTime - m_LastTime;
        #else
        const double DeltaTime=0.01;  //It's best to use sythetic time for simulation to step through code
        #endif
        m_LastTime = CurrentTime;

        m_teleop.TimeSlice(DeltaTime);
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
    m_tester->TimeSlice();
}

#pragma endregion