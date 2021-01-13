
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "Base/Vec2d.h"
#include "Base/Misc.h"
#include "RobotAssembly.h"

#pragma region _description_
//So in here I'll keep various assmeblies to use, these are in order of how they add on and get more developed
//the same is true for the include files as they are mostly independent and can be added in the order of 
//development.  From a birds eye view, we start with just teleop and just the joystick, and then integrate into 
//kinematics the output starts with the SmartDashboard, and some viewer for it on simulations, and finally
//gets routed to WPI itself where the voltage gets routed out, and the encoder and other sensors get read into
//the odometry.  Use the regions to help view each section of interest.
#pragma endregion

#pragma region _01 Test Joystick_
//1. Just joystick
#include "Modules/Input/wpi_Joystick_Controller/wpi_Joystick.h"
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
#pragma region _02 Test03_Swerve_Kinematics_with_Joystick_
//2. kinematics and joystick
#include "Modules/Robot/DriveKinematics/Vehicle_Drive.h"

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
#pragma region _03 Test_Swerve_Entity_Joystick Tele V1_
//3. Add simple form of motion control (does not handle change of direction or centrepetal forces)
#include "Modules/Robot/Entity2D/Entity2D.h"
#include "Modules/Robot/MotionControl2D_simple/MotionControl2D.h"

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
		SmartDashboard::PutNumber("X_ft", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y_ft", Meters2Feet(position.y));
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
#pragma region _04 Test_Swerve_Viewer Tele V2_
//4. Full physics motion profiling, last version assembly before using swerve robot assembly
#include "Modules/Input/JoystickConverter.h"
#include "Modules/Robot/MotionControl2D_physics/MotionControl2D.h"

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
		SmartDashboard::PutNumber("X_ft", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y_ft", Meters2Feet(position.y));
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
#pragma region _05 Test_Swerve_Viewer Tele V3_
//This is the last of the stand-alone tele versions, which introduces the super assembly of swerve robot
//so a lot of the hooking is done inside of it, keeping the minimal hooking here... so when switching over
//from simulation to actual motors can be resolved from within that assembly
//This assembly and onward start to show the use-case of writing hooks through lambda operators, ask me if you need
//help reading this part of the code.

#include "Modules/Robot/SwerveRobot/SwerveRobot.h"

class Test_Swerve_Rotary
{
private:
	#pragma region _member variables_
	Module::Localization::Entity2D m_Entity;
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	Module::Input::wpi_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Input::Analog_EventEntry m_joystick_options;  //for now a simple one-stop option for all
	Module::Robot::SwerveRobot m_robot;  //keep track of our intended velocities

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
		SmartDashboard::PutNumber("Rotation Velocity", m_Entity.GetCurrentAngularVelocity());
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X_ft", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y_ft", Meters2Feet(position.y));
		//This is temporary and handy for now, will change once we get AI started
		//Like with the kinematics if we are not moving we do not update the intended orientation (using this)
		//This is just cosmetic, but may be handy to keep for teleop
		if (!IsZero(linear_velocity.x + linear_velocity.y, 0.02))
        {
			//m_current_state.bits.IntendedOrientation = atan2(velocity_normalized[0], velocity_normalized[1]);
       		//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
    		SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
        }
		else if (!IsZero(entity.GetCurrentAngularVelocity()))
		{
			//point forward locally when rotating in place
			//m_current_state.bits.IntendedOrientation = entity.GetCurrentHeading(); 
            SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		}
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		//To make this interesting, we keep the SmartDashboard to show the intended velocities...
		//SmartDashboard::PutNumber("setpoint_angle", RAD_2_DEG(entity.Get_IntendedOrientation()));
		//kinematic variables-------------------------------------------
		const Module::Robot::SwerveVelocities &cv = m_robot.GetCurrentVelocities();
		const Module::Robot::SwerveVelocities &iv = m_robot.GetIntendedVelocities();
		const Module::Robot::SwerveVelocities &vo = m_robot.GetCurrentVoltages();

		SmartDashboard::PutNumber("Wheel_fl_Velocity", Meters2Feet(iv.Velocity.AsArray[0]));
		SmartDashboard::PutNumber("Wheel_fr_Velocity", Meters2Feet(iv.Velocity.AsArray[1]));
		SmartDashboard::PutNumber("Wheel_rl_Velocity", Meters2Feet(iv.Velocity.AsArray[2]));
		SmartDashboard::PutNumber("Wheel_rr_Velocity", Meters2Feet(iv.Velocity.AsArray[3]));
		SmartDashboard::PutNumber("Wheel_fl_Voltage", vo.Velocity.AsArray[0]);
		SmartDashboard::PutNumber("Wheel_fr_Voltage", vo.Velocity.AsArray[1]);
		SmartDashboard::PutNumber("Wheel_rl_Voltage", vo.Velocity.AsArray[2]);
		SmartDashboard::PutNumber("Wheel_rr_Voltage", vo.Velocity.AsArray[3]);
		SmartDashboard::PutNumber("wheel_fl_Encoder", Meters2Feet(cv.Velocity.AsArray[0]));
		SmartDashboard::PutNumber("wheel_fr_Encoder", Meters2Feet(cv.Velocity.AsArray[1]));
		SmartDashboard::PutNumber("wheel_rl_Encoder", Meters2Feet(cv.Velocity.AsArray[2]));
		SmartDashboard::PutNumber("wheel_rr_Encoder", Meters2Feet(cv.Velocity.AsArray[3]));

		//For the angles either show raw or use simple dial using 180 to -180 with a 45 tick interval
		//its not perfect, but it gives a good enough direction to tell (especially when going down)
		SmartDashboard::PutNumber("Swivel_fl_Voltage", vo.Velocity.AsArray[4]);
		SmartDashboard::PutNumber("Swivel_fr_Voltage", vo.Velocity.AsArray[5]);
		SmartDashboard::PutNumber("Swivel_rl_Voltage", vo.Velocity.AsArray[6]);
		SmartDashboard::PutNumber("Swivel_rr_Voltage", vo.Velocity.AsArray[7]);
		SmartDashboard::PutNumber("swivel_fl_Raw", RAD_2_DEG(cv.Velocity.AsArray[4]));
		SmartDashboard::PutNumber("swivel_fr_Raw", RAD_2_DEG(cv.Velocity.AsArray[5]));
		SmartDashboard::PutNumber("swivel_rl_Raw", RAD_2_DEG(cv.Velocity.AsArray[6]));
		SmartDashboard::PutNumber("swivel_rr_Raw", RAD_2_DEG(cv.Velocity.AsArray[7]));
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
			m_robot.SetAngularVelocity((AnalogConversion(joyinfo.Axis.Named.lZ, m_joystick_options) ) * m_max_heading_rad);
			m_robot.SetLinearVelocity_local(
				Feet2Meters(m_maxspeed*(AnalogConversion(joyinfo.Axis.Named.lY, m_joystick_options) )*-1.0),
				Feet2Meters(m_maxspeed*(AnalogConversion(joyinfo.Axis.Named.lX, m_joystick_options) )));

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
		m_robot.TimeSlice(dTime_s);
		m_Entity.TimeSlice(dTime_s);
		UpdateVariables();
	}

	void SetUpHooks(bool enable)
	{
		if (enable)
		{
			//Now to link up the callbacks for the robot motion control:  Note we can link them up even if we are not using it
			m_robot.Set_UpdateGlobalVelocity([&](const Vec2D &new_velocity)
			{	m_Entity.SetLinearVelocity_global(new_velocity.y(), new_velocity.x());
			});
			m_robot.Set_UpdateHeadingVelocity([&](double new_velocity)
			{	m_Entity.SetAngularVelocity(new_velocity);
			});
			m_robot.Set_GetCurrentPosition([&]() -> Vec2D
			{
				//This is a bit annoying, but for the sake of keeping modules independent (not dependent on vector objects)
				//its worth the hassle
				Vec2D ret = Vec2D(m_Entity.GetCurrentPosition().x, m_Entity.GetCurrentPosition().y);
				return ret;
			});
			m_robot.Set_GetCurrentHeading([&]() -> double
			{
				return m_Entity.GetCurrentHeading();
			});
		}
		else
		{
			m_robot.Set_UpdateGlobalVelocity(nullptr);
			m_robot.Set_UpdateHeadingVelocity(nullptr);
			m_robot.Set_GetCurrentPosition(nullptr);
			m_robot.Set_GetCurrentHeading(nullptr);
		}
	}
public:
	Test_Swerve_Rotary()
	{
		SetUpHooks(true);
	}
	void Reset()
	{
		m_Entity.Reset();
		m_robot.Reset();
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
		//We'll make a square robot for ease to interpret angles
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;

		//Initialize these before calling reset (as the properties can dictate what to reset to)
		//TODO provide properties method here and invoke it, init can have properties sent as a parameter here
		m_robot.Init();

		Reset();  //for entity variables
	}
   	~Test_Swerve_Rotary()
	{
		m_robot.Shutdown(); //detach hooks early
		SetUpHooks(false);
	}
};
#pragma endregion
#pragma region _06 Test_Swerve_Viewer TeleAuton V1_
//This adds autonomous, they use the same slice
#include "Modules/Input/AI_Input/AI_Input_Example.h"
#include "Modules/Input/AI_Input/SmartDashboard_HelperFunctions.h"

class Test_Swerve_TeleAuton
{
private:
	#pragma region _member variables_
	Module::Localization::Entity2D m_Entity;
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	Module::Input::wpi_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
	Module::Input::Analog_EventEntry m_joystick_options;  //for now a simple one-stop option for all
	Module::Input::AI_Example m_Goal;
	Module::Robot::SwerveRobot m_robot;  //keep track of our intended velocities

	//These typically are constants, but if we have gear shifting they can change
	double m_maxspeed; //max velocity forward in feet per second
	double m_max_heading_rad;  //max angular velocity in radians
	double m_dTime_s=0.016;  //cached so other callbacks can access

	enum class game_mode
	{
		eAuton,
		eTele,
		eTest
	} m_game_mode= game_mode::eTele;

	bool m_IsStreaming = false;
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
		SmartDashboard::PutNumber("Rotation Velocity", m_Entity.GetCurrentAngularVelocity());
		Entity2D::Vector2D position = entity.GetCurrentPosition();
		SmartDashboard::PutNumber("X_ft", Meters2Feet(position.x));
		SmartDashboard::PutNumber("Y_ft", Meters2Feet(position.y));
		//If it is angle acceleration is being setpoint driven we read this (e.g. AI controlled)
		if (m_robot.GetIsDrivenAngular())
		{
			//m_current_state.bits.IntendedOrientation = m_robot.Get_IntendedOrientation();
			SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(m_robot.Get_IntendedOrientation()));
		}
		else
		{
			//TeleOp controlled, point forward if we are rotating in place, otherwise, point to the direction of travel
			//Like with the kinematics if we are not moving we do not update the intended orientation (using this)
			//This is just cosmetic, but may be handy to keep for teleop
			if (!IsZero(linear_velocity.x + linear_velocity.y, 0.02))
			{
				//m_current_state.bits.IntendedOrientation = atan2(velocity_normalized[0], velocity_normalized[1]);
				//for swerve the direction of travel is not necessarily the heading, so we show this as well as heading
				SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(atan2(velocity_normalized[0], velocity_normalized[1])));
			}
			else if (!IsZero(entity.GetCurrentAngularVelocity()))
			{
				//point forward locally when rotating in place
				//m_current_state.bits.IntendedOrientation = entity.GetCurrentHeading();
				SmartDashboard::PutNumber("Travel_Heading", RAD_2_DEG(entity.GetCurrentHeading()));
			}
		}
		SmartDashboard::PutNumber("Heading", RAD_2_DEG(entity.GetCurrentHeading()));
		//To make this interesting, we keep the SmartDashboard to show the intended velocities...
		//SmartDashboard::PutNumber("setpoint_angle", RAD_2_DEG(entity.Get_IntendedOrientation()));
		//kinematic variables-------------------------------------------
		const Module::Robot::SwerveVelocities &cv = m_robot.GetCurrentVelocities();
		const Module::Robot::SwerveVelocities &iv = m_robot.GetIntendedVelocities();
		const Module::Robot::SwerveVelocities &vo = m_robot.GetCurrentVoltages();

		SmartDashboard::PutNumber("Wheel_fl_Velocity", Meters2Feet(iv.Velocity.AsArray[0]));
		SmartDashboard::PutNumber("Wheel_fr_Velocity", Meters2Feet(iv.Velocity.AsArray[1]));
		SmartDashboard::PutNumber("Wheel_rl_Velocity", Meters2Feet(iv.Velocity.AsArray[2]));
		SmartDashboard::PutNumber("Wheel_rr_Velocity", Meters2Feet(iv.Velocity.AsArray[3]));
		SmartDashboard::PutNumber("Wheel_fl_Voltage", vo.Velocity.AsArray[0]);
		SmartDashboard::PutNumber("Wheel_fr_Voltage", vo.Velocity.AsArray[1]);
		SmartDashboard::PutNumber("Wheel_rl_Voltage", vo.Velocity.AsArray[2]);
		SmartDashboard::PutNumber("Wheel_rr_Voltage", vo.Velocity.AsArray[3]);
		SmartDashboard::PutNumber("wheel_fl_Encoder", Meters2Feet(cv.Velocity.AsArray[0]));
		SmartDashboard::PutNumber("wheel_fr_Encoder", Meters2Feet(cv.Velocity.AsArray[1]));
		SmartDashboard::PutNumber("wheel_rl_Encoder", Meters2Feet(cv.Velocity.AsArray[2]));
		SmartDashboard::PutNumber("wheel_rr_Encoder", Meters2Feet(cv.Velocity.AsArray[3]));

		//For the angles either show raw or use simple dial using 180 to -180 with a 45 tick interval
		//its not perfect, but it gives a good enough direction to tell (especially when going down)
		SmartDashboard::PutNumber("Swivel_fl_Voltage", vo.Velocity.AsArray[4]);
		SmartDashboard::PutNumber("Swivel_fr_Voltage", vo.Velocity.AsArray[5]);
		SmartDashboard::PutNumber("Swivel_rl_Voltage", vo.Velocity.AsArray[6]);
		SmartDashboard::PutNumber("Swivel_rr_Voltage", vo.Velocity.AsArray[7]);
		SmartDashboard::PutNumber("swivel_fl_Raw", RAD_2_DEG(cv.Velocity.AsArray[4]));
		SmartDashboard::PutNumber("swivel_fr_Raw", RAD_2_DEG(cv.Velocity.AsArray[5]));
		SmartDashboard::PutNumber("swivel_rl_Raw", RAD_2_DEG(cv.Velocity.AsArray[6]));
		SmartDashboard::PutNumber("swivel_rr_Raw", RAD_2_DEG(cv.Velocity.AsArray[7]));
	}
	void GetInputSlice(double dTime_s)
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
		if (m_game_mode==game_mode::eTele)
		{
			//Check if we are being driven by some AI method, we override it if we have any angular velocity (i.e. some teleop interaction)
			const double AngularVelocity = (AnalogConversion(joyinfo.Axis.Named.lZ, m_joystick_options) ) * m_max_heading_rad;
			if (!IsZero(AngularVelocity) || (m_robot.GetIsDrivenAngular()==false))
				m_robot.SetAngularVelocity(AngularVelocity);

			//Get an input from the controllers to feed in... we'll hard code the x and y axis from both joy and keyboard
			//we simply combine them so they can work inter-changeably (e.g. keyboard for strafing, joy for turning)
			const double Forward = Feet2Meters(m_maxspeed * (AnalogConversion(joyinfo.Axis.Named.lY, m_joystick_options) ) * -1.0);
			const double Right = Feet2Meters(m_maxspeed * (AnalogConversion(joyinfo.Axis.Named.lX, m_joystick_options) ));

			//Check if we are being driven by some AI method, we override it if we have any linear velocity (i.e. some teleop interaction)
			//Note: this logic does not quite work right for keyboard if it uses the forward "sticky" button, but since this isn't a real
			//input I am not going to filter it out, but could if needed.  Also the fabs() ensures the forward and right do not cancel
			//each other out
			if (!IsZero(fabs(Forward) + fabs(Right)) || (m_robot.GetIsDrivenLinear() == false))
				m_robot.SetLinearVelocity_local(Forward, Right);
		}
		else if (m_game_mode == game_mode::eAuton)
		{
			using namespace Framework::Base;
			Goal& goal = m_Goal.GetGoal();
			if (goal.GetStatus()==Goal::eActive)
				goal.Process(dTime_s);
		}
		//TODO autonomous and goals here
		//This comes in handy for testing, but could be good to stop robot if autonomous needs to stop
		if (joyinfo.ButtonBank[0] == 1)
			Reset();
	}
	void SetUpHooks(bool enable)
	{
		if (enable)
		{
			#pragma region _Robot Entity Linking_
			//Now to link up the callbacks for the robot motion control:  Note we can link them up even if we are not using it
			m_robot.Set_UpdateGlobalVelocity([&](const Vec2D &new_velocity)
			{	m_Entity.SetLinearVelocity_global(new_velocity.y(), new_velocity.x());
			});
			m_robot.Set_UpdateHeadingVelocity([&](double new_velocity)
			{	m_Entity.SetAngularVelocity(new_velocity);
			});
			m_robot.Set_GetCurrentPosition([&]() -> Vec2D
			{
				//This is a bit annoying, but for the sake of keeping modules independent (not dependent on vector objects)
				//its worth the hassle
				Vec2D ret = Vec2D(m_Entity.GetCurrentPosition().x, m_Entity.GetCurrentPosition().y);
				return ret;
			});
			m_robot.Set_GetCurrentHeading([&]() -> double
			{
				return m_Entity.GetCurrentHeading();
			});
			#pragma endregion
			#pragma region _AI Input Robot linking_
			m_Goal.Set_GetCurrentPosition([&]() -> Vec2D
				{
					return m_robot.GetCurrentPosition();
				});
			m_Goal.Set_GetCurrentHeading([&]() -> double
				{
					return m_robot.GetCurrentHeading();
				});
			m_Goal.Set_DriveToLocation([&](double north, double east, bool absolute, bool stop_at_destination, double max_speed, bool can_strafe)
				{
					m_robot.DriveToLocation(north, east, absolute, stop_at_destination, max_speed, can_strafe);
				});
			m_Goal.Set_SetIntendedOrientation([&](double intended_orientation, bool absolute)
				{
					m_robot.SetIntendedOrientation(intended_orientation, absolute);
				});
			#pragma endregion
		}
		else
		{
			m_robot.Set_UpdateGlobalVelocity(nullptr);
			m_robot.Set_UpdateHeadingVelocity(nullptr);
			m_robot.Set_GetCurrentPosition(nullptr);
			m_robot.Set_GetCurrentHeading(nullptr);
		}
	}
public:
	Test_Swerve_TeleAuton()
	{
		SetUpHooks(true);
	}
	void Reset()
	{
		m_Entity.Reset();
		m_robot.Reset();
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
		//We'll make a square robot for ease to interpret angles
		Vec2D wheel_dimensions(Inches2Meters(24), Inches2Meters(24));
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / wheel_dimensions.length()) * skid;

		//Initialize these before calling reset (as the properties can dictate what to reset to)
		//TODO provide properties method here and invoke it, init can have properties sent as a parameter here
		m_robot.Init();

		Reset();  //for entity variables
	}
	void Start()
	{
		if (!m_IsStreaming)
		{
			m_IsStreaming = true;
			//Give driver station a default testing method by invoking here if we are in test mode
			if (m_game_mode == game_mode::eTest)
			{
				int test_to_run = (int)Auton_Smart_GetSingleValue("AutonTest", 1.0);
				test(test_to_run);
			}
			if (m_game_mode == game_mode::eAuton)
				m_Goal.GetGoal().Activate();
		}
	}
	void Stop()
	{
		if (m_IsStreaming)
		{
			m_IsStreaming = false;
			if (m_game_mode == game_mode::eAuton)
				m_Goal.GetGoal().Terminate();
		}
	}

	void TimeSlice(double dTime_s)
	{
		m_dTime_s = dTime_s;
		//Grab kinematic velocities from controller
		GetInputSlice(dTime_s);
		//Update the predicted motion for this time slice
		m_robot.TimeSlice(dTime_s);
		m_Entity.TimeSlice(dTime_s);
		UpdateVariables();
	}

	void SetGameMode(int mode)
	{
		//If we are leaving from autonomous while still streaming terminate the goal
		if ((m_IsStreaming) && (m_game_mode == game_mode::eAuton) && (game_mode::eAuton != (game_mode)mode))
			m_Goal.GetGoal().Terminate();

		printf("SetGameMode to ");
		bool IsValid = true;
		switch ((game_mode)mode)
		{
		case game_mode::eAuton:
			printf("Auton \n");
			//If we are entering into autonomous already streaming activate the goal
			if ((m_IsStreaming) && (m_game_mode != game_mode::eAuton))
				m_Goal.GetGoal().Activate();
			break;
		case game_mode::eTele:
			printf("Tele \n");
			break;
		case game_mode::eTest:
			printf("Test \n");
			break;
		default:
			printf("Unknown \n");
			IsValid = false;
			break;
		}
		if (IsValid)
			m_game_mode = (game_mode)mode;
	}
	void test(int test)
	{
		switch (test)
		{
		case 1:
			printf("Testing rotate 90\n");
			m_robot.SetIntendedOrientation(PI_2, false);
			//m_robot.SetIntendedOrientation(PI_2, true);
			break;
		case 2:
			m_robot.DriveToLocation(0.0, 0.0);  //simple drive home without managing orientation
			break;
		case 3:
			m_robot.DriveToLocation(0.0, 0.0, true, true, 0.0, false);  //drive facing robot in the direction
			break;
		case 4:
			m_robot.DriveToLocation(0.0, 0.0, true, false);  //hit home at full speed (see if it oscillates)
			break;
		case 5:
			m_robot.DriveToLocation(Feet2Meters(5.0), 0.0, false);  //move forward (whatever direction it is by 5 feet)
			break;
		default:
			printf("Test %d\n", test);
		}
	}
	~Test_Swerve_TeleAuton()
	{
		m_robot.Shutdown(); //detach hooks early
		SetUpHooks(false);
	}
};
#pragma endregion

//We just pick what we test or use from here m_teleop declaration
class RobotAssem_Internal
{
private:
	#pragma region _members_
	#pragma region _testers_
	//TestJoystick m_robot;  //01 joystick
	//Test03_Swerve_Kinematics_with_Joystick m_robot; //02 kinematics and joystick
	//Test_Swerve_Entity_Joystick m_robot;  //03 simple motion profiling  TeleV1
	//Test_Swerve_Viewer m_robot;  //04 full profiling TeleV2
	//Test_Swerve_Rotary m_robot;  //05 full tele with rotary TeleV3
	//enable auton methods for all tests beyond this point	
	#define __HasAutonMethods__
	Test_Swerve_TeleAuton m_robot;  //06 tele auton version 1 (no property integration)
	#pragma endregion

	frc::Timer m_Timer; //use frc timer to take advantage of stepping in simulation (works fine for actual roboRIO too)
	double m_LastTime=0.0;  //used for time slices
	//since some of the testers do not have the game mode, we manage this here
	//also this corresponds to each callback where its one for one on what it updates to
	enum calltype
	{
		eDisabled,
		eAuton,
		eTele,
		eTest
	};

	//current is set from the head of the callback and last is set at the tail of periodic
	calltype m_LastMode=eDisabled;
	calltype m_CurrentMode=eDisabled;
	bool m_IsInit=false;
	#pragma endregion

	void Init()
	{
		if (!m_IsInit)
		{
			m_Timer.Reset();
			m_IsInit=true;
			m_robot.Init();
		}
		#ifdef __HasAutonMethods__
		if (m_CurrentMode!=m_LastMode)
		{
			m_robot.Stop();
			if (m_CurrentMode!=eDisabled)
			{
				m_robot.SetGameMode(m_CurrentMode-1); //we kept the types aligned
				m_robot.Start();
			}
		}
		#endif
	}
	__inline double GetTime() 
	{
		//TODO get the time from WPI so we can use the step effectively
			using std::chrono::duration;
			using std::chrono::duration_cast;
			using std::chrono::system_clock;

			return duration_cast<duration<double>>(system_clock::now().time_since_epoch())
				.count();
	}
  	void TimeSlice()
  {
	  
        const double CurrentTime = m_Timer.GetFPGATimestamp();
        #if 1
        const double DeltaTime = CurrentTime - m_LastTime;
        #else
        const double DeltaTime=0.01;  //It's best to use sythetic time for simulation to step through code
        #endif
        m_LastTime = CurrentTime;
		//sanity check
		//frc::SmartDashboard::PutNumber("time_delta",DeltaTime);
        m_robot.TimeSlice(DeltaTime);
  }
  public:
  	#pragma region _public WPI callbacks_
    void AutonomousInit()
	{
		m_CurrentMode=eAuton;
	    Init();
	}
	void AutonomousPeriodic()
	{
		TimeSlice();
		m_LastMode=eAuton;
	}
	void TeleopInit()
	{
		m_CurrentMode=eTele;
		Init();
	}
	void TeleopPeriodic()
	{
		TimeSlice();
		m_LastMode=eTele;
	}
	void DisabledInit()
	{
		m_CurrentMode=eDisabled;
	}
	void DisabledPeriodic()
	{
		//Nothing to do for the scope of drive, but may need to do things for vision
		m_LastMode=eDisabled;
	}
	void TestInit()
	{
		m_CurrentMode=eTest;
		Init();
	}
	void TestPeriodic()
	{
		TimeSlice();
		m_LastMode=eTest;
	}
	#pragma endregion
};

#pragma region _wrapper_methods_

RobotAssem::RobotAssem()
{
    m_robot=std::make_shared<RobotAssem_Internal>();
}
  void RobotAssem::AutonomousInit()
  {
	  m_robot->AutonomousInit();
  }
  void RobotAssem::AutonomousPeriodic()
  {
	  m_robot->AutonomousPeriodic();
  }
  void RobotAssem::TeleopInit()
  {
	  m_robot->TeleopInit();
  }
  void RobotAssem::TeleopPeriodic()
  {
	  m_robot->TeleopPeriodic();
  }
  void RobotAssem::DisabledInit()
  {
	  m_robot->DisabledInit();
  }
  void RobotAssem::DisabledPeriodic()
  {
	  m_robot->DisabledPeriodic();
  }
  void RobotAssem::TestInit()
  {
	  m_robot->TestInit();
  }
  void RobotAssem::TestPeriodic()
  {
	m_robot->TeleopPeriodic();
  }

#pragma endregion