
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "Base/Vec2d.h"
#include "Base/Misc.h"
#include "RobotAssembly.h"

#pragma region _description_
//So in here I'll keep various assemblies to use, these are in order of how they add on and get more developed
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
		//Set the properties... this should be a one-time setup operation
		m_robot.Init();
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;  
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / m_robot.GetDriveProperties().GetTurningDiameter()) * skid;
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
//3. Add simple form of motion control (does not handle change of direction or centripetal forces)
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
		//Set the properties... this should be a one-time setup operation
		m_robot.Init();
		m_MotionControl2D.Initialize();
		m_Entity_Input.Init();
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / m_robot.GetDriveProperties().GetTurningDiameter()) * skid;
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
		//Set the properties... this should be a one-time setup operation
		m_robot.Init();
		m_robot_display.Init();
		m_MotionControl2D.Initialize();
		m_Entity_Input.Init();
		//Assume our robot's top speed is 12 fps
		m_maxspeed = 12.0;
		//I could omit, but I want to show no skid; however, we can reserve this if in practice the 2-3 degrees
		//of precision loss actually does (I don't believe this would ever be significant)
		const double skid = 1.0;
		//We'll compute the max angular velocity to use in equation
		//like with tank spinning in place needs to be half of spinning with full forward
		//this time... no skid
		m_max_heading_rad = (2 * Feet2Meters(m_maxspeed) / m_robot.GetDriveProperties().GetTurningDiameter()) * skid;
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
	void SimulatorTimeSlice(double dTime_s)
	{
		m_robot.SimulatorTimeSlice(dTime_s);
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
	void SimulatorTimeSlice(double dTime_s)
	{
		m_robot.SimulatorTimeSlice(dTime_s);
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
#pragma region _07 Test_Swerve_Viewer TeleAuton V2_
//In this version it adds making properties by use of an asset manager.  
//This keeps the script method isolated from our code base
//We also start signs of output here where we send off the PID values for evaluation (when pid dump is enabled)
//this is needed during calibration to tune the PID
#include "Modules/Output/SmartDashboard_PID_Monitor.h"
#include "Base/AssetManager.h"
#include "Properties/script_loader.h"
#include "Properties/RegistryV1.h"
class Test_Swerve_Properties
{
protected:
	#pragma region _member variables_
	Framework::Base::asset_manager m_properties;
	properties::script_loader m_script_loader;
	//Here we can choose which motion control to use, this works because the interface
	//between them remain (mostly) identical
	Module::Input::wpi_Joystick m_joystick;  //Note: always late binding, so we can aggregate direct easy here
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

	struct DriveAxisAssignments
	{
		struct AxisEntry
		{
			size_t AxisIndex;
			size_t JoystickIndex;
			Module::Input::Analog_EventEntry Options = {
			0.3,   //filter dead zone
			1.0,   //additional scale
			1.0,   // curve intensity
			false  //is flipped
			};
		};
		AxisEntry Strafe = {0,0};
		AxisEntry Forward = { 1,0 };
		AxisEntry Turn = { 2,0 };
	} m_Axis;

	bool m_IsStreaming = false;
	bool m_FieldCentricDrive = false;
	class AdvancedOdometry
	{
	private:
		Test_Swerve_Properties* m_pParent;
		class LocalizationOverride : public	Module::Localization::Entity2D
		{
		private:
			Test_Swerve_Properties* m_pParent;
			bool m_SupportOdometryHeading = false;
			bool m_SupportOdometryPosition = false;
		public:
			LocalizationOverride(Test_Swerve_Properties* parent) : m_pParent(parent)
			{
			}
			void SetSupportOdometryHeading(bool use_it)
			{
				m_SupportOdometryHeading = use_it;
			}
			void SetSupportOdometryPosition(bool use_it)
			{
				m_SupportOdometryPosition = use_it;
			}

			virtual Vector2D GetCurrentPosition() const
			{
				if (m_SupportOdometryPosition)
				{
					const Vec2D will_fix = m_pParent->m_robot.GetCurrentPosition();
					const Vector2D result = { will_fix.x(), will_fix.y() };
					return result;
				}
				else
					return Module::Localization::Entity2D::GetCurrentPosition();
			}
			virtual double GetCurrentHeading() const
			{
				if (m_SupportOdometryHeading)
					return m_pParent->m_robot.GetCurrentHeading();
				else
					return Module::Localization::Entity2D::GetCurrentHeading();
			}

			Vector2D GetCurrentPosition(bool velocity_only) const
			{
				if (m_SupportOdometryPosition && !velocity_only)
				{
					const Vec2D will_fix = m_pParent->m_robot.GetCurrentPosition();
					const Vector2D result = { will_fix.x(), will_fix.y() };
					return result;
				}
				else
					return Module::Localization::Entity2D::GetCurrentPosition();
			}
			double GetCurrentHeading(bool velocity_only) const
			{
				if (m_SupportOdometryHeading && !velocity_only)
					return m_pParent->m_robot.GetCurrentHeading();
				else
					return Module::Localization::Entity2D::GetCurrentHeading();
			}

		}	m_Entity;
		Module::Robot::SwerveRobot& m_robot;
		bool m_Build_sim_prediction_vars = false;
		bool Get_SupportOdometryPosition() const
		{
			return m_robot.Get_SupportOdometryPosition();
		}
		bool Get_SupportOdometryHeading() const
		{
			return m_robot.Get_SupportOdometryHeading();
		}
	public:
		AdvancedOdometry(Test_Swerve_Properties* parent) : m_Entity(parent),m_pParent(parent),m_robot(parent->m_robot)
		{
		}
		void Init(const Framework::Base::asset_manager* asset_properties = nullptr)
		{
			using namespace properties::registry_v1;
			if ((asset_properties) && (asset_properties->get_bool(csz_Build_sim_prediction_vars, false)))
				m_Build_sim_prediction_vars = true;
			//if we have position odometry hooked, we'll use it instead
			if (Get_SupportOdometryPosition())
			{
				m_Entity.SetSupportOdometryPosition(true);
				m_robot.Set_GetCurrentPosition([&]() -> Vec2D
				{
					return m_robot.Get_OdometryCurrentPosition();
				});
			}
			if (Get_SupportOdometryHeading())
			{
				m_Entity.SetSupportOdometryHeading(true);
				m_robot.Set_GetCurrentHeading([&]()
				{
					return m_robot.Get_OdometryCurrentHeading();
				});
			}
		}
		void UpdateVariables()
		{
			using namespace frc;
			using namespace Module::Localization;
			if (Get_SupportOdometryHeading())
			{
				const double heading = m_Entity.GetCurrentHeading(true);
				if (m_Build_sim_prediction_vars)
					SmartDashboard::PutNumber("predicted_Heading", RAD_2_DEG(heading));
			}
			if (Get_SupportOdometryPosition())
			{
				Entity2D::Vector2D position = m_Entity.GetCurrentPosition(true);
				if (m_Build_sim_prediction_vars)
				{
					SmartDashboard::PutNumber("predicted_X_ft", Meters2Feet(position.x));
					SmartDashboard::PutNumber("predicted_Y_ft", Meters2Feet(position.y));
				}
			}
		}
		Module::Localization::Entity2D& GetEntity()
		{
			return m_Entity;
		}
	} m_Advanced_Odometry=this;
	#pragma endregion

	Module::Localization::Entity2D& GetEntity()
	{
		return m_Advanced_Odometry.GetEntity();
	}
	void UpdateVariables()
	{
		m_Advanced_Odometry.UpdateVariables();
		using namespace frc;
		Module::Localization::Entity2D &entity = GetEntity();
		using namespace Module::Localization;
		const Entity2D::Vector2D linear_velocity = entity.GetCurrentVelocity();
		Vec2D velocity_normalized(linear_velocity.x, linear_velocity.y);
		double magnitude = velocity_normalized.normalize();
		SmartDashboard::PutNumber("linear_velocity_x", linear_velocity.x);
		SmartDashboard::PutNumber("linear_velocity_y", linear_velocity.y);
		//Entity variables-------------------------------------------
		SmartDashboard::PutNumber("Velocity", Meters2Feet(magnitude));
		SmartDashboard::PutNumber("Rotation Velocity", GetEntity().GetCurrentAngularVelocity());
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

		wpi_Joystick::JoyState joyinfo[4];
		//wpi_Joystick::JoyState joyinfo[4] = {0}; //setup joy zero'd out
		memset(&joyinfo[0], 0, sizeof(wpi_Joystick::JoyState));
		memset(&joyinfo[1], 0, sizeof(wpi_Joystick::JoyState));
		memset(&joyinfo[2], 0, sizeof(wpi_Joystick::JoyState));
		memset(&joyinfo[3], 0, sizeof(wpi_Joystick::JoyState));

		size_t NoJoySticks = m_joystick.GetNoJoysticksFound();
		for (size_t i = 0 ; i < NoJoySticks; i++)
		{
			//printf("Button: 2=exit, x axis=strafe, y axis=forward/reverse, z axis turn \n");
			m_joystick.read_joystick(i, joyinfo[i]);
		}
		if (m_game_mode==game_mode::eTele)
		{
			const DriveAxisAssignments& _ = m_Axis;
			//Check if we are being driven by some AI method, we override it if we have any angular velocity (i.e. some teleop interaction)
			const double AngularVelocity = 
				(AnalogConversion(joyinfo[_.Turn.JoystickIndex].Axis.AsArray[_.Turn.AxisIndex], _.Turn.Options) ) * m_max_heading_rad;
			if (!IsZero(AngularVelocity) || (m_robot.GetIsDrivenAngular()==false))
				m_robot.SetAngularVelocity(AngularVelocity);

			//Get an input from the controllers to feed in... we'll hard code the x and y axis from both joy and keyboard
			//we simply combine them so they can work inter-changeably (e.g. keyboard for strafing, joy for turning)
			const double Forward = Feet2Meters(m_maxspeed * (AnalogConversion(joyinfo[_.Forward.JoystickIndex].Axis.AsArray[_.Forward.AxisIndex], _.Forward.Options) ) * -1.0);
			const double Right = Feet2Meters(m_maxspeed * (AnalogConversion(joyinfo[_.Strafe.JoystickIndex].Axis.AsArray[_.Strafe.AxisIndex], _.Strafe.Options) ));

			//Check if we are being driven by some AI method, we override it if we have any linear velocity (i.e. some teleop interaction)
			//Note: this logic does not quite work right for keyboard if it uses the forward "sticky" button, but since this isn't a real
			//input I am not going to filter it out, but could if needed.  Also the fabs() ensures the forward and right do not cancel
			//each other out
			if (!IsZero(fabs(Forward) + fabs(Right)) || (m_robot.GetIsDrivenLinear() == false))
				m_FieldCentricDrive ? m_robot.SetLinearVelocity_global(Forward, Right) : m_robot.SetLinearVelocity_local(Forward, Right);
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
		if (joyinfo[0].ButtonBank[0] == 1)
			Reset();
	}

	virtual void SetUpHooks(bool enable)
	{
		if (enable)
		{
			#pragma region _Robot Entity Linking_
			//Now to link up the callbacks for the robot motion control:  Note we can link them up even if we are not using it
			m_robot.Set_UpdateGlobalVelocity([&](const Vec2D &new_velocity)
			{	GetEntity().SetLinearVelocity_global(new_velocity.y(), new_velocity.x());
			});
			m_robot.Set_UpdateHeadingVelocity([&](double new_velocity)
			{	GetEntity().SetAngularVelocity(new_velocity);
			});
			m_robot.Set_GetCurrentPosition([&]() -> Vec2D
			{
				//This is a bit annoying, but for the sake of keeping modules independent (not dependent on vector objects)
				//its worth the hassle
				Vec2D ret = Vec2D(GetEntity().GetCurrentPosition().x, GetEntity().GetCurrentPosition().y);
				return ret;
			});
			m_robot.Set_GetCurrentHeading([&]() -> double
			{
				return GetEntity().GetCurrentHeading();
			});
			m_robot.SetExternal_Velocity_PID_Monitor_Callback(
				[](double Voltage, double  CurrentVelocity, double  Encoder_Velocity, double  ErrorOffset, double  CalibratedScaler)
				{
					Module::Output::Velocity_PID_Monitor(Voltage, CurrentVelocity, Encoder_Velocity, ErrorOffset, CalibratedScaler);
				});
			m_robot.SetExternal_Position_PID_Monitor_Callback(
				[](double Voltage, double Position, double PredictedPosition, double CurrentVelocity, double Encoder_Velocity, double ErrorOffset)
				{
					Module::Output::Position_PID_Monitor(Voltage, Position, PredictedPosition, CurrentVelocity, Encoder_Velocity, ErrorOffset);
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
	void InitController(const char* prefix, DriveAxisAssignments::AxisEntry &val)
	{
		const Framework::Base::asset_manager *asset=&m_properties;
		using namespace ::properties::registry_v1;
		double ftest = 0.0;  //use to test if an asset exists
		std::string constructed_name;
		#define GET_NUMBER(x,y) \
		constructed_name = prefix, constructed_name += csz_##x; \
		y = asset->get_number(constructed_name.c_str(), y);

		#define GET_SIZE_T(x,y) \
		constructed_name = prefix, constructed_name += csz_##x; \
		y = asset->get_number_size_t(constructed_name.c_str(), y);

		#define GET_BOOL(x,y) \
		constructed_name = prefix, constructed_name += csz_##x; \
		y = asset->get_bool(constructed_name.c_str(), y);
		
		GET_SIZE_T(Control_Key, val.AxisIndex);
		GET_SIZE_T(Control_Joy, val.JoystickIndex);
		GET_NUMBER(Control_FilterRange, val.Options.FilterRange);
		GET_NUMBER(Control_Multiplier, val.Options.Multiplier);
		GET_NUMBER(Control_CurveIntensity, val.Options.CurveIntensity);
		GET_BOOL(Control_IsFlipped, val.Options.IsFlipped);
	}
	void InitControllers()
	{
		using namespace ::properties::registry_v1;
		const char* prefix = csz_AxisStrafe_;
		InitController(prefix, m_Axis.Strafe);
		prefix = csz_AxisForward_;
		InitController(prefix, m_Axis.Forward);
		prefix = csz_AxisTurn_;
		InitController(prefix, m_Axis.Turn);
		//--------------Also need to redo m_maxspeed and m_max_heading_rad
		const Framework::Base::asset_manager* asset = &m_properties;
		using namespace ::properties::registry_v1;
		//assume properties will have none or both to keep code simple
		m_maxspeed = Meters2Feet(asset->get_number(csz_Motion2D_max_speed_linear,Feet2Meters(m_maxspeed)));
		m_max_heading_rad = asset->get_number(csz_Motion2D_max_speed_angular, m_max_heading_rad);
	}
public:
	Test_Swerve_Properties()
	{
		SetUpHooks(true);
	}
	void Reset()
	{
		GetEntity().Reset();
		m_robot.Reset();
	}

	virtual void Init()
	{
		m_joystick.Init();
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
		//TODO: (Maybe) we may put this loading in Start() like before, but will need to ensure code can handle this stress... that is 
		//optional to make life easier for scripting updates, but since the driver station can reset the robot code without rebooting 
		//the firmware this should be an adequate work-flow.  However, I may still do this for partial support, it depends on how
		//much tweaking is needed for PID type operations.
		//populate properties method here and invoke it, init can have properties sent as a parameter here
		m_script_loader.load_script(m_properties);
		InitControllers();
		m_Goal.Initialize(&m_properties);
		m_robot.Init(&m_properties);
		m_Advanced_Odometry.Init(&m_properties);
		m_FieldCentricDrive = m_properties.get_bool(properties::registry_v1::csz_Drive_UseFieldCentric, false);

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
	virtual void TimeSlice(double dTime_s)
	{
		m_dTime_s = dTime_s;
		//Grab kinematic velocities from controller
		GetInputSlice(dTime_s);
		//Update the predicted motion for this time slice
		m_robot.TimeSlice(dTime_s);
		GetEntity().TimeSlice(dTime_s);
		UpdateVariables();
		//m_Advanced_Odometry.TimeSliceLoop(dTime_s);  //reserved
	}
	void SimulatorTimeSlice(double dTime_s)
	{
		m_robot.SimulatorTimeSlice(dTime_s);
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
	~Test_Swerve_Properties()
	{
		//Make sure we are stopped
		Stop();
		m_robot.Shutdown(); //detach hooks early
		SetUpHooks(false);
	}
};
#pragma endregion
#pragma region _08 Main Assembly_
//For now we inherit from TeleAuton V2, since we only need to link up the WPI output to the Swerve Robot
//We may want to copy everything later, but doing it this way helps to read exactly what is changed
#include "Modules/Output/WPI_Output.h"
class Main_Assembly : public Test_Swerve_Properties
{
private:
	Module::Output::WPI_Output m_WPI;
	//#define __BypassPhysicalOdometry__
	//constructor can't virtually call this so, we give it it's own name an manage it here
	void SetUpHooks_main(bool enable)
	{
		Test_Swerve_Properties::SetUpHooks(enable);
		//TODO link Swerve Robot to the physical odometry once it is ready
		if (enable)
		{
			m_WPI.SetVoltageCallback(
				[&]()
				{
					return m_robot.GetCurrentVoltages();
				});
			m_WPI.SetSimOdometry(
				[&]()
				{
					return m_robot.GetSimulatedVelocities();	
				});
			m_WPI.SetSimOdometry_heading(
				[&]()
				{
					return m_robot.Get_SimulatedCurrentHeading();
				});
			//For simulation we can bypass this and use our own internal simulation
			#ifndef __BypassPhysicalOdometry__
			m_robot.SetPhysicalOdometry(
				[&]()
				{
					return m_WPI.GetCurrentVelocities();
				});
			#endif
		}
		else
		{
			m_WPI.SetVoltageCallback(nullptr);
			m_WPI.SetSimOdometry(nullptr);
			m_robot.SetPhysicalOdometry(nullptr);
		}
	}
	public:
	Main_Assembly()
	{
		SetUpHooks_main(true);
	}
	~Main_Assembly()
	{
		SetUpHooks_main(false);
	}
	void Init()
	{
		//call predecessor first to setup the properties
		Test_Swerve_Properties::Init();
		m_WPI.Init(&m_properties);
   		using namespace ::properties::registry_v1;
		const bool HaveGyro = m_properties.get_bool(csz_Misc_have_gyro, false);
        if (HaveGyro)
		{
			m_robot.SetPhysicalOdometry_heading(
				[&]()
				{
					return m_WPI.GyroMag_GetCurrentHeading();
				});
		}
	}
	void TimeSlice(double dTime_s)
	{
		Test_Swerve_Properties::TimeSlice(dTime_s);
		m_WPI.TimeSlice(dTime_s);
	}
	void SimulatorTimeSlice(double dTime_s)
	{
		Test_Swerve_Properties::SimulatorTimeSlice(dTime_s);
		m_WPI.SimulatorTimeSlice(dTime_s);
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
	//Enable simulation time slices for all tests beyond this point
	#define __HasSimulation__
	//Test_Swerve_Rotary m_robot;  //05 full tele with rotary TeleV3
	//enable auton methods for all tests beyond this point	
	#define __HasAutonMethods__
	//Test_Swerve_TeleAuton m_robot;  //06 tele auton version 1 (no property integration)
	//Test_Swerve_Properties m_robot; //07 tele auton version 2, properties and PID viewing when on
	Main_Assembly m_robot;
	#pragma endregion

	frc::Timer m_Timer; //use frc timer to take advantage of stepping in simulation (works fine for actual roboRIO too)
	double m_LastTime=0.0;  //used for time slices
	double m_simLastTime=0.0;  //for simulation time slices
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
        const double DeltaTime=0.01;  //It's best to use synthetic time for simulation to step through code
        #endif
        m_LastTime = CurrentTime;
		//sanity check
		//frc::SmartDashboard::PutNumber("time_delta",DeltaTime);
        m_robot.TimeSlice(DeltaTime);
  }
  void SimulatorTimeSlice()
  {
        const double CurrentTime = m_Timer.GetFPGATimestamp();
        #if 1
        const double DeltaTime = CurrentTime - m_simLastTime;
        #else
        const double DeltaTime=0.01;
        #endif
        m_simLastTime = CurrentTime;
		//sanity check
		//frc::SmartDashboard::PutNumber("time_delta",DeltaTime);
		#ifdef __HasSimulation__
        m_robot.SimulatorTimeSlice(DeltaTime);
		#endif
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
	void SimulationInit ()
	{
		//May need for WPI Simulations
	}
	void SimulationPeriodic ()
	{
		SimulatorTimeSlice();
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
  void RobotAssem::SimulationInit ()
  {
	  m_robot->SimulationInit();
  }
  void RobotAssem::SimulationPeriodic ()
  {
	  m_robot->SimulationPeriodic();
  }

#pragma endregion