#include <frc/smartdashboard/SendableChooser.h>
#include "StdAfx.h"
#include "AI_Input_Example.h"
#include "Goal_Types.h"
#include "SmartDashboard_HelperFunctions.h"
#include "AI_BaseController_goals.h"
#include "../../../Properties/RegistryV1.h"
using namespace Framework::Base;

namespace Module
{
	namespace Input
	{

//now this goal can be whatever we want with access to robot resources
class AI_Example_internal : public AtomicGoal
{
private:
	AI_Input* m_pParent=nullptr;
	double m_Timer=0.0;
	MultitaskGoal m_Primer;
	const Framework::Base::asset_manager* m_properties=nullptr;
	enum AutonType
	{
		eDoNothing,
		eJustMoveForward,
		eJustRotate,
		eSimpleMoveRotateSequence,
		eTestBoxWayPoints,
		eTestSmartWayPoints,
		//eDriveTracking,
		eNoAutonTypes
	};
	frc::SendableChooser<AutonType> m_chooser;

	#pragma region _foundation goals_
	class goal_clock : public AtomicGoal
	{
	private:
		AI_Example_internal* m_Parent;
	public:
		goal_clock(AI_Example_internal* Parent) : m_Parent(Parent) { m_Status = eInactive; }
		void Activate() { m_Status = eActive; }
		Goal_Status Process(double dTime_s)
		{
			const double AutonomousTimeLimit = 30.0 * 60.0; //level 1 30 minutes
			double& Timer = m_Parent->m_Timer;
			if (m_Status == eActive)
			{
				frc::SmartDashboard::PutNumber("Timer", AutonomousTimeLimit - Timer);
				Timer += dTime_s;
				if (Timer >= AutonomousTimeLimit)
					m_Status = eCompleted;
			}
			return m_Status;
		}
		void Terminate() { m_Status = eFailed; }
	};

	class goal_watchdog : public AtomicGoal
	{
	private:
		AI_Example_internal* m_Parent;
	public:
		goal_watchdog(AI_Example_internal* Parent) : m_Parent(Parent) { m_Status = eInactive; }
		void Activate() { m_Status = eActive; }
		Goal_Status Process(double dTime_s)
		{
			if (m_Status == eActive)
			{
				//const bool SafetyLock = SmartDashboard::GetBoolean("SafetyLock_Drive");
				bool SafetyLock=false;
				try
				{
					SafetyLock = frc::SmartDashboard::GetBoolean("SafetyLock_Drive",false);
				}
				catch (...)
				{
					//set up default for nothing
					frc::SmartDashboard::PutBoolean("SafetyLock_Drive", false);
				}

				if (SafetyLock)
					m_Status = eFailed;
			}
			return m_Status;
		}
		void Terminate() { m_Status = eFailed; }
	};

	static Goal* Move_Straight(AI_Input* Parent, double length_ft)
	{
		//Construct a way point
		WayPoint wp;
		const Vec2d Local_GoalTarget(0.0, Feet2Meters(length_ft));
		wp.Position = Local_GoalTarget;
		wp.Power = 1.0;
		//Now to setup the goal
		const bool LockOrientation = true;
		const double PrecisionTolerance = Feet2Meters(1.0);
		Goal_Ship_MoveToPosition* goal_drive = NULL;
		goal_drive = new Goal_Ship_MoveToRelativePosition(Parent, wp, true, LockOrientation, PrecisionTolerance);
		return goal_drive;
	}

	static Goal* Rotate(AI_Input* Parent, double Degrees)
	{
		return new Goal_Ship_RotateToRelativePosition(Parent, DEG_2_RAD(Degrees));
	}

	class RobotQuickNotify : public AtomicGoal
	{
	private:
		std::function<void(bool IsOn)> m_Callback;
		bool m_IsOn;
	public:
		RobotQuickNotify(std::function<void(bool IsOn)> callback,bool On) : m_Callback(callback), m_IsOn(On)
		{
			m_Status = eInactive;
		}
		virtual void Activate() { m_Status = eActive; }
		virtual Goal_Status Process(double dTime_s)
		{
			ActivateIfInactive();
			//m_EventMap.EventOnOff_Map[m_EventName.c_str()].Fire(m_IsOn);
			m_Callback(m_IsOn);
			m_Status = eCompleted;
			return m_Status;
		}
	};
	#pragma endregion
	#pragma region _Drive Tests_
	//Drive Tests----------------------------------------------------------------------
	class MoveForward : public Generic_CompositeGoal
	{
	private:
		AI_Input* m_Parent;
	public:
		MoveForward(AI_Input* Parent, bool AutoActivate = false) : Generic_CompositeGoal(AutoActivate), m_Parent(Parent)
		{
			if (!AutoActivate)
				m_Status = eActive;
		}
		virtual void Activate()
		{
			const char* const MoveSmartVar = "TestMove";
			double DistanceFeet = Auton_Smart_GetSingleValue(MoveSmartVar, 1.0); //should be a safe default

			AddSubgoal(new Goal_Wait(0.500));
			AddSubgoal(Move_Straight(m_Parent, DistanceFeet));
			AddSubgoal(new Goal_Wait(0.500));  //allow time for mass to settle
			m_Status = eActive;
		}
	};

	class RotateWithWait : public Generic_CompositeGoal
	{
	private:
		AI_Input* m_Parent;
	public:
		RotateWithWait(AI_Input* Parent, bool AutoActivate = false) : Generic_CompositeGoal(AutoActivate), m_Parent(Parent)
		{
			if (!AutoActivate)
				m_Status = eActive;
		}
		virtual void Activate()
		{
			const char* const RotateSmartVar = "TestRotate";
			const double RotateDegrees = Auton_Smart_GetSingleValue(RotateSmartVar, 45.0); //should be a safe default

			AddSubgoal(new Goal_Wait(0.500));
			AddSubgoal(Rotate(m_Parent, RotateDegrees));
			AddSubgoal(new Goal_Wait(0.500));  //allow time for mass to settle
			m_Status = eActive;
		}
	};

	class TestMoveRotateSequence : public Generic_CompositeGoal
	{
	private:
		AI_Input* m_pParent;
	public:
		TestMoveRotateSequence(AI_Input* Parent) : m_pParent(Parent) 
		{ 
			m_Status = eActive; 
		}
		virtual void Activate()
		{
			double dNoIterations = 4.0;
			double TestRotateDeg = 90.0;
			double TestMoveFeet = 5.0;
			const char* const SmartNames[] = { "TestMoveRotateIter","TestRotate","TestMove" };
			double* const SmartVariables[] = { &dNoIterations,&TestRotateDeg,&TestMoveFeet };
			Auton_Smart_GetMultiValue(3, SmartNames, SmartVariables);
			size_t NoIterations = (size_t)dNoIterations;

			for (size_t i = 0; i < NoIterations; i++)
			{
				AddSubgoal(new MoveForward(m_pParent, true));
				AddSubgoal(new RotateWithWait(m_pParent, true));
			}
			m_Status = eActive;
		}
	};

	static Goal* GiveRobotSquareWayPointGoal(AI_Input* Parent)
	{
		const char* const LengthSetting = "TestDistance_ft";
		const double Length_m = Feet2Meters(Auton_Smart_GetSingleValue(LengthSetting, 5.0));

		std::list <WayPoint> points;
		struct Locations
		{
			double x, y;
		} test[] =
		{
			{Length_m,Length_m},
			{Length_m,-Length_m},
			{-Length_m,-Length_m},
			{-Length_m,Length_m},
			{0,0}
		};
		for (size_t i = 0; i < _countof(test); i++)
		{
			WayPoint wp;
			wp.Position[0] = test[i].x;
			wp.Position[1] = test[i].y;
			//May want to test slower in real life
			//wp.Power = 0.5;
			//This gets you there but the accuracy is rounded to tolerance
			//(because the swivel wheels are challenged more)
			//wp.Power = 0.0;
			//TODO for now the goals work in power, while the robot works with max speed
			//I may change the interface to use power, but for now just use the speed for power
			//In testing this speed does good enough and pretty quick 
			//(may be different response times once the swivel rates change for simulation or in real life)
			wp.Power = 2.5;
			points.push_back(wp);
		}
		//Now to setup the goal
		Goal_Ship_FollowPath* goal = new Goal_Ship_FollowPath(Parent, points, false, true);
		return goal;
	}
	static Goal* SmartWaypoints(AI_Input* Parent, double max_speed)
	{
		const char* const Waypoint_Count = "waypoint_count";
		const size_t wp_count = (size_t)Auton_Smart_GetSingleValue(Waypoint_Count, 1.0);
		std::list <WayPoint> points;
		//we'll push one group at a time to our list (keeping it simple)
		for (size_t i = 0; i < wp_count; i++)
		{
			//generate our group of points
			std::string WP_x="wp_x_";
			WP_x += std::to_string((double)i);
			std::string WP_y = "wp_y_";
			WP_y += std::to_string((double)i);
			std::string WP_speed = "wp_speed_";
			WP_speed += std::to_string((double)i);
			const char* const SmartNames[] = { WP_x.c_str(),WP_y.c_str(),WP_speed.c_str()};
			double X=0.0, Y=0.0, Speed=1.0;
			double* const SmartVariables[] = { &X,&Y,&Speed};
			Auton_Smart_GetMultiValue(3, SmartNames, SmartVariables);
			//Now to setup this way point
			WayPoint wp;
			wp.Position[0] = Feet2Meters(X);
			wp.Position[1] = Feet2Meters(Y);
			wp.Power = Speed * max_speed;
			points.push_back(wp);
		}
		//Now to setup the goal
		Goal_Ship_FollowPath* goal = new Goal_Ship_FollowPath(Parent, points, false, true);
		return goal;
	}
	#pragma endregion
	//Reserved Drive Tracking
public:

	AI_Example_internal(AI_Input* parent) : m_pParent(parent), m_Timer(0.0),
		m_Primer(false)  //who ever is done first on this will complete the goals (i.e. if time runs out)
	{
		m_Status = eInactive;
		m_chooser.SetDefaultOption("Do Nothing", eDoNothing);
  		m_chooser.AddOption("Just Move Forward", eJustMoveForward);
		m_chooser.AddOption("Just Rotate",eJustRotate);
		m_chooser.AddOption("Move Rotate Sequence",eSimpleMoveRotateSequence);
		m_chooser.AddOption("Test Box Waypoints",eTestBoxWayPoints);
		m_chooser.AddOption("Auto SmartDashboard Waypoints",eTestSmartWayPoints);
  		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}
	void Initialize(const Framework::Base::asset_manager* props)
	{
		m_properties = props;  //just reference it we know it stays active
	}
	void Activate()
	{
		m_Primer.AsGoal().Terminate();  //sanity check clear previous session
		//reset timer
		m_Timer=0.0;
		AutonType AutonTest = eDoNothing;
		const char* const AutonTestSelection = "AutonTest";
		#if 0
		frc::SmartDashboard::SetDefaultNumber(AutonTestSelection, 0.0);
		AutonTest = (AutonType)((size_t)frc::SmartDashboard::GetNumber(AutonTestSelection));
		#else
		AutonTest = m_chooser.GetSelected();
		#endif

		printf("Testing=%d \n", AutonTest);
		switch (AutonTest)
		{
		case eJustMoveForward:
			m_Primer.AddGoal(new MoveForward(m_pParent));
			break;
		case eJustRotate:
			m_Primer.AddGoal(new RotateWithWait(m_pParent));
			break;
		case eSimpleMoveRotateSequence:
			m_Primer.AddGoal(new TestMoveRotateSequence(m_pParent));
			break;
		case eTestBoxWayPoints:
			m_Primer.AddGoal(GiveRobotSquareWayPointGoal(m_pParent));
			break;
		case eTestSmartWayPoints:
		{
			using namespace properties::registry_v1;
			std::string max_speed_name = csz_CommonDrive_;
			max_speed_name += csz_Ship_1D_MAX_SPEED;
			const double max_speed = m_properties ? m_properties->get_number(max_speed_name.c_str(), 1.0) : 1.0;
			m_Primer.AddGoal(SmartWaypoints(m_pParent,max_speed));
		}
			break;
		//case eDriveTracking:
		//	m_Primer.AddGoal(new DriveTracking(this));
		//	break;
		case eDoNothing:
		case eNoAutonTypes: //grrr windriver and warning 1250
			break;
		}
		m_Primer.AddGoal(new goal_clock(this));
		m_Primer.AddGoal(new goal_watchdog(this));
		m_Status = eActive;
	}

	Goal_Status Process(double dTime_s)
	{
		ActivateIfInactive();
		if (m_Status == eActive)
			m_Status = m_Primer.AsGoal().Process(dTime_s);
		return m_Status;
	}
	void Terminate()
	{
		m_Primer.AsGoal().Terminate();
		m_Status = eFailed;
	}

};

AI_Example::AI_Example()
{
	m_AI_Input = std::make_shared<AI_Example_internal>(this);
}

void AI_Example::Initialize(const Framework::Base::asset_manager* props)
{
	m_AI_Input->Initialize(props);
}

Goal& AI_Example::GetGoal()
{
	return *m_AI_Input;
}
	}
}