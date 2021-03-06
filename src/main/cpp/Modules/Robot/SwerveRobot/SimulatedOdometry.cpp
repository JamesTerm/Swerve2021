#pragma region _includes_
//The usual include stuff
#include "stdafx.h"
#include <future>
#include <chrono>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "../../../Base/Base_Includes.h"
#include "../../../Base/Vec2d.h"
#include "../../../Base/Misc.h"
#include "../../../Base/PIDController.h"

#include "../DriveKinematics/Vehicle_Drive.h"
#include "../../../Properties/RegistryV1.h"
#include "SimulatedOdometry.h"
#include "RotaryProperties_Legacy.h"

//Note __UseBypass__ is replaced by a properties driven asset

//TODO
//Get simulator 3 working
//Tweak properties for ideal ride in the example script
//Ensure pot simulator is unlimited

//Keep around if the new simulation has issues
//#define __UseLegacySimulation__

#pragma endregion
namespace Module {
	namespace Robot {

#pragma region _Legacy CalibrationTesting_
//I do not want to use any legacy code for the new simulation
//TODO put macro back once everything is working properly
#ifdef __UseLegacySimulation__
namespace Legacy {
//Note:  This section really belongs with the simulated odometry; however, given the legacy dependency on ship 1D and properties
//it is cleaner to keep the code intact in the order of dependencies, and rewrite a better updated simulation there
#pragma region _CT constants_
const double c_OptimalAngleUp_r = DEG_2_RAD(70.0);
const double c_OptimalAngleDn_r = DEG_2_RAD(50.0);
const double c_ArmToGearRatio = 72.0 / 28.0;
const double c_GearToArmRatio = 1.0 / c_ArmToGearRatio;
//const double c_PotentiometerToGearRatio=60.0/32.0;
//const double c_PotentiometerToArmRatio=c_PotentiometerToGearRatio * c_GearToArmRatio;
const double c_PotentiometerToArmRatio = 36.0 / 54.0;
const double c_PotentiometerToGearRatio = c_PotentiometerToArmRatio * c_ArmToGearRatio;
const double c_ArmToPotentiometerRatio = 1.0 / c_PotentiometerToArmRatio;
const double c_GearToPotentiometer = 1.0 / c_PotentiometerToGearRatio;
//const double c_TestRate=3.0;
//const double c_TestRate=6.0;
//const double c_Potentiometer_TestRate=18.0;
const double c_Potentiometer_TestRate = 24.0;
//const double c_Potentiometer_TestRate=48.0;
const double Polynomial[5] =
{
	0.0,1.0   ,0.0    ,0.0   ,0.0
	//0.0,2.4878,-2.2091,0.7134,0.0
};

const double c_DeadZone = 0.085;
const double c_MidRangeZone = .150;
const double c_MidDistance = c_MidRangeZone - c_DeadZone;
const double c_rMidDistance = 1.0 / c_MidDistance;
const double c_EndDistance = 1.0 - c_MidRangeZone;
const double c_rEndDistance = 1.0 / c_EndDistance;

#define ENCODER_TEST_RATE 1
#if ENCODER_TEST_RATE==0
const double c_Encoder_TestRate = 16.0;
const double c_Encoder_MaxAccel = 120.0;
#endif

#if ENCODER_TEST_RATE==1
const double c_Encoder_TestRate = 2.916;
const double c_Encoder_MaxAccel = 60.0;
#endif

#if ENCODER_TEST_RATE==2
const double c_Encoder_TestRate = 2.4;
const double c_Encoder_MaxAccel = 4.0;
#endif

#if ENCODER_TEST_RATE==3
const double c_Encoder_TestRate = 1.1;
const double c_Encoder_MaxAccel = 2.0;
#endif

const double c_OunceInchToNewton = 0.00706155183333;

const double c_CIM_Amp_To_Torque_oz = (1.0 / (133.0 - 2.7)) * 343.3;
const double c_CIM_Amp_To_Torque_nm = c_CIM_Amp_To_Torque_oz * c_OunceInchToNewton;
const double c_CIM_Vel_To_Torque_oz = (1.0 / (5310 / 60.0)) * 343.4;
const double c_CIM_Vel_To_Torque_nm = c_CIM_Vel_To_Torque_oz * c_OunceInchToNewton;
const double c_CIM_Torque_to_Vel_nm = 1.0 / c_CIM_Vel_To_Torque_nm;

#define __USE_PAYLOAD_MASS__
#pragma endregion
#pragma region _helper functions_
double GetTweakedVoltage(double Voltage)
{
	double VoltMag = fabs(Voltage);
	double sign = Voltage / VoltMag;

	//simulate stresses on the robot
	if (VoltMag > 0.0)
	{
		if (VoltMag < c_DeadZone)
		{
			//DebugOutput("Buzz %f\n", Voltage);   //simulate no movement but hearing the motor
			VoltMag = 0.0;
		}
		else if (VoltMag < c_MidRangeZone)
			VoltMag = (VoltMag - c_DeadZone) * c_rMidDistance * 0.5;
		else
			VoltMag = ((VoltMag - c_MidRangeZone) * c_rEndDistance * 0.5) + 0.5;

		Voltage = VoltMag * sign;
	}
	return Voltage;
}
//Not used
#if 0
static double AngleToHeight_m(double Angle_r)
{
	//TODO fix
	const double c_ArmToGearRatio = 72.0 / 28.0;
	const double c_GearToArmRatio = 1.0 / c_ArmToGearRatio;
	const double c_ArmLength_m = 1.8288;  //6 feet
	const double c_GearHeightOffset = 1.397;  //55 inches

	return (sin(Angle_r * c_GearToArmRatio) * c_ArmLength_m) + c_GearHeightOffset;
}
#endif
#pragma endregion
class COMMON_API Potentiometer_Tester : public Ship_1D
{
private:
	double m_Time_s=0.0;
	Ship_1D_Properties m_PotentiometerProps;
	//GG_Framework::Base::EventMap m_DummyMap;
	bool m_Bypass;  //used for stress test
public:
	Potentiometer_Tester() : m_PotentiometerProps(
		"Potentiometer",
		2.0,    //Mass
		0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
		c_Potentiometer_TestRate,   //Max Speed
		1.0, 1.0, //ACCEL, BRAKE  (These can be ignored)
		c_Potentiometer_TestRate, c_Potentiometer_TestRate,
		//Ship_1D_Props::eRobotArm,
		true,	//Using the range
		DEG_2_RAD(-135.0), DEG_2_RAD(135.0)
	), Ship_1D("Potentiometer")
	{
		m_Bypass = false;
		Initialize(&m_PotentiometerProps);
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		Voltage = GetTweakedVoltage(Voltage);
		if (!m_Bypass)
			SetRequestedVelocity(Voltage * c_GearToPotentiometer * m_PotentiometerProps.GetMaxSpeed());
		else
			SetRequestedVelocity(0.0);
	}
	virtual double GetPotentiometerCurrentPosition()
	{
		//Note this is in native potentiometer ratios
		double Pos_m = GetPos_m();
		//double height = AngleToHeight_m(Pos_m);  //not used... for reference

		//DOUT5("Pot=%f Angle=%f %fft %fin",m_Physics.GetVelocity(),RAD_2_DEG(Pos_m*c_GearToArmRatio),height*3.2808399,height*39.3700787);

		return Pos_m;
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	void TimeChange()
	{
		Ship_1D::TimeChange(m_Time_s);
	}
	void SetBypass(bool bypass) {m_Bypass=bypass;}
};
class COMMON_API Potentiometer_Tester2 : public Ship_1D
{
private:
	void SimulateOpposingForce(double Voltage)
	{
		const double OuterDistance = 3.8;
		const double CoreDistance = 4.0;

		const double Pos = GetPos_m();
		const double Velocity = m_Physics.GetVelocity();
		double ForceScaler = 0.0;
		if ((Pos > OuterDistance) && (Velocity >= 0.0))
		{
			//determine how much force is being applied
			const double Force = m_Physics.GetForceFromVelocity(Velocity + Velocity, m_Time_s);
			if (Pos < CoreDistance)
			{
				//dampen force depending on the current distance
				const double scale = 1.0 / (CoreDistance - OuterDistance);
				ForceScaler = (Pos - OuterDistance) * scale;
			}
			else
				ForceScaler = 1.0;
			m_Physics.ApplyFractionalForce(-Force * ForceScaler, m_Time_s);
		}
		SetRequestedVelocity(Voltage * (1.0 - ForceScaler) * m_PotentiometerProps.GetMaxSpeed());
	}
	double m_Time_s=0.0;
	Ship_1D_Properties m_PotentiometerProps;
	//GG_Framework::Base::EventMap m_DummyMap;
	bool m_SimulateOpposingForce;  //used for stress test
public:
	Potentiometer_Tester2() : m_PotentiometerProps(
		"Potentiometer2",
		2.0,    //Mass
		0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
		10.0,   //Max Speed
		1.0, 1.0, //ACCEL, BRAKE  (These can be ignored)
		10.0, 10.0,
		//Ship_1D_Props::eSwivel,
		true,	//Using the range
		DEG_2_RAD(-180.0), DEG_2_RAD(180.0)
	), Ship_1D("Potentiometer2")
	{
		m_SimulateOpposingForce = false;
	}
	virtual void Init_Ship_1D(const Ship_1D_Properties* props = NULL)
	{
		if (props)
			m_PotentiometerProps = *props;
		Ship_1D::Initialize(&m_PotentiometerProps);
	}
	virtual void Initialize(size_t index, const Framework::Base::asset_manager* props = NULL)
	{
		#pragma region _setup get vars_
		double ftest = 0.0;  //use to test if an asset exists

		#define GET_NUMBER(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = props->get_number(constructed_name.c_str(), y);
		#define GET_BOOL(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = props->get_bool(constructed_name.c_str(), y);

		using namespace properties::registry_v1;
		std::string constructed_name;
		const char * const prefix = csz_CommonSwivel_;
		#pragma endregion
		//just pull the ship props from the asset manager
		Ship_1D_Properties props_to_send;
		#pragma region _Entity1D_
		//entity 1D
		Entity1D_Props& e1d = props_to_send.AsEntityProps();
		//constructed_name = prefix, constructed_name += csz_Entity1D_StartingPosition;
		//e1d.m_StartingPosition = asset_properties->get_number(constructed_name.c_str(), e1d.m_StartingPosition);
		GET_NUMBER(Entity1D_StartingPosition, e1d.m_StartingPosition);
		GET_NUMBER(Entity1D_Mass, e1d.m_Mass);
		GET_NUMBER(Entity1D_Dimension, e1d.m_Dimension);
		GET_BOOL(Entity1D_IsAngular,e1d.m_IsAngular);
		#pragma endregion
		#pragma region _Ship1D_
		Ship_1D_Props& _Ship_1D = props_to_send.GetShip_1D_Props_rw();
		GET_NUMBER(Ship_1D_MAX_SPEED, _Ship_1D.MAX_SPEED);
		//IF we have this asset assign it to forward and reverse
		if (props->get_number_native(constructed_name.c_str(), ftest))
		{
			_Ship_1D.MaxSpeed_Forward = _Ship_1D.MAX_SPEED;
			_Ship_1D.MaxSpeed_Reverse = -_Ship_1D.MAX_SPEED;
		}
		GET_NUMBER(Ship_1D_MaxSpeed_Forward, _Ship_1D.MaxSpeed_Forward);
		GET_NUMBER(Ship_1D_MaxSpeed_Reverse, _Ship_1D.MaxSpeed_Reverse);
		GET_NUMBER(Ship_1D_ACCEL, _Ship_1D.ACCEL);
		GET_NUMBER(Ship_1D_BRAKE, _Ship_1D.BRAKE);

		//I don't do this often but there may be an easier way to handle this, try the simulation one first
		//if none then use MaxAccelForward
		constructed_name = prefix, constructed_name += csz_Ship_1D_MaxAccel_simulation;
		std::string name2 = prefix; 
		name2 += csz_Ship_1D_MaxAccelForward;
		_Ship_1D.MaxAccelForward = props->get_number(constructed_name.c_str(),
			props->get_number(name2.c_str(), _Ship_1D.MaxAccelForward));

		//if I don't have reverse use forward here for default
		//GET_NUMBER(Ship_1D_MaxAccelReverse, _Ship_1D.MaxAccelForward);
		constructed_name = prefix, constructed_name += csz_Ship_1D_MaxAccelReverse;
		_Ship_1D.MaxAccelReverse = props->get_number(constructed_name.c_str(), _Ship_1D.MaxAccelForward);

		GET_NUMBER(Ship_1D_MinRange, _Ship_1D.MinRange);
		GET_NUMBER(Ship_1D_MaxRange, _Ship_1D.MaxRange);
		GET_NUMBER(Ship_1D_DistanceDegradeScaler, _Ship_1D.DistanceDegradeScaler);
		GET_BOOL(Ship_1D_UsingRange, _Ship_1D.UsingRange) //bool
		#pragma endregion
		//finished with macros
		#undef GET_NUMBER
		#undef GET_BOOL
		//send these props
		Init_Ship_1D(&props_to_send);
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		//Voltage=GetTweakedVoltage(Voltage);
		if (!m_SimulateOpposingForce)
			SetRequestedVelocity(Voltage * m_PotentiometerProps.GetMaxSpeed());
		else
			SimulateOpposingForce(Voltage);
	}
	virtual double GetPotentiometerCurrentPosition()
	{
		//Note this is in native potentiometer ratios
		double Pos_m = GetPos_m();
		return Pos_m;
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	void TimeChange()
	{
		Ship_1D::TimeChange(m_Time_s);
	}
	void SetSimulateOpposingForce(bool Simulate) {m_SimulateOpposingForce=Simulate;}
};
class COMMON_API Encoder_Simulator : public Ship_1D
{
private:
	double m_Time_s;
	Ship_1D_Properties m_EncoderProps;
	//GG_Framework::Base::EventMap m_DummyMap;
	Framework::Base::LatencyFilter m_Latency;
	double m_EncoderScaler; //used to implement reverse
	bool m_GetEncoderFirstCall;  //allows GetEncoderVelocity to know when a new set of calls occur within a time slice
public:
	Encoder_Simulator(const char *EntityName="EncSimulator") : m_Time_s(0.0), m_EncoderProps(
		EntityName,
		68.0,    //Mass
		0.0,   //Dimension  (this really does not matter for this, there is currently no functionality for this property, although it could impact limits)
		c_Encoder_TestRate,   //Max Speed
		1.0, 1.0, //ACCEL, BRAKE  (These can be ignored)
		c_Encoder_MaxAccel, c_Encoder_MaxAccel,
		//Ship_1D_Props::eRobotArm,
		false	//Not using the range
	), Ship_1D(EntityName), m_Latency(0.300),
		m_EncoderScaler(1.0), m_GetEncoderFirstCall(false)
	{
	}
	virtual void Initialize(const Ship_1D_Properties* props = NULL)
	{
		if (props)
			m_EncoderProps = *props;
		Ship_1D::Initialize(&m_EncoderProps);
	}
	void UpdateEncoderVoltage(double Voltage)
	{
		//Here is some stress to emulate a bad curved victor
		#if 0
		//This is how it would be if the motor was set to non-coast
		double VoltageMag = pow(fabs(Voltage), 0.5);
		Voltage = (Voltage > 0.0) ? VoltageMag : -VoltageMag;
		#endif
		SetRequestedVelocity(Voltage * m_EncoderProps.GetMaxSpeed());
		#if 0
		//For coast it is more like applying force
		double Velocity = m_Physics.GetVelocity();
		double MaxSpeed = m_EncoderProps.GetMaxSpeed();
		double Accel = Voltage * 5000.0 * MaxSpeed * m_Time_s;  //determine current acceleration by applying the ratio with a scaler  
		double filter = 1.0 - (fabs(Velocity) / MaxSpeed);  //as there is more speed there is less torque
		//double friction=((Velocity>0.04)?-0.025 : (Velocity<-0.04)?+0.025 : 0.0 )* rTime ;  //There is a constant amount of friction opposite to current velocity
		double friction = ((Velocity > 0.04) ? -250 : (Velocity < -0.04) ? +250 : 0.0) * m_Time_s;  //There is a constant amount of friction opposite to current velocity
		SetCurrentLinearAcceleration((Accel * filter * filter) + friction); //square the filter (matches read out better)
		#endif
	}
	virtual double GetEncoderVelocity()
	{
		#if 0
		return m_Physics.GetVelocity() * m_EncoderScaler;
		#else
		//if (!m_GetEncoderFirstCall) return m_Latency();

		double Voltage = m_Physics.GetVelocity() / m_EncoderProps.GetMaxSpeed();
		double Direction = Voltage < 0 ? -1.0 : 1.0;
		Voltage = fabs(Voltage); //make positive
		//Apply the victor curve
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double* c = Polynomial;
			double x2 = Voltage * Voltage;
			double x3 = Voltage * x2;
			double x4 = x2 * x2;
			Voltage = (c[4] * x4) + (c[3] * x3) + (c[2] * x2) + (c[1] * Voltage) + c[0];
			Voltage *= Direction;
		}
		double ret = Voltage * m_EncoderProps.GetMaxSpeed() * m_EncoderScaler;
		//ret=m_Latency(ret,m_Time_s);
		m_GetEncoderFirstCall = false; //weed out the repeat calls
		return ret;
		#endif
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	void TimeChange()
	{
		m_GetEncoderFirstCall = true;
		Ship_1D::TimeChange(m_Time_s);
	}
	void SetReverseDirection(bool reverseDirection)
	{
		//emulates functionality of the encoder (needed because kids put them in differently)
		m_EncoderScaler = reverseDirection ? -1.0 : 1.0;
	}
	void SetEncoderScaler(double value) {m_EncoderScaler=value;}  //This helps to simulate differences between sides
	void SetFriction(double StaticFriction,double KineticFriction) {}
};

//TorqueAccelerationDampener - formerly known as ForceAppliedOnWheelRadius has become the solution to account for empirical testing of torque acceleration 
//measured in the motor, unfortunately I haven't yet found a way to compute for this, but this is quite effective as its factoring happens in the best
//place (same place as it was before)
struct EncoderSimulation_Props
{
	double Wheel_Mass;  //This is a total mass of all the wheels and gears for one side
	double COF_Efficiency;
	double GearReduction;  //In reciprocal form of spread sheet   driving gear / driven gear
	double TorqueAccelerationDampener; //ratio 1.0 no change
	double DriveWheelRadius; //in meters
	double NoMotors;  //Used to get total torque
	double PayloadMass;  //The robot weight in kg
	double SpeedLossConstant;
	double DriveTrainEfficiency;

	struct Motor_Specs
	{
		double FreeSpeed_RPM;
		double Stall_Torque_NM;
		double Stall_Current_Amp;
		double Free_Current_Amp;
	} motor;
};

//This is used in calibration testing to simulate encoder readings
class COMMON_API EncoderSimulation_Properties
{
	public:
		EncoderSimulation_Properties()
		{
			EncoderSimulation_Props props;
			memset(&props, 0, sizeof(EncoderSimulation_Props));

			props.Wheel_Mass = 1.5;
			props.COF_Efficiency = 1.0;
			props.GearReduction = 12.4158;
			props.TorqueAccelerationDampener = 0.0508;
			props.DriveWheelRadius = 0.0762;
			props.NoMotors = 1.0;
			props.PayloadMass = 200.0 * 0.453592;  //in kilograms
			props.SpeedLossConstant = 0.81;
			props.DriveTrainEfficiency = 0.9;


			props.motor.FreeSpeed_RPM = 5310;
			props.motor.Stall_Torque_NM = 343.4 * c_OunceInchToNewton;
			props.motor.Stall_Current_Amp = 133.0;
			props.motor.Free_Current_Amp = 2.7;

			m_EncoderSimulation_Props = props;
		}
		//virtual void LoadFromScript(GG_Framework::Logic::Scripting::Script& script);
		const EncoderSimulation_Props &GetEncoderSimulationProps() const {return m_EncoderSimulation_Props;}
		//Get and Set the properties
		EncoderSimulation_Props &EncoderSimulationProps() {return m_EncoderSimulation_Props;}

	protected:
		EncoderSimulation_Props m_EncoderSimulation_Props;
};

class COMMON_API Drive_Train_Characteristics
{
	public:
		Drive_Train_Characteristics()
		{
			EncoderSimulation_Properties default_props;
			m_Props = default_props.GetEncoderSimulationProps();
		}
		void UpdateProps(const EncoderSimulation_Props &props) {m_Props=props;}
		void UpdateProps(const Framework::Base::asset_manager* props = NULL)
		{ 
			if (!props)
				return;
			using namespace ::properties::registry_v1;
			#define GET_NUMBER(x,y) \
			y = props->get_number(csz_##x, y);

			GET_NUMBER(EncoderSimulation_Wheel_Mass,m_Props.Wheel_Mass);
			GET_NUMBER(EncoderSimulation_COF_Efficiency, m_Props.COF_Efficiency);
			GET_NUMBER(EncoderSimulation_GearReduction, m_Props.GearReduction);
			GET_NUMBER(EncoderSimulation_TorqueAccelerationDampener, m_Props.TorqueAccelerationDampener);
			GET_NUMBER(EncoderSimulation_DriveWheelRadius, m_Props.DriveWheelRadius);
			GET_NUMBER(EncoderSimulation_NoMotors, m_Props.NoMotors);
			GET_NUMBER(EncoderSimulation_PayloadMass, m_Props.PayloadMass);
			GET_NUMBER(EncoderSimulation_SpeedLossConstant, m_Props.SpeedLossConstant);
			GET_NUMBER(EncoderSimulation_DriveTrainEfficiency, m_Props.DriveTrainEfficiency);
			//	struct Motor_Specs
			EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			GET_NUMBER(EncoderSimulation_FreeSpeed_RPM, motor.FreeSpeed_RPM);
			GET_NUMBER(EncoderSimulation_Stall_Torque_NM, motor.Stall_Torque_NM);
			GET_NUMBER(EncoderSimulation_Stall_Current_Amp, motor.Stall_Current_Amp);
			GET_NUMBER(EncoderSimulation_Free_Current_Amp, motor.Free_Current_Amp);
			#undef GET_NUMBER
		}

		__inline double GetAmp_To_Torque_nm(double Amps) const
		{
			const EncoderSimulation_Props::Motor_Specs& _ = m_Props.motor;
			const double c_Amp_To_Torque_nm = (1.0 / (_.Stall_Current_Amp - _.Free_Current_Amp)) * _.Stall_Torque_NM;
			return std::max((Amps - _.Free_Current_Amp) * c_Amp_To_Torque_nm, 0.0);
		}
		__inline double INV_GetVel_To_Torque_nm(double Vel_rps) const
		{
			//depreciated
			const EncoderSimulation_Props::Motor_Specs& _ = m_Props.motor;
			const double c_Vel_To_Torque_nm = (1.0 / (_.FreeSpeed_RPM / 60.0)) * _.Stall_Torque_NM;
			return (Vel_rps * c_Vel_To_Torque_nm);
		}
		__inline double GetVel_To_Torque_nm(double motor_Vel_rps) const
		{
			const EncoderSimulation_Props::Motor_Specs& _ = m_Props.motor;
			const double FreeSpeed_RPS = (_.FreeSpeed_RPM / 60.0);
			//ensure we don't exceed the max velocity
			const double x = std::min(fabs(motor_Vel_rps) / FreeSpeed_RPS, 1.0);  //working with normalized positive number for inversion
			const double Torque_nm((1.0 - x) * _.Stall_Torque_NM);
			return (motor_Vel_rps >= 0) ? Torque_nm : -Torque_nm;
		}
		__inline double GetTorque_To_Vel_nm_V1(double motor_Vel_rps) const
		{  //depreciated
			return 0.0;
		}
		__inline double GetWheelTorque(double Torque) const
		{
			return Torque / m_Props.GearReduction * m_Props.DriveTrainEfficiency;
		}
		__inline double INV_GetWheelTorque(double Torque) const
		{
			//depreciated
			return Torque * m_Props.GearReduction * m_Props.COF_Efficiency;
		}
		__inline double GetWheelStallTorque() const 
		{
			return m_Props.motor.Stall_Torque_NM / m_Props.GearReduction * m_Props.DriveTrainEfficiency;
		}
		__inline double GetTorqueAtWheel(double Torque) const
		{
			return (GetWheelTorque(Torque) / m_Props.DriveWheelRadius);
		}
		__inline double GetWheelRPS(double LinearVelocity) const
		{
			return LinearVelocity / (M_PI * 2.0 * m_Props.DriveWheelRadius);
		}
		__inline double GetLinearVelocity(double wheel_RPS) const
		{
			return wheel_RPS * (M_PI * 2.0 * m_Props.DriveWheelRadius);
		}
		__inline double GetLinearVelocity_WheelAngular(double wheel_AngularVelocity) const
		{
			return wheel_AngularVelocity * m_Props.DriveWheelRadius;
		}
		__inline double GetMotorRPS(double LinearVelocity) const
		{
			return GetWheelRPS(LinearVelocity) / m_Props.GearReduction;
		}
		__inline double GetWheelRPS_Angular(double wheel_AngularVelocity) const
		{
			return wheel_AngularVelocity / (M_PI * 2.0);
		}
		__inline double GetWheelAngular_RPS(double wheel_RPS) const
		{
			return wheel_RPS * (M_PI * 2.0);
		}
		__inline double GetWheelAngular_LinearVelocity(double LinearVelocity) const
		{
			//accurate as long as there is no skid
			return LinearVelocity / m_Props.DriveWheelRadius;
		}
		__inline double GetMotorRPS_Angular(double wheel_AngularVelocity) const
		{
			return GetWheelRPS_Angular(wheel_AngularVelocity) / m_Props.GearReduction;
		}
		__inline double INV_GetMotorRPS_Angular(double wheel_AngularVelocity) const
		{
			//depreciated
			return GetWheelRPS_Angular(wheel_AngularVelocity) * m_Props.GearReduction;
		}
		__inline double GetTorqueFromLinearVelocity(double LinearVelocity) const
		{
			const double MotorTorque = GetVel_To_Torque_nm(GetMotorRPS(LinearVelocity));
			return GetTorqueAtWheel(MotorTorque);
		}
		__inline double GetWheelTorqueFromVoltage(double Voltage) const
		{
			const EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			const double Amps = fabs(Voltage * motor.Stall_Current_Amp);
			const double MotorTorque = GetAmp_To_Torque_nm(Amps);
			const double WheelTorque = GetTorqueAtWheel(MotorTorque * 2.0);
			return (Voltage > 0) ? WheelTorque : -WheelTorque;  //restore sign
		}
		__inline double GetTorqueFromVoltage_V1(double Voltage) const
		{
			//depreciated
			const EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			const double Amps = fabs(Voltage * motor.Stall_Current_Amp);
			const double MotorTorque = GetAmp_To_Torque_nm(Amps);
			const double WheelTorque = INV_GetWheelTorque(MotorTorque * m_Props.NoMotors);
			return (Voltage > 0) ? WheelTorque : -WheelTorque;  //restore sign
		}
		__inline double GetTorqueFromVoltage(double Voltage) const
		{
			const EncoderSimulation_Props::Motor_Specs& motor = m_Props.motor;
			const double Amps = fabs(Voltage * motor.Stall_Current_Amp);
			const double MotorTorque = GetAmp_To_Torque_nm(Amps);
			const double WheelTorque = GetWheelTorque(MotorTorque * m_Props.NoMotors);
			return (Voltage > 0) ? WheelTorque : -WheelTorque;  //restore sign
		}
		__inline double INV_GetTorqueFromVelocity(double wheel_AngularVelocity) const
		{
			//depreciated
			const double MotorTorque_nm = INV_GetVel_To_Torque_nm(INV_GetMotorRPS_Angular(wheel_AngularVelocity));
			return INV_GetWheelTorque(MotorTorque_nm * m_Props.NoMotors);
		}
		__inline double GetTorqueFromVelocity(double wheel_AngularVelocity) const
		{
			const double MotorTorque_nm = GetVel_To_Torque_nm(GetMotorRPS_Angular(wheel_AngularVelocity));
			return GetWheelTorque(MotorTorque_nm * m_Props.NoMotors);
		}
		__inline double GetMaxTraction() const {return m_Props.PayloadMass*m_Props.COF_Efficiency;}
		__inline double GetMaxDriveForce() const {return GetWheelStallTorque()/m_Props.DriveWheelRadius*2.0;}
		__inline double GetCurrentDriveForce(double WheelTorque) const {return WheelTorque/m_Props.DriveWheelRadius*2.0;}
		__inline double GetMaxPushingForce() const {	return std::min(GetMaxTraction()*9.80665,GetMaxDriveForce());}

		const EncoderSimulation_Props &GetDriveTrainProps() const {return m_Props;}
		void SetGearReduction(double NewGearing) {m_Props.GearReduction=NewGearing;}
	private:
		EncoderSimulation_Props m_Props;
};
class COMMON_API Encoder_Simulator2
{
protected:
	//The wheel physics records the velocity of the wheel in radians
	Framework::Base::PhysicsEntity_1D m_Physics;
	Drive_Train_Characteristics m_DriveTrain;
	double m_Time_s;

	double m_Position;  //also keep track of position to simulate distance use case (i.e. used as a potentiometer)
	double m_EncoderScaler; //used for position updates
	double m_ReverseMultiply; //used to implement set reverse direction
public:
	Encoder_Simulator2(const char* EntityName = "EncSimulator") : m_Time_s(0.0), m_EncoderScaler(1.0), m_ReverseMultiply(1.0), m_Position(0.0)
	{
	}
	virtual void Initialize(const Framework::Base::asset_manager *props = NULL)
	{
		//const Rotary_Properties* rotary_props = dynamic_cast<const Rotary_Properties*>(props);
		//if (rotary_props)
		//{
			//m_DriveTrain.UpdateProps(rotary_props->GetEncoderSimulationProps());
		//	m_EncoderScaler = rotary_props->GetRotaryProps().EncoderToRS_Ratio;
		//}
		m_DriveTrain.UpdateProps(props);
		if (props)
			m_EncoderScaler = props->get_number(properties::registry_v1::csz_EncoderSimulation_EncoderScaler, 1.0);

		#if 0
		//m_Physics.SetMass(68);  //(about 150 pounds)
		m_Physics.SetMass(50);  //adjust this to match latency we observe
		m_Physics.SetFriction(0.8, 0.08);
		#else
		m_Physics.SetMass(m_DriveTrain.GetDriveTrainProps().Wheel_Mass);
		m_Physics.SetFriction(0.8, 0.2);
		m_Physics.SetRadiusOfConcentratedMass(m_DriveTrain.GetDriveTrainProps().DriveWheelRadius);
		#endif
	}
	virtual void UpdateEncoderVoltage(double Voltage)
	{
		double Direction=Voltage<0 ? -1.0 : 1.0;
		Voltage=fabs(Voltage); //make positive
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double *c=Polynomial;
			double x2=Voltage*Voltage;
			double x3=Voltage*x2;
			double x4=x2*x2;
			Voltage = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*Voltage) + c[0]; 
			Voltage *= Direction;
		}
		//From this point it is a percentage (hopefully linear distribution after applying the curve) of the max force to apply... This can be
		//computed from stall torque ratings of motor with various gear reductions and so forth
		//on JVN's spread sheet torque at wheel is (WST / DWR) * 2  (for nm)  (Wheel stall torque / Drive Wheel Radius * 2 sides)
		//where stall torque is ST / GearReduction * DriveTrain efficiency
		//This is all computed in the drive train
		#if 0
		double ForceToApply=m_DriveTrain.GetWheelTorqueFromVoltage(Voltage);
		const double ForceAbsorbed=m_DriveTrain.GetWheelTorqueFromVelocity(m_Physics.GetVelocity());
		ForceToApply-=ForceAbsorbed;
		m_Physics.ApplyFractionalForce(ForceToApply,m_Time_s);
		#else
		double TorqueToApply=m_DriveTrain.GetTorqueFromVoltage_V1(Voltage);
		const double TorqueAbsorbed=m_DriveTrain.INV_GetTorqueFromVelocity(m_Physics.GetVelocity());
		TorqueToApply-=TorqueAbsorbed;
		m_Physics.ApplyFractionalTorque_depreciated(TorqueToApply,m_Time_s,m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		#endif
	}
	virtual double GetEncoderVelocity() const
	{
		#if 0
		static size_t i = 0;
		double Velocity = m_Physics.GetVelocity();
		if (((i++ % 50) == 0) && (Velocity != 0.0))
			printf("Velocty=%f\n", Velocity * m_DriveTrain.GetDriveTrainProps().DriveWheelRadius);
		return 0.0;
		#else
		//return m_Physics.GetVelocity();
		return m_Physics.GetVelocity() * m_DriveTrain.GetDriveTrainProps().DriveWheelRadius * m_ReverseMultiply;
		#endif
	}
	double GetDistance() const
	{
		return m_Position;
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s) {m_Time_s=dTime_s;}
	virtual void TimeChange()
	{
		//m_GetEncoderFirstCall=true;
		const double Ground = 0.0;  //ground in radians
		double FrictionForce = m_Physics.GetFrictionalForce(m_Time_s, Ground);
		m_Physics.ApplyFractionalForce(FrictionForce, m_Time_s);  //apply the friction
		double PositionDisplacement;
		m_Physics.TimeChangeUpdate(m_Time_s, PositionDisplacement);
		m_Position += PositionDisplacement * m_EncoderScaler * m_ReverseMultiply;
	}
	void SetReverseDirection(bool reverseDirection)
	{
		//emulates functionality of the encoder (needed because kids put them in differently)
		m_ReverseMultiply = reverseDirection ? -1.0 : 1.0;
	}
	void SetEncoderScaler(double value) {m_EncoderScaler=value;}  //This helps to simulate differences between sides
	void SetFriction(double StaticFriction,double KineticFriction) {m_Physics.SetFriction(StaticFriction,KineticFriction);}
	virtual void ResetPos()
	{
		m_Physics.ResetVectors();
		m_Position = 0;
	}
	void SetGearReduction(double NewGearing) {m_DriveTrain.SetGearReduction(NewGearing);}
};

class COMMON_API Encoder_Simulator3 : public Encoder_Simulator2
{
private:
	static void TimeChange_UpdatePhysics(double Voltage, Drive_Train_Characteristics dtc,Framework::Base::PhysicsEntity_1D &PayloadPhysics, 
	Framework::Base::PhysicsEntity_1D &WheelPhysics,double Time_s,bool UpdatePayload)
{
	//local function to avoid redundant code
	const double PayloadVelocity=PayloadPhysics.GetVelocity();
	const double WheelVelocity=WheelPhysics.GetVelocity();

	double PositionDisplacement;
	//TODO add speed loss force
	if (UpdatePayload)
	{
		//apply a constant speed loss using new velocity - old velocity
		if (Voltage==0.0)
		{
			const double MaxStop=fabs(PayloadVelocity/Time_s);
			const double SpeedLoss=std::min(5.0,MaxStop);
			const double acceleration=PayloadVelocity>0.0?-SpeedLoss:SpeedLoss;
			//now to factor in the mass
			const double SpeedLossForce = PayloadPhysics.GetMass() * acceleration;
			PayloadPhysics.ApplyFractionalTorque(SpeedLossForce,Time_s);
		}
		PayloadPhysics.TimeChangeUpdate(Time_s,PositionDisplacement);
	}

	#ifdef __USE_PAYLOAD_MASS__
	//Now to add force normal against the wheel this is the difference between the payload and the wheel velocity
	//When the mass is lagging behind it add adverse force against the motor... and if the mass is ahead it will
	//relieve the motor and make it coast in the same direction
	//const double acceleration = dtc.GetWheelAngular_RPS(dtc.GetWheelRPS(PayloadVelocity))-WheelVelocity;
	const double acceleration = dtc.GetWheelAngular_LinearVelocity(PayloadVelocity)-WheelVelocity;
	if (PayloadVelocity!=0.0)
	{
		#if 0
		if (UpdatePayload)
			SmartDashboard::PutNumber("Test",dtc.GetWheelAngular_LinearVelocity(PayloadVelocity));
		if (fabs(acceleration)>50)
			int x=8;
		#endif
		//make sure wheel and payload are going in the same direction... 
		if (PayloadVelocity*WheelVelocity >= 0.0)
		{
			//now to factor in the mass
			const double PayloadForce = WheelPhysics.GetMomentofInertia()*(1.0/dtc.GetDriveTrainProps().TorqueAccelerationDampener)* acceleration;
			WheelPhysics.ApplyFractionalTorque(PayloadForce,Time_s);
		}
		//else
		//	printf("skid %.2f\n",acceleration);
		//if not... we have skidding (unless its some error of applying the force normal) for now let it continue its discourse by simply not applying the payload force
		//this will make it easy to observe cases when skids can happen, but eventually we should compute the kinetic friction to apply
	}
	else
		WheelPhysics.SetVelocity(0.0);
	#endif
}
protected:
	double m_Voltage; //cache voltage for speed loss assistance
	//We are pulling a heavy mass this will present more load on the wheel, we can simulate a bench test vs. an actual run by factoring this in
	static Framework::Base::PhysicsEntity_1D s_PayloadPhysics_Left;
	static Framework::Base::PhysicsEntity_1D s_PayloadPhysics_Right;
	//Cache last state to determine if we are accelerating or decelerating
	static double s_PreviousPayloadVelocity_Left;
	static double s_PreviousPayloadVelocity_Right;
	double m_PreviousWheelVelocity;
	size_t m_EncoderKind;
public:
	//The payload mass is shared among multiple instances per motor, so each instance will need to identify the kind it is
	//multiple instance can read, but only one write per side.  Use ignore and the payload computations will be bypassed
	enum EncoderKind
	{
		eIgnorePayload,  //an easy way to make it on the bench
		eReadOnlyLeft,
		eReadOnlyRight,
		eRW_Left,
		eRW_Right
	};
	Encoder_Simulator3(const char *EntityName="EncSimulator") :
		Encoder_Simulator2(EntityName), m_EncoderKind(eIgnorePayload), m_PreviousWheelVelocity(0.0)
	{
	}
	//has to be late binding for arrays
	void SetEncoderKind(EncoderKind kind)
	{
		//Comment this out to do bench testing
		m_EncoderKind = kind;
	}
	virtual void Initialize(const Framework::Base::asset_manager *props = NULL)
	{

		const double Pounds2Kilograms = 0.453592;
		const double wheel_mass = 1.5;
		const double cof_efficiency = 0.9;
		const double gear_reduction = 1.0;
		const double torque_on_wheel_radius = Inches2Meters(1.8);
		const double drive_wheel_radius = Inches2Meters(4.0);
		const double number_of_motors = 1;
		const double payload_mass = 200 * Pounds2Kilograms;
		const double speed_loss_constant = 0.81;
		const double drive_train_effciency = 0.9;

		const double free_speed_rpm = 263.88;
		const double stall_torque = 34;
		const double stall_current_amp = 84;
		const double free_current_amp = 0.4;
		EncoderSimulation_Props defaults_for_sim3 =
		{
			wheel_mass,   //This is a total mass of all the wheels and gears for one side
			cof_efficiency,  //double COF_Efficiency;
			gear_reduction,  //In reciprocal form of spread sheet   driving gear / driven gear
			torque_on_wheel_radius, //double TorqueAccelerationDampener; //ratio.. where 1.0 is no change
			drive_wheel_radius, //double DriveWheelRadius; //in meters
			number_of_motors, //double NoMotors;  //Used to get total torque
			payload_mass, //double PayloadMass;  //The robot weight in kg
			speed_loss_constant, //double SpeedLossConstant;
			drive_train_effciency, //double DriveTrainEfficiency;
			//struct Motor_Specs
			{
				free_speed_rpm, //double FreeSpeed_RPM;
				stall_torque, //double Stall_Torque_NM;
				stall_current_amp, //double Stall_Current_Amp;
				free_current_amp //double Free_Current_Amp;
			} 
		};
		m_DriveTrain.UpdateProps(defaults_for_sim3);
		Encoder_Simulator2::Initialize(props);
		m_Physics.SetAngularInertiaCoefficient(0.5);  //Going for solid cylinder
		//now to setup the payload physics   Note: each side pulls half the weight
		if (m_EncoderKind == eRW_Left)
			s_PayloadPhysics_Left.SetMass(m_DriveTrain.GetDriveTrainProps().PayloadMass / 2.0);
		else if (m_EncoderKind == eRW_Right)
			s_PayloadPhysics_Right.SetMass(m_DriveTrain.GetDriveTrainProps().PayloadMass / 2.0);
		//TODO see if this is needed
		//m_PayloadPhysics.SetFriction(0.8,0.2);
	}
	virtual void UpdateEncoderVoltage(double Voltage)
	{
		m_Voltage = Voltage;
		//depreciated
		#if 0
		const double Direction = Voltage < 0 ? -1.0 : 1.0;
		Voltage = fabs(Voltage); //make positive
		//Apply the victor curve
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double* c = Polynomial;
			double x2 = Voltage * Voltage;
			double x3 = Voltage * x2;
			double x4 = x2 * x2;
			Voltage = (c[4] * x4) + (c[3] * x3) + (c[2] * x2) + (c[1] * Voltage) + c[0];
			Voltage *= Direction;
		}
		#endif
		//This is line is somewhat subtle... basically if the voltage is in the same direction as the velocity we use the linear distribution of the curve, when they are in
		//opposite directions zero gives the stall torque where we need max current to switch directions (same amount as if there is no motion)
		const double VelocityToUse = m_Physics.GetVelocity() * Voltage > 0 ? m_Physics.GetVelocity() : 0.0;
		double TorqueToApply = m_DriveTrain.GetTorqueFromVelocity(VelocityToUse);
		//Avoid ridiculous division and zero it out... besides a motor can't really turn the wheel in the dead zone range, but I'm not going to factor that in yet
		if ((TorqueToApply != 0.0) && (fabs(TorqueToApply) < m_Physics.GetMass() * 2))
			TorqueToApply = 0.0;
		//Note: Even though TorqueToApply has direction, if it gets saturated to 0 it loses it... ultimately the voltage parameter is sacred to the correct direction
		//in all cases so we'll convert TorqueToApply to magnitude
		m_Physics.ApplyFractionalTorque_depreciated(fabs(TorqueToApply) * Voltage, m_Time_s, m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);

		//Apply the appropriate change in linear velocity to the payload.  Note:  This is not necessarily torque because of the TorqueAccelerationDampener... the torque
		//acts as voltage acts to a circuit... it gives the potential but the current doesn't reflect this right away... its the current, or in our case the velocity
		//change of he wheel itself that determines how much force to apply.
		const double Wheel_Acceleration = (m_Physics.GetVelocity() - m_PreviousWheelVelocity) / m_Time_s;
		//Now to reconstruct the torque Ia, where I = cm R^2 /torquedampner
		const double Wheel_Torque = Wheel_Acceleration * m_Physics.GetMomentofInertia()*(1.0/m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		//Grrr I have to blend this... mostly because of the feedback from the payload... I'll see if there is some way to avoid needing to do this
		const double smoothing_value = 0.75;
		const double BlendTorqueToApply = (Wheel_Torque * smoothing_value) + (TorqueToApply * (1.0 - smoothing_value));
		//TODO check for case when current drive force is greater than the traction
		//Compute the pushing force of the mass and apply it just the same
		if (m_EncoderKind == eRW_Left)
		{
			//Testing
			#if 0
			SmartDashboard::PutNumber("Wheel_Torque_Left", Wheel_Torque);
			SmartDashboard::PutNumber("Motor_Torque_Left", TorqueToApply);
			#endif
			s_PayloadPhysics_Left.ApplyFractionalForce(fabs(m_DriveTrain.GetCurrentDriveForce(BlendTorqueToApply)) * Voltage, m_Time_s);
		}
		else if (m_EncoderKind == eRW_Right)
			s_PayloadPhysics_Right.ApplyFractionalForce(fabs(m_DriveTrain.GetCurrentDriveForce(BlendTorqueToApply)) * Voltage, m_Time_s);

	}
	virtual double GetEncoderVelocity() const
	{
		//we return linear velocity (which is native to the ship)
		return  m_DriveTrain.GetLinearVelocity(m_DriveTrain.GetWheelRPS_Angular(m_Physics.GetVelocity())) * m_ReverseMultiply;
	}
	virtual void TimeChange()
	{
		//For best results if we locked to the payload we needn't apply a speed loss constant here
		#ifndef __USE_PAYLOAD_MASS__
		//first apply a constant speed loss using new velocity - old velocity
		//if (fabs(m_Voltage)<0.65)
		{
			if ((fabs(m_Physics.GetVelocity()) > 0.01))
			{
				const double acceleration = m_DriveTrain.GetDriveTrainProps().SpeedLossConstant * m_Physics.GetVelocity() - m_Physics.GetVelocity();
				//now to factor in the mass
				const double SpeedLossForce = m_Physics.GetMass() * acceleration;
				const double VoltageMagnitue = fabs(m_Voltage);
				m_Physics.ApplyFractionalTorque(SpeedLossForce, m_Time_s, VoltageMagnitue > 0.5 ? (1.0 - VoltageMagnitue) * 0.5 : 0.25);
			}
			else
				m_Physics.SetVelocity(0.0);
		}
		#endif

		double PositionDisplacement;
		m_Physics.TimeChangeUpdate(m_Time_s, PositionDisplacement);
		m_Position += PositionDisplacement * m_EncoderScaler * m_ReverseMultiply;

		//cache velocities
		m_PreviousWheelVelocity = m_Physics.GetVelocity();
		if ((m_EncoderKind == eRW_Left) || (m_EncoderKind == eReadOnlyLeft))
		{
			TimeChange_UpdatePhysics(m_Voltage, m_DriveTrain, s_PayloadPhysics_Left, m_Physics, m_Time_s, m_EncoderKind == eRW_Left);
			if (m_EncoderKind == eRW_Left)
			{
				s_PreviousPayloadVelocity_Left = s_PayloadPhysics_Left.GetVelocity();
				//SmartDashboard::PutNumber("PayloadLeft", Meters2Feet(s_PreviousPayloadVelocity_Left));
				//SmartDashboard::PutNumber("WheelLeft", Meters2Feet(GetEncoderVelocity()));
			}
		}
		else if ((m_EncoderKind == eRW_Right) || (m_EncoderKind == eReadOnlyRight))
		{
			TimeChange_UpdatePhysics(m_Voltage, m_DriveTrain, s_PayloadPhysics_Right, m_Physics, m_Time_s, m_EncoderKind == eRW_Right);
			if (m_EncoderKind == eReadOnlyRight)
			{
				s_PreviousPayloadVelocity_Right = s_PayloadPhysics_Right.GetVelocity();
				//SmartDashboard::PutNumber("PayloadRight", Meters2Feet(s_PreviousPayloadVelocity_Right));
				//SmartDashboard::PutNumber("WheelRight", Meters2Feet(GetEncoderVelocity()));
			}
		}
	}
	virtual void ResetPos()
	{
		Encoder_Simulator2::ResetPos();
		if (m_EncoderKind == eRW_Left)
			s_PayloadPhysics_Left.ResetVectors();
		if (m_EncoderKind == eRW_Right)
			s_PayloadPhysics_Right.ResetVectors();
	}
};
#pragma region _Encoder simulator 3 global variables_
Framework::Base::PhysicsEntity_1D Encoder_Simulator3::s_PayloadPhysics_Left;
Framework::Base::PhysicsEntity_1D Encoder_Simulator3::s_PayloadPhysics_Right;
double Encoder_Simulator3::s_PreviousPayloadVelocity_Left = 0.0;
double Encoder_Simulator3::s_PreviousPayloadVelocity_Right = 0.0;
#pragma endregion

class COMMON_API Potentiometer_Tester3 : public Encoder_Simulator2
{
private:
	std::queue<double> m_Slack;  //when going down this will grow to x frames, and going up with shrink... when not moving it will shrink to nothing
	double m_SlackedValue=0.0; // ensure the pop of the slack only happens in the time change
protected:
	double m_InvEncoderToRS_Ratio;
public:
	Potentiometer_Tester3(const char* EntityName = "PotSimulator3") : Encoder_Simulator2(EntityName), m_InvEncoderToRS_Ratio(1.0)
	{
	}
	virtual void Initialize(const Framework::Base::asset_manager* props = NULL)
	{
		Encoder_Simulator2::Initialize(props);
		//const Rotary_Properties* rotary = dynamic_cast<const Rotary_Properties*>(props);
		//if (rotary)
		//	m_InvEncoderToRS_Ratio = 1.0 / rotary->GetRotaryProps().EncoderToRS_Ratio;
		m_SlackedValue = GetDistance();
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		double Direction = Voltage < 0 ? -1.0 : 1.0;
		Voltage = fabs(Voltage); //make positive
		//Apply the victor curve
		//Apply the polynomial equation to the voltage to linearize the curve
		{
			const double* c = Polynomial;
			double x2 = Voltage * Voltage;
			double x3 = Voltage * x2;
			double x4 = x2 * x2;
			Voltage = (c[4] * x4) + (c[3] * x3) + (c[2] * x2) + (c[1] * Voltage) + c[0];
			Voltage *= Direction;
		}
		//From this point it is a percentage (hopefully linear distribution after applying the curve) of the max force to apply... This can be
		//computed from stall torque ratings of motor with various gear reductions and so forth
		//on JVN's spread sheet torque at wheel is (WST / DWR) * 2  (for nm)  (Wheel stall torque / Drive Wheel Radius * 2 sides)
		//where stall torque is ST / GearReduction * DriveTrain efficiency
		//This is all computed in the drive train
		double TorqueToApply = m_DriveTrain.GetTorqueFromVoltage(Voltage);
		const double TorqueAbsorbed = m_DriveTrain.INV_GetTorqueFromVelocity(m_Physics.GetVelocity());
		TorqueToApply -= TorqueAbsorbed;
		m_Physics.ApplyFractionalTorque_depreciated(TorqueToApply, m_Time_s, m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		//If we have enough voltage and enough velocity the locking pin is not engaged... gravity can apply extra torque
		//if ((fabs(Voltage)>0.04)&&(fabs(m_Physics.GetVelocity())>0.05))
		if (fabs(Voltage) > 0.04)
		{
			// t=Ia 
			//I=sum(m*r^2) or sum(AngularCoef*m*r^2)
			const double Pounds2Kilograms = 0.453592;
			const double mass = 16.27 * Pounds2Kilograms;
			const double r2 = 1.82 * 1.82;
			//point mass at radius is 1.0
			const double I = mass * r2;
			double Torque = I * 9.80665 * -0.55;  //the last scaler gets the direction correct with negative... and tones it down from surgical tubing
			double TorqueWithAngle = cos(GetDistance() * m_InvEncoderToRS_Ratio) * Torque * m_Time_s;  //gravity has most effect when the angle is zero
			//add surgical tubing simulation... this works in both directions  //1.6 seemed closest but on weaker battery, and can't hit 9 feet well
			TorqueWithAngle += sin(GetDistance() * m_InvEncoderToRS_Ratio) * -1.5;
			//The pin can protect it against going the wrong direction... if they are opposite... saturate the max opposing direction
			if (((Torque * TorqueToApply) < 0.0) && (fabs(TorqueWithAngle) > fabs(TorqueToApply)))
				TorqueWithAngle = -TorqueToApply;
			m_Physics.ApplyFractionalTorque_depreciated(TorqueWithAngle, m_Time_s, m_DriveTrain.GetDriveTrainProps().TorqueAccelerationDampener);
		}
	}
	virtual double GetPotentiometerCurrentPosition()
	{
		#if 1
		return m_SlackedValue;
		#else
		return GetDistance();
		#endif
	}
	virtual void TimeChange()
	{
		Encoder_Simulator2::TimeChange();
		double CurrentVelociy = m_Physics.GetVelocity();
		m_Slack.push(GetDistance());
		const size_t MaxLatencyCount = 40;

		if (CurrentVelociy >= 0.0)  //going up or still shrink
		{
			if (m_Slack.size() > (MaxLatencyCount >> 1))
				m_Slack.pop();
			if (!m_Slack.empty())
			{
				m_SlackedValue = m_Slack.front();
				m_Slack.pop();
			}
			else
				m_SlackedValue = GetDistance();
		}
		else  //going down expand
		{
			if (m_Slack.size() >= MaxLatencyCount)
				m_Slack.pop();
			m_SlackedValue = m_Slack.front();
		}
		//SmartDashboard::PutNumber("TestSlack",(double)m_Slack.size());
	}
	virtual void ResetPos()
	{
		Encoder_Simulator2::ResetPos();
		while (!m_Slack.empty())
			m_Slack.pop();
		m_SlackedValue = GetDistance();
	}
};

class COMMON_API Encoder_Tester
{
private:
	#if 0
	//This is still good for a lesser stress (keeping the latency disabled)
	Encoder_Simulator m_LeftEncoder;
	Encoder_Simulator m_RightEncoder;
	#else
	Encoder_Simulator2 m_LeftEncoder;
	Encoder_Simulator2 m_RightEncoder;
	#endif
public:
	Encoder_Tester() : m_LeftEncoder("LeftEncoder"), m_RightEncoder("RightEncoder")
	{
		m_LeftEncoder.Initialize(NULL);
		m_RightEncoder.Initialize(NULL);
	}
	virtual void Initialize(const Framework::Base::asset_manager* props = NULL)
	{
		m_LeftEncoder.Initialize(props);
		m_RightEncoder.Initialize(props);
		//Cool way to add friction uneven to simulate 
		//m_RightEncoder.SetFriction(0.8,0.12);
	}
	virtual void GetLeftRightVelocity(double& LeftVelocity, double& RightVelocity)
	{
		LeftVelocity = m_LeftEncoder.GetEncoderVelocity();
		RightVelocity = m_RightEncoder.GetEncoderVelocity();
	}
	virtual void UpdateLeftRightVoltage(double LeftVoltage, double RightVoltage)
	{
		//LeftVoltage=GetTweakedVoltage(LeftVoltage);
		//RightVoltage=GetTweakedVoltage(RightVoltage);
		m_LeftEncoder.UpdateEncoderVoltage(LeftVoltage);
		m_RightEncoder.UpdateEncoderVoltage(RightVoltage);
	}
	void SetTimeDelta(double dTime_s)
	{
		m_LeftEncoder.SetTimeDelta(dTime_s);
		m_RightEncoder.SetTimeDelta(dTime_s);
	}
	void TimeChange()
	{
		m_LeftEncoder.TimeChange();
		m_RightEncoder.TimeChange();
	}
	void SetLeftRightReverseDirectionEncoder(bool Left_reverseDirection,bool Right_reverseDirection)
	{
		m_LeftEncoder.SetReverseDirection(Left_reverseDirection),m_RightEncoder.SetReverseDirection(Right_reverseDirection);
	}
	void SetLeftRightScaler(double LeftScaler,double RightScaler)
	{
		m_LeftEncoder.SetEncoderScaler(LeftScaler),m_RightEncoder.SetEncoderScaler(RightScaler);
	}
};
}
#else
#pragma endregion

#pragma region _Simulation_
class Potentiometer_Tester4
{
#pragma region _Description_
	//This is specialized for motor simulation turning a loaded mass
	//This version is much simpler than the others in that it works with the motor curves to determine 
	//This will model a solid sphere for the moment of inertia
#pragma endregion
private:
	#pragma region _members_
	Framework::Base::PhysicsEntity_1D m_motor_wheel_model;
	double m_free_speed_rad = 18730 * (1.0 / 60.0) * Pi2; //radians per second of motor
	double m_stall_torque = 0.71;  //Nm
	double m_gear_reduction = (1.0 / 81.0) * (3.0 / 7.0);
	//This will account for the friction, and the load torque lever arm
	double m_gear_box_effeciency = 0.65;
	double m_mass = Pounds2Kilograms(1.0);  //just motor
	double m_RadiusOfConcentratedMass = Inches2Meters(1.12);  //just the motor's diameter
	double m_AngularInertiaCoefficient = 0.5;  //solid cylinder
	//The dead zone defines the opposing force to be added to the mass we'll clip it down to match the velocity
	double m_dead_zone = 0.10; //this is the amount of voltage to get motion can be tested
	double m_anti_backlash_scaler = 0.85; //Any momentum beyond the steady state will be consumed 1.0 max is full consumption
	double m_Time_s = 0.010;
	double m_Heading = 0.0;
	size_t m_InstanceIndex = 0;  //for ease of debugging
	#pragma endregion
public:
	void Initialize(size_t index, const Framework::Base::asset_manager* props = NULL,
		const char* prefix = properties::registry_v1::csz_CommonSwivel_)
	{
		if (props)
		{
			using namespace ::properties::registry_v1;
			std::string constructed_name;
			#define GET_NUMBER(x,y) \
			constructed_name = prefix, constructed_name += csz_##x; \
			y = props->get_number(constructed_name.c_str(), y);

			GET_NUMBER(Pot4_free_speed_rad, m_free_speed_rad);
			GET_NUMBER(Pot4_stall_torque_NM, m_stall_torque);
			GET_NUMBER(Pot4_motor_gear_reduction, m_gear_reduction);
			GET_NUMBER(Pot4_gear_box_effeciency, m_gear_box_effeciency);
			GET_NUMBER(Pot4_mass, m_mass);
			GET_NUMBER(Pot4_RadiusOfConcentratedMass, m_RadiusOfConcentratedMass);
			GET_NUMBER(Pot4_AngularInertiaCoefficient, m_AngularInertiaCoefficient);
			GET_NUMBER(Pot4_dead_zone, m_dead_zone);
			GET_NUMBER(Pot4_anti_backlash_scaler, m_anti_backlash_scaler);
			//GET_NUMBER();
			#undef GET_NUMBER
		}
		m_motor_wheel_model.SetMass(m_mass);
		m_motor_wheel_model.SetAngularInertiaCoefficient(m_AngularInertiaCoefficient);
		m_motor_wheel_model.SetRadiusOfConcentratedMass(m_RadiusOfConcentratedMass);
		m_InstanceIndex = index;
	}
	double GetTorqueFromVoltage(double Voltage)
	{
		//Compute the torque to apply from the speed of a scaled voltage curve of a scaled max torque; 
		//Note: the motor curve max torque is scaled
		//Important: The only one voltage factor here will establish the direction of torque
		const double max_torque = m_stall_torque * Voltage;
		//The speed ratio is a ratio of speed to "max speed" of a voltage scaled motor curve and must only be magnitude
		const double speed_ratio = fabs(Voltage) > 0.0 ?
			std::max(1.0 - (fabs(m_motor_wheel_model.GetVelocity()) / (m_free_speed_rad * fabs(Voltage))), 0.0) : 0.0;
		const double torque = max_torque * speed_ratio;
		return torque;
	}
	double GetMechanicalRestaintTorque(double Voltage, double next_current_velocity)
	{
		double adverse_torque = 0.0;
		const double max_torque = m_stall_torque;
		//Compute adverse torque as the equilibrium sets in to no motion, note the current velocity is motor velocity (no reduction)
		//as this will be easier to read
		if (next_current_velocity != 0)
		{
			//Don't have this mess with small increments, friction will take the final bite here
			if (fabs(next_current_velocity) < 0.01)
			{
				m_motor_wheel_model.ResetVectors();
			}
			else if (m_Time_s > 0.0)  // no division by zero
			{
				//May want to provide properties to control how much dead-zone torque to blend
				//Found a straight constant gives correct result, but have to adjust mass to get correct latency
				#if 0
				const double dead_zone_torque = m_dead_zone * max_torque;
				adverse_torque = (1.0 - fabs(0.6*Voltage)) * dead_zone_torque + (0.4*Voltage)* dead_zone_torque; //this is the minimum start
				#else
				adverse_torque = (m_dead_zone * max_torque); //this is the minimum start
				#endif

				//Evaluate anti backlash, on deceleration the momentum of the payload that exceeds the current steady state
				//will be consumed by the gearing, we apply a scaler to module the strength of this
				//Note: this could work without reduction if next_current_velocity didn't have it as well, but it is easier to read with it in
				const double steady_state_velocity = Voltage * m_free_speed_rad; //for now not factoring in efficiency
				//both the voltage and velocity must be in the same direction... then see if we have deceleration
				if ((steady_state_velocity * next_current_velocity > 0.0) && (fabs(next_current_velocity) > fabs(steady_state_velocity)))
				{
					//factor all in to evaluate note we restore direction at the end
					const double anti_backlash_accel = (fabs(next_current_velocity) - fabs(steady_state_velocity)) / m_Time_s;
					const double anti_backlash_torque = m_motor_wheel_model.GetMomentofInertia() * anti_backlash_accel;
					//Backlash only kicks in when the excess momentum exceed the dead zone threshold
					adverse_torque += anti_backlash_torque * m_anti_backlash_scaler;
				}
				//just compute in magnitude, then restore the opposite direction of the velocity
				const double adverse_accel_limit = fabs(next_current_velocity) / m_Time_s; //acceleration in radians enough to stop motion
				//Now to convert into torque still in magnitude
				const double adverse_torque_limit = m_motor_wheel_model.GetMomentofInertia() * adverse_accel_limit;
				//clip adverse torque as-needed and restore the opposite sign (this makes it easier to add in the next step
				adverse_torque = std::min(adverse_torque_limit, adverse_torque) * ((next_current_velocity > 0) ? -1.0 : 1.0);
			}
		}
		return adverse_torque;
	}
	void UpdatePotentiometerVoltage(double Voltage)
	{
		//Note: these are broken up for SwerveEncoders_Simulator4 to obtain both torques before applying them
		const double voltage_torque = GetTorqueFromVoltage(Voltage);
		m_motor_wheel_model.ApplyFractionalTorque(voltage_torque, m_Time_s);
		const double adverse_torque = GetMechanicalRestaintTorque(Voltage, m_motor_wheel_model.GetVelocity());
		m_motor_wheel_model.ApplyFractionalTorque(adverse_torque, m_Time_s);
	}
	double GetPotentiometerCurrentPosition() const
	{
		return m_Heading;
	}
	void TimeChange()
	{
		double PositionDisplacement;
		m_motor_wheel_model.TimeChangeUpdate(m_Time_s, PositionDisplacement);
		//No normalizing here, we represent the actual position!
		m_Heading += PositionDisplacement * m_gear_reduction;  //velocity is motor, so convert to output rate with the gear reduction
	}
	//This is broken up so that the real interface does not have to pass time
	void SetTimeDelta(double dTime_s)
	{
		m_Time_s = dTime_s;
	}
	void ResetPos()
	{
		m_motor_wheel_model.ResetVectors();
	}
	Framework::Base::PhysicsEntity_1D& GetWheelModel_rw()
	{
		return m_motor_wheel_model;
	}
	double GetMaxSpeed(bool with_reduction = false) const
	{
		return m_free_speed_rad * (with_reduction ? m_gear_reduction : 1.0);
	}
	double GetGearReduction() const
	{
		return m_gear_reduction;
	}
	double GetGearBoxEffeciency() const
	{
		return m_gear_box_effeciency;
	}
};

class SwerveEncoders_Simulator4
{
#pragma region _Description_
	//This works on the same motor concepts as Potentiometer_Tester4 to compute the torque out, but there is an additional load working 
	//against the motor, to work out what this is the interface mechanism will be cleaner to write by providing all at the same time
	//Ultimately we can make use of Torque = F X R equation and apply this to the mass where R is the wheel radius.  To properly simulate
	//We need to move a 2D mass (using 2D physics) and then break down the vectors that are n line with the wheel, and apply.  This can go
	//in either direction.  In the case of opposing torques the magnitude of the forces (before they cancel each other) must be less than
	//weight * static CoF.  The force is torque / wheel radius and the acceleration is known from the torque applied if this is greater
	//clamp down torque to max and put in kinetic friction mode while the forces are clamped.  When it kinetic mode the torque is detached
	//somewhat where the limit is lower (that goes for how much torque can be transferred to mass as well.  Once the forces subside to the
	//lower limit (weight * kinetic CoF) this can flip it back to static CoF mode.
#pragma endregion
private:
	#pragma region _member variables_
	Potentiometer_Tester4 m_Encoders[4];
	Framework::Base::PhysicsEntity_2D m_Payload;
	Framework::Base::PhysicsEntity_1D m_Friction;  //apply friction to skid
	std::function< SwerveVelocities&()> m_CurrentVelocities_callback;
	std::function<SwerveVelocities()> m_VoltageCallback;
	Inv_Swerve_Drive m_Input;
	double m_KineticFriction = 0.50; //this looks about right, but we can try to fix
	class SwerveDrive_Plus : public Swerve_Drive
	{
	private:
		SwerveVelocities m_FromExistingOutput;
	public:
		const SwerveVelocities &UpdateFromExisting(double FWD, double STR, double RCW,const SwerveVelocities &CurrentVelocities)
		{
			using namespace Framework::Base;
			//Essentially there is a common vector that has to work with the wheel angles from current velocities. Because the wheel 
			//angles can be reversed we have to approach this per each wheel angle.  The result is that the magnitude that occupies the
			//same direction will be preserved, and the rest will be discarded to friction. We'll also have to be mindful the direction
			//may be opposite in which case we reverse the sign to keep the same direction.
			//First we let the kinematics we inherit from work like before, and then work from those quad vectors
			Swerve_Drive::UpdateVelocities(FWD, STR, RCW);
			//If the wheel angles are not know, then grab then grab them from existing
			if (IsZero(fabs(FWD) + fabs(STR) + fabs(RCW)))
			{
				for (size_t i = 0; i < 4; i++)
					SetVelocitiesFromIndex(i + 4, CurrentVelocities.Velocity.AsArray[i + 4]);
			}
			for (size_t i = 0; i < 4; i++)
			{
				//This next part is tricky... we take our projected vector and rotate it to a global view using the direction of our actual
				//wheel angle.  Everything that exists in the Y component will be what we consume... X will be discarded
				const Vec2D projected_vector_local(0,GetIntendedVelocitiesFromIndex(i));  //local magnitude pointing up
				const double projected_vector_heading = GetSwerveVelocitiesFromIndex(i);
				//point it in it's current direction
				const Vec2D projected_vector_global = LocalToGlobal(projected_vector_heading,projected_vector_local);
				//Point it back locally to the direction of our actual wheel angle
				const double actual_vector_heading = CurrentVelocities.Velocity.AsArray[i + 4];
				const Vec2D actual_vector_local = GlobalToLocal(actual_vector_heading, projected_vector_global);
				//TODO see if there is ever a need to reverse the direction, there is a case where 2 wheels on one side are
				//opposite causing the velocity to go half speed (not sure if that is here though)
				const double IsReversed = 1.0;

				//We have our velocity in the Y component, now to evaluate if it needs to be reversed
				//const double IsReversed = fabs(projected_vector_heading) - fabs(actual_vector_heading) > Pi / 2 ? -1.0 : 1.0;

				//Now to populate our result
				m_FromExistingOutput.Velocity.AsArray[i] = actual_vector_local.y() * IsReversed;
				m_FromExistingOutput.Velocity.AsArray[i + 4] = CurrentVelocities.Velocity.AsArray[i + 4];  //pedantic
			}
			return m_FromExistingOutput;
		}
	};
	SwerveDrive_Plus m_Output;
	Swerve_Drive m_TestForce; //used to measure acceleration
	double m_WheelDiameter_In=4.0;
	bool m_IsSkidding=false; //cache last state to determine or force normal
	#pragma endregion
public:
	virtual void Initialize(const Framework::Base::asset_manager* props = NULL)
	{
		//TODO get properties for our local members
		m_Input.Init(props);
		m_TestForce.Init(props);
		m_Output.Init(props);
		Framework::Base::asset_manager DefaultMotorProps;
		using namespace properties::registry_v1;
		
		if (props)
		{
			m_WheelDiameter_In = props->get_number(csz_Drive_WheelDiameter_in,m_WheelDiameter_In);
			//This loads up the defaults first and then any scripted override
			for (size_t i = 0; i < 4; i++)
			{
				m_Encoders[i].Initialize(i, &DefaultMotorProps, csz_CommonDrive_);
				m_Encoders[i].Initialize(i, props, csz_CommonDrive_);
			}
		}
		else
		{
			//We only have defaults
			for (size_t i = 0; i < 4; i++)
				m_Encoders[i].Initialize(i, &DefaultMotorProps, csz_CommonDrive_);
		}
		//TODO: We may want to add property to the mass
		m_Payload.SetMass(Pounds2Kilograms(148));
		m_Friction.SetMass(Pounds2Kilograms(148));
		//May want friction coefficients as well
		m_Friction.SetStaticFriction(1.0);  //rated from wheels
		m_Friction.SetKineticFriction(m_KineticFriction);  //not so easy to measure
	}
	void SetVoltageCallback(std::function<SwerveVelocities()> callback)
	{
		m_VoltageCallback = callback;
	}
	void TimeChange(double dTime_s)
	{
		using namespace Framework::Base;
		//Make sure the potentiometer angles are set in current velocities before calling in here
		SwerveVelocities InitialVelocitiesForPayload = m_CurrentVelocities_callback();
		for (size_t i = 0; i < 4; i++)
		{
			const double Voltage = m_VoltageCallback().Velocity.AsArray[i];
			Potentiometer_Tester4& encoder = m_Encoders[i];
			encoder.UpdatePotentiometerVoltage(Voltage);
			//This is our initial velocity to submit to payload
			const double LinearVelocity = encoder.GetWheelModel_rw().GetVelocity() * encoder.GetGearReduction() *
				Inches2Meters(m_WheelDiameter_In * 0.5);

			#if 0
			m_CurrentVelocities_callback().Velocity.AsArray[i] = LinearVelocity;
			return;
			#else
			InitialVelocitiesForPayload.Velocity.AsArray[i] = LinearVelocity;
			#endif
		}
		//Now we can interpolate the velocity vectors with the same kinematics of the drive
		m_Input.InterpolateVelocities(InitialVelocitiesForPayload);

		//printf("force=%.2f\n",ForcesForPayload.Velocity.AsArray[0]);
		//Before we apply the forces grab the current velocities to compute the new forces
		const Vec2D last_payload_velocity = m_Payload.GetLinearVelocity();
		const double last_payload_angular_velocity = m_Payload.GetAngularVelocity();
		const double drive_train_efficiency = m_Encoders[0].GetGearBoxEffeciency();
		//Now to convert the inputs velocities into force, which is the velocity change (per second) and divide out the time
		//to get our acceleration impulse, finally multiple by mass for the total.
		//Note: I may need to consider conservation of momentum here, but for now scaling down using drive train efficiency does add the latency
		Vec2D InputAsForce = (Vec2D(m_Input.GetLocalVelocityX(), m_Input.GetLocalVelocityY()) - last_payload_velocity) / dTime_s * 
			m_Payload.GetMass() * drive_train_efficiency;
		double InputAsTorque = (m_Input.GetAngularVelocity() - last_payload_angular_velocity) / dTime_s * 
			m_Payload.GetMomentofInertia() * drive_train_efficiency;

		double summed_y_force = 0.0;
		//see if we are decelerating
		const Vec2D current_local_velocity(m_Input.GetLocalVelocityX(), m_Input.GetLocalVelocityY());
		const double current_local_speed = current_local_velocity.length();  //aka magnitude
		const double last_payload_speed = last_payload_velocity.length();
		bool IsDecelerating = current_local_speed < last_payload_speed;
		if (IsDecelerating)
		{
			//Make sure the rotation isn't overpowering this indication
			if (fabs(m_Input.GetAngularVelocity()) > fabs(last_payload_angular_velocity))
			{
				//compare apples to apples
				const double angular_velocity_asLinear = m_Input.GetAngularVelocity() * Pi * m_Input.GetDriveProperties().GetTurningDiameter();
				if (fabs(angular_velocity_asLinear) > current_local_speed)
					IsDecelerating = false;
			}
		}
		if ((!IsDecelerating) && (fabs(m_Input.GetAngularVelocity()) < fabs(last_payload_angular_velocity)))
		{
			IsDecelerating = true; //kind of redundant but makes the logic a mirror reflection of above
			if (current_local_speed > last_payload_speed)
			{
				//compare oranges to oranges
				const double angular_velocity_asLinear = m_Input.GetAngularVelocity() * Pi * m_Input.GetDriveProperties().GetTurningDiameter();
				if (current_local_speed > fabs(angular_velocity_asLinear))
					IsDecelerating = false;
			}
		}

		if (!IsDecelerating)
		{
			//Testing the total force start by adding up the total velocity delta
			m_TestForce.UpdateVelocities(m_Input.GetLocalVelocityY() - last_payload_velocity.y(), 
				m_Input.GetLocalVelocityX() - last_payload_velocity.x(), m_Input.GetAngularVelocity() - last_payload_angular_velocity);
			double max_wheel_velocity=0.0;
			for (size_t i = 0; i < 4; i++)
				if (m_TestForce.GetIntendedVelocitiesFromIndex(i) > max_wheel_velocity)
					max_wheel_velocity = m_TestForce.GetIntendedVelocitiesFromIndex(i);
			//sum all 4 wheels... if one of them exceeds force they all do
			summed_y_force = max_wheel_velocity / dTime_s * m_Payload.GetMass() * drive_train_efficiency;
			//printf("|sum=%.2f,y=%.2f|",summed_y_force,InputAsForce.y());
		}
		//Now we can easily evaluate the y-component of force to see if we have a skid, for now we only care about linear motion
		//as rotation with motion only impacts the x-component of the wheels and is addressed below
		//The spin in place may be added here later, but leaving out for now as this is not a typical stress that needs to be simulated
		const bool IsSkidding = fabs(summed_y_force) > m_Friction.GetForceNormal(g,m_IsSkidding?1:0);
		const int FrictionMode = IsSkidding ? 1 : 0;
		//Scale down even further if we are skidding
		if (IsSkidding)
		{
			#if 0
			static int counter = 0;
			printf("[%d]skid [%.2f>%.2f]\n",counter++,InputAsForce.y(),m_Friction.GetForceNormal(g, m_IsSkidding ? 1 : 0));
			#endif
			//the scaled amount is fudged to what looks about right since the summed forces is not quite accurate
			const double scale = m_Friction.GetForceNormal(g, m_IsSkidding ? 1 : 0) / std::max(fabs(summed_y_force)*2.0,m_KineticFriction);
			//This may be a bit more scale than in reality but should be effective
			InputAsForce *= scale;
			InputAsTorque *= scale;
		}
		m_IsSkidding = IsSkidding; //stays skid until we slow down enough below the kinetic friction

		//Apply our torque forces now, before working with friction forces
		//Note: each vector was averaged (and the multiply by 0.25 was the last operation)
		m_Payload.ApplyFractionalForce(InputAsForce, dTime_s);
		m_Payload.ApplyFractionalTorque(InputAsTorque, dTime_s);
		//printf("|pay_t=%.2f|", m_Input.GetAngularVelocity());

		//Check direction, while the controls may not take centripetal forces into consideration, or if in open loop and no feedback
		//It is possible to skid, where the intended direction of travel doesn't match the existing, to simulate, we localize the
		//the existing velocity to the new intended direction of travel, and then apply friction force to the x component which is force normal
		//by the kinetic friction coefficient.  This has to clip as the x components velocity reaches zero
		{
			const Vec2D IntendedVelocities = Vec2D(m_Input.GetLocalVelocityX(),m_Input.GetLocalVelocityY());
			Vec2D IntendedVelocities_normalized(m_Input.GetLocalVelocityX(), m_Input.GetLocalVelocityY());
			double magnitude = IntendedVelocities_normalized.normalize();
			//Use this heading to rotate our velocity to global
			const double IntendedDirection=atan2(IntendedVelocities_normalized[0], IntendedVelocities_normalized[1]);
			const Vec2D local_payload_velocity = m_Payload.GetLinearVelocity();
			//Note: aligned_to_Intended this is the current velocity rotated to align with the intended velocity, when the current velocity direction
			//is mostly the same as the intended, the Y component will consume it.
			const Vec2D aligned_to_Intended_velocity = GlobalToLocal(IntendedDirection, local_payload_velocity);
			const Vec2D IntendedVelocity_fromWheelAngle = GlobalToLocal(IntendedDirection, IntendedVelocities);
			const double Skid_Velocity = aligned_to_Intended_velocity.x();  //velocity to apply friction force to can be in either direction
			m_Friction.SetVelocity(Skid_Velocity);
			const double friction_x = m_Friction.GetFrictionalForce(dTime_s, FrictionMode);
			//printf("|fnx=%.2f|",friction_x);
			double friction_y = 0.0;
			//Test for friction Y (only when controls for Y are idle)
			if (IsZero(IntendedVelocity_fromWheelAngle.y(),0.01))
			{
				double combined_velocity_magnitude=0.0;
				for (size_t i = 0; i < 4; i++)
					combined_velocity_magnitude += m_CurrentVelocities_callback().Velocity.AsArray[i];
				//while we are here... if we are stopped then we need to do the same for the y Component
				if (IsZero(combined_velocity_magnitude,0.01))
				{
					m_Friction.SetVelocity(aligned_to_Intended_velocity.y());
					friction_y = m_Friction.GetFrictionalForce(dTime_s, FrictionMode);
				}
			}
			//now to get friction force to be applied to x component
			const Vec2D global_friction_force(friction_x, friction_y);
			const Vec2D local_friction_force = LocalToGlobal(IntendedDirection, global_friction_force);
			//We can now consume these forces into the payload
			m_Payload.ApplyFractionalForce(local_friction_force, dTime_s);
			//Same goes for the angular velocity as this should not be sliding around
			if (IsZero(m_Input.GetAngularVelocity(), 0.01)&&(m_Payload.GetAngularVelocity()!=0.00))
			{
				const double AngularVelocity = m_Payload.GetAngularVelocity();
				//Avoid small fractions by putting a bite in the threshold
				if (fabs(AngularVelocity) < 0.0001)
				{
					m_Payload.SetAngularVelocity(0.0);
				}
				else
				{
					//The way to think of this is that in space something could spin forever and it is friction that would stop it
					//lack of friction (like a top or something on bearings demonstrates this as well) once the robot spins it is
					//the friction of the wheels that stops the spin.  To be compatible to our units of force (which work with KMS i.e. meters)
					//we convert our angular velocity into linear compute this force and scale back down
					//force = mass * acceleration, acceleration = vel_a-vel_b, vel = distance / time
					//proof... using 1 for time distance a is 3 and distance b is 2.    3-2 (meters) a =1 meters per second square
					//for radians, for now let's assume Pi * D= 2.   1/D(3-2)= 1/D  so we can scale this back down to radians, where D is the turning diameter
					const double turning_diameter = m_Input.GetDriveProperties().GetTurningDiameter();
					const double to_linear = Pi * turning_diameter;
					const double rot_linear = AngularVelocity * to_linear;
					assert(turning_diameter != 0.0);  //sanity check
					m_Friction.SetVelocity(rot_linear);
					const double friction_rot = m_Friction.GetFrictionalForce(dTime_s, FrictionMode) / to_linear; //as radians
					m_Payload.ApplyFractionalTorque(friction_rot, dTime_s);
				}
			}
		}
		// we could m_Payload.TimeChangeUpdate() at some point
		const Vec2D current_payload_velocity = m_Payload.GetLinearVelocity();
		//printf("--%.2f,", current_payload_velocity.y());
		const double current_payload_angular_velocity = m_Payload.GetAngularVelocity();
		const SwerveVelocities &AdjustedToExisting=m_Output.UpdateFromExisting(current_payload_velocity.y(), current_payload_velocity.x(), 
			current_payload_angular_velocity, InitialVelocitiesForPayload);
		//Now we can apply the torque to the wheel
		for (size_t i = 0; i < 4; i++)
		{
			Potentiometer_Tester4& encoder = m_Encoders[i];
			Framework::Base::PhysicsEntity_1D& wheel_model = encoder.GetWheelModel_rw();
			//const double Force = m_Output.GetIntendedVelocitiesFromIndex(i);
			//const double Torque = Force * Inches2Meters(m_WheelDiameter_In * 0.5);
			const double WheelVelocity_fromPayload = AdjustedToExisting.Velocity.AsArray[i] / Inches2Meters(m_WheelDiameter_In * 0.5);
			#if 0
			//To set the velocity directly it works with motor velocity so we'll need to apply the inverse gear ratio
			wheel_model.SetVelocity(WheelVelocity_fromPayload * (1.0/encoder.GetGearReduction()));
			#else
			//Instead of setting the velocity directly, we can apply as a torque, this way if we skid we can apply less force for adjustments
			//Note: In simulation we work with motor velocity, and do not care about where the physical encoder is, because the actual encoder
			//output is linear velocity.  Outside the simulation using a real encoder will need to use the encoder's reduction to have the 
			//same output of linear velocity
			const double motor_velocity = WheelVelocity_fromPayload * (1.0 / encoder.GetGearReduction());
			double torque_load = (motor_velocity - wheel_model.GetVelocity()) / dTime_s * wheel_model.GetMomentofInertia();
			//separating this out for easy of debugging:
			if (IsSkidding)
				torque_load = 0.0; //This is not correct, but will help us visualize the skid this is some impact on the velocity though
			wheel_model.ApplyFractionalTorque(torque_load, dTime_s);

			#endif
			//Now that the velocity has taken effect we can add in the adverse torque
			const double WheelVelocity= wheel_model.GetVelocity() * encoder.GetGearReduction();
			const double LinearVelocity = WheelVelocity * Inches2Meters(m_WheelDiameter_In * 0.5);
			//finally update our odometry, note we work in linear velocity so we multiply by the wheel radius
			m_CurrentVelocities_callback().Velocity.AsArray[i] = LinearVelocity;
		}
	}
	void SetCurrentVelocities_Callback(std::function<SwerveVelocities&()> callback)
	{
		m_CurrentVelocities_callback=callback;
	}
	void ResetPos()
	{
		for (size_t i = 0; i < 4; i++)
			m_Encoders[i].ResetPos();
		m_Payload.ResetVectors();
	}
	const Framework::Base::PhysicsEntity_2D& GetPayload() const
	{
		//read access for advanced sensors to use
		return m_Payload;
	}
};
#endif  //If __UseLegacySimulation__
#pragma endregion


#pragma region _Simulated Odometry Internal_
class SimulatedOdometry_Internal
{
private:
	static inline double NormalizeRotation2(double Rotation)
	{
		const double Pi2 = M_PI * 2.0;
		//Normalize the rotation
		if (Rotation > M_PI)
			Rotation -= Pi2;
		else if (Rotation < -M_PI)
			Rotation += Pi2;
		return Rotation;
	}

	#pragma region _member vars_
	SwerveVelocities m_CurrentVelocities;
	std::function<SwerveVelocities()> m_VoltageCallback;
	double m_maxspeed = Feet2Meters(12.0); //max velocity forward in meters per second
	double m_current_position[4] = {};  //keep track of the pot's position of each angle
	//const Framework::Base::asset_manager *m_properties=nullptr;  <----reserved
	struct properties
	{
		double swivel_max_speed[4];
	} m_bypass_properties;
	#ifdef __UseLegacySimulation__
	Legacy::Potentiometer_Tester2 m_Potentiometers[4]; //simulate a real potentiometer for calibration testing
	std::shared_ptr<Legacy::Encoder_Simulator2> m_Encoders[4];
	#else
	Potentiometer_Tester4 m_Potentiometers[4];
	SwerveEncoders_Simulator4 m_EncoderSim4;

	class AdvancedSensors
	{
	private:
		SimulatedOdometry_Internal* m_pParent;
		Vec2D m_current_position;
		double m_current_heading = 0.0;
	public:
		AdvancedSensors(SimulatedOdometry_Internal* parent) : m_pParent(parent)
		{
		}
		void TimeSlice(double d_time_s)
		{
			using namespace Framework::Base;
			//grab our velocities
			const Framework::Base::PhysicsEntity_2D& payload = m_pParent->m_EncoderSim4.GetPayload();
			const double AngularVelocity = payload.GetAngularVelocity();
			//update our heading
			m_current_heading += AngularVelocity * d_time_s;
			m_current_heading=NormalizeRotation2(m_current_heading);
			//With latest heading we can adjust our position delta to proper heading
			const Vec2D local_velocity = payload.GetLinearVelocity();
			const Vec2D global_velocity = LocalToGlobal(m_current_heading, local_velocity);
			//Now we can advance with the global
			m_current_position += global_velocity * d_time_s;
		}
		void Reset()
		{
			m_current_position = Vec2D(0.0, 0.0);
			m_current_heading = 0.0;
		}
		Vec2D Vision_GetCurrentPosition() const
		{
			return m_current_position;
		}
		double GyroMag_GetCurrentHeading() const
		{
			return m_current_heading;
		}
	} m_AdvancedSensors=this;
	#endif
	bool m_UseBypass = true;
	#pragma endregion
	void SetHooks(bool enabled)
	{
		#ifndef __UseLegacySimulation__
		if (enabled)
		{
			m_EncoderSim4.SetCurrentVelocities_Callback(
				[&]() -> SwerveVelocities &
			{
				return m_CurrentVelocities;
			}
			);
		}
		else
			m_EncoderSim4.SetCurrentVelocities_Callback(nullptr);
		#endif
	}
public:
	void Init(const Framework::Base::asset_manager *props)
	{
		using namespace ::properties::registry_v1;
		//m_properties = props;  <---- reserved... most likely should restrict properties here
		m_UseBypass = props ? props->get_bool(csz_Build_bypass_simulation, m_UseBypass) : true;
		//Since we define these props as we need them the client code will not need default ones
		m_bypass_properties.swivel_max_speed[0] =
			m_bypass_properties.swivel_max_speed[1] =
			m_bypass_properties.swivel_max_speed[2] =
			m_bypass_properties.swivel_max_speed[3] = 8.0;

		//If we are using bypass, we are finished
		if (m_UseBypass)
			return;
		#ifdef __UseLegacySimulation__
		for (size_t i = 0; i < 4; i++)
		{
			//TODO hook up to properties... the defaults do not work with sim 3, as they were made for sim 2
			//I may want to make the defaults for 3 override as well
			const bool Simulator3 = props ? !props->get_bool(csz_EncoderSimulation_UseEncoder2,true) : false;
			if (Simulator3)
			{
				using Encoder_Simulator3 = Legacy::Encoder_Simulator3;
				m_Encoders[i] = std::make_shared<Encoder_Simulator3>();
				Encoder_Simulator3* ptr = dynamic_cast<Encoder_Simulator3 *>(m_Encoders[i].get());

				switch (i)
				{

				case 0:
					ptr->SetEncoderKind(Encoder_Simulator3::eRW_Left);
					break;
				case 1:
					ptr->SetEncoderKind(Encoder_Simulator3::eRW_Right);
					break;
				case 2:
					ptr->SetEncoderKind(Encoder_Simulator3::eReadOnlyLeft);
					break;
				case 3:
					ptr->SetEncoderKind(Encoder_Simulator3::eReadOnlyRight);
					break;
				}
			}
			else
				m_Encoders[i] = std::make_shared<Legacy::Encoder_Simulator2>();
			m_Encoders[i]->Initialize(props);
		}
		#else
		m_EncoderSim4.Initialize(props);
		#endif
		for (size_t i = 0; i < 4; i++)
			m_Potentiometers[i].Initialize(i, props);

		ResetPos();
	}
	void ResetPos()
	{
		if (m_UseBypass)
		{
			m_current_position[0]= m_current_position[1] = m_current_position[2] = m_current_position[3] = 0.0;
		}
		else
		{
			#ifdef __UseLegacySimulation__
			for (size_t i = 0; i < 4; i++)
			{
				m_Encoders[i]->ResetPos();
				m_Potentiometers[i].ResetPos();
			}
			#else
			for (size_t i = 0; i < 4; i++)
				m_Potentiometers[i].ResetPos();
			m_EncoderSim4.ResetPos();
			m_AdvancedSensors.Reset();
			#endif
		}
	}
	void ShutDown()
	{
		SetHooks(false);
	}
	SimulatedOdometry_Internal()
	{
		SetHooks(true);
	}
	~SimulatedOdometry_Internal()
	{
		ShutDown();
	}
	void SetVoltageCallback(std::function<SwerveVelocities()> callback)
	{
		//Input get it from client
		m_VoltageCallback = callback;
		#ifndef __UseLegacySimulation__
		m_EncoderSim4.SetVoltageCallback(callback);
		#endif
	}
	//Run the simulation time-slice
	void TimeSlice(double d_time_s)
	{
		if (m_UseBypass)
		{
			//If only life were this simple, but alas robots are not god-ships
			m_CurrentVelocities = m_VoltageCallback();
			//convert voltages back to velocities
			//for drive this is easy
			for (size_t i = 0; i < 4; i++)
			{
				m_CurrentVelocities.Velocity.AsArray[i] *= m_maxspeed;
			}
			//for position we have to track this ourselves
			for (size_t i = 0; i < 4; i++)
			{
				//go ahead and apply the voltage to the position... this is over-simplified but effective for a bypass
				//from the voltage determine the velocity delta
				const double velocity_delta = m_CurrentVelocities.Velocity.AsArray[i + 4] * m_bypass_properties.swivel_max_speed[i];
				m_current_position[i] = NormalizeRotation2(m_current_position[i] + velocity_delta * d_time_s);
				m_CurrentVelocities.Velocity.AsArray[i + 4] = m_current_position[i];
			}
		}
		else
		{
			SwerveVelocities CurrentVoltage;
			CurrentVoltage = m_VoltageCallback();
			for (size_t i = 0; i < 4; i++)
			{
				//We'll put the pot update first in case the simulation factors in the direction (probably shouldn't matter though)
				m_Potentiometers[i].SetTimeDelta(d_time_s);
				m_Potentiometers[i].UpdatePotentiometerVoltage(CurrentVoltage.Velocity.AsArray[i + 4]);
				m_Potentiometers[i].TimeChange();
				//cannot normalize here
				//m_current_position[i] = NormalizeRotation2(m_Potentiometers[i].GetPotentiometerCurrentPosition());
				m_current_position[i] = m_Potentiometers[i].GetPotentiometerCurrentPosition();
				m_CurrentVelocities.Velocity.AsArray[i + 4] = m_current_position[i];
			}

			#ifdef __UseLegacySimulation__

			for (size_t i = 0; i < 4; i++)
			{
				m_Encoders[i]->SetTimeDelta(d_time_s);
				m_Encoders[i]->UpdateEncoderVoltage(CurrentVoltage.Velocity.AsArray[i]);
				m_Encoders[i]->TimeChange();
				m_CurrentVelocities.Velocity.AsArray[i] = m_Encoders[i]->GetEncoderVelocity();
			}

			#else
			//Ensure potentiometers are updated before calling sim4
			m_EncoderSim4.TimeChange(d_time_s);
			m_AdvancedSensors.TimeSlice(d_time_s);
			#endif
		}
	}
	const SwerveVelocities &GetCurrentVelocities() const
	{
		//Output: contains the current speeds and positions of any given moment of time
		return m_CurrentVelocities;
	}
	#pragma region _advanced_sensor_methods_
	bool Sim_SupportVision() const
	{
		#ifdef __UseLegacySimulation__
		return false;
		#else
		return true;
		#endif
	}
	Vec2D Vision_GetCurrentPosition() const
	{
		#ifdef __UseLegacySimulation__
		return Vec2D(0, 0);
		#else
		return m_AdvancedSensors.Vision_GetCurrentPosition();
		#endif
	}
	bool Sim_SupportHeading() const
	{
		#ifdef __UseLegacySimulation__
		return false;
		#else
		return true;
		#endif
	}
	double GyroMag_GetCurrentHeading() const
	{
		#ifdef __UseLegacySimulation__
		return 0.0;
		#else
		return m_AdvancedSensors.GyroMag_GetCurrentHeading();
		#endif
	}
	#pragma endregion
};
#pragma endregion
#pragma region _wrapper methods_
SimulatedOdometry::SimulatedOdometry()
{
	m_simulator = std::make_shared<SimulatedOdometry_Internal>();
}
void SimulatedOdometry::Init(const Framework::Base::asset_manager* props)
{
	m_simulator->Init(props);
}
void SimulatedOdometry::ResetPos()
{
	m_simulator->ResetPos();
}
void SimulatedOdometry::Shutdown()
{
	m_simulator->ShutDown();
}
void SimulatedOdometry::SetVoltageCallback(std::function<SwerveVelocities()> callback)
{
	m_simulator->SetVoltageCallback(callback);
}
void SimulatedOdometry::TimeSlice(double d_time_s)
{
	m_simulator->TimeSlice(d_time_s);
}
const SwerveVelocities &SimulatedOdometry::GetCurrentVelocities() const
{
	return m_simulator->GetCurrentVelocities();
}
bool SimulatedOdometry::Sim_SupportVision() const
{
	return m_simulator->Sim_SupportVision();
}
Vec2D SimulatedOdometry::Vision_GetCurrentPosition() const
{
	return m_simulator->Vision_GetCurrentPosition();
}
bool SimulatedOdometry::Sim_SupportHeading() const
{
	return m_simulator->Sim_SupportHeading();
}
double SimulatedOdometry::GyroMag_GetCurrentHeading() const
{
	return m_simulator->GyroMag_GetCurrentHeading();
}
#pragma endregion
}}