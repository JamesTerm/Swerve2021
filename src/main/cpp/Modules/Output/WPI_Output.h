#pragma once
#include <memory>
#include <functional>
//need SwerveVelocities
#include "../Robot/DriveKinematics/Vehicle_Drive.h"
//Use asset manager to configure assignments, and encoder settings
#include "../../Base/AssetManager.h"

namespace Module
{
	namespace Output
	{

class WPI_Output_Internal;
class WPI_Output
{
public:
    WPI_Output();
    void Init(const Framework::Base::asset_manager *props=nullptr);
    //We pull voltage from swerve robot
    void TimeSlice(double dTime_s);
    //WPI has a SimulationPeriodic() which allows updates to encoders here
    void SimulatorTimeSlice(double dTime_s);
    //Optional: use our simulated odometry for simulator time slice
    void SetSimOdometry(std::function<Robot::SwerveVelocities ()> callback);
    //Optional: use our simulated odometry for gyro
    void SetSimOdometry_heading(std::function<double ()> callback);
    
   	//Input: grab the voltages from each rotary system
	void SetVoltageCallback(std::function<Robot::SwerveVelocities ()> callback);
   	//Output: contains the current speeds and positions of any given moment of time
	const Robot::SwerveVelocities &GetCurrentVelocities() const;
private:
    std::shared_ptr<WPI_Output_Internal> m_WPI;
};

    }
}