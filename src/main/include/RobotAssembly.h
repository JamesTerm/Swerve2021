#include <memory>

class RobotAssem_Internal;
class RobotAssem
{
public:
  RobotAssem();
  //Note: Timed Robot presents it this way so we make our own time delta within the implementation
  void AutonomousInit();
  void AutonomousPeriodic();
  void TeleopInit();
  void TeleopPeriodic();
  void DisabledInit();
  void DisabledPeriodic();
  void TestInit();
  void TestPeriodic();
  void SimulationInit ();
  void SimulationPeriodic ();
private:
  std::shared_ptr<RobotAssem_Internal> m_robot;
};
