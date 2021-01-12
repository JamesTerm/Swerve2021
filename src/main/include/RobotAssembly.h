#include <memory>

class TeleOp_Internal;
class TeleOp
{
public:
  TeleOp();
  void Init();
  //Note: Timed Robot presents it this way so we make our own time delta within the implementation
  void TimeSlice();
private:
  std::shared_ptr<TeleOp_Internal> m_tester;
};
