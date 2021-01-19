#pragma once
#include <string>
#include <memory>

namespace Module
{
	namespace Output
	{

//All vendor controllers here

class SparkMaxItem;
class SparkMaxController
{
public:
	SparkMaxController(int _channel, std::string _name, bool _reversed);
	SparkMaxController(int _channel);
	//encoder methods
	void SetDistancePerPulse(double distancePerPulse);
	double GetEncoderPosition() const;
	double GetEncoderVelocity() const;
	void ResetEncoderValue();
	//motor methods
	double Get() const;
	int GetPolarity() const;
	void Reset();
	void Set(double val) ;
	void Stop() ;
	std::string GetName() const;
private:
    std::shared_ptr<SparkMaxItem> m_controller;
};

class TalonSRXItem;
class TalonSRX_Controller
{
public:
	TalonSRX_Controller(int channel, std::string name, bool reversed, bool enableEncoder);
	TalonSRX_Controller(int _channel, bool enableEncoder=false);
	//encoder methods
	void SetDistancePerPulse(double distancePerPulse);
	double GetEncoderPosition() const;
	double GetEncoderVelocity() const;
	//motor methods
	double Get() const;
	void Set(double val);
	void Stop();
private:
	std::shared_ptr<TalonSRXItem> m_controller;
};

    }
}