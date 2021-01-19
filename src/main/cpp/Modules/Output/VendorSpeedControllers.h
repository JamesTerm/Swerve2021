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

	void SetDistancePerPulse(double distancePerPulse);
	double GetEncoderPosition() const;
	double GetEncoderVelocity() const;
    
	double Get() const;
	int GetPolarity() const;
	void Reset();
	void Set(double val) ;
	void Stop() ;
	std::string GetName() const;
    void ResetEncoderValue(); 
private:
    std::shared_ptr<SparkMaxItem> m_controller;
};

    }
}