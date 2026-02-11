#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	Arrive(ASteeringAgent & Agent);
	
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
	
private:
	
	float m_CachedMaxVelocity{};
	
	float m_SlowRadius{10.f};
	float m_TargetRadius{1.f};
};
//draw debug directional arrow