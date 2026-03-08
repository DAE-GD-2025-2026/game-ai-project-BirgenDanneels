#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	if (pFlock->GetNrOfNeighbors() == 0)
		return SteeringOutput{};

	FSteeringParams NewTarget{};
	NewTarget.Position = pFlock->GetAverageNeighborPos();
	SetTarget(NewTarget);
	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	if (pFlock->GetNrOfNeighbors() == 0)
		return Steering;

	FVector2D AgentPos = pAgent.GetPosition();
	for (int i = 0; i < pFlock->GetNrOfNeighbors(); ++i)
	{
		FVector2D ToAgent = AgentPos - pFlock->GetNeighbors()[i]->GetPosition();
		float Dist = ToAgent.Length();
		if (Dist > 0.f)
			Steering.LinearVelocity += ToAgent.GetSafeNormal() / Dist;
	}

	// Normalize vector then scale to max speed
	Steering.LinearVelocity = Steering.LinearVelocity.GetSafeNormal();

	Steering.IsValid = true;
	return Steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	if (pFlock->GetNrOfNeighbors() == 0)
		return Steering;

	FVector2D AverageVelocity = pFlock->GetAverageNeighborVelocity();;

	Steering.LinearVelocity = AverageVelocity.GetSafeNormal();

	Steering.IsValid = true;
	return Steering;
}
