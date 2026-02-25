
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	
	for (const BlendedSteering::WeightedBehavior& weightedBehaviour : WeightedBehaviors)
	{
		if (!weightedBehaviour.pBehavior || weightedBehaviour.Weight <= 0.f) continue;
		
		SteeringOutput steering = weightedBehaviour.pBehavior->CalculateSteering(DeltaT, Agent);
		BlendedSteering.LinearVelocity += steering.LinearVelocity * weightedBehaviour.Weight;
		BlendedSteering.AngularVelocity += steering.AngularVelocity * weightedBehaviour.Weight;
	}
	
	if (Agent.GetDebugRenderingEnabled())
	{
		float DebugArrowLength = 150.f;
        
		// Draw direction arrow
		DrawDebugDirectionalArrow(Agent.GetWorld(),
			FVector(Agent.GetPosition(), 0),
			FVector(Agent.GetPosition(), 0) + FVector(BlendedSteering.LinearVelocity.GetClampedToSize(-1, 1), 0) * DebugArrowLength,
			20,
			FColor::Green);
	}

	return BlendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}