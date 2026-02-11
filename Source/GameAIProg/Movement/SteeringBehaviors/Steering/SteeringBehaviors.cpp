#include "SteeringBehaviors.h"

#include "SkeletalMeshAttributes.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
    SteeringOutput Steering = {};
    Steering.LinearVelocity = Target.Position - Agent.GetPosition();
    
    //Debug drawing
    // DrawDebugDirectionalArrow(Agent.GetWorld(), Agent.GetPosition(), 
    //     Agent.GetPosition() + FVector2D(Steering.LinearVelocity * Agent.GetMaxLinearSpeed())
    //     , 10.f
    //     , FColor::Red);
    
    return Steering;
}


//FLEE
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
    return Steering;
}

Arrive::Arrive(ASteeringAgent& Agent)
{
    m_CachedMaxVelocity = Agent.GetMaxLinearSpeed();
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    
    //Set Direction
    Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
    
    //Calculate Speed
    double distance { Agent.GetPosition().Length()};
    if (distance > m_SlowRadius)
        Agent.SetMaxLinearSpeed(m_CachedMaxVelocity);
    else if (distance > m_TargetRadius)
        Agent.SetMaxLinearSpeed(0);
    else
    {
        float speedMultiplier = (distance - m_TargetRadius) / (m_SlowRadius - m_TargetRadius);
        
        Agent.SetMaxLinearSpeed(m_CachedMaxVelocity * speedMultiplier);
    }
    
    return Steering;
}

// TODO: Do the Week01 assignment :^)
