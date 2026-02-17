#include "SteeringBehaviors.h"

#include "SkeletalMeshAttributes.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "Runtime/Engine/Internal/Kismet/BlueprintTypeConversions.h"

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

Arrive::Arrive(const ASteeringAgent* Agent)
{
    m_CachedMaxVelocity = Agent->GetMaxLinearSpeed();
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    
    //Set Direction
    Steering.LinearVelocity = Target.Position - Agent.GetPosition();
    
    //Calculate Speed
    double distance {(Target.Position - Agent.GetPosition()).Length()};
    if (distance > m_SlowRadius)
        Agent.SetMaxLinearSpeed(m_CachedMaxVelocity);
    else if (distance < m_TargetRadius)
        Agent.SetMaxLinearSpeed(0);
    else
    {
        float speedMultiplier = (distance - m_TargetRadius) / (m_SlowRadius - m_TargetRadius);
        
        Agent.SetMaxLinearSpeed(m_CachedMaxVelocity * speedMultiplier);
    }
    
    if (Agent.GetDebugRenderingEnabled())
    {
        //Draw Slow radius
        DrawDebugCircle(Agent.GetWorld(),
            FVector(Agent.GetPosition(), 0),
            m_SlowRadius,
            32,
            FColor::Blue,
            false,
            -1,
            0,
            2.f,
            FVector(1,0,0),
            FVector(0,1,0),
            false );
        // Draw Target radius
        DrawDebugCircle(Agent.GetWorld(),
            Agent.GetActorLocation(),
            m_TargetRadius,
            32,
            FColor::Red,
            false,
            -1,
            0,
            2.f,
            FVector(1,0,0),
            FVector(0,1,0),
            false );
    }
    
    return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    
    // const FVector2D dstnVector{ Target.Position - Agent.GetPosition() };
    // const FVector2D forwardVector{ cos(Agent.GetRotation()), sin(Agent.GetRotation()) };
    // const float angle1 = FMath::Atan2(dstnVector.X, dstnVector.Y);
    // const float angle2 = FMath::Atan2(forwardVector.X, forwardVector.Y);
    // float angle = FMath::RadiansToDegrees(angle1 - angle2);
    // if(angle > 180.0f) angle -= 360.0f; else if(angle < -180.0f) angle += 360.0f;
    //
    // Steering.AngularVelocity = angle;
    
    return Steering;
}

// TODO: Do the Week01 assignment :^)
