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
    double Distance {(Target.Position - Agent.GetPosition()).Length()};
    if (Distance > m_SlowRadius)
        Agent.SetMaxLinearSpeed(m_CachedMaxVelocity);
    else if (Distance < m_TargetRadius)
        Agent.SetMaxLinearSpeed(0);
    else
    {
        float SpeedMultiplier = (Distance - m_TargetRadius) / (m_SlowRadius - m_TargetRadius);
        
        Agent.SetMaxLinearSpeed(m_CachedMaxVelocity * SpeedMultiplier);
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
        FVector(Agent.GetPosition(), 0),
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

    // Direction to target
    FVector ToTarget = FVector(Target.Position, 0) - Agent.GetActorLocation();
    ToTarget.Z = 0.f; // ignore vertical rotation
    ToTarget.Normalize();

    // Agent forward vector
    FVector Forward = Agent.GetActorForwardVector();
    Forward.Z = 0.f;
    Forward.Normalize();

    // Compute signed angle (rad)
    const float AngleDiffRad = FMath::Atan2(ToTarget.Y, ToTarget.X) - FMath::Atan2(Forward.Y, Forward.X);

    // Convert to degrees
    float AngleDiffDeg = FMath::RadiansToDegrees(AngleDiffRad);
    AngleDiffDeg = FMath::UnwindDegrees(AngleDiffDeg);

    // Desired angular velocity
    const float MaxAngularSpeed = Agent.GetMaxAngularSpeed();
    Steering.AngularVelocity = FMath::Clamp(AngleDiffDeg, -MaxAngularSpeed, MaxAngularSpeed);

    // Draw debug
    if (Agent.GetDebugRenderingEnabled())
    {
        const float DebugArrowLength = 150.f;
        
        // Draw forward vector
        DrawDebugDirectionalArrow(Agent.GetWorld(),
            FVector(Agent.GetPosition(), 0),
            FVector(Agent.GetPosition(), 0) + Forward * DebugArrowLength,
            20,
            FColor::Blue);
        
        //Draw desired vector
        DrawDebugDirectionalArrow(Agent.GetWorld(),
            FVector(Agent.GetPosition(), 0),
            FVector(Agent.GetPosition(), 0) + ToTarget * DebugArrowLength,
            20,
            FColor::Red);
        
        //Draw angle difference
        FVector TextLocation = Agent.GetActorLocation() + FVector(0,0,50);
        DrawDebugString(Agent.GetWorld(), 
            TextLocation, 
            FString::Printf(TEXT("Angle: %.1f"), 
                AngleDiffDeg), 
                nullptr, 
                FColor::Yellow, 
                0.f, 
                true);

    }
    
    return Steering;
}

SteeringOutput Persuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    
    const float TimeToReachT{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) / Agent.GetMaxLinearSpeed()};
    FVector2D predictedPos{Target.Position + (Target.LinearVelocity * TimeToReachT)};

    Steering.LinearVelocity = predictedPos - Agent.GetPosition();

    if (Agent.GetDebugRenderingEnabled())
    {
        float DebugArrowLength = 150.f;
        
        // Draw direction arrow
        DrawDebugDirectionalArrow(Agent.GetWorld(),
            FVector(Agent.GetPosition(), 0),
            FVector(Agent.GetPosition(), 0) + FVector(Steering.LinearVelocity.GetClampedToSize(-1, 1), 0) * DebugArrowLength,
            20,
            FColor::Red);
    }
 
    return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    
    const float TimeToReachT{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) / Agent.GetMaxLinearSpeed()};
    FVector2D predictedPos{Target.Position + (Target.LinearVelocity * TimeToReachT)};

    Steering.LinearVelocity = Agent.GetPosition() - predictedPos;

    if (Agent.GetDebugRenderingEnabled())
    {
        float DebugArrowLength = 150.f;
        
        // Draw direction arrow
        DrawDebugDirectionalArrow(Agent.GetWorld(),
            FVector(Agent.GetPosition(), 0),
            FVector(Agent.GetPosition(), 0) + FVector(Steering.LinearVelocity.GetClampedToSize(-1, 1), 0) * DebugArrowLength,
            20,
            FColor::Red);
    }
 
    return Steering;
}


// TODO: Do the Week01 assignment :^)
