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
    Steering.LinearVelocity.Normalize();
    
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

//FLEE
//*******
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
    Steering.LinearVelocity.Normalize();
    
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

//ARRIVE
//*******
Arrive::Arrive(const ASteeringAgent* Agent)
{
    m_CachedMaxVelocity = Agent->GetMaxLinearSpeed();
    m_HasCachedMaxVel = true;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    if (!m_HasCachedMaxVel)
    {
        m_CachedMaxVelocity = Agent.GetMaxLinearSpeed();
        m_HasCachedMaxVel = true;
    }
    
    SteeringOutput Steering = {};
    
    //Set Direction
    Steering.LinearVelocity = Target.Position - Agent.GetPosition();
    
    //Calculate Speed
    double Distance {(Target.Position - Agent.GetPosition()).Length()};
    float SpeedMultiplier{};
    
    if (Distance > m_SlowRadius) SpeedMultiplier = 1.f;
    else if (Distance < m_TargetRadius) SpeedMultiplier = 0;
    else SpeedMultiplier = (Distance - m_TargetRadius) / (m_SlowRadius - m_TargetRadius);
    
    Agent.SetMaxLinearSpeed(m_CachedMaxVelocity * SpeedMultiplier);
    
    Steering.LinearVelocity.Normalize();
    
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
        
        float DebugArrowLength = 150.f;
        
        DrawDebugDirectionalArrow(Agent.GetWorld(),
           FVector(Agent.GetPosition(), 0),
           FVector(Agent.GetPosition(), 0) + FVector(Steering.LinearVelocity.GetClampedToSize(-1, 1), 0) * DebugArrowLength * SpeedMultiplier,
           20,
           FColor::Red);
    }
    
    return Steering;
}

void Arrive::SetTargetRadius(float TargetRadius)
{
    m_TargetRadius = TargetRadius;
    m_SlowRadius = TargetRadius * 5;
}

void Arrive::ResetMaxVelocity(ASteeringAgent& Agent)
{
    if (m_HasCachedMaxVel)
        Agent.SetMaxLinearSpeed(m_CachedMaxVelocity);
}

//FACE
//*******
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

//PERSUIT
//*******
SteeringOutput Persuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    
    const float TimeToReachT{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) / Agent.GetMaxLinearSpeed()};
    FVector2D predictedPos{Target.Position + (Target.LinearVelocity * TimeToReachT)};

    Steering.LinearVelocity = predictedPos - Agent.GetPosition();
    Steering.LinearVelocity.Normalize();

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

//EVADE
//*******
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    SteeringOutput Steering = {};
    const float Distance{ static_cast<float>((Target.Position - Agent.GetPosition()).Length()) };
    
    if (Distance < EvasionRadius)
    {
        const float TimeToReachT{ Distance / Agent.GetMaxLinearSpeed()};
        FVector2D predictedPos{Target.Position + (Target.LinearVelocity * TimeToReachT)};

        Steering.LinearVelocity = Agent.GetPosition() - predictedPos;
        Steering.LinearVelocity.Normalize();
    }
    else Steering.IsValid = false;
    if (Agent.GetDebugRenderingEnabled())
    {
        float DebugArrowLength = 150.f;
        
        // Draw direction arrow
        DrawDebugDirectionalArrow(Agent.GetWorld(),
            FVector(Agent.GetPosition(), 0),
            FVector(Agent.GetPosition(), 0) + FVector(Steering.LinearVelocity.GetClampedToSize(-1, 1), 0) * DebugArrowLength,
            20,
            FColor::Red);
        
        // Draw Evade radius
        DrawDebugCircle(Agent.GetWorld(),
            FVector( Agent.GetPosition(), 0),
            EvasionRadius,
            32,
            FColor::Blue,
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

//WANDER
//*******
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
    //Random Angle on Circle
    const float randomAngle = FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange);
    m_WanderAngle += randomAngle;

    FVector2D angleVect{ FVector2D(sin(m_WanderAngle), cos(m_WanderAngle)) };
    angleVect *= m_Radius;

    //Position of point on circle
    const FVector2D forwardVector{ Agent.GetActorForwardVector() };
    const FVector2D posPoint{ Agent.GetPosition() + ((forwardVector * m_OffsetDistance ) + angleVect)};

    Target.Position = posPoint;

    if (Agent.GetDebugRenderingEnabled())
    {
        // Draw Debug circle
        DrawDebugCircle(Agent.GetWorld(),
            FVector( Agent.GetPosition() + (forwardVector * m_OffsetDistance ), 0),
            m_Radius,
            32,
            FColor::Blue,
            false,
            -1,
            0,
            2.f,
            FVector(1,0,0),
            FVector(0,1,0),
            false );
        
        // Draw Circlepoint
        DrawDebugCircle(Agent.GetWorld(),
            FVector( posPoint, 0),
            10.f,
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
    
    return Seek::CalculateSteering(DeltaT, Agent);
}
