#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
    steeringOutput Steering = {};
    Steering.LinearVelocity = Target.Position - Agent.GetPosition();

    return Steering;
}


// TODO: Do the Week01 assignment :^)