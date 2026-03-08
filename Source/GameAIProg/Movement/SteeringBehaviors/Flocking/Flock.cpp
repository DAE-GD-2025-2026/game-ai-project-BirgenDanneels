#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);
	OldPositions.SetNum(FlockSize);
	
	//Memory pool: NrOfNeighbors can only be the amount of active agents minus itself
	Neighbors.SetNum(FlockSize - 1);
	
	//Spawn Agents
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride =
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	
	for (int i = 0; i < FlockSize; ++i)
	{
		FVector SpawnPos{
			FMath::RandRange(-WorldSize, WorldSize),
			FMath::RandRange(-WorldSize, WorldSize),
			90.f
		};
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, SpawnPos, FRotator::ZeroRotator, SpawnParams);
		if (IsValid(Agents[i]))
		{
			Agents[i]->SetActorTickEnabled(false); //Manual Tick
			Agents[i]->SetDebugRenderingEnabled(false);
		}
	}
	
	if (DebugRenderSteering)
		Agents[0]->SetDebugRenderingEnabled(true);
	
	//Create behaviors
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>();
	
	pBlendedSteering = std::make_unique<BlendedSteering>(
	std::vector<BlendedSteering::WeightedBehavior>{ 
		{pCohesionBehavior.get(),0.14f},
		{pSeparationBehavior.get(), 0.43f},
		{pVelMatchBehavior.get(), 0.58f},
		{pWanderBehavior.get(), 0.14f},
		{pSeekBehavior.get(), 0.5f}
	});
	
	pPrioritySteering = std::make_unique<PrioritySteering>(
	std::vector<ISteeringBehavior*>{
		pEvadeBehavior.get(),
		pBlendedSteering.get()
	});
	
	//Create cellSpace
	pPartitionedSpace = std::make_unique<CellSpace>(pWorld, WorldSize, WorldSize, NrOfCellsX, NrOfCellsX, FlockSize);
	
	for (int i = 0; i < FlockSize; ++i)
	{
		if (IsValid(Agents[i]))
		{
			Agents[i]->SetSteeringBehavior(pPrioritySteering.get());
			
			//Add agents to space partition
			OldPositions[i] = Agents[i]->GetPosition();
			pPartitionedSpace->AddAgent(*Agents[i]);
			
		}
	}
	
	//Set neighborhood radius to cell size
	NeighborhoodRadius = WorldSize / NrOfCellsX * 1.5;
	
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	// Update Evade target
	if (IsValid(pAgentToEvade) && pEvadeBehavior)
	{
		FSteeringParams evadeTarget{};
		evadeTarget.Position       = pAgentToEvade->GetPosition();
		evadeTarget.Orientation    = pAgentToEvade->GetRotation();
		evadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		pEvadeBehavior->SetTarget(evadeTarget);
	}
	
	for (int Idx{0}; Idx < Agents.Num(); ++Idx)
	{
		
		if (!IsValid(Agents[Idx])) continue;
		
		if (UseSpatialPartitioning)
		{
			//Space Partitioning
			pPartitionedSpace->RegisterNeighbors(*Agents[Idx], NeighborhoodRadius);
			
			Agents[Idx]->Tick(DeltaTime);

			pPartitionedSpace->UpdateAgentCell(*Agents[Idx], OldPositions[Idx]);
			OldPositions[Idx] = Agents[Idx]->GetPosition();
		}
		else
		{
			//Normal Flock
			RegisterNeighbors(Agents[Idx]);
			Agents[Idx]->Tick(DeltaTime); 	
		}
	}
}

void Flock::RenderDebug()
{
	if (DebugRenderPartitions)
		pPartitionedSpace->RenderCells();
	
	if (DebugRenderNeighborhood)
		RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();
	
		bool Temp {DebugRenderSteering};
		if (ImGui::Checkbox("Debug Steering", &Temp))
			Agents[0]->SetDebugRenderingEnabled(Temp);
		DebugRenderSteering = Temp;
		
		Temp = DebugRenderNeighborhood;
		ImGui::Checkbox("Debug Neighborhood", &Temp);
		DebugRenderNeighborhood = Temp;
		
		ImGui::Spacing();
		ImGui::Text("Partitioning");
		ImGui::Spacing();
		
		Temp = UseSpatialPartitioning;
		ImGui::Checkbox("Use Partitioning", &Temp);
		UseSpatialPartitioning = Temp;
		
		Temp = DebugRenderPartitions;
		ImGui::Checkbox("Debug Partitions", &Temp);
		DebugRenderPartitions = Temp;
		
		if (!UseSpatialPartitioning)
			DebugRenderPartitions = false;
		
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion",
			 pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			 [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation",
			 pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
			 [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Velocity Match",
			 pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight, 0.f, 1.f,
			 [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
			 pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight, 0.f, 1.f,
			 [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight = InVal; }, "%.2f");
		
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			 pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight, 0.f, 1.f,
			 [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight = InVal; }, "%.2f");
		
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (UseSpatialPartitioning)
	{
		pPartitionedSpace->RenderNeighborHood(*Agents[0], NeighborhoodRadius);
		return;
	}
	
	RegisterNeighbors(Agents[0]);
	
	const float DebugCircleHeight{90};
	
	// Draw Neighborhood Radius
	DrawDebugCircle(pWorld,
		FVector( Agents[0]->GetPosition(), DebugCircleHeight),
		NeighborhoodRadius,
		32,
		FColor::Black,
		false,
		-1,
		0,
		4.f,
		FVector(1,0,0),
		FVector(0,1,0),
		false );
	
	// Draw a circle for every agent in the neighborhood
	const float NeighborDebugRadius{20};
	
	for (int idx{0}; idx < NrOfNeighbors; ++idx)
	{
		DrawDebugCircle(pWorld,
			FVector( Neighbors[idx]->GetPosition(), DebugCircleHeight),
			NeighborDebugRadius,
			32,
			FColor::Green,
			false,
			-1,
			0,
			6.f,
			FVector(1,0,0),
			FVector(0,1,0),
			false );
	} 
}

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	NrOfNeighbors = 0;
	
	for (ASteeringAgent* other : Agents)
	{
		if (other == pAgent || !IsValid(other)) continue;
		float Distance = FVector2D::Distance(pAgent->GetPosition(), other->GetPosition());
		if (Distance < NeighborhoodRadius)
		{
			Neighbors[NrOfNeighbors] = other;
			++NrOfNeighbors;
		}
	}
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D AveragePosition = FVector2D::ZeroVector;

	for (int i = 0; i < GetNrOfNeighbors(); ++i)
		AveragePosition += GetNeighbors()[i]->GetPosition();

	if (GetNrOfNeighbors() > 0)
		AveragePosition /= static_cast<float>(GetNrOfNeighbors());

	return AveragePosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D AverageVelocity = FVector2D::ZeroVector;

	for (int i = 0; i < GetNrOfNeighbors(); ++i)
		AverageVelocity += GetNeighbors()[i]->GetLinearVelocity();

	if (GetNrOfNeighbors() > 0)
		AverageVelocity /= static_cast<float>(GetNrOfNeighbors());

	return AverageVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}

