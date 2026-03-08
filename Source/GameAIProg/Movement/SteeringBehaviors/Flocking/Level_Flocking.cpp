// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_Flocking.h"

// Sets default values
ALevel_Flocking::ALevel_Flocking()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_Flocking::BeginPlay()
{
	Super::BeginPlay();

	TrimWorld->SetTrimWorldSize(1500.f);
	TrimWorld->bShouldTrimWorld = true;

	//Create Agent to evade
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride =
		ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	
	pAgentToEvade = GetWorld()->SpawnActor<ASteeringAgent>(EvadeAgentClass, FVector{0.f, 0.f, 90.f}, FRotator::ZeroRotator, SpawnParams);
	
	if (IsValid(pAgentToEvade))
	{
		pEvadeAgentWander = std::make_unique<Wander>();
		pAgentToEvade->SetSteeringBehavior(pEvadeAgentWander.get());
		pAgentToEvade->SetIsAutoOrienting(true);
	}
	
	
	
	pFlock = TUniquePtr<Flock>(
		new Flock(
			GetWorld(),
			SteeringAgentClass,
			FlockSize,
			TrimWorld->GetTrimWorldSize(),
			pAgentToEvade,
			true)
			);
}

// Called every frame
void ALevel_Flocking::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	pFlock->ImGuiRender(WindowPos, WindowSize);
	pFlock->Tick(DeltaTime);
	pFlock->RenderDebug();
	if (bUseMouseTarget)
		pFlock->SetTarget_Seek(MouseTarget);
}

