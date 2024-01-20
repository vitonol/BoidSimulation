//  Copyright Vitalii Voronkin. All Rights Reserved.


#include "BFlock.h"

#include "Components/BoxComponent.h"
#include "Components/InstancedStaticMeshComponent.h"

ABFlock::ABFlock()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	Box = CreateDefaultSubobject<UBoxComponent>(FName("Box"));
	Box->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Box->SetBoxExtent(FVector(1000.f));
	Box->SetHiddenInGame(false);
	SetRootComponent(Box);
	
	ISMComp = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("ISM"));
	ISMComp->SetupAttachment(Box);
	ISMComp->SetMobility(EComponentMobility::Static);
	ISMComp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	ISMComp->SetGenerateOverlapEvents(false);
}

void ABFlock::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
}

void ABFlock::BeginPlay()
{
	Super::BeginPlay();
	
	// No need to reserve since SetNum does it
	// BoidCurrentLocations.Reserve(NumOfInstances);
	// BoidsVelocities.Reserve(NumOfInstances);
	
	BoidsVelocities.SetNum(NumInstances);
	BoidCurrentLocations.SetNum(NumInstances);

	if ( GetInstanceCount() != 0) ISMComp->ClearInstances();

	TArray<FTransform> Transforms;
	Transforms.Reserve(NumInstances);
	
	for (int32 i = 0; i < NumInstances; i++)    
	{
		//Alternatively spawn boids in the box
		// FVector SpawnPoint = RandomPointInBoundingBox(Box->GetCenterOfMass(), Box->GetUnscaledBoxExtent());
		FVector SpawnPoint = FVector(FMath::RandRange(-1.f, 1.f) * BoidsSpawnRadius,
										FMath::RandRange(-1.f, 1.f) * BoidsSpawnRadius,
										FMath::RandRange(-1.f, 1.f) * BoidsSpawnRadius);

		FRotator RandomRotator = FRotator(FMath::RandRange(0.f, 359.998993f),
													FMath::RandRange(0.f, 359.998993f),
													FMath::RandRange(0.f, 359.998993f));

		FTransform Transform(RandomRotator, SpawnPoint, FVector(1.f, 0.5f, 0.3f));
		Transforms.Add(Transform);		
	}
	ISMComp->AddInstances(Transforms, false, false);
	
}

void ABFlock::AddInstances(int32 NumToAdd)
{
	SetActorTickEnabled(false);
	if (NumToAdd <= 0) return;

	TArray<FTransform> InstancesToAdd;
	InstancesToAdd.Reserve(NumToAdd);

	for (int i = 0; i < NumToAdd; ++i)
	{
		// Create new instances and add their transforms to the array
		// You can adjust the position and rotation based on your requirements
		FTransform NewTransform = FTransform::Identity;
		InstancesToAdd.Add(NewTransform);
	}

	ISMComp->AddInstances(InstancesToAdd, false, false);
	SetActorTickEnabled(true);
	// NumInstances = GetInstanceCount();
}

void ABFlock::RemoveInstances(int32 NumToRemove)
{
	SetActorTickEnabled(false);
	if (NumToRemove <= 0) return;

	int32 CurrentInstanceCount = GetInstanceCount();
	int32 NumToRemoveClamped = FMath::Clamp(NumToRemove, 0, GetInstanceCount());

	TArray<int32> InstancesToRemove;
	InstancesToRemove.Reserve(NumToRemoveClamped);

	// Collect indices of instances to remove
	for (int i = 0; i < NumToRemoveClamped; ++i)
	{
		InstancesToRemove.Add(CurrentInstanceCount - 1 - i);
	}
	
	ISMComp->RemoveInstances(InstancesToRemove);
	SetActorTickEnabled(true);

	// NumInstances = GetInstanceCount();
}

FVector ABFlock::GetVectorArrayAverage(const TArray<FVector>& Vectors)
{
	FVector Sum(0.f);
	FVector Average(0.f);

	if (Vectors.Num() > 0)
	{
		for (int32 VecIdx=0; VecIdx<Vectors.Num(); VecIdx++)
		{
			Sum += Vectors[VecIdx];
		}
		Average = Sum / ((float)Vectors.Num());
	}
	return Average;
}

void ABFlock::SetRandomColor()
{
	TArray<float> CustomData;
	CustomData.Init(0.f, 3);
	CustomData[0] = FMath::RandRange(0.95f, 1.f);
	CustomData[1] = FMath::RandRange(0.24f, 0.3f);
	CustomData[2] = FMath::RandRange(0.28f, 0.32f);
	
	for (int32 i = 0; i < NumInstances; i++)
	{
		ISMComp->SetCustomData(i, CustomData, false);
	}
	ISMComp->MarkRenderStateDirty();
}

int ABFlock::GetInstanceCount()
{
	if (ISMComp != nullptr) return ISMComp->GetInstanceCount();
	return 0;
}

// FVector ABFlock::RandomPointInBoundingBox(const FVector Center, const FVector HalfSize)
// {
// 	const FVector BoxMin = Center - HalfSize;
// 	const FVector BoxMax = Center + HalfSize;
// 	return FMath::RandPointInBox(FBox(BoxMin, BoxMax));
// }

FVector ABFlock::Separate(TArray<FVector>& BoidsPositions, int CurrentIndex)
{
	FVector Steering = FVector::ZeroVector;
	int32 FlockCount = 0.0f;
	FVector SeparationDirection = FVector::ZeroVector;
	float ProximityFactor = 0.0f;
	
	FTransform InstanceTransform;
	ISMComp->GetInstanceTransform(CurrentIndex,InstanceTransform);
	const FVector ForwardVector = InstanceTransform.GetUnitAxis(EAxis::X);
	
	const FVector CurrentPosition = BoidsPositions[CurrentIndex];
	
	//get separation steering force for each of the boid's flockmates
	for (FVector OtherPosition : BoidsPositions)
	{
		// Ignore self and filer out irrelevant far away birds
		const float ProximityDistance = FVector::Dist(CurrentPosition, OtherPosition);
		if (CurrentPosition == OtherPosition || ProximityDistance > ProximityRadius)
		{
			continue;
		}
		
		SeparationDirection = CurrentPosition - OtherPosition;
		SeparationDirection = SeparationDirection.GetSafeNormal();
		ProximityFactor = 1.0 - (SeparationDirection.Size() / ProximityRadius);

		if (ProximityFactor < 0.1f)
		{
			continue;
		}

		// filter other boids which outside of the field of view
		//TODO Impmlement view angle for other functions as well, and test
		if (FVector::DotProduct(ForwardVector, (OtherPosition - CurrentPosition).GetSafeNormal()) <= -1.0f)
		{
			continue;
		}

		//add steering force for each nearby boid
		Steering += (ProximityFactor * SeparationDirection);
		FlockCount++;
	}

	if (FlockCount > 0)
	{
		Steering /= FlockCount;
		Steering.GetSafeNormal() -= BoidsVelocities[CurrentIndex].GetSafeNormal();
		Steering *= SeparationStrength;
		return Steering;
	}
	else
	{
		return FVector::ZeroVector;
	}
}

FVector ABFlock::Align(TArray<FVector>& BoidsPositions, int CurrentIndex)
{
	FVector Steering = FVector::ZeroVector;
	int32 FlockCount = 0.0f;

	FTransform InstanceTransform;
	ISMComp->GetInstanceTransform(CurrentIndex,InstanceTransform);
	const FVector ForwardVector = InstanceTransform.GetUnitAxis(EAxis::X);	
	
	const FVector CurrentPosition = BoidsPositions[CurrentIndex];
	for (FVector OtherPosition : BoidsPositions)
	{		
		const float ProximityDistance = FVector::Dist(CurrentPosition, OtherPosition);
		
		// Ignore self and filter out other birds that are far away and irrelevant
		if (OtherPosition == CurrentPosition || ProximityDistance > ProximityRadius)
		{
			continue;
		}

		// filter other birds which are outside of the field of view angle
		if (FVector::DotProduct(ForwardVector, (OtherPosition - CurrentPosition).GetSafeNormal()) <= 0.5f)
		{
			continue;
		}
		
		FVector OtherVelocity = (CurrentPosition - OtherPosition) / GetWorld()->GetDeltaSeconds();
		Steering += OtherVelocity.GetSafeNormal();
		FlockCount++;
	}
	
	if (FlockCount > 0)
	{
		//get alignment force to average flock direction
		Steering /= FlockCount;
		Steering.GetSafeNormal() -= BoidsVelocities[CurrentIndex].GetSafeNormal();
		Steering *= AlignmentStrength;
		return Steering;
	}
	else
	{
		return FVector::ZeroVector;
	}
}

FVector ABFlock::Cohere(TArray<FVector>& BoidsPositions, int CurrentIndex)
{
	FVector Steering = FVector::ZeroVector;
	int32 FlockCount = 0.0f;
	FVector AveragePosition = FVector::ZeroVector;

	FTransform InstanceTransform;
	ISMComp->GetInstanceTransform(CurrentIndex,InstanceTransform);
	const FVector ForwardVector = InstanceTransform.GetUnitAxis(EAxis::X);

	const FVector CurrentPosition = BoidsPositions[CurrentIndex];
	for (FVector OtherPosition : BoidsPositions)
	{
		// Filter out birds that are far away and irrelevant
		const float ProximityDistance = FVector::Dist(CurrentPosition, OtherPosition);
		if (ProximityDistance > ProximityRadius )
		{
			continue;
		}
		// ignore self
		if (OtherPosition == CurrentPosition)
		{
			continue;
		}
		// filter other boids which outside of the field of view
		//TODO Impmlement view angle for other functions as well, and test
		if (FVector::DotProduct(ForwardVector, (OtherPosition - CurrentPosition).GetSafeNormal()) <= -0.5f)
		{
			continue; //flockmate is outside viewing angle, disregard this flockmate and continue the loop
		}
		AveragePosition += OtherPosition;
		FlockCount++;
	}
	
	if (FlockCount > 0)
	{
		AveragePosition /= FlockCount;
		Steering = AveragePosition - CurrentPosition;
		Steering.GetSafeNormal() -= BoidsVelocities[CurrentIndex].GetSafeNormal();
		Steering *= CohesionStrength;
		return Steering;
	}
	else
	{
		return FVector::ZeroVector;
	}
}

void ABFlock::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	TArray<FTransform> TempBuffer;
	TempBuffer.Empty(NumInstances);
	for (int32 i = 0; i < NumInstances; i++)
	{
		FVector Acceleration = FVector::ZeroVector;
		
		FTransform InstanceTransform;
		ISMComp->GetInstanceTransform(i,InstanceTransform);
		FVector CurrentBoidLoc = InstanceTransform.GetLocation();

		FVector CurrentBoidVelocity = BoidsVelocities[i];

		//Update Stored Locations
		BoidCurrentLocations[i] = CurrentBoidLoc;

		//update position
		InstanceTransform.SetLocation(CurrentBoidLoc + (CurrentBoidVelocity * DeltaTime));
		//update rotation
		InstanceTransform.SetRotation(CurrentBoidVelocity.ToOrientationQuat());
		
		//apply steering to acceleration vector
		Acceleration += Separate(BoidCurrentLocations,i);
		Acceleration += Align(BoidCurrentLocations,i);
		Acceleration += Cohere(BoidCurrentLocations,i);

		//update velocities
		BoidsVelocities[i] += (Acceleration * DeltaTime);
		BoidsVelocities[i] = BoidsVelocities[i].GetClampedToSize(400.f, 800.f);

		TempBuffer.Add(InstanceTransform);
		//Alternative is below, outside of the loop
		// ISMComp->UpdateInstanceTransform(i, InstanceTransform, false, false, false);

#if DEBUG_ENABLED
		if (bToggleProximityDebug)
			DrawDebugSphere(GetWorld(), CurrentBoidLoc, ProximityRadius, 4, FColor::Purple );
#endif
	}
	
	//Optimization? 
	ISMComp->BatchUpdateInstancesTransforms(0, TempBuffer);
	
#if DEBUG_ENABLED
	FString str = TEXT("Average Velocity: " ) + GetVectorArrayAverage(BoidsVelocities).ToString();
	GEngine->AddOnScreenDebugMessage(-1, 0.f, FColor::Red, *str);
#endif
	
	SetRandomColor(); // Moving to blueprints

	if ( GetInstanceCount() > 0) ISMComp->MarkRenderTransformDirty();
	
	// Possibly usefull
	// ISMComp->ReleasePerInstanceRenderData();
	
#if DEBUG_ENABLED
	FString msg1 = FString::Printf(TEXT("InstanceCount: %i"), GetInstanceCount() );
	GEngine->AddOnScreenDebugMessage(-1, 0.f, FColor::White, *msg1);
#endif
}

