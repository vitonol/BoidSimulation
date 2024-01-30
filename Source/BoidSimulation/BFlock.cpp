//  Copyright Vitalii Voronkin. All Rights Reserved.


#include "BFlock.h"

#include "Components/InstancedStaticMeshComponent.h"

// Declare performance profiling stats
DECLARE_STATS_GROUP(TEXT("BoidProfiling"), STATGROUP_BoidProfiling, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Simulate (GT)"), STAT_Simulate_GameThread, STATGROUP_BoidProfiling);
DECLARE_CYCLE_STAT(TEXT("Simulate (Task)"), STAT_Simulate_WorkerThread, STATGROUP_BoidProfiling);

ABFlock::ABFlock()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
	
	ISMComp = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("ISM"));
	SetRootComponent(ISMComp);
	ISMComp->SetMobility(EComponentMobility::Static);
	ISMComp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	ISMComp->SetGenerateOverlapEvents(false);
	
	InitialSpawnScale = FVector{1.f};
}

void ABFlock::BeginPlay()
{
	Super::BeginPlay();
	
	// Update buffers and clear instances
	UpdateBuffers(NumInstances);
	InstanceIndices.SetNum(NumInstances);

	if ( GetInstanceCount() != 0) ISMComp->ClearInstances();

	// Spawn instances with random transforms
	TArray<FTransform> Transforms;
	Transforms.Reserve(NumInstances);

	float SpawnBounds = SpreadRadius * 0.5f;
	
	for (int32 i = 0; i < NumInstances; i++)    
	{
		FVector SpawnPoint = FVector(FMath::RandRange(-1.f, 1.f) * SpawnBounds,
										FMath::RandRange(-1.f, 1.f) * SpawnBounds,
										FMath::RandRange(-1.f, 1.f) * SpawnBounds);

		FRotator RandomRotator = FRotator(0.f,
										FMath::RandRange(0.f, 359.998993f),
										0.f);

		FTransform Transform(RandomRotator, SpawnPoint, InitialSpawnScale);
		Transforms.Add(Transform);
	}
	InstanceIndices = ISMComp->AddInstances(Transforms, true, false);
}

void ABFlock::UpdateBuffers(int32 NewCount)
{
	BoidCurrentLocations.SetNum(NewCount);
	BoidsVelocities.SetNum(NewCount);
}

void ABFlock::AddInstances(int32 NumToAdd)
{
	// SetActorTickEnabled(false);
	if (NumToAdd <= 0) return;
	
	UpdateBuffers(GetInstanceCount() + NumToAdd);
	
	TArray<FTransform> InstancesToAdd;
	InstancesToAdd.Reserve(NumToAdd);
	
	for (int i = 0; i < NumToAdd; ++i)
	{
		// Create new instances and add their transforms to the array
		FTransform NewTransform = FTransform::Identity;
		InstancesToAdd.Add(NewTransform);
	}

	InstanceIndices.Append(ISMComp->AddInstances(InstancesToAdd, true));
	
	// SetActorTickEnabled(true);
	NumInstances = GetInstanceCount();
}

//TODO Rework the logic for below function
void ABFlock::RemoveInstances(int32 NumToRemove)
{
	SetActorTickEnabled(false);
	if (ensure(NumToRemove <= 0)) return;
	
	const int32 NewInstanceCount = FMath::Max(0, GetInstanceCount() - NumToRemove);

	// Ensure the indices are within bounds
	if (NumToRemove > InstanceIndices.Num())
	{
		// Print an error or use UE_LOG to indicate the issue
		UE_LOG(LogTemp, Error, TEXT("Attempting to remove more instances than available."));
		return;
	}
	UpdateBuffers(NewInstanceCount);
	
	TArray<int32> InstancesToRemove;
	InstancesToRemove.Reserve(NumToRemove);

	// Collect indices of instances to remove
	for (int32 i = InstanceIndices.Num() - 1; i >= InstanceIndices.Num() - 1 - NumToRemove ; --i)
	{
		InstancesToRemove.Add(InstanceIndices[i]);
		UE_LOG(LogTemp, Log, TEXT("Removing: %i"), InstanceIndices[i] );
	}
	if (ISMComp->RemoveInstances(InstancesToRemove))
	{
		UE_LOG(LogTemp, Log, TEXT("Succesfully Removed: %i instances"), NumToRemove);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Could not remove instances"));
	}
	SetActorTickEnabled(true);

	NumInstances = GetInstanceCount();
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

int ABFlock::GetInstanceCount()
{
	if (ISMComp != nullptr) return ISMComp->GetInstanceCount();
	return 0;
}

// Linearly interpolates between two normalized vectors using spherical linear interpolation
UE_NODISCARD FORCEINLINE FVector LerpNormals(const FVector& A, const FVector& B, const double Alpha)
{
	const FQuat RotationDifference = FQuat::FindBetweenNormals(A, B);

	FVector Axis; double Angle;
	RotationDifference.ToAxisAndAngle(Axis, Angle);

	return FQuat{Axis, Angle * Alpha}.RotateVector(A);
}


// Calculate separation steering force
FVector ABFlock::Separate(TArray<FVector>& BoidsPositions, int CurrentIndex) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Separate"), STAT_Separate, STATGROUP_BoidProfiling);
	
	FVector Steering = FVector::ZeroVector;
	int32 FlockCount = 0;
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
		const float ProximityDistanceSquared = FVector::DistSquared(CurrentPosition,OtherPosition);
		if (CurrentPosition == OtherPosition || ProximityDistanceSquared > FMath::Square(ProximityRadius))
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
		Steering *= SeparationStrength;
		return Steering;
	}
	else return FVector::ZeroVector;
}

FVector ABFlock::Separate(const TArrayView<FVector>& Velocities, const int32 CurrentIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Separate_Rel"), STAT_Separate, STATGROUP_BoidProfiling);

	FTransform CurrentTransform{NoInit};
	ISMComp->GetInstanceTransform(CurrentIndex, CurrentTransform);
	
	FVector Steering = Velocities[CurrentIndex];
	
	for (const int32 OtherBoidIndex : OtherRelevantBoidIndices)
	{
		FTransform OtherTransform{NoInit};
		ISMComp->GetInstanceTransform(OtherBoidIndex, OtherTransform);

		const FVector Translation = FTransform::SubtractTranslations(CurrentTransform, OtherTransform);
		if (UNLIKELY(Translation.SizeSquared() < UE_DOUBLE_KINDA_SMALL_NUMBER)) continue;

		const double Dist = Translation.Size();

		Steering += Translation * (((1.0 - (Dist / ProximityRadius)) / Dist) * SeparationStrength);
	}
	Steering.Normalize();
	
	return  Steering;
}

FVector ABFlock::Align(TArray<FVector>& BoidsPositions, const int32 CurrentIndex) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Align"), STAT_Align, STATGROUP_BoidProfiling);
	
	FVector Steering = FVector::ZeroVector;
	int32 FlockCount = 0;

	FTransform InstanceTransform;
	ISMComp->GetInstanceTransform(CurrentIndex,InstanceTransform);
	const FVector ForwardVector = InstanceTransform.GetUnitAxis(EAxis::X);

	const FVector CurrentPosition = BoidsPositions[CurrentIndex];
	for (FVector OtherPosition : BoidsPositions)
	{		
		const float ProximityDistanceSquared = FVector::DistSquared(CurrentPosition,OtherPosition);
		
		// Ignore self and filter out other birds that are far away and irrelevant
		if (OtherPosition == CurrentPosition || ProximityDistanceSquared > FMath::Square(ProximityRadius))
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
		Steering *= AlignmentStrength;
		return Steering;
	}
	else return FVector::ZeroVector;
}

// Calculate aligning steering force of relevant boids only
FVector ABFlock::Align(const TArrayView<FVector>& Velocities, const int32 CurrentIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Align_rel"), STAT_Align, STATGROUP_BoidProfiling);

	FVector Steering = FVector::ZeroVector;
	if (OtherRelevantBoidIndices.IsEmpty()) return Steering;

	for (const int32 OtherBoidIndex : OtherRelevantBoidIndices)
	{
		Steering += BoidCurrentLocations[OtherBoidIndex];
	}

	Steering /= OtherRelevantBoidIndices.Num();
	Steering.Normalize();

	const double Alpha = FMath::Min(1.0, static_cast<double>(OtherRelevantBoidIndices.Num()) / 15.0) * AlignmentStrength;
	Steering = LerpNormals(Velocities[CurrentIndex], Steering, Alpha);

	return Steering;
}

// Calculate Grouping-up steering force
FVector ABFlock::Cohere(TArray<FVector>& BoidsPositions, int CurrentIndex) const 
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Cohere"), STAT_Cohere, STATGROUP_BoidProfiling);
	
	FVector Steering = FVector::ZeroVector;
	int32 FlockCount = 0;
	FVector AveragePosition = FVector::ZeroVector;

	FTransform InstanceTransform;
	ISMComp->GetInstanceTransform(CurrentIndex,InstanceTransform);
	const FVector ForwardVector = InstanceTransform.GetUnitAxis(EAxis::X);

	const FVector CurrentPosition = BoidsPositions[CurrentIndex];
	for (FVector OtherPosition : BoidsPositions)
	{
		// Filter out birds that are far away and irrelevant
		const float ProximityDistanceSquared = FVector::DistSquared(CurrentPosition, OtherPosition);
		if (ProximityDistanceSquared > FMath::Square(ProximityRadius))
		{
			continue;
		}
		// ignore self
		if (OtherPosition == CurrentPosition)
		{
			continue;
		}
		// filter other boids which outside of the field of view
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
	else return FVector::ZeroVector;
}

FVector ABFlock::Cohere(const TArrayView<FVector>& Velocities, const int32 CurrentIndex, const TConstArrayView<int32>& OtherRelevantBoidIndices) const
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Cohere_rel"), STAT_Cohere, STATGROUP_BoidProfiling);

	FVector Steering = FVector::ZeroVector;

	if (OtherRelevantBoidIndices.IsEmpty()) return Steering;
	
	for (const int32 OtherBoidIndex : OtherRelevantBoidIndices)
	{
		FTransform OtherTransform{NoInit};
		ISMComp->GetInstanceTransform(OtherBoidIndex, OtherTransform);

		Steering += OtherTransform.GetTranslation();
	}
	Steering /= OtherRelevantBoidIndices.Num();

	const FVector DirToAverageLocation = (Steering - BoidCurrentLocations[CurrentIndex].GetSafeNormal());

	const double Alpha = FMath::GetMappedRangeValueClamped<double, double>({0.0, 15.0}, {0.0, CohesionStrength}, static_cast<double>(OtherRelevantBoidIndices.Num()));
	Steering = LerpNormals(Velocities[CurrentIndex], DirToAverageLocation, Alpha);
	
	return Steering;
}

// Change Direction when boids get near the bounds
void ABFlock::Redirect(FVector& Direction, const int32 CurrentIndex)
{
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Redirect"), STAT_Redirect, STATGROUP_BoidProfiling);
	
	FTransform Transform{NoInit};
	ISMComp->GetInstanceTransform(CurrentIndex,Transform);

	if (Transform.GetTranslation().SizeSquared() > FMath::Square(SpreadRadius - ProximityRadius - UE_DOUBLE_KINDA_SMALL_NUMBER))
	{
		const double Dist = Transform.GetTranslation().Size();
		const FVector Dir = Transform.GetTranslation() / Dist;

		// Calculate CrossProduct
		// FVector RightAxis = Direction ^ Dir;
		FVector RightAxis = Direction.Cross(Dir);

		FVector TargetDirection;
		if(RightAxis.SizeSquared() > UE_DOUBLE_KINDA_SMALL_NUMBER)
		{
			RightAxis = RightAxis.GetUnsafeNormal();
			TargetDirection = RightAxis.RotateAngleAxisRad(UE_DOUBLE_PI / 2.0, Dir).RotateAngleAxisRad(-UE_DOUBLE_PI / 4.0, RightAxis);
		}
		else
		{
			TargetDirection = -Dir;
		}

		const double Alpha = FMath::GetMappedRangeValueUnclamped<double, double>({SpreadRadius - ProximityRadius - UE_DOUBLE_KINDA_SMALL_NUMBER, SpreadRadius},
																				{0.0, 1.0}, Dist);
		Direction = LerpNormals(Direction, TargetDirection, Alpha);
	}
}

void ABFlock::Tick(float DeltaTime)
{
	SCOPE_CYCLE_COUNTER(STAT_Simulate_GameThread);
	Super::Tick(DeltaTime);

	// Fetch Boid Locations
	ParallelFor(NumInstances, [&](const int32 i) -> void
	{
		FTransform InstanceTransform{NoInit};
		ISMComp->GetInstanceTransform(i,InstanceTransform);
		
		//Update Stored Locations
		BoidCurrentLocations[i] = InstanceTransform.GetLocation();
	});

	//Multithreading allowing multiple iterations to be executed concurrently
	ParallelFor(NumInstances, [&](const int32 i) -> void
	{
		FVector Acceleration = FVector::ZeroVector;

		//TODO Pre-filter near flockmates and only pass relevant array of boids for force calculations
		//apply steering forces to acceleration vector
		Acceleration += Separate(BoidCurrentLocations,i);
		Acceleration += Align(BoidCurrentLocations,i);
		Acceleration += Cohere(BoidCurrentLocations,i);
		
		//Keep boids inside the bounds
		Redirect(BoidsVelocities[i], i);

		//update velocities
		BoidsVelocities[i] += (Acceleration * DeltaTime);
		BoidsVelocities[i] = BoidsVelocities[i].GetClampedToSize(MinMovementSpeed, MaxMovementSpeed);
	});

	// Move Boids
	TArray<FTransform> TempBuffer;
	TempBuffer.SetNum(NumInstances);
	
	ParallelFor(NumInstances, [&](const int32 i) -> void
	{
		FVector CurrentBoidVelocity = BoidsVelocities[i];
		FVector NewBoidLocation = BoidCurrentLocations[i] + (CurrentBoidVelocity * DeltaTime);
		TempBuffer[i] = FTransform(CurrentBoidVelocity.ToOrientationQuat(), NewBoidLocation);		
	});
	ISMComp->BatchUpdateInstancesTransforms(0, TempBuffer, false, true);
}


	


