//  Copyright Vitalii Voronkin. All Rights Reserved.


#include "BFlock.h"

#include "Components/InstancedStaticMeshComponent.h"

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
}

void ABFlock::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	DrawDebugSphere(GetWorld(), GetActorLocation(), SpreadRadius, 8, FColor::Orange);
}

void ABFlock::BeginPlay()
{
	Super::BeginPlay();
	
	// BoidsVelocities.SetNum(NumInstances);
	// BoidCurrentLocations.SetNum(NumInstances);
	UpdateBuffers(NumInstances);
	InstanceIndecies.SetNum(NumInstances);

	if ( GetInstanceCount() != 0) ISMComp->ClearInstances();

	TArray<FTransform> Transforms;
	Transforms.Reserve(NumInstances);

	float SpawnBounds = SpreadRadius * 0.5f;
	
	for (int32 i = 0; i < NumInstances; i++)    
	{
		//Alternatively spawn boids in the box
		// FVector SpawnPoint = RandomPointInBoundingBox(Box->GetCenterOfMass(), Box->GetUnscaledBoxExtent());
		FVector SpawnPoint = FVector(FMath::RandRange(-1.f, 1.f) * SpawnBounds,
										FMath::RandRange(-1.f, 1.f) * SpawnBounds,
										FMath::RandRange(-1.f, 1.f) * SpawnBounds);

		FRotator RandomRotator = FRotator(0.f,
										FMath::RandRange(0.f, 359.998993f),
										0.f);

		FTransform Transform(RandomRotator, SpawnPoint);
		Transforms.Add(Transform);
	}
	InstanceIndecies = ISMComp->AddInstances(Transforms, true, false);
	
}

void ABFlock::AddInstances(int32 NumToAdd)
{
	// SetActorTickEnabled(false);
	if (NumToAdd <= 0) return;
	
	UpdateBuffers(GetInstanceCount() + NumToAdd);
	// BoidCurrentLocations.SetNum(NewInstanceCount);
	// BoidsVelocities.SetNum(NewInstanceCount);

	TArray<FTransform> InstancesToAdd;
	InstancesToAdd.Reserve(NumToAdd);

	for (int i = 0; i < NumToAdd; ++i)
	{
		// Create new instances and add their transforms to the array
		FTransform NewTransform = FTransform::Identity;
		InstancesToAdd.Add(NewTransform);
	}

	InstanceIndecies.Append(ISMComp->AddInstances(InstancesToAdd, true));
	
	// SetActorTickEnabled(true);
	NumInstances = GetInstanceCount();
}

void ABFlock::UpdateBuffers(int32 NewCount)
{
	BoidCurrentLocations.SetNum(NewCount);
	BoidsVelocities.SetNum(NewCount);
}

void ABFlock::RemoveInstances(int32 NumToRemove)
{
	SetActorTickEnabled(false);
	if (ensure(NumToRemove <= 0)) return;

	const int32 InitialInstanceCount = GetInstanceCount();
	const int32 NewInstanceCount = FMath::Max(0, InitialInstanceCount - NumToRemove);

	// Ensure the indices are within bounds
	if (NumToRemove > InstanceIndecies.Num())
	{
		// Print an error or use UE_LOG to indicate the issue
		UE_LOG(LogTemp, Error, TEXT("Attempting to remove more instances than available."));
		return;
	}
	UpdateBuffers(NewInstanceCount);
	
	// BoidCurrentLocations.SetNum(NewInstanceCount);
	// BoidsVelocities.SetNum(NewInstanceCount);
	
	TArray<int32> InstancesToRemove;
	InstancesToRemove.Reserve(NumToRemove);

	// Collect indices of instances to remove
	for (int32 i = InstanceIndecies.Num() - 1; i >= InstanceIndecies.Num() - 1 - NumToRemove ; --i)
	{
		InstancesToRemove.Add(InstanceIndecies[i]);
		UE_LOG(LogTemp, Log, TEXT("Removing: %i"), InstanceIndecies[i] );
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
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Separate"), STAT_Separate, STATGROUP_BoidProfiling);
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
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Align"), STAT_Align, STATGROUP_BoidProfiling);
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
	DECLARE_SCOPE_CYCLE_COUNTER(TEXT("Cohere"), STAT_Cohere, STATGROUP_BoidProfiling);
	
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

UE_NODISCARD FORCEINLINE FVector LerpNormals(const FVector& A, const FVector& B, const double Alpha)
{
	const FQuat RotationDifference = FQuat::FindBetweenNormals(A, B);

	FVector Axis; double Angle;
	RotationDifference.ToAxisAndAngle(Axis, Angle);

	return FQuat{Axis, Angle * Alpha}.RotateVector(A);
}

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

	// TArray<FTransform> TempBuffer;
	// TempBuffer.Reserve(NumInstances);
	for (int32 i = 0; i < NumInstances; i++)
	{
		FVector Acceleration = FVector::ZeroVector;
		
		FTransform InstanceTransform;
		ISMComp->GetInstanceTransform(i,InstanceTransform);
		FVector CurrentBoidLoc = InstanceTransform.GetLocation();

		if (i >= BoidsVelocities.Num())
		{
			// Print an error or use UE_LOG to indicate the issue
			UE_LOG(LogTemp, Error, TEXT("BoidsVelocities index out of bounds: %d"), i);
			continue;
		}
		
		FVector CurrentBoidVelocity = BoidsVelocities[i];

		//Update Stored Locations
		BoidCurrentLocations[i] = CurrentBoidLoc;

		//Update position
		FVector NewBoidLocation = CurrentBoidLoc + (CurrentBoidVelocity * DeltaTime);		

		Redirect(BoidsVelocities[i], i);
		//update position
		InstanceTransform.SetLocation(NewBoidLocation);
		//update rotation
		InstanceTransform.SetRotation(CurrentBoidVelocity.ToOrientationQuat());

		//Keep them inside
		
		//apply steering to acceleration vector
		Acceleration += Separate(BoidCurrentLocations,i);
		Acceleration += Align(BoidCurrentLocations,i);
		Acceleration += Cohere(BoidCurrentLocations,i);

		//update velocities
		BoidsVelocities[i] += (Acceleration * DeltaTime);
		BoidsVelocities[i] = BoidsVelocities[i].GetClampedToSize(MinMovementSpeed, MaxMovementSpeed);

		// TempBuffer.Add(InstanceTransform);
		//Alternative is below, outside of the loop
		ISMComp->UpdateInstanceTransform(i, InstanceTransform);

#if DEBUG_ENABLED
		if (bToggleProximityDebug)
			DrawDebugSphere(GetWorld(), CurrentBoidLoc, ProximityRadius, 4, FColor::Purple );
#endif
	}
	
	//Optimization? 
	// ISMComp->BatchUpdateInstancesTransforms(0, TempBuffer);
	
#if DEBUG_ENABLED
	// FString str = TEXT("Average Velocity: " ) + GetVectorArrayAverage(BoidsVelocities).ToString();
	// GEngine->AddOnScreenDebugMessage(-1, 0.f, FColor::Red, *str);
#endif
	
	// SetRandomColor(); // Moving to blueprints

	if ( GetInstanceCount() > 0) ISMComp->MarkRenderTransformDirty();
	
	// Possibly usefull
	// ISMComp->ReleasePerInstanceRenderData();

#if UE_BUILD_DEVELOPMENT
	// DrawDebugSphere(GetWorld(), GetActorLocation(), SpreadRadius, 8, FColor::Orange);

#endif
	
	
#if DEBUG_ENABLED
	FString msg1 = FString::Printf(TEXT("InstanceCount: %i"), GetInstanceCount() );
	GEngine->AddOnScreenDebugMessage(-1, 0.f, FColor::White, *msg1);
#endif
}

