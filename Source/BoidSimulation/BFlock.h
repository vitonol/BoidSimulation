//  Copyright Vitalii Voronkin. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "BFlock.generated.h"

class UInstancedStaticMeshComponent;
class UBoxComponent;

#define DEBUG_ENABLED 1

UCLASS()
class BOIDSIMULATION_API ABFlock : public AActor
{
	GENERATED_BODY()
	
public:	

	ABFlock();

	virtual void OnConstruction(const FTransform& Transform) override;

	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

	//COMPONENTS
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	UInstancedStaticMeshComponent* ISMComp;
	
	TArray<FVector> BoidCurrentLocations;
	
	float BoidsSpawnRadius = 1500.f;

public:
	// void MoveBoids(float D);
	
	// void UpdateLocations();
	
	//MOVEMENT
protected:
	TArray<FVector> BoidsVelocities;

	TArray<FRotator> BoidsRotations;
	
	FVector Align(TArray<FVector>& BoidsPositions, int CurrentIndex);
	
	FVector Separate(TArray<FVector>& BoidsPositions, int CurrentIndex);
	
	FVector Cohere(TArray<FVector>& BoidsPositions, int CurrentIndex);

public:

	UFUNCTION(BlueprintCallable)
	inline int GetInstanceCount();    
	
	void SetRandomColor();
	
	UFUNCTION(BlueprintCallable)
	void AddInstances(int32 NumToAdd);

	UFUNCTION(BlueprintCallable)
	void RemoveInstances(int32 NumToRemove);
	
protected:
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	UBoxComponent* Box;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments");
	int NumInstances = 50;

	// UPROPERTY(EditAnywhere, BlueprintReadWrite);
	// float MovementSpeed = 400.f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "10.0", ClampMax = "500.0", UIMin = "10.0", UIMax = "500.0"));
	float AlignmentStrength = 302.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "5.0", ClampMax = "60.0", UIMin = "5.0", UIMax = "60.0"));
	float SeparationStrength = 25.f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "1.0", ClampMax = "20.0", UIMin = "1.0", UIMax = "20.0"));
	float CohesionStrength = 4.f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "80.0", ClampMax = "1200.0", UIMin = "80.0", UIMax = "1200.0"));
	float ProximityRadius = 360.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid DEBUG")
	bool bToggleProximityDebug = true;

	inline FVector RandomPointInBoundingBox(const FVector Center, const FVector HalfSize) { return FMath::RandPointInBox(FBox(Center - HalfSize, Center + HalfSize)); }

	FVector GetVectorArrayAverage(const TArray<FVector>& Vectors);
};
