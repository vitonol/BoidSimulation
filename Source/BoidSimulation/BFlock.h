//  Copyright Vitalii Voronkin. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "BFlock.generated.h"

class UInstancedStaticMeshComponent;

#define DEBUG_ENABLED 0

/**
 * The ABFlock class represents a flocking behavior simulation using instanced static meshes.
 * Adjustable parameters are exposed to UI and help to dial in specific behaviour.
 */

UCLASS()
class BOIDSIMULATION_API ABFlock : public AActor
{
	GENERATED_BODY()
	
public:	

	ABFlock();
	
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

	//COMPONENTS
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	UInstancedStaticMeshComponent* ISMComp;

	// Arrays to store current locations and velocities of boids
	TArray<FVector> BoidCurrentLocations;
	TArray<FVector> BoidsVelocities;

	// Set in UI before spawning this actor
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector InitialSpawnScale;
	
	//MOVEMENT
protected:
	
	FVector Align(TArray<FVector>& BoidsPositions, const int32 CurrentIndex);
	
	FVector Separate(TArray<FVector>& BoidsPositions, const int32 CurrentIndex);
	
	FVector Cohere(TArray<FVector>& BoidsPositions, const int32 CurrentIndex);

	void Redirect(FVector& Direction, const int32 CurrentIndex);

public:

	UFUNCTION(BlueprintCallable)
	inline int GetInstanceCount();    

	UFUNCTION(BlueprintCallable)
	void UpdateBuffers(int32 NewCount);
	
	UFUNCTION(BlueprintCallable)
	void AddInstances(int32 NumToAdd);

	UFUNCTION(BlueprintCallable)
	void RemoveInstances(int32 NumToRemove);

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<int32> InstanceIndecies;

	//Default Configurations
protected:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments");
	int NumInstances = 50;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "10.0", ClampMax = "500.0", UIMin = "10.0", UIMax = "500.0"));
	float AlignmentStrength = 302.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "5.0", ClampMax = "60.0", UIMin = "5.0", UIMax = "60.0"));
	float SeparationStrength = 25.f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "0.5", ClampMax = "15.0", UIMin = "1.0", UIMax = "15.0"));
	float CohesionStrength = 1.3f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (ClampMin = "30.0", ClampMax = "1200.0", UIMin = "30.0", UIMax = "1200.0"));
	float ProximityRadius = 70.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (UIMin = "90.0", UIMax = "650.0"));
	float MinMovementSpeed = 90.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (UIMin = "90.0", UIMax = "650.0"));
	float MaxMovementSpeed = 650.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid Adjustments", meta = (UIMin = "400.0", UIMax = "3500.0"));
	float SpreadRadius = 400.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boid DEBUG")
	bool bToggleProximityDebug = true;

	//Not Currenly Used
	static inline FVector RandomPointInBoundingBox(const FVector& Center, const FVector& HalfSize) { return FMath::RandPointInBox(FBox(Center - HalfSize, Center + HalfSize)); }

	//Helper function
	FVector GetVectorArrayAverage(const TArray<FVector>& Vectors);
};
