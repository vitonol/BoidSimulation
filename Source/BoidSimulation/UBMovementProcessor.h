#pragma once

#include "CoreMinimal.h"
#include "MassProcessor.h"

#include "UBMovementProcessor.generated.h"

UCLASS()
class UBMovementProcessor : public UMassProcessor
{
	GENERATED_BODY()
public:
	UBMovementProcessor();
	
protected:
	// Request processing operation on the data
	FMassEntityQuery EntityQuery;
	
	virtual void ConfigureQueries() override;
	
	// virtual void Execute(UMassEntitySubsystem& EntitySubsystem, FMassExecutionContext& Context) override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;
	
};