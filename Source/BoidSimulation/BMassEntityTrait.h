// Copyright Vitalii Voronkin. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "MassEntityTraitBase.h"
#include "MassEntityTypes.h"
#include "BMassEntityTrait.generated.h"

/**
 * 
 */


USTRUCT() 
struct FBMovementFragment : public FMassFragment
{
	GENERATED_BODY()
	FVector Target = FVector::ZeroVector;
};

UCLASS()
class BOIDSIMULATION_API UBMassEntityTrait : public UMassEntityTraitBase
{
	GENERATED_BODY()

protected:
	virtual void BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const override;
	
};
