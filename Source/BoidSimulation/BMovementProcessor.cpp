// Copyright Vitalii Voronkin. All Rights Reserved.

#include "UBMovementProcessor.h"
#include "BMassEntityTrait.h"
#include "MassCommonFragments.h"

#include "MassCommonTypes.h"
#include "MassExecutionContext.h"

UBMovementProcessor::UBMovementProcessor() : EntityQuery(*this)
{
	bAutoRegisterWithProcessingPhases = true;
	ExecutionFlags = static_cast<int32>(EProcessorExecutionFlags::All);
	ExecutionOrder.ExecuteInGroup = UE::Mass::ProcessorGroupNames::Movement;
	ExecutionOrder.ExecuteBefore.Add(UE::Mass::ProcessorGroupNames::Avoidance);
}

void UBMovementProcessor::ConfigureQueries()
{
	EntityQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadWrite);
	EntityQuery.AddRequirement<FBMovementFragment>(EMassFragmentAccess::ReadWrite);
	// EntityQuery.RegisterWithProcessor(*this); // setting queries in the constructor instead 
}


void UBMovementProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
	// Data is tightly packed in memory
	EntityQuery.ForEachEntityChunk(EntityManager, Context, ([&](FMassExecutionContext& Context)
		{
			 
			const TArrayView<FTransformFragment> TransformsList = Context.GetMutableFragmentView<FTransformFragment>();
		
			const TArrayView<FBMovementFragment> MovementsList = Context.GetMutableFragmentView<FBMovementFragment>();
			const float WorldDeltaTime = Context.GetDeltaTimeSeconds();

			
			for (int32 EntityIndex = 0; EntityIndex < Context.GetNumEntities(); EntityIndex++)
			{
				FTransform& Transform = TransformsList[EntityIndex].GetMutableTransform();
				FVector& MoveTarget = MovementsList[EntityIndex].Target;

				FVector CurrentLoc = Transform.GetLocation();
				FVector TargetLoc = MoveTarget - CurrentLoc;

				// Move entities
				if(TargetLoc.Size() <= 20.f)
				{
					MoveTarget = FVector(FMath::RandRange(-1.f, 1.f) * 1000.f,
										FMath::RandRange(-1.f, 1.f) * 1000.f,
										FMath::RandRange(-1.f, 1.f) * 1000.f);
				}
				else
				{
					Transform = FTransform(TargetLoc.ToOrientationQuat(),CurrentLoc + TargetLoc.GetSafeNormal() * 400.f * WorldDeltaTime );
					// Transform.SetLocation(CurrentLoc + TargetLoc.GetSafeNormal() * 400.f * WorldDeltaTime);
				}
			}
		}));
}
