// Copyright Vitalii Voronkin. All Rights Reserved.


#include "BMassEntityTrait.h"
#include "MassEntityTemplateRegistry.h"

void UBMassEntityTrait::BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const
{
	// Adds Fragment to the Entity
	BuildContext.AddFragment<FBMovementFragment>();

}
