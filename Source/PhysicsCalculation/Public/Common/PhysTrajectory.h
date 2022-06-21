#pragma once

#include "CoreMinimal.h"
#include "HandyMath/Public/HMStructs/Trajectory.h"

#include "PhysTrajectory.generated.h"

USTRUCT(BlueprintType)
struct FPhysTrajectory
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	FTrajectory Trajectory;

};

UCLASS()
class PHYSICSCALCULATION_API UPhysTrajectoryLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
};

