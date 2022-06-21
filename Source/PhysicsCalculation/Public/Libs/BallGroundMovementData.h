#pragma once

#include "CoreMinimal.h"
#include "Common/PhysRigidBodyParams.h"
#include "BallGroundMovementData.generated.h"

static float ONE_FRAME_SEC_48_FPS = 1.0f/48.0f;

USTRUCT(BlueprintType)
struct FBallGroundMovementData
{
	GENERATED_BODY()

	int MaxCompSteps = 500000;
	
	UPROPERTY(BlueprintReadOnly)
	float TimeStep =  0.0002f;

	float AbsToleranceVelocitySel = 5;
	float AbsToleranceDistanceSel = 5;

	UPROPERTY(BlueprintReadOnly)
	TArray<float> LinVelXY;

	// debug
	UPROPERTY(BlueprintReadOnly)
	TArray<FVector> VelocityArr;

	UPROPERTY(BlueprintReadOnly)
	TArray<float> DistanceDeltaArr;

	float GetInitialVelocityFromEndVelocity(float EndVelocity, float Time);
	float GetInitialVelocityDistancePassedVelocity(float DistanceToPass, float Time);
	float GetNumSteps(float Time) const;

	void Simulate(FPhysRigidBodyParams P, float Time_Step);
	void Simulate(FPhysRigidBodyParams P);
};