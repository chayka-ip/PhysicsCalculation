#include "Libs/BallGroundMovementData.h"
#include "HandyMathLibrary.h"
#include "Libs/PhysicsSimulation.h"
#include "Common/PhysTransform.h"

float FBallGroundMovementData::GetInitialVelocityFromEndVelocity(float EndVelocity, float Time)
{
	if (Time >= 0)
	{
		int Index;
		if(UHM::FloatBinSearch(LinVelXY, EndVelocity, AbsToleranceVelocitySel, Index))
		{
			const int NumSteps = Time / TimeStep;
			const int StepOffset = Index - NumSteps;
			Index = StepOffset > 0 ? StepOffset : 0;
			return LinVelXY[Index];
        }
	}
	return 0;
}

float FBallGroundMovementData::GetInitialVelocityDistancePassedVelocity(float DistanceToPass, float Time)
{
	const int Steps = GetNumSteps(Time);

	
	if(Steps > 0 && DistanceToPass > 0)
	{
		int StartIndex;
		if(UHM::FloatBinSearchSum(DistanceDeltaArr, DistanceToPass, Steps, AbsToleranceDistanceSel, StartIndex))
		{
			// compensate shifting
			StartIndex -= 2;
			if (StartIndex < 0) StartIndex = 0;
			return LinVelXY[StartIndex]; 
		}
	}
	return 0;
}

float FBallGroundMovementData::GetNumSteps(float Time) const
{
	return UHM::SafeDivision(Time, TimeStep);
}

void FBallGroundMovementData::Simulate(FPhysRigidBodyParams P, float Time_Step)
{
	return;
	
	TimeStep = Time_Step;

	// cm/sec

	float MinVel = 10;
	float StartVel = 50000;

	FVector StartLocation = FVector(0, 0, P.Radius);
	FVector StartVelocity = FVector(StartVel, 0, 0);
	FPhysTransform TPrev = FPhysTransform(StartLocation, FQuat::Identity, StartVelocity, FVector::ZeroVector);

	LinVelXY.Add(StartVel);

	bool DoComp = true;
	int StepsPassed = 0;

	float DistanceSum = 0;

	while (DoComp)
	{
		constexpr float SkipTime = 0;
		constexpr bool bApplyExtraForces = false;
		FPhysTransform TNew;
		// UPhysicsSimulation::PhysicsSimulateDelta(TPrev, P, TimeStep, SkipTime, bApplyExtraForces, TNew);
		TPrev = TNew;

		// add velocity
		
		FVector VL = TPrev.LinearVelocity;
		VelocityArr.Add(VL);
		VL.Z = 0;		
		
		float V = VL.X;
		LinVelXY.Add(V);

		// add location

		const float DistanceFull = TPrev.Location.X;
		float DistDelta = DistanceFull - DistanceSum;
		if(DistanceSum > 0.0f)
		{
			DistanceDeltaArr.Add(DistDelta);
		}
		DistanceSum += DistDelta;
		
		// check exit
		
		if (V <= MinVel)
		{
			DoComp = false;
		}

		StepsPassed++;
		if (StepsPassed >= MaxCompSteps)
		{
			DoComp = false;
		}
	}
	
	LinVelXY.Add(0);
}

void FBallGroundMovementData::Simulate(FPhysRigidBodyParams P)
{
	Simulate(P, TimeStep);
}
