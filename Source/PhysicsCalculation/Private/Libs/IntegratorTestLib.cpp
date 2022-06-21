// Fill out your copyright notice in the Description page of Project Settings.

#include "Libs/IntegratorTestLib.h"

#include "Libs/PhysicsSimulation.h"

FVector UIntegratorTestLib::GetIntegratedLocation(const FPhysTransform& TIn, float Time, const FPhysRigidBodyParams& Params)
{
    return {};
}

FPhysTransform UIntegratorTestLib::GetIntegratedTransform(const FPhysTransform& TIn, float DeltaTime, const FPhysRigidBodyParams& RbParams)
{
    FPhysTransform OutT = TIn;
    
    OutT.Location += OutT.LinearVelocity * DeltaTime;
    
    if(RbParams.bGravityEnabled)
    {
        OutT.LinearVelocity +=  UPhysicsSimulation::ForceToImpulse(RbParams.GetGravity(), DeltaTime);
    }

    return OutT;
}

TArray<FPhysTransform> UIntegratorTestLib::GetIntegralTrajectory(const FPhysTransform& TIn, float Time, int  NumSimSteps, const FPhysRigidBodyParams& Params)
{
    if(NumSimSteps < 1) NumSimSteps = 1;
    TArray<FPhysTransform> Out = {TIn};
    const float SimStep = Time / NumSimSteps;
    
    for (int i = 0; i < NumSimSteps; ++i)
    {
        auto TPrev = Out.Last();
        auto TNext = GetIntegratedTransform(TPrev, SimStep, Params);
        Out.Add(TNext);
    }
    
    return Out;
}

TArray<FVector> UIntegratorTestLib::GetIntegralTrajectoryPoints(const FPhysTransform& TIn, float Time, int NumSimSteps,
    const FPhysRigidBodyParams& Params)
{
    TArray<FVector> Out;
    for (auto T : GetIntegralTrajectory(TIn, Time, NumSimSteps, Params)) Out.Add(T.Location);
    return Out;
}
