// Fill out your copyright notice in the Description page of Project Settings.

#include "Libs/PhysicsCacheLib_old.h"

#include "Libs/PhysicsSimulation.h"
#include "Components/AdvancedPhysicsComponent.h"
#include "DataAssets/PhysicsCache_DataAsset.h"
#include "DataAssets/PhysicsCacheParams_DataAsset.h"
#include "PhysicsCache/PhysCachedTrajectory.h"

void UPhysicsCacheLib_old::CleanUpTrajectoryData(UAdvancedPhysicsComponent* Target)
{
    
    /*
    const auto PCache = Target->PhysicsCache;
    const auto Var = Target->PhysicsCacheParams;
    const auto SrcTrajectories = PCache->Trajectories;
    const int SrcTrajectoryCount = SrcTrajectories.Num();
    
    TArray<int> SkipIndices = {};
    
    for (int i = 0; i < SrcTrajectoryCount; ++i)
    {
        if(SkipIndices.Contains(i)) continue;
        GetIdenticalTrajectoryIndices(SrcTrajectories, i, SkipIndices);
    }

    //
    
    PCache->Trajectories.Empty();
    for (int i = 0; i < SrcTrajectoryCount; ++i)
    {
        if(SkipIndices.Contains(i)) continue;
        PCache->AddTrajectory(SrcTrajectories[i]);
    }
    */
    
}

void UPhysicsCacheLib_old::GetIdenticalTrajectoryIndices(const TArray<FPhysCachedTrajectoryFull>& Array, int StartIndex, TArray<int>& InOutSkipIndices)
{
    const auto Ref = Array[StartIndex];
    const int Next = StartIndex + 1;
    for (int i = Next; i < Array.Num(); ++i)
    {
        if(InOutSkipIndices.Contains(i)) continue;
        if(Ref.IsSimilarTo(Array[i]))
        {
            InOutSkipIndices.Add(i);
        }
    }
}

void UPhysicsCacheLib_old::CalculatePhysicsCache(UAdvancedPhysicsComponent* Target, const TArray<UCustomPhysicsBaseComponent*> OtherObj)
{
    const auto PCache = Target->PhysicsCache;
    const auto Var = Target->PhysicsCacheParams;
    
    auto LinVelArray = Var->GetAllStartLinearVelocitiesDetailed();
    auto AngVelArray = Var->GetAllStartAngularVelocitiesDetailed();

    for (const auto LinVel : LinVelArray)
    {
        for (const auto AngVel : AngVelArray)
        {
            // todo: it's pointless to simulate all XY angles when angular velocity is zero
            
            FPhysCachedTrajectoryFull Trajectory;
            FPhysTransform TStart = GetStartTransformForCalculation(Target->GetRadius(), LinVel.V, AngVel.V);
            CalculateCachedTrajectory(Target, OtherObj, TStart, Trajectory);
            // PCache->AddTrajectory(Trajectory);
        }
    }
}

void UPhysicsCacheLib_old::CalculateCachedTrajectory(UAdvancedPhysicsComponent* Target, const TArray<UCustomPhysicsBaseComponent*> OtherObj,
    const FPhysTransform& TStart, FPhysCachedTrajectoryFull& OutTrajectory)
{
    return;
    
    auto RbParams = Target->PhysicsParams;
    const auto Var = Target->PhysicsCacheParams;
    float SimDT = Var->TimeData.SimulationDeltaTime;
    int SimSteps = Var->TimeData.GetMaxSimulationStepsCount();

    OutTrajectory.NoCollisions.AddPoint(0.0f, TStart.Location, TStart.LinearVelocity, TStart.AngularVelocity);
    
    FPhysTransform TPrev = TStart;
    for (int i = 1; i < SimSteps; ++i)
    {
        FPhysTransform TNew;
        float CurrentTime = SimDT * i;
        
        // UPhysicsSimulation::PhysicsSimulateDelta(TPrev, RbParams, SimDT, 0.0f, false, TNew);
        OutTrajectory.NoCollisions.AddPoint(CurrentTime, TNew.Location, TNew.LinearVelocity, TNew.AngularVelocity);

        bool bContinue = CanContinueCalculation(Target, TNew);

        if(bContinue)
        {
            TPrev = TNew;
        }
        else
        {
            break;
        }
    }
}

FPhysTransform UPhysicsCacheLib_old::GetStartTransformForCalculation(float Radius, const FVector& LV, const FVector& AV)
{
    const FVector Location = FVector(0.0f, 0.0f, Radius);
    return  FPhysTransform(Location, FQuat::Identity, LV, AV);
}

bool UPhysicsCacheLib_old::CanContinueCalculation(UAdvancedPhysicsComponent* Target, const FPhysTransform& T)
{
    const auto Var = Target->PhysicsCacheParams;
    const auto DistanceLimits = Var->DistanceLimits;
    if(!DistanceLimits.bEnabled)return true;
    return DistanceLimits.IsLocationApplicable(T.Location);
}
