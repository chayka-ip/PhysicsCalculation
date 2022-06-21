// Fill out your copyright notice in the Description page of Project Settings.

#include "Libs/SpinMovementLib.h"
#include "Components/AdvancedPhysicsComponent.h"
#include "DataAssets/SpinMovementParams_DataAsset.h"
#include "ParabolicMotion/ParabolicMotionToRealLib.h"
#include "DataAssets/PhysicsCache_DataAsset.h"
#include "ImpulseDistribution/ImpulseDistributionLib.h"

FBallLaunchCache_Data USpinMovementLib::CalculateBallLaunchCache(UAdvancedPhysicsComponent* PC)
{
    const auto Params = &PC->SpinMovementParams->Data;
    const auto LaunchSpeedData = Params->GetLaunchSpeedCmSec();
    const auto LaunchAngleData = Params->LaunchAngle.GetValueArrayFullRange();
    const auto FrontSpinAngleData = Params->FrontSpinAngle.GetValueArrayFullRange();
    const auto SideSpinAngleData = Params->SideSpinAngle.GetValueArrayFullRange();
    const auto VerticalLevelWidth = Params->GetVerticalLevelWidth();

    FBallLaunchCache_Data OutData;
    OutData.SetLaunchSpeedVector(LaunchSpeedData);
    OutData.SetLaunchAngleVector(LaunchAngleData);
    OutData.SetFrontSpinAngleVector(FrontSpinAngleData);
    OutData.SetSideSpinAngleVector(SideSpinAngleData);
    OutData.SetVerticalLevelWidth(VerticalLevelWidth);
    OutData.Validate();

    for (int launch_speed_ind = 0; launch_speed_ind < LaunchSpeedData.Num(); ++launch_speed_ind)
    {
        for (int launch_angle_ind = 0; launch_angle_ind < LaunchAngleData.Num(); ++launch_angle_ind)
        {
            for (int front_spin_angle_ind = 0; front_spin_angle_ind < FrontSpinAngleData.Num(); ++front_spin_angle_ind)
            {
                for (int side_spin_angle_ind = 0; side_spin_angle_ind < SideSpinAngleData.Num(); ++side_spin_angle_ind)
                {
                    const float launch_speed = LaunchSpeedData[launch_speed_ind];
                    const float launch_angle = LaunchAngleData[launch_angle_ind];
                    const float front_spin_angle = FrontSpinAngleData[front_spin_angle_ind];
                    const float side_spin_angle = SideSpinAngleData[side_spin_angle_ind];
                    
                    FBallLaunchParams LaunchParams;
                    LaunchParams.Init(launch_speed, launch_angle, front_spin_angle, side_spin_angle);

                    const bool CanUseLaunchParams = Params->CanUseLaunchParams(LaunchParams);
                    if(CanUseLaunchParams)
                    {
                        CalculateDataFromLaunchParams(PC, LaunchParams, OutData);
                    }
                }
            }
        }
    }
    return OutData;
}

void USpinMovementLib::CalculateDataFromLaunchParams(UAdvancedPhysicsComponent* PC, const FBallLaunchParams& LaunchParams, FBallLaunchCache_Data& OutData)
{
    const auto Params = &PC->SpinMovementParams->Data;
    const float SimStep = Params->SimulationStep;
    const int   NumSteps = Params->GetComputationNumSteps();
    const FVector COM = FVector(0.0f, 0.0f, PC->GetRadius());
                    
    FPhysTransform TLaunch;
    const auto Impulse = GetImpulseFromBallLaunchParams(PC, LaunchParams, COM);
    const auto Curve = CalculateSpinTrajectory(PC, Impulse, COM, SimStep, NumSteps, TLaunch);
    FVector GroundLocation;
    UParabolicMotionToRealLib::GetTrajectoryGroundLocation(Curve, GroundLocation, true);
    
    const float MaxDistance = (GroundLocation - COM).Size2D();
    OutData.AddDistanceValue(LaunchParams, MaxDistance);

    const auto VerticalDistribution = CalculateVerticalDistributionFromTrajectory(PC, Curve);
    OutData.AddVerticalDistribution(LaunchParams, VerticalDistribution);
}


FImpulseReconstructed USpinMovementLib::GetImpulseFromBallLaunchParams(UAdvancedPhysicsComponent* PC, const FBallLaunchParams& P, FVector COM, FVector BaseVector)
{
    BaseVector = UHMV::TrimVectorZ(BaseVector).GetSafeNormal();
    const FVector ParabolicVelocity = P.LaunchSpeed * UHMV::GetVectorRotated_XY_XZ(BaseVector, 0.0, P.LaunchAngle);
    const auto ImpulseData = UImpulseDistributionLib::ReconstructImpulseFromVelocities(PC, ParabolicVelocity, FVector::ZeroVector, COM);
    const FVector ParabolicApplyLocation = ImpulseData.ApplyLocation;

    const auto CorrectImpulse = UImpulseDistributionLib::GetImpulseFromParabolicAndSpin(PC, ParabolicVelocity, ParabolicApplyLocation, COM, P.FrontSpinAngle, P.SideSpinAngle,true);

    const FVector Impulse = CorrectImpulse.Impulse;
    const FVector ApplyLocation = CorrectImpulse.ApplyLocation;
    return FImpulseReconstructed(Impulse, ApplyLocation);
}

FCustomVectorCurve USpinMovementLib::CalculateSpinTrajectory(UAdvancedPhysicsComponent* Obj, const FImpulseReconstructed& ImpulseData, FVector COM, float TimeStep, int NumSteps, FPhysTransform& TLaunch)
{
    TLaunch = Obj->CalculateImpulseImpactAtLocationOtherCOM(ImpulseData.Impulse, ImpulseData.ApplyLocation, COM, FVector::ZeroVector, true);
    return  UParabolicMotionToRealLib::CalculateMotionCurveFromTransform(Obj, TLaunch, TimeStep, NumSteps);
}

FBallLaunchVerticalDistribution USpinMovementLib::CalculateVerticalDistributionFromTrajectory(UAdvancedPhysicsComponent* PC, const FCustomVectorCurve& Curve)
{
    FBallLaunchVerticalDistribution Out;

    const auto Params = &PC->SpinMovementParams->Data;
    auto VerticalLevels = Params->GetTargetVerticalLevels();
    const float LevelWidth = Params->GetVerticalLevelWidth();
    const float Radius = PC->GetRadius() + 1.0f;
    const float TimeMaxZ = Curve.GetMaxZTime();
    const FVector StartLocation = Curve.GetValueAtZero();
    
    // we limit results by distance from launch position to save more space
    
    for (int i = 0; i < VerticalLevels.Num(); ++i)
    {
        const auto Level = VerticalLevels[i];
        const auto LevelIndex = GetVerticalLevelIndexFromDistanceZ(Level.GetAverage(), LevelWidth);
        
        const float MinZToStore = Level.Min;
        const float MaxZToStore = Level.Max;
        const float MinZ = MinZToStore < Radius ? Radius : MinZToStore;
        const float MaxZ = MaxZToStore < Radius ? Radius : MaxZToStore;

        TArray<float> MinTimes = {};
        TArray<float> MaxTimes = {};
        const bool HasMin = Curve.GetTimesWhenZEquals(MinZ, MinTimes);
        const bool HasMax = Curve.GetTimesWhenZEquals(MaxZ, MaxTimes);
        const bool HasMinMax = HasMin && HasMax;

        if(HasMinMax)
        {
            auto TimePairs = UHM::MakeConsequentTimePairsFromArrays(MinTimes, MaxTimes, TimeMaxZ);
                        
            for (const auto TimePair : TimePairs)
            {
                FVector MinLocation = Curve.GetVectorValue(TimePair.Min);
                FVector MaxLocation = Curve.GetVectorValue(TimePair.Max);

                const float MinDistanceXY = (MinLocation - StartLocation).Size2D();
                const float MaxDistanceXY = (MaxLocation - StartLocation).Size2D();
                                
                Out.AddItem(LevelIndex, MinDistanceXY, MaxDistanceXY);

                if(LevelIndex == static_cast<uint8>(8))
                {
                    FString MD = FString::SanitizeFloat(MinDistanceXY);
                    FString MaD = FString::SanitizeFloat(MaxDistanceXY);
                    // PrintToLog(MD + " | " + MaD);
                }
            }
        }
    }

    return Out;
}

FCustomVectorCurve USpinMovementLib::CalculateSpinTrajectory(UAdvancedPhysicsComponent* Obj, const FBallLaunchParams& P, FVector COM, float TimeStep, int NumSteps, FPhysTransform& TLaunch)
{
    const auto Impulse = GetImpulseFromBallLaunchParams(Obj, P, COM);
    return CalculateSpinTrajectory(Obj, Impulse, COM, TimeStep, NumSteps, TLaunch);
}

uint8 USpinMovementLib::GetVerticalLevelIndexFromDistanceZ(float DistanceZ, float LevelWidth)
{
    return FMath::CeilToInt(DistanceZ / LevelWidth);
}
