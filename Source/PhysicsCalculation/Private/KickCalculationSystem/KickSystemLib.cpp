// Fill out your copyright notice in the Description page of Project Settings.

#include "KickCalculationSystem/KickSystemLib.h"
#include "ImpulseDistribution/ImpulseDistributionLib.h"
#include "DataAssets/PhysicsCache_DataAsset.h"
#include "HMStructs/FloatMinMax.h"
#include "Libs/PhysicsSimulation.h"
#include "Libs/SpinMovementLib.h"
#include "ColorsLib.h"
#include "VisualDebugLib.h"
#include "Components/PhysicsComponent.h"
#include "DataAssets/SpinMovementParams_DataAsset.h"

FImpulseReconstructed UKickSystemLib::CalculateKickImpulse(UPhysicsComponent* PC, const FKickCompData& Data)
{
    auto Result = KickComputePossibleImpact(PC, Data);
    if(Result.IsNotEmpty())
    {
        FPhysTransform T;
        FPhysicsPredictionCurveWithMeta Meta;
        Result.GetRandomDataPair(T, Meta);

        const auto ImpReconstructed = UImpulseDistributionLib::ReconstructImpulseFromImpactTransform(PC, T);

        const FVector CurrentLocation = PC->GetCurrentLocation();
        const FVector VToCachedLocation = Meta.GetCachedVector() - CurrentLocation;
        const FVector V_ToTarget = Data.GetVectorToTarget();
        const float Angle = UHMV::SignedAngleBetweenVectorsDegXY(VToCachedLocation, V_ToTarget);
        const auto ImpRotated = UImpulseDistributionLib::RotateImpulseZAxis(ImpReconstructed, CurrentLocation, Angle);
        return ImpRotated;        
    }
    return {};
}

FKickCompResultTmp UKickSystemLib::KickComputeParabolicWithSpin_prev(UPhysicsComponent* PC, const FKickCompData& Data)
{
    const auto ParabolicCache = PC->PhysicsCache->ParabolicMotionCache;
    const float MaxSpeed = Data.MaxSpeed;
    const float AngleAlpha = Data.GetLaunchAngleAlpha();
    const float MinParabolicAngle = ParabolicCache->GetMinLaunchAngle();
    const float MaxParabolicAngle = ParabolicCache->GetMaxLaunchAngle();
    const float MaxDistanceAngle = ParabolicCache->GetAngleMaxDistanceForLaunchSpeed(MaxSpeed);
    const FVector VToKickTarget = Data.GetVectorToTarget();
    const FVector NormalToTarget_XY = UHMV::TrimVectorZ(VToKickTarget).GetSafeNormal();

    const int DirectLaunchAngle = Data.GetDirectLaunchAngleRounded();
    const float MinLaunchAngle = MinParabolicAngle > DirectLaunchAngle ? MinParabolicAngle : DirectLaunchAngle;
    const float MaxLaunchAngle = MaxParabolicAngle < MaxDistanceAngle ?  MaxParabolicAngle : MaxDistanceAngle;
    
    FFloatMinMax AngleRange;
    const bool bCanLaunchParabolic = PC->CalculateParabolicMinMaxLaunchAngle(MaxSpeed, MinLaunchAngle, MaxLaunchAngle, VToKickTarget, AngleRange);

    if(bCanLaunchParabolic)
    {
        const float LaunchAngle = UHM::GetValueFromFloatRangeByAlphaFromMin(AngleRange.Min, AngleRange.Max, AngleAlpha);
        const float ParabolicSpeed = PC->GetRealRequiredParabolicSpeedForAngle(LaunchAngle, VToKickTarget);
        const FVector ParabolicVelocity = ParabolicSpeed * UHMV::GetVectorRotated_XY_XZ(NormalToTarget_XY, 0.0f, LaunchAngle);

        bool bFlipSideSpin = false;
        if(Data.bRandomizeSideSpinFlip) bFlipSideSpin = FMath::RandBool();

        const auto SpecialCompData = GetKickSpecialPredictionData(PC, ParabolicVelocity, Data.SpinRange, bFlipSideSpin);
        const auto Result = GetKickComputedTrajectories(PC, SpecialCompData, Data.PredictionLimits, Data.PredictionExtraData);
        DisplayComputedTrajectories(PC, Data, Result);
        return Result;
    }

    PrintToLog("KickComputeParabolicWithSpin: can't find trajectory");
    return {};
}

FKickCompResultTmp UKickSystemLib::KickComputePossibleImpact(UPhysicsComponent* PC, const FKickCompData& Data)
{
    const FSpinCacheIterCheckData IterData = Data.CacheIterCheckData;
    const auto LaunchParamsArray = MakeLaunchParamsArray(PC, IterData);
    const FVector VToTarget = IterData.GetVectorToTarget();
    const auto ImpactArray = CalculateApplicableImpactDataCacheBased(PC, LaunchParamsArray, VToTarget);
    const auto Result = GetKickComputedTrajectoriesFromImpactArray(PC, Data.PredictionLimits, Data.PredictionExtraData, ImpactArray);
    DisplayComputedTrajectories(PC, Data, Result);
    return Result;
}

void UKickSystemLib::DisplayComputedTrajectories(UPhysicsComponent* PC, const FKickCompData& Data, const FKickCompResultTmp& Result)
{
    if(Data.bDrawResultTrajectories)
    {
        auto Transforms = Result.ImpactTransformArray;
        if(Data.bRotateResultTrajectories)
        {
            FVector KickTarget = Data.GetVectorToTarget() + PC->GetCurrentLocation();
            auto Rotated = RotateImpactTransformsToKickTarget(PC, Result, KickTarget);
            Transforms = Rotated.ImpactTransformArray;
        }

        FPhysCurveSpecialPrediction DData;
        DData.SetDeltaTime(PC->GetRoughPredictSimStep());
        DData.SetMaxIterations(5000);
        DData.Limits = Data.PredictionLimits;
        DData.ExtraData = Data.PredictionExtraData;
        DData.ExtraData.EnableStoreLocationVectors();
            
        TArray<FPhysicsPredictionCurveWithMeta> CurveArray = {};
        for (auto T : Transforms)
        {
            DData.SetInitialTransform(T);

            FPhysicsPredictionCurveWithMeta Curve;
            UPhysicsSimulation::CalculatePredictionCurveWithMeta(PC, DData, Curve);
            CurveArray.Add(Curve);
        }
            
        int NumCurves = CurveArray.Num();
        auto Colors = UColorsLib::MakeColorsArray(NumCurves);
        const float DrawTime = Data.DrawTime;
        for (int i = 0; i < CurveArray.Num(); ++i)
        {
            const auto Color = Colors[i];
            auto P = CurveArray[i].Locations;
            UVisualDebugLib::DrawTrajectory(PC, P, DrawTime, Color, ETDT_Line, 4);
        }
    }
}

FKickSpecialPredictionImpulseData UKickSystemLib::GetKickSpecialPredictionData(UPhysicsComponent* PC, FVector ParabolicVelocity, const FKickSpinRange& KickSpinRange, bool bFlipSideSpin)
{
    FKickSpecialPredictionImpulseData Out;
    
    const FVector ParabolicVelocityNormal = ParabolicVelocity.GetSafeNormal();
    const FVector CurrentLocation = PC->GetCurrentLocation();

    const auto ImpulseData = UImpulseDistributionLib::ReconstructImpulseFromVelocities(PC, ParabolicVelocity, FVector::ZeroVector, CurrentLocation);
    const FVector ApplyLocation = ImpulseData.ApplyLocation;
    const FVector ParabolicImpulse = ImpulseData.Impulse;
    const float ImpulseSize = ParabolicImpulse.Size();

    const auto ImpulseQuatArray = UImpulseDistributionLib::GetImpulseQuatArray(ParabolicVelocityNormal, KickSpinRange, bFlipSideSpin);

    Out.Init(ApplyLocation, ImpulseQuatArray, ImpulseSize);
    
    return Out;
}

void UKickSystemLib::CalculateSpecialPrediction(UPhysicsComponent* PC, const FSpecialPrediction_ExtraData& ExtraData, const FPhysCurveSpecialPrediction& PredictionData, const FPhysTransform& Impact, FKickCompResultTmp
                                                & Out)
{
    if(ExtraData.bApplyAllResults)
    {
        Out.AddImpactTransform(Impact, FPhysicsPredictionCurveWithMeta());
        return;
    }
        
    FPhysicsPredictionCurveWithMeta CompResult;
    if(UPhysicsSimulation::CalculatePredictionCurveWithMeta(PC, PredictionData, CompResult))
    {
        if(CompResult.IsVectorCached())
        {
            const float Min = ExtraData.GetMinZ();
            const float Max = ExtraData.GetMaxZ();
            const bool bApplicable = UHM::IsFloatInRange(CompResult.GetCachedVectorZ(), Min, Max);
            if(bApplicable) Out.AddImpactTransform(Impact, CompResult);
        }
    }
}

FKickCompResultTmp UKickSystemLib::GetKickComputedTrajectories(UPhysicsComponent* PC,
                                                               const FKickSpecialPredictionImpulseData& Data, const FSpecialPrediction_Limits& Limits, const FSpecialPrediction_ExtraData& ExtraData)
{
    FKickCompResultTmp Out;
    
    const FVector ApplyLocation = Data.ApplyLocation;

    // todo: convert special prediction data to impact array and delete this func
    
    FPhysCurveSpecialPrediction PredictionData;
    PredictionData.SetDeltaTime(PC->GetRoughPredictSimStep());
    PredictionData.SetMaxIterations(5000);
    PredictionData.Limits = Limits;
    PredictionData.ExtraData = ExtraData;

    for (int i = 0; i < Data.GetNum(); ++i)
    {
        const FVector Impulse = Data.GetImpulseAtIndex(i);
        constexpr bool bRemoveVelocity = true;
        const auto Impact = PC->CalculateImpulseImpactAtLocation(Impulse, ApplyLocation, bRemoveVelocity);
        PredictionData.SetInitialTransform(Impact);
        CalculateSpecialPrediction(PC, ExtraData, PredictionData, Impact, Out);
    }
    
    return Out;
}

FKickCompResultTmp UKickSystemLib::GetKickComputedTrajectoriesFromImpactArray(UPhysicsComponent* PC,
                                                                              const FSpecialPrediction_Limits& Limits, const FSpecialPrediction_ExtraData& ExtraData, const TArray<FPhysTransform>& TArray)
{
    FKickCompResultTmp Out;

    FPhysCurveSpecialPrediction PredictionData;
    PredictionData.SetDeltaTime(PC->GetRoughPredictSimStep());
    PredictionData.SetMaxIterations(5000);
    PredictionData.Limits = Limits;
    PredictionData.ExtraData = ExtraData;

    for (auto Impact : TArray)
    {
        PredictionData.SetInitialTransform(Impact);
        CalculateSpecialPrediction(PC, ExtraData, PredictionData, Impact, Out);
    }

    return Out;
}

FKickCompResultTmp UKickSystemLib::RotateImpactTransformsToKickTarget(UPhysicsComponent* PC, const FKickCompResultTmp& Data, FVector KickTarget)
{
    FKickCompResultTmp Out = Data;
    Out.MarkMeta();

    Out.ImpactTransformArray.Empty();
    for (int i = 0; i < Data.GetNum(); ++i)
    {
        auto TImpact = Data.ImpactTransformArray[i];
        auto Meta = Data.CurveMetaArray[i];
        auto ImpReconstructed = UImpulseDistributionLib::ReconstructImpulseFromImpactTransform(PC, TImpact);
        FVector CurrentLocation = PC->GetCurrentLocation();
        FVector TargetLocation = KickTarget;
        FVector VToCachedLocation = Meta.GetCachedVector() - CurrentLocation;
        FVector V_ToTarget = TargetLocation - CurrentLocation;
        float Angle = UHMV::SignedAngleBetweenVectorsDegXY(VToCachedLocation, V_ToTarget);
        auto ImpRotated = UImpulseDistributionLib::RotateImpulseZAxis(ImpReconstructed, CurrentLocation, Angle);
        bool bRemoveVelocities = true;
        auto TNewImpact = PC->CalculateImpulseImpactAtLocation(ImpRotated.Impulse, ImpRotated.ApplyLocation, bRemoveVelocities);
        Out.ImpactTransformArray.Add(TNewImpact);
    }
    return Out;
}

TArray<FBallLaunchParams> UKickSystemLib::MakeLaunchParamsArray(UPhysicsComponent* PC, const FSpinCacheIterCheckData& Data)
{
    const auto ParabolicCache = PC->PhysicsCache->ParabolicMotionCache;
    const auto SpinCache = PC->PhysicsCache->BallLaunchCache;
    const auto SpinCacheParams = &PC->SpinMovementParams->Data;

    const int MinParabolicCachedAngle = ParabolicCache->GetMinLaunchAngle();
    
    // passed
    
    const FVector VToTarget = Data.GetVectorToTarget();

    const int MinLaunchAngle = Data.GetMinLaunchAngle();
    const int MaxLaunchAngle = Data.GetMaxLaunchAngle();
    const int LaunchAngleDelta = Data.GetLaunchAngleDelta();

    const int MinFrontSpinAngle = Data.GetMinFrontSpinAngle();
    const int MaxFrontSpinAngle = Data.GetMaxFrontSpinAngle();
    const int FrontSpinAngleDelta = Data.GetFrontSpinAngleDelta();

    const int MinSideSpinAngle = Data.GetMinSideSpinAngle();
    const int MaxSideSpinAngle = Data.GetMaxSideSpinAngle();
    const int SideSpinAngleDelta = Data.GetSideSpinAngleDelta();

    const int MinLaunchSpeedPassed_KMpH = Data.GetMinLaunchSpeed_KMpH();
    const int MaxLaunchSpeed_KMpH = Data.GetMaxLaunchSpeed_KMpH();
    const int LaunchSpeedDelta_KMpH = Data.GetLaunchSpeedDelta_KMpH();
    
    //
    
    const float DistanceXY = VToTarget.Size2D() * 1.01f;
    const float DistanceZ = VToTarget.Z;
    constexpr float AbsDerivationZ = 0.0f;
    constexpr float MulDistanceXY = 1.01f;

    TArray<FBallLaunchParams> Out = {};
    
    // todo: clamp values by max in each loop
    
    for (int launch_angle = MinLaunchAngle; launch_angle <= MaxLaunchAngle; launch_angle += LaunchAngleDelta)
    {
        int MinLaunchSpeed_KMpH = MinLaunchSpeedPassed_KMpH;
        if(launch_angle >= MinParabolicCachedAngle)
        {
            const float ParabolicSpeed = PC->GetRealRequiredParabolicSpeedForAngle(launch_angle, VToTarget);
            const float ParabolicSpeedKMpH = UHM::FCMSec2Kmph(ParabolicSpeed);
            if(ParabolicSpeedKMpH > MinLaunchSpeed_KMpH)
            {
                MinLaunchSpeed_KMpH = SpinCacheParams->GetLaunchSpeedSnappedToGrid_KMpH(ParabolicSpeedKMpH);
            }
        }
        
        for (int launch_speed_kmph = MinLaunchSpeed_KMpH; launch_speed_kmph <= MaxLaunchSpeed_KMpH; launch_speed_kmph += LaunchSpeedDelta_KMpH)
        {
            for (int front_spin_angle = MinFrontSpinAngle; front_spin_angle <= MaxFrontSpinAngle; front_spin_angle += FrontSpinAngleDelta)
            {
                for (int side_spin_angle = MinSideSpinAngle; side_spin_angle <= MaxSideSpinAngle; side_spin_angle += SideSpinAngleDelta)
                {
                    const float launch_speed_cm_sec = UHM::FKmph2CMSec(launch_speed_kmph);
                    auto LaunchParams = FBallLaunchParams(launch_speed_cm_sec, launch_angle, front_spin_angle, side_spin_angle);

                    const bool CanUseLaunchParams = SpinCacheParams->CanUseLaunchParams(LaunchParams);
                    if(!CanUseLaunchParams) continue;

                    const bool bReaching = SpinCache->CanInputReachTarget(LaunchParams, DistanceXY, DistanceZ, AbsDerivationZ, MulDistanceXY);
                    if(bReaching && !Out.Contains(LaunchParams))
                    {
                        Out.Add(LaunchParams);
                    }
                }
            }
        }
    }
    
    return Out;
}

TArray<FPhysTransform> UKickSystemLib::CalculateApplicableImpactDataCacheBased(UPhysicsComponent* PC, const TArray<FBallLaunchParams>& Data, FVector VToTarget)
{
    TArray<FPhysTransform> Out = {};

    for (auto LaunchParams : Data)
    {
        constexpr bool bRemoveVelocities = true;
        auto TImpact = BallLaunchParamsToImpact(PC, LaunchParams, VToTarget, bRemoveVelocities);
        const bool HasSameImpact = Out.Contains(TImpact);
        if(!HasSameImpact)  Out.Add(TImpact);
    }
    
    return Out;
}

FPhysTransform UKickSystemLib::BallLaunchParamsToImpact(UPhysicsComponent* PC, const FBallLaunchParams& P, FVector VToTarget, bool bRemoveVelocities)
{
    VToTarget = UHMV::TrimVectorZ(VToTarget);
    const auto Impulse = USpinMovementLib::GetImpulseFromBallLaunchParams(PC, P, PC->GetCurrentLocation(), VToTarget);
    return PC->CalculateImpulseImpactAtLocation(Impulse.Impulse, Impulse.ApplyLocation, bRemoveVelocities);
}
