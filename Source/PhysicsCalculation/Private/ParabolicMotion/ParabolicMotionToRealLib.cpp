// Fill out your copyright notice in the Description page of Project Settings.

#include "ParabolicMotion/ParabolicMotionToRealLib.h"
#include "VisualDebugLib.h"
#include "Components/AdvancedPhysicsComponent.h"
#include "DataAssets/ParabolicCacheParams_DataAsset.h"
#include "ImpulseDistribution/ImpulseDistributionLib.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "ParabolicMotion/ParabolicMotionCache.h"
#include "ParabolicMotion/ParabolicMotionLib.h"

float UParabolicMotionToRealLib::CalculateDirectAngle(FVector TargetLocation, FVector LaunchLocation)
{
    const FVector Opposite = FVector(0.0f, 0.0f, TargetLocation.Z - LaunchLocation.Z);
    const FVector H = TargetLocation - LaunchLocation;
    return UKismetMathLibrary::DegAsin(Opposite.Size() / H.Size());
}

UParabolicMotionCache* UParabolicMotionToRealLib::ComputeParabolicMultipliers(UAdvancedPhysicsComponent* Obj)
{
    UParabolicMotionCache* Out = NewObject<UParabolicMotionCache>();
    Out->Data = ComputeParabolicMultipliersData(Obj);
    return Out;
}

FParabolicMotionCache_Data UParabolicMotionToRealLib::ComputeParabolicMultipliersData(UAdvancedPhysicsComponent* Obj)
{
    FParabolicMotionCache_Data Out;
    
    const auto Params = Obj->ParabolicCacheParams;
    const float ObjRadius = Obj->GetRadius();
    
    auto LaunchAngles = Params->GetLaunchAngles();
    const auto LaunchLocations = Params->GetLaunchLocations(ObjRadius);
    const auto TargetLocation = Params->TargetLocation;

    for (const auto Angle : LaunchAngles)
    {
        auto MultiplierCurve = GetParabolicMultiplierCurveForLaunchAngle(Obj, TargetLocation, Angle, LaunchLocations);
        Out.AddLaunchAngleData(Angle, MultiplierCurve);
    }

    int Min, Max;
    Out.ComputeMinMaxLaunchAngle(Min, Max);
    Out.MaxLaunchCurve = CalculateMaxLaunchAngleCurve(Obj, Min, Max);
    
    return Out;
}

FParabolicMotionCurve UParabolicMotionToRealLib::GetParabolicMultiplierCurveForLaunchAngle(UAdvancedPhysicsComponent* Obj, FVector TargetLocation,
                                                                                           float LaunchAngle, const TArray<FVector>& LaunchLocations)
{
    FPMCompData CompData = CreateParabolicCompParams(Obj, TargetLocation, FVector::ZeroVector, LaunchAngle);

    FParabolicMotionCurve Out;
    Out.LaunchAngle = LaunchAngle;
    
    const auto RefCurve = Out.MultiplierCurve.GetRichCurve();
    const auto MaxDistanceCurve = Out.MaxDistanceCurve.GetRichCurve();
    
    for (const auto Location : LaunchLocations)
    {
        CompData.ChangeLaunchLocation(Location);

        const float DirectAngle = CalculateDirectAngle(TargetLocation, Location);
        if(LaunchAngle >= DirectAngle)
        {
            const float DistanceX = TargetLocation.X - Location.X;
            float LaunchSpeed, RealMaxDistance;
            const float Multiplier = GetParabolicMultiplier(Obj, CompData, LaunchSpeed, RealMaxDistance);
            RefCurve->AddKey(DistanceX, Multiplier);
            MaxDistanceCurve->AddKey(LaunchSpeed, RealMaxDistance);
        }
        else
        {
            constexpr bool bLog = !true;
            if(bLog)
            {
                const FString S = "Skip location " + Location.ToString() + " for angle " + FString::SanitizeFloat(LaunchAngle) + " (direct is: "+ FString::SanitizeFloat(DirectAngle) + ")";
                PrintToLog(S);
            }
        }
    }
    
    return Out;
}

FPMCompData UParabolicMotionToRealLib::CreateParabolicCompParams(UAdvancedPhysicsComponent* Obj, FVector TargetLocation, FVector LaunchLocation, float LaunchAngle)
{
    FPMCompData CompData;
    
    const auto Params = Obj->ParabolicCacheParams;
    const float TimeStep = Params->ComputationTimeStep;
    const int NumSteps = Params->GetComputationNumSteps();
    const float Tolerance = Params->ComputationLocationTolerance;
    const float MultiplierChangeStep = Params->MultiplierChangeStep;
    CompData.Init(TargetLocation, LaunchLocation, LaunchAngle, TimeStep, NumSteps, Tolerance, MultiplierChangeStep);

    return CompData;
}

float UParabolicMotionToRealLib::GetParabolicMultiplier(UAdvancedPhysicsComponent* Obj, const FPMCompData& CompData, float& LaunchSpeed, float& MaxDistance)
{
    const float LaunchAngle = CompData.LaunchAngle;
    const FVector TargetLocation = CompData.TargetLocation;
    const FVector LaunchLocation = CompData.LaunchLocation;
    const FVector V = CompData.GetVectorToTargetRelative();

    check(V.X >= 0.0f)

    const float DirectAngle = CalculateDirectAngle(TargetLocation, LaunchLocation);
    const bool bInvalidAngle = LaunchAngle < DirectAngle;
    if(bInvalidAngle)
    {
        PrintToLog("Invalid angle for parabolic multiplier");
        return 1.0f;
    }
    
    const float ParabolicSpeed = UParabolicMotionLib::GetRequiredLaunchSpeedForAngle(LaunchAngle, V.X, V.Z);
    const FVector LaunchDirection = UHMV::GetWorldForwardRotated_XY_XZ(0.0f, LaunchAngle);

    const FVector LV_Base = LaunchDirection * ParabolicSpeed;
    const FVector AV = FVector::ZeroVector;
    const auto T_Parabolic = FPhysTransform(LaunchLocation, FQuat::Identity, LV_Base, AV);

    const float OutMultiplier = GetParabolicMultiplier_Body(Obj, CompData, T_Parabolic, 1.0f);

    {
        auto TResult = T_Parabolic;
        TResult.ScaleLinearVelocity(OutMultiplier);
        const auto RealCurve = CalculateMotionCurveFromTransform(Obj, TResult, CompData.TimeStep, CompData.NumSteps);
        float GroundTime;
        const bool bValid = RealCurve.GetLastTimeWhenZEquals(GroundTime, 0.0f);
        check(bValid)
        
        LaunchSpeed = TResult.LinearVelocity.Size();
        MaxDistance = (RealCurve.GetVectorValue(GroundTime) - TResult.Location).Size2D();
    }
    
    return OutMultiplier;
}

float UParabolicMotionToRealLib::GetParabolicMultiplier_Body(UAdvancedPhysicsComponent* Obj, const FPMCompData& CompData, const FPhysTransform& T_Parabolic, float BaseMultiplier)
{
    const auto Comparison = GetComparisonBaseToOneStep(Obj, T_Parabolic, CompData.TimeStep, CompData.NumSteps, BaseMultiplier, CompData.MultiplierChangeStep);

    const float MulIncreaseStep = CompData.MultiplierChangeStep;
    const FVector Target = CompData.TargetLocation;
    const float TargetX = Target.X;
    const float TargetZ = Target.Z;
    const auto CurveBase = &Comparison.CurveBase;
    const float StepDeltaX = Comparison.GetStepDeltaX();
    const float StepDeltaZ = Comparison.GetStepDeltaZ();
    constexpr float XAdditionalOffset = 15.0f;
    const float TargetDeltaX = TargetX  + XAdditionalOffset - Comparison.GetBaseXWhenZeroZ();

    TArray<FVector> LocationsSearchX;
    const bool bBaseReachX = CurveBase->GetAllVectorsWhereXEquals(TargetX, LocationsSearchX);

    if(bBaseReachX)
    {
        const FVector NearestLocation = UHMV::GetNearestLocation(LocationsSearchX, Target);
        const float TargetDeltaZ = TargetZ - NearestLocation.Z;
        const float Distance = FMath::Abs(TargetDeltaZ);
        
        if(Distance <= CompData.LocationTolerance)
        {
            return BaseMultiplier;
        }

        const float MulChange = ComputeMultiplierChangeValue(TargetDeltaZ, StepDeltaZ, MulIncreaseStep);
        const float NewMultiplier = BaseMultiplier + MulChange;
        return GetParabolicMultiplier_Body(Obj, CompData, T_Parabolic, NewMultiplier);
    }
    
    const float MulChange = ComputeMultiplierChangeValue(TargetDeltaX, StepDeltaX, MulIncreaseStep);
    const float NewMultiplier = BaseMultiplier + MulChange;
    return GetParabolicMultiplier_Body(Obj, CompData, T_Parabolic, NewMultiplier);
}

float UParabolicMotionToRealLib::ComputeMultiplierChangeValue(float DistanceDelta, float StepDelta, float MulIncreaseStep)
{
    return (DistanceDelta / StepDelta) * MulIncreaseStep;
}

FTrajectoryStepComparison UParabolicMotionToRealLib::GetComparisonBaseToOneStep(UAdvancedPhysicsComponent* Obj, const FPhysTransform& T_Parabolic, float TimeStep, int NumSteps, float BaseMultiplier, float MultiplierStep)
{
    FTrajectoryStepComparison Out;
    
    auto T_Base = T_Parabolic;
    T_Base.LinearVelocity *= BaseMultiplier;
    
    auto T_FirstIter = T_Base;
    T_FirstIter.LinearVelocity *= BaseMultiplier + MultiplierStep;

    const auto CurveBase = CalculateMotionCurveFromTransform(Obj, T_Base, TimeStep, NumSteps);
    const auto CurveFirstIter = CalculateMotionCurveFromTransform(Obj, T_FirstIter, TimeStep, NumSteps);

    float t0, t1;
    const bool b0 = CurveBase.GetLastTimeWhenZEquals(t0);
    const bool b1 = CurveFirstIter.GetLastTimeWhenZEquals(t1);
    check(b0)
    check(b1)

    const float X_BaseZero = CurveBase.GetVectorValue(t0).X;
    const float X_FirstIterZero = CurveFirstIter.GetVectorValue(t1).X;
    const float StepDeltaZ = CurveFirstIter.GetVectorValue(t0).Z;

    const bool bExpectedChangeX = X_FirstIterZero > X_BaseZero;

    if(!bExpectedChangeX)
    {
        PrintToLog("Base: " + FString::SanitizeFloat(X_BaseZero) + " Iter: " + FString::SanitizeFloat(X_FirstIterZero));
    }
    
    check(bExpectedChangeX)

    Out.CurveBase = CurveBase;
    Out.CurveFirstIter = CurveFirstIter;
    Out.X_BaseWhenZeroZ = X_BaseZero;
    Out.X_OneStepWhenZeroZ = X_FirstIterZero;
    Out.StepDeltaZ = StepDeltaZ;

    return Out;
}

void UParabolicMotionToRealLib::DrawCurveTrajectory(UObject* Obj, FCustomVectorCurve Curve, FLinearColor Color)
{
    UVisualDebugLib::DrawTrajectory(Obj, Curve.GetVectorKeys(), 100000000, Color);
}

int UParabolicMotionToRealLib::GetLaunchAngleMaxDistance(UAdvancedPhysicsComponent* Obj, float LaunchSpeed, float TimeStep, int NumSteps, int MinAngle, int MaxAngle)
{
    float MaxDistance = 0.0f;
    
    for (int Angle = MinAngle; Angle <=MaxAngle; ++Angle)
    {
        auto T = Obj->CurrentTransform;
        T.LinearVelocity = UHMV::GetWorldForwardRotatedScaled_XY_XZ(LaunchSpeed, 0.0f, Angle);;

        auto Curve = Obj->PredictMovementFromTransformAndGetTrajectoryAsCurveAnyTimeStep(T, TimeStep, NumSteps, false, true);

        float Time;
        const bool bGroundContact = Curve.GetLastTimeWhenZEquals(Time,0.0f);
        check(bGroundContact)

        FVector Location = Curve.GetVectorValue(Time);
        FVector V = Location - T.Location;
        const float Distance = V.Size2D();

        if(Distance > MaxDistance)
        {
            MaxDistance = Distance;
        }
        else
        {
            return Angle - 1;
        }
    }
    return -1;
}

FRuntimeFloatCurve UParabolicMotionToRealLib::CalculateMaxLaunchAngleCurve(UAdvancedPhysicsComponent* Obj, int MinLaunchAngle, int MaxLaunchAngle)
{
    FRuntimeFloatCurve OutCurve;
    const auto Curve = OutCurve.GetRichCurve();

    constexpr float MaxLaunchSpeed_Kmph = 230.0f;
    constexpr float LaunchSpeedDelta_Kmph = 2.0f;
    const float LaunchSpeedDelta = UHM::FKmph2CMSec(LaunchSpeedDelta_Kmph);
    constexpr int NumSpeedIter = MaxLaunchSpeed_Kmph / LaunchSpeedDelta_Kmph;

    const float PhysDeltaTime = Obj->GetRoughPredictSimStep();
    constexpr int NumPhysSteps = 5000;

    for (int i = 1; i <= NumSpeedIter ; ++i)
    {
        const float LaunchSpeed = LaunchSpeedDelta * i;
        const int Angle = GetLaunchAngleMaxDistance(Obj, LaunchSpeed, PhysDeltaTime, NumPhysSteps, MinLaunchAngle, MaxLaunchAngle);
        Curve->AddKey(LaunchSpeed, Angle);
    }
    
    return OutCurve;
}

void UParabolicMotionToRealLib::CalculateSpinTrajectories(UAdvancedPhysicsComponent* Obj, float LaunchSpeed, float LaunchAngle, float LaunchAngleXY, float FrontSpinAngle, float SideSpinAngle)
{
    checkNoEntry()
    return;
    
    const FVector COM = Obj->GetCurrentLocation();
    const FVector VZ = FVector::ZeroVector;
    const FVector ParabolicVelocity = LaunchSpeed * UHMV::GetVectorRotated_XY_XZ(FVector::ForwardVector, LaunchAngleXY, LaunchAngle);
    const auto ImpulseData = UImpulseDistributionLib::ReconstructImpulseFromVelocities(Obj, ParabolicVelocity, VZ, COM);
    const FVector ParabolicApplyLocation = ImpulseData.ApplyLocation;
    const FVector ParabolicImpulse = ImpulseData.Impulse;

    const auto CorrectImpulse = UImpulseDistributionLib::GetImpulseFromParabolicAndSpin(Obj, ParabolicVelocity, ParabolicApplyLocation, COM, FrontSpinAngle, SideSpinAngle, true);
    
    const FVector Impulse = CorrectImpulse.Impulse;
    const FVector ApplyLocation = CorrectImpulse.ApplyLocation;

    UKismetSystemLibrary::DrawDebugSphere(Obj, ParabolicApplyLocation, 1, 10, FLinearColor::White, 1000000, 0);
    // UKismetSystemLibrary::DrawDebugLine(Obj, ApplyLocation, ImpulseQuat.GetForwardVector() * 100 + ApplyLocation, FLinearColor::Green, 10000, 3);
    UKismetSystemLibrary::DrawDebugLine(Obj, ApplyLocation, ParabolicVelocity.GetSafeNormal() * 100 + ApplyLocation, FLinearColor::White, 10000, 1);
    UKismetSystemLibrary::DrawDebugLine(Obj, ApplyLocation, Impulse.GetSafeNormal() * 100 + ApplyLocation, FLinearColor::Green, 10000, 1);
    
    const auto TSpin = Obj->CalculateImpulseImpactAtLocationOtherCOM(Impulse, ApplyLocation, COM, FVector::ZeroVector, true);

    // const auto InvImpulseQuat = UImpulseDistributionLib::GetImpulseQuatFromParabolicVelocityAndSpin(ParabolicVelocity, -FrontSpinAngle, SideSpinAngle);
    // const FVector InvImpulse = InvImpulseQuat.GetForwardVector() * ParabolicImpulse.Size();
    // const auto TInvSpin = Obj->CalculateImpulseImpactAtLocation(InvImpulse, ApplyLocation, FVector::ZeroVector);

    auto TParabolic = Obj->CurrentTransform;
    TParabolic.LinearVelocity = ParabolicVelocity;
    TParabolic.Location = COM;

    const float TimeStep = Obj->ParabolicCacheParams->ComputationTimeStep;
    constexpr int NumSteps = 5000;

    const auto ParabolicCurve = CalculateMotionCurveFromTransform(Obj, TParabolic, TimeStep, NumSteps);
    const auto SpinCurve = CalculateMotionCurveFromTransform(Obj, TSpin, TimeStep, NumSteps);
    // const auto InvSpinCurve = CalculateMotionCurveFromTransform(Obj, TInvSpin, TimeStep, NumSteps);

    // todo: spin vector must be clamped; if we are clamp -> break after this iteration

    // PrintToLog("PV: " + ParabolicVelocity.ToString() + " (" + FString::SanitizeFloat(ParabolicVelocity.Size()) + ") " + " | SV: " + TSpin.LinearVelocity.ToString() + " (" + FString::SanitizeFloat(TSpin.LinearVelocity.Size()) + ")");

    FVector G;
    GetTrajectoryGroundLocation(SpinCurve, G, true);
    float Distance = (G - COM).Size2D();
    PrintToLog(FString::SanitizeFloat(Distance));
    
    DrawCurveTrajectory(Obj, ParabolicCurve, FLinearColor::White);
    DrawCurveTrajectory(Obj, SpinCurve, FLinearColor::Red);
    // DrawCurveTrajectory(Obj, InvSpinCurve, FLinearColor::Blue);
}

void UParabolicMotionToRealLib::TestTrajectoryCalcTime(UAdvancedPhysicsComponent* Obj, float LaunchSpeed, float LaunchAngle, float MaxFrontSpinAngle,
    float MaxSideSpinAngle, int NumSpeedIterations, int NumAngleIterations, TArray<FVector>& Out)
{
    checkNoEntry()
    return;
    
    const int TrajectoryCountForSpeed = MaxFrontSpinAngle * MaxSideSpinAngle;
    const int TotalIterations = TrajectoryCountForSpeed * NumSpeedIterations * NumAngleIterations;

    const float TimeStep = Obj->GetRoughPredictSimStep();
    constexpr int NumSteps = 5000;

    const auto Before = UKismetMathLibrary::UtcNow();
    
    for (int i = 0; i < TotalIterations; ++i)
    {
        const FVector ParabolicVelocity = LaunchSpeed * UHMV::GetVectorRotated_XY_XZ(FVector::ForwardVector, 0, LaunchAngle);
        auto T = Obj->CurrentTransform;
        T.LinearVelocity = ParabolicVelocity;
        auto D = CalculateMotionCurveFromTransform(Obj, T, TimeStep, NumSteps);
        auto V = D.GetValueAtZero();
        Out.Add(V);
    }
    const auto After = UKismetMathLibrary::UtcNow();

    const auto TimeDelta = After - Before;
    const auto Passed = TimeDelta.ToString();
    PrintToLog("Time passed: " + Passed);
}

bool UParabolicMotionToRealLib::GetTrajectoryGroundLocation(const FCustomVectorCurve& Trajectory, FVector& OutLocation, bool bCheckForTrue)
{
    float GroundTime;
    const bool bGround = Trajectory.GetLastTimeWhenZEquals(GroundTime, 0.0f);
    if(bCheckForTrue) check(bGround)
    OutLocation = Trajectory.GetVectorValue(GroundTime);
    return bGround;
}

FCustomVectorCurve UParabolicMotionToRealLib::CalculateMotionCurveFromTransform(UAdvancedPhysicsComponent* Obj, const FPhysTransform& T, float TimeStep, int NumSteps)
{
    constexpr float LimitZ = 0.0f;
    constexpr bool bLimitZ = true;
    constexpr bool bCheckCollisions = false;
    return Obj->PredictMovementFromTransformAndGetTrajectoryAsCurveAnyTimeStep(T, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
}

FCustomVectorCurve UParabolicMotionToRealLib::CalculateParabolicMotionCurve(UAdvancedPhysicsComponent* Obj, const FVector& LinearVelocity,
    const FVector& LaunchLocation, float TimeStep, int NumSteps)
{
    const FPhysTransform T = FPhysTransform(LaunchLocation, FQuat::Identity, LinearVelocity, FVector::ZeroVector);
    return CalculateMotionCurveFromTransform(Obj, T, TimeStep, NumSteps);
}
