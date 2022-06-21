// Fill out your copyright notice in the Description page of Project Settings.

#include "Libs/TrajectoryPredictionLib.h"
#include "Libs/PhysicsSimulation.h"
#include "Components/AdvancedPhysicsComponent.h"

FTrajectoryData UTrajectoryPredictionLib::CalculateCorrectedTrajectoryDataFromImpulse(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData)
{
    CompData.Validate();
    const auto TrajectoryData = CalculateTrajectoryDataFromImpulse(Obj, CompData);
    const float MinDistance = TrajectoryData.GetMinDistance();
    const bool bAdjustRequired = MinDistance > CompData.ApplicableDistanceToTarget;

    if(CompData.bDebug)
    {
        return TrajectoryData;
    }
    
    return  bAdjustRequired ? GetCorrectedTrajectoryData(Obj, CompData, TrajectoryData) : TrajectoryData;
}

FTrajectoryData UTrajectoryPredictionLib::CalculateTrajectoryDataFromImpulse(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData)
{
    FTrajectoryData OutData;

    const bool bLimitSpinVector = CompData.bLimitSpinVector;
    const float FrontSpinLimit = CompData.GetFrontSpinLimitRadSec();
    const float SideSpinLimit = CompData.GetSideSpinLimitRadSec();
    
    constexpr bool  bCheckCollisions = false;
    constexpr bool  bLimitZ = true;
    const float LimitZ = CompData.LimitMinZ;

    const FVector Target = CompData.Target;
    const FVector Impulse = CompData.Impulse;
    const FVector ApplyLocation = CompData.ApplyLocation;
    const float SimStep = CompData.SimStep;
    const int NumSteps = CompData.NumSteps;

    FVector COM = Obj->GetCurrentLocation();

    bool bRemoveLV = false;
    auto Impact = Obj->CalculateImpulseImpactAtLocationOtherCOM(Impulse, ApplyLocation, COM, FVector::ZeroVector, bRemoveLV);
    if(bLimitSpinVector)
    {
        Impact.AngularVelocity = LimitSpinVector(Impact.AngularVelocity, FrontSpinLimit, SideSpinLimit);       
    }
    
    const auto Points = Obj->PredictMovementFromTransformAnyTimeStepAndGetLocations(Impact, SimStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
    const auto Trajectory = FCustomVectorCurve(Points, SimStep);
        
    OutData.Init(Impact, Trajectory, Target);
    
    return OutData;
}

FTrajectoryData UTrajectoryPredictionLib::GetCorrectedTrajectoryData(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData)
{
    float DistanceDeltaXY = 0.0f;
    FVector ClosestPoint = FVector::ZeroVector;
    const bool CanReachXY = CanTrajectoryReachTarget(Obj, CompData, PrevData, ClosestPoint, DistanceDeltaXY);
    
    if(CanReachXY)
    {
        return GetCorrectedTrajectoryFromReachable(CompData, PrevData, ClosestPoint);
    }
    
    return GetCorrectedTrajectoryFromUnreachable(Obj, CompData, PrevData, DistanceDeltaXY);
}

bool UTrajectoryPredictionLib::CanTrajectoryReachTarget(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, FVector& OutClosestPoint, float& OutDistanceDeltaXY)
{
    const float OverallApplicableDistance = CompData.ApplicableDistanceToTarget; 
    const float TrajectoryFarCheckDeclineAngleOffset = CompData.TrajectoryFarCheckDeclineAngleOffset;
    const FVector Target = CompData.Target;
    const FVector CurrentLocation = PrevData.Trajectory.GetValueAtZero();
    OutClosestPoint = PrevData.GetVectorOfMinDistance();

    const FVector VToTarget = Target - CurrentLocation;
    const FVector FarPoint = GetTrajectoryFarPointInBaseDirection(Target, CurrentLocation, OutClosestPoint, PrevData, TrajectoryFarCheckDeclineAngleOffset);

    const float MaxDistance = (FarPoint - CurrentLocation).Size2D();
    OutDistanceDeltaXY = MaxDistance - VToTarget.Size2D();
    
    return  OutDistanceDeltaXY >= OverallApplicableDistance;
}

FTrajectoryData UTrajectoryPredictionLib::GetCorrectedTrajectoryFromReachable(const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, FVector ClosestPoint)
{
    const FVector Target = CompData.Target;
    const FVector CurrentLocation = PrevData.Trajectory.GetValueAtZero();

    const auto VToTarget = FDetailedVector(Target - CurrentLocation);
    const auto VToClosest = FDetailedVector(ClosestPoint - CurrentLocation);
    
    const FVector VTargetXY = VToTarget.GetV_XY();
    const FVector VClosestXY = VToClosest.GetV_XY();
    const FVector RightVectorTargetDir = UHMV::RotateVectorZAxis(VTargetXY, 90.0f).GetSafeNormal();
    
    const float AngleTargetToPlaneXY = UHMV::GetAngleVectorToPlaneXY(VToTarget.GetV());
    const float AngleClosestToPlaneXY = UHMV::GetAngleVectorToPlaneXY(VToClosest.GetV());

    const float AngleXY = UHMV::SignedAngleBetweenVectorsDegXY(VClosestXY, VTargetXY);
    const float AngleXZ = AngleClosestToPlaneXY - AngleTargetToPlaneXY;

    FQuat QuatXY = UHMR::QuatZAxisAngleDeg(AngleXY);
    FQuat QuatXZ = UHMR::QuatAxisAngleDeg(RightVectorTargetDir, AngleXZ);

    if(CompData.bRemoveQuatXY) QuatXY = FQuat::Identity;
    if(CompData.bRemoveQuatXZ) QuatXZ = FQuat::Identity;

    const FQuat QLinear = QuatXZ * QuatXY;
    const FQuat QAngular = QuatXY;

    auto NewData = PrevData;
    NewData.RotateByQuat(QLinear, QAngular);

    // todo: make final adjust after rotation applied
    
    return NewData;
}

FTrajectoryData UTrajectoryPredictionLib::GetCorrectedTrajectoryFromUnreachable(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, float DistanceDeltaXY)
{
    PrintToLog("Trajectory cant reach target");
    
    const float PhysSimStep = CompData.SimStep;
    const int PhysNumSteps = CompData.NumSteps;
    const float VelocityMultiplier = CompData.GetVelocityComparisonMultiplier();
    const float VelocityMultiplierStep = CompData.GetVelocityComparisonMultiplierStep();
    const float FrontSpinMul = CompData.GetFrontSpinDecreaseMul();
    const float SideSpinMul = CompData.GetSideSpinDecreaseMul();
    const FVector Target = CompData.Target;
    const FVector CurrentLocation = PrevData.Trajectory.GetValueAtZero();

    FTrajectoryData ValidTrajectoryData;
    auto StepDeltaX = -1;

    const bool bOld = true;
    if(bOld)
    {
        StepDeltaX = GetTrajectoryDeltaX(Obj, PrevData, Target, CurrentLocation, PhysSimStep, PhysNumSteps, VelocityMultiplier);
        const bool bValidStepDelta = StepDeltaX > 0.0f;
        if(!bValidStepDelta)
        {
            // decrease angular velocity by fixed amount in % and recalculate
            // best option where we are decreasing only top spin component
            // when velocity is pointed to X top spin is positive Y axis
            // but this happens on back spin as well, we should decrease
            // front spin always is XY component of angular velocity -> easy to deal with
        
            auto TNew = PrevData.TOrigin;
            ScaleTransformAngularVelocity(TNew, FrontSpinMul, SideSpinMul);
        
            checkNoEntry()
        }
    }
    else
    {
        StepDeltaX = GetTrajectoryValidDeltaX(Obj, CompData, PrevData, ValidTrajectoryData);

        // if we change params -> pointless to use DistanceDeltaXY its incorrect
    }
    
    float VelocityMultiplierChange = ComputeMultiplierChangeValue(DistanceDeltaXY, StepDeltaX, VelocityMultiplierStep);
    VelocityMultiplierChange = FMath::Abs(VelocityMultiplierChange);
    const float NewVelocityMultiplier = VelocityMultiplier + VelocityMultiplierChange;
    
    auto T = PrevData.TOrigin;
    T.ScaleLinearVelocity(NewVelocityMultiplier);

    auto OutData = FTrajectoryData();
    const auto Trajectory = ComputeTrajectoryFromTransform(Obj, T, PhysSimStep, PhysNumSteps);
    OutData.Init(T, Trajectory, CompData.Target);

    return GetCorrectedTrajectoryData(Obj, CompData, OutData);
}

FVector UTrajectoryPredictionLib::GetTrajectoryFarPointInBaseDirection(FVector Target, FVector Origin, FVector ClosestPoint, const FTrajectoryData& Data, float AngleOffset)
{
    float GroundContactTime;
    const bool HasGroundContact = Data.Trajectory.GetLastTimeWhenZEquals(GroundContactTime);
    if(HasGroundContact)
    {
        const FVector GroundContactPoint  = Data.Trajectory.GetVectorValue(GroundContactTime);

        const FVector ToGroundXY = UHMV::TrimVectorZ(GroundContactPoint - Origin);
        const FVector ToTargetXY = UHMV::TrimVectorZ(Target - Origin);
        const FVector ToClosestXY = UHMV::TrimVectorZ(ClosestPoint - Origin);

        const float DistanceToClosest = ToClosestXY.Size();
        const float DistanceToGround = ToGroundXY.Size();
        const bool IsGroundMoreDistant = DistanceToGround > DistanceToClosest;
        
        const float Angle_Ground = UHMV::AngleBetweenVectorsDeg(ToTargetXY, ToGroundXY);
        const float Angle_Closest = UHMV::AngleBetweenVectorsDeg(ToTargetXY, ToClosestXY);
        const bool CanUseGround = Angle_Ground <= Angle_Closest + AngleOffset;

        const bool B = IsGroundMoreDistant && CanUseGround;
        return B ? GroundContactPoint : ClosestPoint;        
    }
    return ClosestPoint;
}

float UTrajectoryPredictionLib::ComputeMultiplierChangeValue(float DistanceDelta, float StepDelta, float MulIncreaseStep)
{
    return (DistanceDelta / StepDelta) * MulIncreaseStep;
}

float UTrajectoryPredictionLib::GetTrajectoryDeltaX(UAdvancedPhysicsComponent* Obj, const FTrajectoryData& BaseData, FVector Target, FVector Origin, float TimeStep, int NumSteps, float VelocityMul)
{
    check(VelocityMul > 1.005f)

    const FVector BaseDir = (Target - Origin).GetSafeNormal2D();
    
    auto T_FirstIter = BaseData.TOrigin;
    T_FirstIter.LinearVelocity *= VelocityMul;

    constexpr float LimitZ = 0.0f;
    constexpr bool bLimitZ = true;
    constexpr bool bCheckCollisions = false;

    const auto CurveBase = BaseData.Trajectory;
    const auto CurveFirstIter = Obj->PredictMovementFromTransformAndGetTrajectoryAsCurveAnyTimeStep(T_FirstIter, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);

    float t0, t1;
    const bool b0 = CurveBase.GetLastTimeWhenZEquals(t0);
    const bool b1 = CurveFirstIter.GetLastTimeWhenZEquals(t1);

    check(b0)
    check(b1)

    const FVector V0 = CurveBase.GetVectorValue(t0);
    const FVector V1 = CurveFirstIter.GetVectorValue(t1);
    const FVector VP0 = V0.ProjectOnToNormal(BaseDir);
    const FVector VP1 = V1.ProjectOnToNormal(BaseDir);

    const float Distance0 = VP0.Size();
    const float Distance1 = VP1.Size();
    
    return  Distance1 - Distance0;
}

float UTrajectoryPredictionLib::GetTrajectoryValidDeltaX(UAdvancedPhysicsComponent* Obj, const FTrajectoryDataCompData& CompData, const FTrajectoryData& PrevData, FTrajectoryData& OutData)
{
    const float PhysSimStep = CompData.SimStep;
    const int PhysNumSteps = CompData.NumSteps;
    const float VelocityMultiplier = CompData.GetVelocityComparisonMultiplier();
    const float FrontSpinMul = CompData.GetFrontSpinDecreaseMul();
    const float SideSpinMul = CompData.GetSideSpinDecreaseMul();
    const FVector Target = CompData.Target;
    const FVector CurrentLocation = PrevData.Trajectory.GetValueAtZero();

    const auto StepDeltaX = GetTrajectoryDeltaX(Obj, PrevData, Target, CurrentLocation, PhysSimStep, PhysNumSteps, VelocityMultiplier);
    const bool bValidStepDelta = StepDeltaX > 0.0f;

    if(bValidStepDelta) return StepDeltaX;

    auto TNew = PrevData.TOrigin;
    ScaleTransformAngularVelocity(TNew, FrontSpinMul, SideSpinMul);

    // todo: recompute data and return result

    // return GetTrajectoryValidDeltaX()
    
    if(!bValidStepDelta)
    {
        // decrease angular velocity by fixed amount in % and recalculate
        // best option where we are decreasing only top spin component
        // when velocity is pointed to X top spin is positive Y axis
        // but this happens on back spin as well, we should decrease
        // front spin always is XY component of angular velocity -> easy to deal with
        checkNoEntry()
    }
    OutData = PrevData;
    return 0;
}

FCustomVectorCurve UTrajectoryPredictionLib::ComputeTrajectoryFromTransform(UCustomPhysicsComponent* Obj, const FPhysTransform& T, float TimeStep, int NumSteps)
{
    constexpr float LimitZ = 0.0f;
    constexpr bool bLimitZ = true;
    constexpr bool bCheckCollisions = false;

    return Obj->PredictMovementFromTransformAndGetTrajectoryAsCurveAnyTimeStep(T, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
}

FVector UTrajectoryPredictionLib::LimitSpinVector(const FVector& V, float FrontSpinLimit, float SideSpinLimit)
{
    const float SideSpin = FMath::Clamp(V.Z, -SideSpinLimit, SideSpinLimit);
    FVector VFrontSpin = FVector(V.X, V.Y, 0.0f).GetClampedToMaxSize(FrontSpinLimit);
    VFrontSpin.Z = SideSpin;
    return VFrontSpin;
}

FVector UTrajectoryPredictionLib::ScaleSpinVector(const FVector& V, float FrontSpinMul, float SideSpinMul)
{
    FVector AV = V;
    AV.X *= FrontSpinMul;
    AV.Y *= FrontSpinMul;
    AV.Z *= SideSpinMul;
    return AV;
}

void UTrajectoryPredictionLib::ScaleTransformAngularVelocity(FPhysTransform& T, float FrontSpinMul, float SideSpinMul)
{
    T.AngularVelocity = ScaleSpinVector(T.AngularVelocity, FrontSpinMul, SideSpinMul);    
}

bool UTrajectoryPredictionLib::IsTopSpin(float FrontSpinAngle)
{
    return FrontSpinAngle < 0.0f;;
}
