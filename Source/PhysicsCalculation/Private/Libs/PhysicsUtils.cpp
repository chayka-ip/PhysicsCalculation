// Fill out your copyright notice in the Description page of Project Settings.

#include "Libs/PhysicsUtils.h"

#include "Libs/BallGroundMovementData.h"
#include "HandyMathVectorLibrary.h"
#include "Libs/PhysicsSimulation.h"
#include "Components/CustomPhysicsBaseComponent.h"

float UPhysicsUtils::CombinePhysValue(float A, float B, EPhysicsCombineMode Mode)
{
    switch (Mode) {
        case Average: return (A + B)/2.0f ;
        case Min: return FMath::Min(A, B);
        case Multiply: return A * B;
        case Max: return FMath::Max(A, B);
        default: return  0.0f;
    }
}

EPhysicsCombineMode UPhysicsUtils::GetCombineModeFromDefaultPhysics(EFrictionCombineMode::Type Mode)
{
    switch (Mode)
    {
        case EFrictionCombineMode::Average: return Average;
        case EFrictionCombineMode::Min: return Min;
        case EFrictionCombineMode::Multiply: return Multiply;
        case EFrictionCombineMode::Max: return Max;
        default: return Average;
    }
}

EPhysicsCombineMode UPhysicsUtils::SelectPhysCombineMode(EPhysicsCombineMode A, EPhysicsCombineMode B)
{
    if(A == Multiply || B == Multiply) return  Multiply;
    if(A == Max || B == Max) return  Max;
    if(A == Average || B == Average) return Average;
    if(A == Min || B == Min) return Min;
    return Average;
}

FPhysTransform UPhysicsUtils::DiffTransform(const FPhysTransform& A, const FPhysTransform& B)
{
    return A-B;
}

FPhysTransform UPhysicsUtils::DiffTransformPure(const FPhysTransform& A, const FPhysTransform& B)
{
    return DiffTransform(A, B);
}

FTransform UPhysicsUtils::PhysTransformToTransform(const FPhysTransform PhysTransform, FVector Scale)
{
    return PhysTransform.ToTransform(Scale);
}

FTransform UPhysicsUtils::PhysTransformToTransformPure(const FPhysTransform PhysTransform, const FVector Scale)
{
    return PhysTransformToTransform(PhysTransform, Scale);
}

FPhysTransform UPhysicsUtils::TransformToPhysTransform(const FTransform Transform, const FVector LinearVelocity, const FVector AngularVelocity)
{
    return FPhysTransform(Transform, LinearVelocity, AngularVelocity);
}

FPhysTransform UPhysicsUtils::TransformToPhysTransformPure(const FTransform Transform, const FVector LinearVelocity, const FVector AngularVelocity)
{
    return TransformToPhysTransform(Transform, LinearVelocity, AngularVelocity);
}

FPhysTransform UPhysicsUtils::TLerp(const FPhysTransform& A, const FPhysTransform& B, float Alpha)
{
    constexpr float Tolerance = 0.0000001f;
    const FQuat Orientation = UMathUtils::Slerp(A.Orientation.GetNormalized(), B.Orientation.GetNormalized(), Alpha).GetNormalized();
    const FVector Location = UHMV::LerpVector(A.Location, B.Location, Alpha, Tolerance);
    const FVector LinearVelocity = UHMV::LerpVector(A.LinearVelocity, B.LinearVelocity, Alpha, Tolerance);
    const FVector AngularVelocity = UHMV::LerpVector(A.AngularVelocity, B.AngularVelocity, Alpha, Tolerance);

    return FPhysTransform(Location, Orientation, LinearVelocity, AngularVelocity);
}

FBallGroundMovementData UPhysicsUtils::ComputeBallGroundMovementData(const FPhysRigidBodyParams& P, float Time_Step)
{
    FBallGroundMovementData D;
    D.Simulate(P, Time_Step);
    return D;
}

float UPhysicsUtils::GetBallInitialVelocityFromEndVelocity(FBallGroundMovementData P, float EndVelocity, float Time)
{
    return P.GetInitialVelocityFromEndVelocity(EndVelocity, Time);
}

// float UPhysicsUtils::GetInitialVelocityDistancePassedVelocity2(FPhysRigidBodyParams P, float DistanceToPass, float Time)
// {
//     checkNoEntry()
//     return 0;
//     // fix error for times around 0.41 sec (10 frames => for hitframe)
//     const float magic_fix = 0.92f;
//     DistanceToPass *= magic_fix;
//
//     const float AbsTolerance = 2;
//     const float DT = 0.02f;
//     const int Steps = Time / DT;
//
//     float StartVelMin = 0;
//     float StartVelMax = 30000;
//
//     float StartVel = StartVelMax;
//
//     int Iter = 0;
//
//     while (true)
//     {
//         const FVector StartLocation = FVector(0, 0, P.Radius);
//         const FVector StartVelocity = FVector(StartVel, 0, 0);
//         const FPhysTransform TStart = FPhysTransform(StartLocation, FQuat::Identity, StartVelocity, FVector::ZeroVector);
//
//         bool FullTimeComputed = true;
//         float DistancePassed = 0;
//         FPhysTransform TPrev = TStart;
//
//         for (int i = 0; i < Steps; ++i)
//         {
//             constexpr float SkipTime = 0;
//             constexpr bool bApplyExtraForces = false;
//             FPhysTransform TNew;
//             // UPhysicsSimulation::PhysicsSimulateDelta(TPrev, P, DT, SkipTime, bApplyExtraForces, TNew);
//             TPrev = TNew;
//
//             DistancePassed = TPrev.Location.X;
//
//             bool B = (i < Steps - 2) && (DistancePassed > DistanceToPass + AbsTolerance);
//             if (B)
//             {
//                 FullTimeComputed = false;
//                 break;
//             }
//         }
//
//         const float Delta = DistancePassed - DistanceToPass;
//         const float DeltaAbs = FMath::Abs(Delta);
//         if ((DeltaAbs < AbsTolerance) && FullTimeComputed)
//         {
//             // found value
//             return StartVel;
//         }
//         else
//         {
//             if (Delta > 0)
//             {
//                 StartVelMax = StartVel;
//                 if (StartVelMax < 10) return false;
//             }
//             else
//             {
//                 StartVelMin = StartVel;
//             }
//
//             if (StartVelMax <= StartVelMin)
//             {
//                 StartVelMax = 1.25f * StartVelMin;
//             }
//
//             const float VelChange = 0.5f * (StartVelMax - StartVelMin);
//             StartVel = StartVelMax - VelChange;
//
//             Iter++;
//         }
//     }
// }

// float UPhysicsUtils::GetDistancePassedOnGroundWithStartVelocityAfterTime(FPhysRigidBodyParams P, float StartVelocity, float Time, float TimeStep)
// {
//     checkNoEntry()
//     return 0;
//     if(TimeStep <= 0) TimeStep = 0.0002f;
//     const int Steps = Time / TimeStep;
//
//     const FVector StartLocation = FVector(0, 0, P.Radius);
//     const FVector StartVel = FVector(StartVelocity, 0, 0);
//     const FPhysTransform TStart = FPhysTransform(StartLocation, FQuat::Identity, StartVel, FVector::ZeroVector);
//     FPhysTransform TPrev = TStart;
//     
//     for (int i = 0; i < Steps; ++i)
//     {
//         constexpr float SkipTime = 0;
//         constexpr bool bApplyExtraForces = false;
//         FPhysTransform TNew;
//         // UPhysicsSimulation::PhysicsSimulateDelta(TPrev, P, TimeStep, SkipTime, bApplyExtraForces, TNew);
//         TPrev = TNew;
//     }
//
//     return TPrev.Location.X;
//     
// }

float UPhysicsUtils::GetTotalMassInv(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B)
{
    return  A->GetMassInv()  + B->GetMassInv();
}

float UPhysicsUtils::GetRestitutionFromBodies(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B)
{
    const float RA = A->GetRestitution();
    const float RB = B->GetRestitution();
    const EPhysicsCombineMode Mode = UPhysicsUtils::SelectPhysCombineMode(A->GetRestitutionCombineMode(), B->GetRestitutionCombineMode());
    return CombinePhysValue(RA, RB, Mode);
}

float UPhysicsUtils::GetFrictionFromBodies(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B, EPhysicsCombineMode Mode)
{
    const float FA = A->GetFriction();
    const float FB = B->GetFriction();
    return CombinePhysValue(FA, FB, Mode);
}
