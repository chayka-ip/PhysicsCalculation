// Fill out your copyright notice in the Description page of Project Settings.

#include "ImpulseDistribution/ImpulseDistributionLib.h"
#include "DataAssets/PhysicsCache_DataAsset.h"
#include "Kick/KickSpinRange.h"
#include "Libs/PhysicsSimulation.h"

FImpulseDistributionCache_Data UImpulseDistributionLib::CalculateImpulseDistribution(UAdvancedPhysicsComponent* Obj)
{
    FImpulseDistributionCache_Data Data;

    Data.AngleDistribution = GetImpulseDistributionCurve(Obj);
    
    return Data;
}

FRuntimeFloatCurve UImpulseDistributionLib::GetImpulseDistributionCurve(UAdvancedPhysicsComponent* Obj)
{
    FRuntimeFloatCurve Curve;
    const auto CurveRef = Curve.GetRichCurve();
    
    constexpr float ImpulseMagnitude = 1000.0f;
    
    for (int Angle = 1; Angle <= 90; ++Angle)
    {
        const float Ratio = GetImpulseDistributionRatioForSphere(Obj, ImpulseMagnitude, Angle);
        CurveRef->AddKey(Angle, Ratio);
    }

    const float V0 = 1.2f * CurveRef->Eval(1.0f);
    CurveRef->AddKey(0, V0);
    
    return Curve;
}

float UImpulseDistributionLib::GetImpulseDistributionRatioForSphere(UAdvancedPhysicsComponent* Obj, float ImpulseMagnitude, float AngleToNormalDeg)
{
    const FVector Impulse = ImpulseMagnitude * UHMV::GetWorldForwardRotated_XY_XZ(0.0f, AngleToNormalDeg);
    const FVector ApplyLocation = Obj->GetCurrentLocation() + FVector::BackwardVector * Obj->GetRadius();
    const bool bRemoveVelocities = true;
    const auto Impact = Obj->CalculateImpulseImpactAtLocation(Impulse, ApplyLocation, bRemoveVelocities);
    const FVector LV = Impact.LinearVelocity;
    const FVector AV = Impact.AngularVelocity;
    return UHM::SafeDivision(LV.Size(), AV.Size());
}

bool UImpulseDistributionLib::CheckAngle90Deg(FVector LinearVelocity, FVector AngularVelocity)
{
    constexpr float Angle90 = 90.0f;
    constexpr float Tolerance = 1.0f;
    return UHMV::IsAngleBetweenVectorsInRange(LinearVelocity, AngularVelocity, Angle90, Angle90, Tolerance);
}

FVector UImpulseDistributionLib::GetImpulseApplyLocationFromVelocity(UAdvancedPhysicsComponent* PC, FVector LinearVelocity, FVector AngularVelocity, FVector COM)
{
    const float Radius = PC->GetRadius();
    const auto ImpulseCache = PC->PhysicsCache->ImpulseDistributionCache;
    const FVector AVN = AngularVelocity.GetSafeNormal();
    const FVector BackwardVelocity = -LinearVelocity.GetSafeNormal();
    const float RawAngle = UHMV::AngleBetweenVectorsDeg(LinearVelocity, AngularVelocity);
    const bool bZeroAngle = FMath::IsNearlyZero(RawAngle, 1.0f);
    const bool bAngle90Deg = CheckAngle90Deg(LinearVelocity, AngularVelocity);

    if(bZeroAngle)
    {
        // works wrong when impulse is 90 degrees to ball surface but this is not realistic case
        return COM + BackwardVelocity * Radius;
    }
    
    if(bAngle90Deg && ImpulseCache)
    {
        const float Angle = ImpulseCache->GetImpulseAngleFromVelocities(LinearVelocity, AngularVelocity);
        const FVector Offset = BackwardVelocity.RotateAngleAxis(Angle, AVN).GetSafeNormal();
        return COM + Offset * Radius;
    }
    
    return COM;
}

FVector UImpulseDistributionLib::GetImpulseFromLinearVelocity(UAdvancedPhysicsComponent* Obj, FVector LinearVelocity, FVector ApplyLocation, FVector COM)
{
    const FVector ModifiedImpulse = UPhysicsSimulation::ImpulseToSphereLinearImpulse(LinearVelocity, ApplyLocation, COM);
    const float ImpulseRatio = LinearVelocity.Size() / ModifiedImpulse.Size();
    const FVector DirectImpulse = LinearVelocity * ImpulseRatio;
    return UPhysicsSimulation::GetForceFromImpulse(DirectImpulse, Obj->GetMass());
}

FVector UImpulseDistributionLib::GetImpulseFromVelocities(UAdvancedPhysicsComponent* Obj, FVector LinearVelocity, FVector AngularVelocity, FVector ApplyLocation, FVector COM)
{
    // todo: will have problem with zero linear velocity when angle is 90
    if(LinearVelocity.IsZero()) return FVector::ZeroVector;
    return GetImpulseFromLinearVelocity(Obj, LinearVelocity, ApplyLocation, COM);
}

FImpulseReconstructed UImpulseDistributionLib::ReconstructImpulseFromVelocities(UAdvancedPhysicsComponent* PC, FVector LinearVelocity,
                                                                                FVector AngularVelocity, FVector COM)
{
    FImpulseReconstructed Out;

    Out.ApplyLocation = GetImpulseApplyLocationFromVelocity(PC, LinearVelocity, AngularVelocity, COM);
    Out.Impulse = GetImpulseFromVelocities(PC, LinearVelocity, AngularVelocity, Out.ApplyLocation, COM);
    
    return Out;
}

FImpulseReconstructed UImpulseDistributionLib::ReconstructImpulseFromImpactTransform(UAdvancedPhysicsComponent* Obj, const FPhysTransform& T)
{
    return ReconstructImpulseFromVelocities(Obj, T.LinearVelocity, T.AngularVelocity, T.Location);
}

FImpulseReconstructed UImpulseDistributionLib::GetImpulseFromParabolicAndSpin(UAdvancedPhysicsComponent* PC, FVector ParabolicVelocity,
                                                                              FVector ApplyLocation, FVector COM, float FrontSpin, float SideSpin, bool bRemoveVelocity)
{
    const FVector InitialImpulse = GetImpulseFromVelocities(PC, ParabolicVelocity, FVector::ZeroVector, ApplyLocation, COM);
    const float ParabolicImpulsePower = InitialImpulse.Size();
    
    const FQuat QNoSpin = GetImpulseQuatFromParabolicVelocityAndSpin(ParabolicVelocity);
    const FQuat QFrontSpin = GetImpulseQuatFromParabolicVelocityAndSpin(ParabolicVelocity, FrontSpin);

    const FVector CenterToApply = ApplyLocation - COM;
    const FVector FrontSpinRight = QFrontSpin.GetRightVector();
    const FVector FrontSpinApplyLocation = COM + CenterToApply.RotateAngleAxis(-FrontSpin, FrontSpinRight);
    const FVector FrontSpinImpulse = QNoSpin.GetForwardVector() * ParabolicImpulsePower;

    const auto TImpact = PC->CalculateImpulseImpactAtLocationOtherCOM(FrontSpinImpulse, FrontSpinApplyLocation, COM,
        FVector::ZeroVector, bRemoveVelocity,bRemoveVelocity);
    
    auto ImpulseAfterFrontSpin = ReconstructImpulseFromVelocities(PC, ParabolicVelocity, TImpact.AngularVelocity, COM);

    ImpulseAfterFrontSpin.Impulse = UHMV::RotateVectorZAxis(ImpulseAfterFrontSpin.Impulse, SideSpin);
    return ImpulseAfterFrontSpin;
}

FQuat UImpulseDistributionLib::GetImpulseQuatFromParabolicVelocityAndSpin(FVector V, float FrontSpin, float SideSpin)
{
    const FQuat Q = UHMR::GetOrientationFromWorldForwardRotated_XY_XZ_FromV(V.GetSafeNormal());
    return UHMR::QuatApplyRightUpRotation(Q, FrontSpin, SideSpin);
}

void UImpulseDistributionLib::GetSpinFromImpulseQuatAndParabolicVelocity(FVector V, FQuat Q, float& FrontSpin, float& SideSpin)
{
    const FQuat QV = UHMR::GetOrientationFromWorldForwardRotated_XY_XZ_FromV(V.GetSafeNormal());
    const FQuat QRightUp = UHMR::GetDeltaRotation(Q, QV);

    const float SignFrontSpin = UHM::Sign(UHMV::GetAngleVectorToPlaneXY(QRightUp.GetAxisX()));
    FrontSpin = SignFrontSpin * (UHMV::GetAngleVectorToPlaneXY(QRightUp.GetAxisZ()) - 90.0f);

    const FQuat QFront = UHMR::QuatFromAxisAngleDeg(QV.GetRightVector(), FrontSpin, true);
    const FQuat QFrontApplied = QFront * QV;
    const FQuat QSide = UHMR::GetDeltaRotation(Q, QFrontApplied);

    SideSpin = UHMV::AngleBetweenVectorsDeg(QFrontApplied.GetRightVector(), QSide.GetRightVector());

    const FQuat QSideCheck = UHMR::QuatFromAxisAngleDeg(QV.GetUpVector(), SideSpin, true);
    const FQuat QCheck = QFrontApplied * QSideCheck;
    
    if(!Q.Equals(QCheck, 0.1f)) SideSpin *= -1.0f;
}

FImpulseReconstructed UImpulseDistributionLib::RotateImpulseZAxis(const FImpulseReconstructed& ImpulseData, const FVector& COM, float AngleDeg)
{
    const FVector COMToContact = ImpulseData.ApplyLocation - COM;
    
    FImpulseReconstructed OutImpulse;
    OutImpulse.Impulse = UHMV::RotateVectorZAxis(ImpulseData.Impulse, AngleDeg);
    OutImpulse.ApplyLocation = UHMV::RotateVectorZAxis(COMToContact, AngleDeg) + COM;
    
    return OutImpulse;
}

TArray<FQuat> UImpulseDistributionLib::GetImpulseQuatArray(FVector ParabolicVelocity, const FKickSpinRange& SpinRange, bool bFlipSideSpin)
{
    TArray<FQuat> Out;

    const int MinFrontAngle = SpinRange.GetMinFrontSpinAngleInt();
    const int MaxFrontSpinAngle = SpinRange.GetMaxFrontSpinAngleInt();
    const int MinSideAngle = SpinRange.GetMinSideSpinAngleInt();
    const int MaxSideSpinAngle = 0;
    
    for (int front_spin_angle = MinFrontAngle; front_spin_angle <= MaxFrontSpinAngle; ++front_spin_angle)
    {
        for (int side_spin_angle = MinSideAngle; side_spin_angle <= MaxSideSpinAngle; ++side_spin_angle)
        {
            float side_spin_final = side_spin_angle;
            if(bFlipSideSpin) side_spin_final *= -1.0f;
            FQuat Q = GetImpulseQuatFromParabolicVelocityAndSpin(ParabolicVelocity, front_spin_angle, side_spin_final);
            Out.Add(Q);
        }            
    }
    
    return Out;
}
