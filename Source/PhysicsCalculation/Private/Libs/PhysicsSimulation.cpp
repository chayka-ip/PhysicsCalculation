// Fill out your copyright notice in the Description page of Project Settings.

#include "Libs/PhysicsSimulation.h"
#include "Aerodynamics/AerodynamicsSimulation.h"
#include "Components/CustomPhysicsBaseComponent.h"
#include "Components/CustomPhysicsComponent.h"
#include "Common/PhysRigidBodyParams.h"
#include "Kismet/KismetMathLibrary.h"

FVector UPhysicsSimulation::PTransformGetLinearVelocityAtPoint(const FPhysTransform& T, const FVector& P)
{
    const FVector Arm = P - T.Location;
    return T.LinearVelocity + (T.AngularVelocity ^ Arm);
}

void UPhysicsSimulation::PTransformApplyLinearImpulse(FPhysTransform& InOut, const FVector& Force, float MassInv)
{
    InOut.LinearVelocity += Force * MassInv;
}

void UPhysicsSimulation::PTransformApplyAngularImpulse(FPhysTransform& InOut, const FVector& Force, const FSimpleMatrix3& TInertiaInv)
{
    InOut.AngularVelocity += TInertiaInv.MultiplyByVector(Force);
}

void UPhysicsSimulation::CalculateFriction(const FVector &COM, const FVector& CP, const FVector &CPVelocity,  const FVector& CollisionImpulse,const FSimpleMatrix3& TInertiaInv, float MassInv, float Friction, FVector& LinearForce,
    FVector& AngularForce)
{
    const FVector N = CollisionImpulse.GetSafeNormal();
    const float ImpulseMagnitude = CollisionImpulse.Size();

    const FVector Arm = CP - COM;
    const FVector TangentVelocity = UMathUtils::GetVectorTangentComponent(CPVelocity, N);
    const FVector Tangent = TangentVelocity.GetSafeNormal();

    const float vt = CPVelocity | Tangent;
    const FVector ArmCrossTangent = Arm ^ Tangent;

    check(!Tangent.ContainsNaN())

    const FVector Inertia = CalcInertiaEffect(TInertiaInv, Arm, Tangent);
    const float AngularEffect = Inertia | Tangent;
    const float kt = MassInv + AngularEffect;

    const float ReactionMulFriction = Friction * ImpulseMagnitude;
    const float jt = FMath::Clamp( -vt/kt, -ReactionMulFriction, ReactionMulFriction);

    LinearForce = jt * Tangent;
    AngularForce = jt * ArmCrossTangent;
}

void UPhysicsSimulation::ApplyFriction(UCustomPhysicsBaseComponent* Obj, const FVector &CP, const FVector &CollisionImpulse, float Friction, float Time, bool bRecomputePredict)
{
    const FVector COM = Obj->GetCurrentLocation();
    const FVector CPVelocity = Obj->GetFullVelocityAtPoint(CP);
    const float MassInv = Obj->GetMassInv();
    const auto TInertiaInv = Obj->GetInertiaTensorInverted();
    
    FVector LinearForce, AngularForce;
    CalculateFriction(COM, CP, CPVelocity, CollisionImpulse, TInertiaInv, MassInv, Friction, LinearForce, AngularForce);

    // Obj->AddLinearImpulseAsApplied(LinearForce, Time ,bRecomputePredict);
    // Obj->AddAngularImpulseAsApplied(AngularForce, Time,  bRecomputePredict);
    
    Obj->AddLinearImpulse(LinearForce, bRecomputePredict);
    Obj->AddAngularImpulse(AngularForce, bRecomputePredict);
}

void UPhysicsSimulation::ApplyFriction(FPhysTransform& InOut, const FVector& CP, const FVector &CPVelocity, const FVector& CollisionImpulse, const FSimpleMatrix3& TInertiaInv, float MassInv, float Friction)
{
    const FVector COM = InOut.Location;
    
    FVector LinearForce, AngularForce;
    CalculateFriction(COM, CP, CPVelocity, CollisionImpulse, TInertiaInv, MassInv, Friction, LinearForce, AngularForce);

    PTransformApplyLinearImpulse(InOut, LinearForce, MassInv);
    PTransformApplyAngularImpulse(InOut, AngularForce,TInertiaInv);
}

FVector UPhysicsSimulation::DeltaLocationToForce(FVector DeltaLocation, float Mass, float DeltaTime)
{
    return DeltaLocation * Mass / DeltaTime;
}

FPhysTransform UPhysicsSimulation::PhysicsSimulateDelta(FPSI_Data& Data)
{
    FPhysTransform Out;
    PhysicsSimulateDelta(Data, Out);
    return Out;
}

FVector UPhysicsSimulation::CalcInertiaEffect(FSimpleMatrix3 InertiaTensorInverted, FVector Arm, FVector Normal)
{
    return (InertiaTensorInverted.MultiplyByVector(Arm ^ Normal))^ Arm;
}

void UPhysicsSimulation::UpdateTransformOrientation(const FPhysRigidBodyParams& RbParams, float DeltaTime, FPhysTransform& InOutT)
{
    FVector AngularVelocity = InOutT.AngularVelocity;
    if(RbParams.Rendering.MaxAngularVelocity.bClamp)
    {
        AngularVelocity = RbParams.Rendering.GetAngularVelocityClamped(InOutT.AngularVelocity);
    }
    UMathUtils::ApplyAngularVelocityToRotation(AngularVelocity, DeltaTime, InOutT.Orientation);
}

void UPhysicsSimulation::ApplyConstrains(const FPhysConstrains& Constrains, FPhysTransform& InOutT)
{
    Constrains.ClampVelocity(InOutT.LinearVelocity, InOutT.AngularVelocity);
}

void UPhysicsSimulation::ApplyVelocityDamping(const FPhysRigidBodyParams& RbParams, float DeltaTime, FPhysTransform& OutT)
{
    const auto Linear = &RbParams.LinearDamping;
    const auto Angular = &RbParams.AngularDamping;

    if(Linear->bEnabled)
    {
        SimulateVelocityDamping(OutT.LinearVelocity,Linear->Value, DeltaTime);
    }
    if(Angular->bEnabled)
    {
        SimulateVelocityDamping(OutT.AngularVelocity, Angular->Value, DeltaTime);
    }    
}

void UPhysicsSimulation::PhysicsSimulateDelta(FPSI_Data& Data, FPhysTransform& OutT)
{
    check(Data.Obj)
    auto InT = Data.GetTransform();
    auto RbParams = Data.Obj->PhysicsParams;
    float DeltaTime = Data.GetDeltaTime();
    
    OutT = InT;

    FVector locationOffset = FVector::ZeroVector;
    {
        FVector LinearVelocityAccumulated = FVector::ZeroVector;
        FVector AngularVelocityAccumulated = FVector::ZeroVector;

        if(RbParams.bGravityEnabled) 
        {
            FVector GForce = RbParams.GetGravityForce();
            SimulateAddImpulseFromForce(LinearVelocityAccumulated, GForce, DeltaTime, RbParams.GetMass());
        }

        {
            FVector ADLinearVelocity = FVector::ZeroVector;
            FVector ADAngularVelocity = FVector::ZeroVector;

            UAerodynamicsSimulation::CalculateAerodynamicImpact(Data, ADLinearVelocity, ADAngularVelocity, locationOffset);

            OutT.Location += locationOffset;
            LinearVelocityAccumulated += ADLinearVelocity;
            AngularVelocityAccumulated += ADAngularVelocity;
        }

        OutT.LinearVelocity += LinearVelocityAccumulated;
        OutT.AngularVelocity += AngularVelocityAccumulated;
    }
    
    ApplyVelocityDamping(RbParams, DeltaTime, OutT);
    ApplyConstrains(RbParams.Constrains, OutT);
    
    OutT.Location += OutT.LinearVelocity * DeltaTime;
    
    UpdateTransformOrientation(RbParams, DeltaTime, OutT);
}

void UPhysicsSimulation::SimulateAddImpulseFromForce(FVector& InOutLinearVelocity, const FVector& Force, const float DeltaTime, const float Mass)
{
    InOutLinearVelocity += Force * DeltaTime / Mass;
}

void UPhysicsSimulation::SimulateAddTorqueFromForce(FVector& InOutAngularVelocity, const FVector& Force, const float DeltaTime, const float Mass)
{
    InOutAngularVelocity += Force*DeltaTime/(1000*Mass);
}

FVector UPhysicsSimulation::ForceToImpulse(FVector Force, float DeltaTime)
{
    return Force * DeltaTime;
}

FVector UPhysicsSimulation::ImpulseToAngularImpulse(FVector Impulse, FVector ApplyLocation, FVector COM)
{
    const FVector Arm = ApplyLocation - COM;
    return  Arm ^ Impulse;
}

FVector UPhysicsSimulation::ImpulseToSphereLinearImpulse(FVector Impulse, FVector ApplyLocation, FVector COM)
{
    //todo: decide should we chang direction into radial of left it as is??
    constexpr float AngleClamp = 1.0f;
    const FVector Arm =  COM - ApplyLocation;
    float Angle = UHMV::AngleBetweenVectorsDeg(Arm, Impulse);
    if(FMath::Abs(Angle) <= AngleClamp) Angle = 0.0f;
    const float Cos = UKismetMathLibrary::DegCos(Angle);
    return Impulse * Cos;
}

FVector UPhysicsSimulation::LinearImpulseGetDeltaVelocity(FVector Force, float Mass)
{
    return  Force / Mass;
}

FVector UPhysicsSimulation::GetForceFromImpulse(FVector Impulse, float Mass)
{
    return Impulse * Mass;
}

float UPhysicsSimulation::GetDeltaSpeedAfterLinearImpulseForSphereCentralApplied(float Magnitude, float Mass)
{
    const FVector Force = FVector::ForwardVector * Magnitude;
    return LinearImpulseGetDeltaVelocity(Force, Mass).Size();
}

void UPhysicsSimulation::SimulateAddImpulseAtLocationForSphere2(FVector& InOutLinearVelocity, FVector& InOutAngularVelocity, const FVector& Impulse,
                                                                float Mass, const FVector& ApplyLocation, const FVector& COM, FSimpleMatrix3 InertiaTensorInv)
{
    const FVector LinearImpulse = ImpulseToSphereLinearImpulse(Impulse, ApplyLocation, COM);
    const FVector AngularImpulse = ImpulseToAngularImpulse(Impulse, ApplyLocation, COM);
    
    InOutLinearVelocity = SimulateAddLinearImpulse(LinearImpulse, InOutLinearVelocity, Mass);
    InOutAngularVelocity = SimulateAddAngularImpulseConvCOM(AngularImpulse, InOutAngularVelocity, InertiaTensorInv);
}

FVector UPhysicsSimulation::SimulateAddLinearImpulse(FVector Force, FVector LinearVelocity, float Mass)
{
    return LinearVelocity + LinearImpulseGetDeltaVelocity(Force, Mass);
}

FVector UPhysicsSimulation::SimulateAddAngularImpulseConvCOM(FVector Impulse, FVector AngularVelocity, FSimpleMatrix3 InertiaTensorInv)
{
    return  AngularVelocity + InertiaTensorInv.MultiplyByVector(Impulse);
}

void UPhysicsSimulation::SimulateAddImpulseToAreaForSphere2(FVector& InOutLinearVelocity, FVector& InOutAngularVelocity, const FVector& Impulse,
    float Mass, const TArray<FVector>& AreaPoints, const FVector& COM, FSimpleMatrix3 InertiaTensorInv)
{
    const int Num = AreaPoints.Num();
    if(Num == 0) return;
    
    const FVector Imp = Impulse /  Num;
    for (auto P : AreaPoints)
    {
        SimulateAddImpulseAtLocationForSphere2(InOutLinearVelocity, InOutAngularVelocity, Imp, Mass, P, COM, InertiaTensorInv);
    }
}

void UPhysicsSimulation::SimulateVelocityDamping(FVector& Velocity, const float VelocityDamping, const float DeltaTime)
{
    Velocity *=   1.0f - (VelocityDamping * DeltaTime);
}

void UPhysicsSimulation::UpdateTransformLock(UCustomPhysicsComponent* Obj, const FVector& PrevLocation, const FQuat& PrevOrientation)
{
    const bool bLockX = Obj->IsLockLocationX();
    const bool bLockY = Obj->IsLockLocationY();
    const bool bLockZ = Obj->IsLockLocationZ();

    UpdateTransformLock(Obj->CurrentTransform, PrevLocation, PrevOrientation, bLockX, bLockY, bLockZ);
}

void UPhysicsSimulation::UpdateTransformLock(FPhysTransform& InOutT, const FVector& PrevLocation, const FQuat& PrevOrientation, bool bLockX, bool bLockY, bool bLockZ)
{
    /*
     const bool bLockXR = BodyInstance->bLockXRotation > 0;
	 const bool bLockYR = BodyInstance->bLockYRotation > 0;
	 const bool bLockZR = BodyInstance->bLockZRotation > 0;
	 */

    if (bLockX) InOutT.Location.X = PrevLocation.X;
    if (bLockY) InOutT.Location.Y = PrevLocation.Y;
    if (bLockZ) InOutT.Location.Z = PrevLocation.Z;
	
}

bool UPhysicsSimulation::CalculatePredictionCurveWithMeta(UCustomPhysicsComponent* Obj, const FPhysCurveSpecialPrediction& Data, FPhysicsPredictionCurveWithMeta& Out)
{

    /*
     * todo
     *
     * optimization:
     * track for max Z passed and compare with target Z min (not computational limit but real target) - > if less than this value - break
     */
    
    const int MaxSteps = Data.GetMaxIterations();
    const float SimStep = Data.GetDeltaTime();
    const auto TInitial = Data.GetInitialTransform();

    const auto Limits = &Data.Limits;
    const auto ExtraData = &Data.ExtraData;
    const bool HasLimit_Z = Limits->HasLimitZ();
    const bool bLimit_ZMin = Limits->Limit_MinZ.IsLimit();
    const bool bLimit_ZMax = Limits->Limit_MaxZ.IsLimit();
    const bool bLimit_DistanceFromStartXY = Limits->Limit_DistanceFromStartXY.IsLimit();
    const bool bStoreLocationOnDistance = ExtraData->IsStoringLocationOnDistance();
    
    const float Limit_ZMin = Limits->Limit_MinZ.GetValue();
    const float Limit_ZMax = Limits->Limit_MaxZ.GetValue();
    const float DistanceFromStartXY = Limits->Limit_DistanceFromStartXY.GetValue();
    const float XYDistanceToStoreZ = ExtraData->GetDistanceToStoreZ();

    bool bStoreLocations = Data.ExtraData.bStoreLocationVectors;
    
    FPSI_Data PSI_Data;
    PSI_Data.Obj = Obj;
    PSI_Data.SetTransform(TInitial);
    PSI_Data.SetDeltaTime(SimStep);

    bool bNeededStoreLocationOnDistanceFlag = bStoreLocationOnDistance;

    FVector StartLocation = TInitial.Location;
    FVector PrevLocation = TInitial.Location;

    if(bStoreLocations) Out.AddLocationToArray(PrevLocation);
    
    for (int i = 1; i < MaxSteps; ++i)
    {
        auto TNew = PhysicsSimulateDelta(PSI_Data);
        PSI_Data.SetTransform(TNew);

        bool bContinueIteration  = true;

        FVector NewLocation = TNew.Location;
        const float DistanceZ = NewLocation.Z;
        const float DistanceXY = (NewLocation - StartLocation).Size2D();

        if(HasLimit_Z)
        {
            if(bLimit_ZMin) bContinueIteration = DistanceZ > Limit_ZMin;
            if(bLimit_ZMax && bContinueIteration) bContinueIteration = DistanceZ < Limit_ZMax;
        }

        if(bLimit_DistanceFromStartXY && bContinueIteration)
        {
            bContinueIteration = DistanceXY < DistanceFromStartXY;
        }

        if(bNeededStoreLocationOnDistanceFlag)
        {
            const bool bDistanceNearlyEqual = FMath::IsNearlyEqual(DistanceXY, XYDistanceToStoreZ);
            const bool bDistanceOverPassed = DistanceXY > XYDistanceToStoreZ;
            const bool CanStore = bDistanceNearlyEqual || bDistanceOverPassed;
            if(CanStore)
            {
                FVector VToStore;

                if(bDistanceNearlyEqual) VToStore = NewLocation;
                else if(bDistanceOverPassed)
                {
                    const float PrevDistanceXY = (PrevLocation - StartLocation).Size2D();
                    float Alpha = UHM::GetRatioFromPositiveRange(XYDistanceToStoreZ, PrevDistanceXY, DistanceXY);
                    VToStore = UKismetMathLibrary::VLerp(PrevLocation, NewLocation, Alpha);
                }
                
                Out.SetLocationCached(VToStore);
                bNeededStoreLocationOnDistanceFlag = false;
            }
        }

        PrevLocation = NewLocation;
        if(bStoreLocations) Out.AddLocationToArray(NewLocation);
        
        if(!bContinueIteration)
        {
            return true;
        }
    }
    return true;
}
