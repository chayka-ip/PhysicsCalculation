// Fill out your copyright notice in the Description page of Project Settings.

#include "Aerodynamics/AerodynamicsSimulation.h"
#include "Aerodynamics/SideforceGenerator.h"
#include "Common/PhysRigidBodyParams.h"
#include "Common/PhysTransform.h"
#include "Components/CustomPhysicsComponent.h"
#include "Libs/PhysicsSimulation.h"
#include "Structs/DataFromTime.h"

void UAerodynamicsSimulation::CalculateAerodynamicImpact(FPSI_Data& Data, FVector& OutLinearVelocity, FVector& OutAngularVelocity, FVector& locationOffset)
{
    const auto T = Data.GetTransform();
    const float DeltaTime = Data.GetDeltaTime();
    const float SkipTime = Data.GetSkipTime();
    const float Mass = Data.Obj->PhysicsParams.GetMass();
    const bool bApplyExtraForces = Data.HasExtraForces();

    const FVector ADForce = CalculateAerodynamicForce(Data.Obj->PhysicsParams, T.LinearVelocity, T.AngularVelocity);
    UPhysicsSimulation::SimulateAddImpulseFromForce(OutLinearVelocity, ADForce, DeltaTime, Mass);

    if(bApplyExtraForces)
    {
        FVector ExtraAV = FVector::ZeroVector;
        CalculateSideForceImpact(DeltaTime, SkipTime, Data.Obj->PhysicsParams.Aerodynamics, T, ExtraAV, locationOffset);
        OutAngularVelocity += ExtraAV;
    }
}

FVector UAerodynamicsSimulation::CalculateAerodynamicForce(const FPhysRigidBodyParams& RbParams, const FVector& LinearVelocity, const FVector& AngularVelocity)
{
    FVector AerodynamicForce = FVector::ZeroVector;

    const auto ADParams = &RbParams.Aerodynamics;

    const bool bDrag = ADParams->AirDragImpact.bEnabled;
    const bool bMagnus = ADParams->MagnusImpact.bEnabled;
    const bool bEnabled = bDrag || bMagnus;
    
    if(!bEnabled) return AerodynamicForce;

    const float VelocityMagnitude = LinearVelocity.Size();
    const float Radius = RbParams.Radius;
    const float CrossSectionArea = RbParams.GetSphericalCrossSectionArea();
    
    const float AirDensity = ADParams->AirDrag.GetAirDensityKgCm3();
    const float DragCoefficient = ADParams->AirDrag.GetAirDragCoefficient(VelocityMagnitude);
    const float LiftCoefficient = ADParams->AirDrag.GetLiftCoefficient(VelocityMagnitude);

    const float DragMul = ADParams->AirDragImpact.Value * DragCoefficient;
    const float MagnusMul = ADParams->MagnusImpact.Value * LiftCoefficient;
        
    if(bDrag)
    {
        const FVector Drag = DragMul * ComputeSphereAirDragForce(CrossSectionArea, AirDensity, LinearVelocity);
        AerodynamicForce += Drag;
    }

    if (bMagnus)
    {
        const FVector Magnus = MagnusMul * ComputeSphereLiftForceIdeal(Radius, AirDensity, LinearVelocity, AngularVelocity);
        AerodynamicForce += Magnus;
    }

    return AerodynamicForce;
}

FVector UAerodynamicsSimulation::ComputeSphereAirDragForce(float CrossSectionArea, float AirDensity, FVector LinearVelocity)
{
    const float Vm = LinearVelocity.Size();
    if(FMath::IsNearlyZero(Vm, 0.1f)) return  FVector::ZeroVector;
    
    const float M = 0.5f * AirDensity * CrossSectionArea *  Vm * Vm;
    const FVector Direction = -LinearVelocity.GetSafeNormal();
    return M * Direction;
}

FVector UAerodynamicsSimulation::ComputeSphereLiftForceIdeal(float Radius, float AirDensity, FVector LinearVelocity, FVector AngularVelocity)
{
    /*
     * L = (4 * pi^2 * r^3 * s * rho * V) * 4 / 3
     *
     * V - relative velocity;
     * s - spin; revolutions/sec;
     * r - sphere radius;
     * rho - air density;
     * 
     */

    const FVector Cross = LinearVelocity ^ AngularVelocity;
    if(FMath::IsNearlyZero(Cross.Size(), 0.1f)) return  FVector::ZeroVector;
    
    constexpr float ConstCf = 16.0f * Pi * Pi * RadToTurn / 3.0f;
    const float VarCf = AirDensity * Radius * Radius * Radius;
    return  ConstCf * VarCf * Cross;
}

FVector UAerodynamicsSimulation::GetOffsetToApplyWithTimeSkip(FSideforceBaseGenerator& S, FVector ForwardVector, FVector RightVector, FVector UpVector,
                                                                float DeltaTime, float SkipTime, float VelocityImpactAlpha)
{
    const float InitialTimeBNP = S.GetTimeBeforeUpdate();
    const float SimStep = S.GetUpdateTimeInterval();
    const auto Data = FDataFromTime(DeltaTime, SkipTime, SimStep, InitialTimeBNP);

    FVector VectorTotal = FVector::ZeroVector;

    if(Data.HasInitialPart)
    {
        const FVector A = S.GetProperVectorAfterStepCount(ForwardVector, RightVector, UpVector, Data.InitialStartIndex);
        VectorTotal += A * Data.InitialPartMultiplier;
    }

    if(Data.HasIntegerPart)
    {
        int Index = Data.IntegerStartIndex;

        const int MaxIter = Data.NumStepsToGet - 1;
        for (int i = 0; i < MaxIter; ++i)
        {
            VectorTotal += S.GetProperVectorAfterStepCount(ForwardVector, RightVector, UpVector, Index) * SimStep;
            Index ++;
        }
    }
    
    if(Data.HasFuturePart)
    {
        const FVector A = S.GetProperVectorAfterStepCount(ForwardVector, RightVector, UpVector, Data.FutureStartIndex);
        VectorTotal += A * Data.FuturePartMultiplier;
    }

    return VectorTotal * VelocityImpactAlpha;
}

void UAerodynamicsSimulation::CalculateSideForceImpact(float DeltaTime, float SkipTime, FBodyAerodynamics& ADP, const FPhysTransform& T, FVector& OutAngularVelocity, FVector& locationOffset)
{
    if(ADP.Sideforce.bEnabled)
    {
        FVector LocationOffset = FVector::ZeroVector;
        FVector AngularVelocityDelta = FVector::ZeroVector;
        
        ADP.Sideforce.GetOffsetsToApply(T, DeltaTime, SkipTime, LocationOffset, AngularVelocityDelta);

        locationOffset = LocationOffset;
        OutAngularVelocity += AngularVelocityDelta;
    }
}

FVector UAerodynamicsSimulation::ComputeMagnusAccelerationForBody(const UCustomPhysicsComponent* Body)
{
    const auto ADParams = &Body->PhysicsParams.Aerodynamics;
    const auto  RbParams = &Body->PhysicsParams;

    const FVector LinearVelocity = Body->GetCurrentLinearVelocity();
    const FVector AngularVelocity = Body->GetCurrentAngularVelocityRadians();
    
    const float VelocityMagnitude = LinearVelocity.Size();
    const float Radius = RbParams->Radius;
    const float MassInv = RbParams->GetMassInv();
    const float LiftCoefficient = ADParams->AirDrag.GetLiftCoefficient(VelocityMagnitude);
    const float AirDensity = ADParams->AirDrag.GetAirDensityKgCm3();
    const float MagnusMul = ADParams->MagnusImpact.Value * LiftCoefficient;

    return  MagnusMul * ComputeSphereLiftForceIdeal(Radius, AirDensity, LinearVelocity, AngularVelocity) * MassInv;
}

FVector UAerodynamicsSimulation::ComputeMagnusAccelerationForBodyForVelocity(const UCustomPhysicsComponent* Body, FVector LV, FVector AV)
{
    const auto ADParams = &Body->PhysicsParams.Aerodynamics;
    const auto  RbParams = &Body->PhysicsParams;

    const float VelocityMagnitude = LV.Size();
    const float Radius = RbParams->Radius;
    const float MassInv = RbParams->GetMassInv();
    const float LiftCoefficient = ADParams->AirDrag.GetLiftCoefficient(VelocityMagnitude);
    const float AirDensity = ADParams->AirDrag.GetAirDensityKgCm3();
    const float MagnusMul = ADParams->MagnusImpact.Value * LiftCoefficient;

    return  MagnusMul * ComputeSphereLiftForceIdeal(Radius, AirDensity, LV, AV) * MassInv;
}
