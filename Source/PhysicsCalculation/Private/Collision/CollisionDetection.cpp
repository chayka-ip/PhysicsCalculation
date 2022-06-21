// Fill out your copyright notice in the Description page of Project Settings.

#include "Collision/CollisionDetection.h"

#include "Collision/CollisionPair.h"
#include "constants.h"
#include "Components/CustomPhysicsComponent.h"
#include "HandyMathLibrary.h"
#include "Libs/PhysicsSimulation.h"
#include "Libs/PhysicsUtils.h"

bool UCollisionDetection::DetectCollisionAgainstSphere(UPrimitiveComponent* PC, const FVector& SLocation, const float& SRadius, FVector& CollisionPointPC,
                                                       float& Penetration)
{
    if(PC)
    {
        const float CollisionDistance = PC->GetClosestPointOnCollision(SLocation, CollisionPointPC);
        Penetration = SRadius - CollisionDistance;
        return Penetration > 0;
    }
    return  false;
}

bool UCollisionDetection::TestCustomCollisionAgainstSphere(UCustomPhysicsBaseComponent* ObjA, UCustomPhysicsComponent* ObjB,
    FCollisionPair& CollisionPair)
{
    if(!ObjA || !ObjB) return false;
    
    const auto PA = ObjA->GetPrimitiveComponent();

    const FVector SphereLocation = ObjB->GetCurrentLocation();
    const float SphereRadius = ObjB->GetRadius();
	
    if(DetectCollisionAgainstSphere(PA, SphereLocation, SphereRadius, CollisionPair.CollisionPoint, CollisionPair.Penetration))
    {
        CollisionPair.ObjA = ObjA;
        CollisionPair.ObjB = ObjB;
        CollisionPair.CollisionNormal = (SphereLocation - CollisionPair.CollisionPoint).GetSafeNormal();
        return true;
    }
    return  false;
}

bool UCollisionDetection::FindCollisionsAgainstSphereArray(const TArray<UCustomPhysicsComponent*>& FullObjects,
    const TArray<UCustomPhysicsBaseComponent*>& SimplifiedObjects, TArray<FCollisionPair>& CollisionPairs)
{
    if(FullObjects.Num() > 0)
    {
        for (int i = 0; i < FullObjects.Num(); ++i)
        {
            const auto FullObj = FullObjects[i];
            if(!FullObj) continue;
			
            for (int j = 0; j < SimplifiedObjects.Num(); ++j)
            {
                const auto SimpleObj = SimplifiedObjects[j];
                if(!SimpleObj) continue;

                FCollisionPair CollisionPair;
                if(TestCustomCollisionAgainstSphere(SimpleObj, FullObj, CollisionPair))
                {
                    CollisionPairs.Add(CollisionPair);
                }
            }
        }
    }
    return  CollisionPairs.Num() > 0;
}

bool UCollisionDetection::FindCollisionAgainstSpherePredictMode(FVector SphereLocation, UCustomPhysicsComponent* Obj,
                                                                const TArray<UCustomPhysicsBaseComponent*>& StaticObjects,
                                                                TArray<FCollisionPair>& CollisionPairs)
{
    // Sphere location is used because actual location is determined by some location in the future

    const float Radius = Obj->GetRadius();
    
    for (const auto StaticObj : StaticObjects)
    {
        if(!StaticObj) continue;

        const auto P = StaticObj->GetPrimitiveComponent();
        
        FCollisionPair CollisionPair;

        if(DetectCollisionAgainstSphere(P, SphereLocation, Radius, CollisionPair.CollisionPoint, CollisionPair.Penetration))
        {
            CollisionPair.ObjA = StaticObj;
            CollisionPair.ObjB = Obj;
            CollisionPair.CollisionNormal = (SphereLocation - CollisionPair.CollisionPoint).GetSafeNormal();
            CollisionPairs.Add(CollisionPair);
        }
    }
    return  CollisionPairs.Num() > 0;
}

bool UCollisionDetection::FindFirstCollisionAgainstSphere(FVector StartLocation, FVector EndLocation, UCustomPhysicsComponent* Obj,
    const TArray<UCustomPhysicsBaseComponent*>& StaticObjects, FCollisionPair& Collision, FVector &SphereLocation, float &Alpha)
{
    Alpha = 1.0f;

    const float Radius = Obj->GetRadius();
    const float Offset = TunnelingDetectionSphereRadiusMul * Radius; 

    const FVector DeltaLocation = EndLocation - StartLocation;
    const FVector Dir = DeltaLocation.GetSafeNormal();
    const float Distance = DeltaLocation.Size();

    int NumSteps = 0;
    const float LeftOver = UHM::SplitFloatsIntWithLeftOver(Distance, NumSteps, Offset);

    TArray<FCollisionPair> CollisionPairs;
    
    for (int i = 0; i < NumSteps; ++i)
    {
        const float AddDistance = i * Offset;
        SphereLocation = StartLocation + AddDistance * Dir;
        if(FindCollisionAgainstSpherePredictMode(SphereLocation, Obj, StaticObjects, CollisionPairs))
        {
            Alpha = AddDistance / Distance;
            Collision = CollisionPairs[0];
            return true;
        }
    }

    if (LeftOver > 0)
    {
        if(FindCollisionAgainstSpherePredictMode(EndLocation, Obj, StaticObjects, CollisionPairs))
        {
            SphereLocation = EndLocation;
            Collision = CollisionPairs[0];
            return true;
        }
    }

    return false;
}

void UCollisionDetection::ResolveCustomCollisionAgainstSphere(const FCollisionPair& CollisionPair)
{
    if(!CollisionPair.IsValid()) return;
        
    const auto ObjA = CollisionPair.ObjA;
    const auto ObjB = CollisionPair.ObjB;
    
    // separate objects
    
    const FVector Penetration = CollisionPair.GetPenetrationVector();
    FVector OffsetA, OffsetB;
    CalcSeparationOffsets(ObjA->GetMassInv(), ObjB->GetMassInv(), Penetration, OffsetA, OffsetB);

    constexpr bool bRecomputePredict = false;
    
    ObjA->AddWorldLocation(OffsetA, bRecomputePredict);
    ObjB->AddWorldLocation(OffsetB, bRecomputePredict);

    // calculate impulse and friction
    
    const FVector CP = CollisionPair.CollisionPoint;
    const FVector N = CollisionPair.CollisionNormal;
    const float FrictionA = UPhysicsUtils::GetFrictionFromBodies(ObjA, ObjB, ObjA->GetFrictionCombineMode());
    const float FrictionB = UPhysicsUtils::GetFrictionFromBodies(ObjA, ObjB, ObjB->GetFrictionCombineMode());
    
    const FVector FullImpulse = CalcCollisionFullImpulse(ObjA, ObjB, CP, N);

    // apply forces

    constexpr float DEPRECATED_TIME = 0; // left here after attempt to apply impulses to default objects as location change
    
    ApplyCollisionImpulse(ObjA, -FullImpulse, CP, DEPRECATED_TIME, bRecomputePredict);
    ApplyCollisionImpulse(ObjB, FullImpulse, CP, DEPRECATED_TIME, bRecomputePredict);

    UPhysicsSimulation::ApplyFriction(ObjA, CP, -FullImpulse, FrictionA, DEPRECATED_TIME, bRecomputePredict);
    UPhysicsSimulation::ApplyFriction(ObjB, CP, FullImpulse, FrictionB, DEPRECATED_TIME, bRecomputePredict);
}

void UCollisionDetection::ResolveCustomCollisionAgainstSpherePredictMode(const FPhysTransform& T, const FCollisionPair& CollisionPair, FPhysTransform& OutT)
{
    if(!CollisionPair.IsValid()) return;

    OutT = T;

    const auto ObjA = CollisionPair.ObjA;
    const auto ObjB = CollisionPair.ObjB;
    
    // separate objects
    
    const FVector Penetration = CollisionPair.GetPenetrationVector();
    FVector OffsetA, OffsetB;
    CalcSeparationOffsets(ObjA->GetMassInv(), ObjB->GetMassInv(), Penetration, OffsetA, OffsetB);

    OutT.Location += OffsetB;

    // calculate impulse

    const FVector LocationA = ObjA->GetCurrentLocation();
    const FVector LocationB = OutT.Location;
    
    const FVector CP = CollisionPair.CollisionPoint;
    const FVector N = CollisionPair.CollisionNormal;

    const float MassInvB = ObjB->GetMassInv();
    const float TotalMassInv = UPhysicsUtils::GetTotalMassInv(ObjA, ObjB);
    const float FrictionB = UPhysicsUtils::GetFrictionFromBodies(ObjA, ObjB, ObjB->GetFrictionCombineMode());
    const float Restitution = UPhysicsUtils::GetRestitutionFromBodies(ObjA, ObjB);

    const auto TInertiaInvA =  ObjA->GetInertiaTensorInverted();
    const auto TInertiaInvB =  ObjB->GetInertiaTensorInverted();

    const FVector CPVelocityA = ObjA->GetFullVelocityAtPoint(CP);
    const FVector CPVelocityB = UPhysicsSimulation::PTransformGetLinearVelocityAtPoint(OutT, CP);
    
    const FVector FullImpulse = CalcCollisionFullImpulse(LocationA, LocationB, TInertiaInvA, TInertiaInvB, CPVelocityA, CPVelocityB, TotalMassInv, Restitution, CP, N);

    // apply forces
    
    ApplyCollisionImpulse(OutT, FullImpulse, CP, TInertiaInvB, MassInvB);
    UPhysicsSimulation::ApplyFriction(OutT, CP, CPVelocityB, FullImpulse, TInertiaInvB, MassInvB, FrictionB);
}

void UCollisionDetection::CalcSeparationOffsets(float InvMassA, float InvMassB, FVector PV, FVector& OffsetA, FVector& OffsetB)
{
    const float TotalMassInv = InvMassA  + InvMassB;
    const FVector Portion = PV * SeparationSphereMultiplier / TotalMassInv;
    OffsetA = -Portion * InvMassA;
    OffsetB = Portion * InvMassB;
}

void UCollisionDetection::ApplyCollisionImpulse(UCustomPhysicsBaseComponent* Obj, const FVector& FullImpulse, const FVector& CP, float Time,
    bool bRecomputePredict)
{
    // We don't convert Full Impulse to linear impulse because collision is resolved as radial impact
    const FVector COM = Obj->GetCurrentLocation();
    const FVector LinearImpulse = UPhysicsSimulation::ImpulseToSphereLinearImpulse(FullImpulse, CP, COM);
    const FVector AngularImpulse = UPhysicsSimulation::ImpulseToAngularImpulse(FullImpulse, CP, COM);

    // Obj->AddLinearImpulseAsApplied(FullImpulse, Time ,bRecomputePredict);
    // Obj->AddAngularImpulseAsApplied(AngularImpulse, Time, bRecomputePredict);
    
    Obj->AddLinearImpulse(LinearImpulse, bRecomputePredict);
    Obj->AddAngularImpulse(AngularImpulse, bRecomputePredict);
}

void UCollisionDetection::ApplyCollisionImpulse(FPhysTransform& InOut, const FVector& FullImpulse, const FVector& CP,
    const FSimpleMatrix3& TInertiaInv, float MassInv)
{
    const FVector Arm = CP - InOut.Location;
    const FVector AngularImpulse = Arm ^ FullImpulse;

    UPhysicsSimulation::PTransformApplyLinearImpulse(InOut, FullImpulse, MassInv);
    UPhysicsSimulation::PTransformApplyAngularImpulse(InOut, AngularImpulse,TInertiaInv);
}

FVector UCollisionDetection::CalcCollisionFullImpulse(FVector LocationA, FVector LocationB, const FSimpleMatrix3& TInertiaInvA,
    const FSimpleMatrix3& TInertiaInvB, FVector CPVelocityA, FVector CPVelocityB, float TotalMassInv, float Restitution, const FVector& CP,
    const FVector& N)
{
    const FVector ArmA = CP - LocationA;
    const FVector ArmB = CP - LocationB;

    const FVector InertiaA = UPhysicsSimulation::CalcInertiaEffect(TInertiaInvA, ArmA, N);
    const FVector InertiaB = UPhysicsSimulation::CalcInertiaEffect(TInertiaInvB, ArmB, N);

    const float AngularEffect = (InertiaA + InertiaB) | N;

    const FVector ContactVelocity = CPVelocityB - CPVelocityA;
    const float FullImpulseForceMagnitude = ContactVelocity | N;

    const float J = CalcCollisionImpulseMagnitude(FullImpulseForceMagnitude, Restitution, TotalMassInv + AngularEffect);

    return  J * N;
}

FVector UCollisionDetection::CalcCollisionFullImpulse(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B, const FVector& CP,
                                                      const FVector& N)
{
    const FVector LocationA = A->GetCurrentLocation();
    const FVector LocationB = B->GetCurrentLocation();
    
    const float TotalMassInv = UPhysicsUtils::GetTotalMassInv(A, B);
    const float Restitution = UPhysicsUtils::GetRestitutionFromBodies(A, B);

    const auto TInertiaInvA =  A->GetInertiaTensorInverted();
    const auto TInertiaInvB =  B->GetInertiaTensorInverted();

    const FVector CPVelocityA = A->GetFullVelocityAtPoint(CP);
    const FVector CPVelocityB = B->GetFullVelocityAtPoint(CP);

    return CalcCollisionFullImpulse(LocationA, LocationB, TInertiaInvA, TInertiaInvB, CPVelocityA, CPVelocityB, TotalMassInv, Restitution, CP, N);
}

float UCollisionDetection::CalcCollisionImpulseMagnitude(float ImpulseForceMagnitude, float Restitution, float TotalMassInv)
{
    if(FMath::IsNearlyZero(TotalMassInv)) return  0;
    return  -1.0f * (1.0f + Restitution) * ImpulseForceMagnitude / TotalMassInv;
}
