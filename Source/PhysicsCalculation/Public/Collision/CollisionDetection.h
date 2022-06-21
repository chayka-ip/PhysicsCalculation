// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysRigidBodyParams.h"
#include "Common/PhysTransform.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "CollisionDetection.generated.h"

struct FCollisionPair;
class UCustomPhysicsComponent;
class UCustomPhysicsBaseComponent;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UCollisionDetection : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()


public:

	UFUNCTION(BlueprintCallable)
	static bool DetectCollisionAgainstSphere(UPrimitiveComponent* PC, const FVector& SLocation, const float& SRadius, FVector& CollisionPointPC, float& Penetration);

	static bool TestCustomCollisionAgainstSphere(UCustomPhysicsBaseComponent* ObjA, UCustomPhysicsComponent* ObjB, FCollisionPair& CollisionPair);
	static bool FindCollisionsAgainstSphereArray(const TArray<UCustomPhysicsComponent*>& FullObjects, const TArray<UCustomPhysicsBaseComponent*>& SimplifiedObjects, TArray<FCollisionPair>& CollisionPairs);
	static bool FindCollisionAgainstSpherePredictMode(FVector SphereLocation, UCustomPhysicsComponent* Obj, const TArray<UCustomPhysicsBaseComponent*>& StaticObjects, TArray<FCollisionPair>& CollisionPairs);
	static bool FindFirstCollisionAgainstSphere(FVector StartLocation, FVector EndLocation, UCustomPhysicsComponent* Obj,
												const TArray<UCustomPhysicsBaseComponent*>& StaticObjects, FCollisionPair& Collision,
												FVector& SphereLocation, float& Alpha);
	static void ResolveCustomCollisionAgainstSphere(const FCollisionPair& CollisionPair);
	static void ResolveCustomCollisionAgainstSpherePredictMode(const FPhysTransform& T, const FCollisionPair& CollisionPair, FPhysTransform& OutT);

	static void CalcSeparationOffsets(float InvMassA, float InvMassB, FVector PV, FVector& OffsetA, FVector& OffsetB);
	static void ApplyCollisionImpulse(UCustomPhysicsBaseComponent* Obj, const FVector& FullImpulse, const FVector& CP, float Time, bool bRecomputePredict);
	static void ApplyCollisionImpulse(FPhysTransform& InOut, const FVector& FullImpulse, const FVector& CP, const FSimpleMatrix3& TInertiaInv, float MassInv);
	static FVector CalcCollisionFullImpulse(FVector LocationA, FVector LocationB, const FSimpleMatrix3 &TInertiaInvA, const FSimpleMatrix3 &TInertiaInvB, FVector CPVelocityA, FVector CPVelocityB,  float TotalMassInv, float Restitution,   const FVector& CP,
														 const FVector& N);
	static FVector CalcCollisionFullImpulse(UCustomPhysicsBaseComponent* A, UCustomPhysicsBaseComponent* B, const FVector& CP, const FVector& N);
	static float CalcCollisionImpulseMagnitude(float ImpulseForceMagnitude, float Restitution, float TotalMassInv);
};
