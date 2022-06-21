// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysicsPredictionCurveWithMeta.h"
#include "Common/PhysTransform.h"
#include "Common/PSI_Data.h"
#include "Common/SimpleMatrix3.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "PhysicsSimulation.generated.h"

/*
 * https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials
 * nice stuff 
 */

/**
*
* p = mV
* F = ma
* F = m (dV/dt)
* F = dp/dt
*
* CP - collision point
* N - collision normal
* PV - penetration vector = penetration distance * collision normal
* prefer use radians for angular velocity
*/

/*
 * Even if restitution is set to one - we loose some energy; But original physics will gain some energy in the same situations;
 * Therefore both implementations are not realistic and we can use own
 */


struct FPhysConstrains;
class UCustomPhysicsBaseComponent;
struct FPhysRigidBodyParams;

UCLASS()
class PHYSICSCALCULATION_API UPhysicsSimulation : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:
	static FVector PTransformGetLinearVelocityAtPoint(const FPhysTransform& T, const FVector &P); 
	static void PTransformApplyLinearImpulse(FPhysTransform& InOut, const FVector& Force, float MassInv);
	static void PTransformApplyAngularImpulse(FPhysTransform& InOut, const FVector& Force, const FSimpleMatrix3& TInertiaInv);

	static FVector CalcInertiaEffect(FSimpleMatrix3 InertiaTensorInverted, FVector Arm, FVector Normal);
	
	static void CalculateFriction(const FVector& COM, const FVector& CP, const FVector& CPVelocity, const FVector& CollisionImpulse, const FSimpleMatrix3& TInertiaInv, float MassInv, float
	                              Friction, FVector& LinearForce, FVector& AngularForce);
	
	static void ApplyFriction(UCustomPhysicsBaseComponent* Obj, const FVector& CP, const FVector& CollisionImpulse, float Friction, float Time, bool bRecomputePredict);
	static void ApplyFriction(FPhysTransform& InOut, const FVector& CP, const FVector& CPVelocity, const FVector& CollisionImpulse, const FSimpleMatrix3& TInertiaInv, float
	                          MassInv, float Friction);

	static FVector DeltaLocationToForce(FVector DeltaLocation, float Mass, float DeltaTime);
	static FPhysTransform PhysicsSimulateDelta(FPSI_Data& Data);
	static void PhysicsSimulateDelta(FPSI_Data& Data, FPhysTransform& OutT);
	static void UpdateTransformOrientation(const FPhysRigidBodyParams& RbParams, float DeltaTime, FPhysTransform& InOutT);
	static void ApplyConstrains(const FPhysConstrains& Constrains, FPhysTransform& InOutT);
	static void ApplyVelocityDamping(const FPhysRigidBodyParams& RbParams, float DeltaTime, FPhysTransform& OutT);

	static void SimulateAddImpulseFromForce(FVector &InOutLinearVelocity, const FVector &Force, float DeltaTime, float Mass);
	static void SimulateAddTorqueFromForce(FVector &InOutAngularVelocity, const FVector &Force, float DeltaTime, float Mass);
	static FVector ForceToImpulse(FVector Force, float DeltaTime);
	static FVector ImpulseToAngularImpulse(FVector Impulse, FVector ApplyLocation, FVector COM);
	static FVector ImpulseToSphereLinearImpulse(FVector Impulse, FVector ApplyLocation, FVector COM);
	static FVector LinearImpulseGetDeltaVelocity(FVector Force, float Mass);
	static FVector GetForceFromImpulse(FVector Impulse, float Mass);
	static float GetDeltaSpeedAfterLinearImpulseForSphereCentralApplied(float Magnitude, float Mass);

	static void SimulateAddImpulseAtLocationForSphere2(FVector& InOutLinearVelocity, FVector& InOutAngularVelocity,
	                                                   const FVector& Impulse, float Mass, const FVector& ApplyLocation, const FVector& COM, FSimpleMatrix3 InertiaTensorInv);

	static FVector SimulateAddLinearImpulse(FVector Force, FVector LinearVelocity, float Mass);
	// requires force converted to COM (using arm vector from COM to Apply point)
	static FVector SimulateAddAngularImpulseConvCOM(FVector Impulse, FVector AngularVelocity, FSimpleMatrix3 InertiaTensorInv);
	
	static void SimulateAddImpulseToAreaForSphere2(FVector& InOutLinearVelocity, FVector& InOutAngularVelocity, const FVector& Impulse,
	                                               float Mass, const TArray<FVector>& AreaPoints, const FVector& COM, FSimpleMatrix3 InertiaTensorInv);

	static void SimulateVelocityDamping(FVector &Velocity, float VelocityDamping,  float DeltaTime);

	static  void UpdateTransformLock(UCustomPhysicsComponent* Obj, const FVector& PrevLocation, const FQuat& PrevOrientation);
	static  void UpdateTransformLock(FPhysTransform& InOutT, const FVector& PrevLocation, const FQuat& PrevOrientation, bool bLockX, bool bLockY, bool bLockZ);

public:
	static bool CalculatePredictionCurveWithMeta(UCustomPhysicsComponent* Obj, const FPhysCurveSpecialPrediction& Data, FPhysicsPredictionCurveWithMeta& Out);
	
};
