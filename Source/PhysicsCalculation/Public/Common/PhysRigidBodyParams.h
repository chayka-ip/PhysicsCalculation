#pragma once

#include "CoreMinimal.h"
#include "Aerodynamics/BodyAerodynamics.h"
#include "PhysConstrains.h"
#include "VelocityDamping.h"
#include "Libs/InertiaLib.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "PhysRigidBodyParams.generated.h"


USTRUCT(BlueprintType)
struct FPhysRigidBodyParams
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly)
	float GravityZ =  UPhysicsSettings::Get()->DefaultGravityZ;
	
	UPROPERTY(BlueprintReadOnly)
	TEnumAsByte<EShape> Shape;
	
	/*Spherical Radius*/
	UPROPERTY(BlueprintReadWrite)
	float Radius = 0.0f;
	
	UPROPERTY(BlueprintReadWrite)
	float Mass = 1.0f;
	
	UPROPERTY(BlueprintReadWrite)
	FVelocityDamping LinearDamping;
	
	UPROPERTY(BlueprintReadWrite)
	FVelocityDamping AngularDamping;
	
	UPROPERTY(BlueprintReadWrite)
	bool bGravityEnabled;
	
	UPROPERTY(BlueprintReadOnly)
	FBodyAerodynamics Aerodynamics;

	UPROPERTY(BlueprintReadOnly)
	FPhysConstrains Constrains;

	UPROPERTY(BlueprintReadOnly)
	FPhysRendering Rendering;
	
	FSimpleMatrix3 GetInertiaTensorInverted() const
	{
		return UInertiaLib::GetCentralAxisInertiaTensor(GetMass(), Radius, Shape, true);
	}

	float GetSphericalCrossSectionArea() const
	{
		return Pi *  Radius * Radius;
	}

	FVector GetGravity() const {return FVector(0, 0, GravityZ);}
	FVector GetGravityForce() const {return GetGravity() * GetMass();}

	float GetMass() const {return Mass;}
	float GetMassInv() const {return 1.0f / GetMass();}
};
