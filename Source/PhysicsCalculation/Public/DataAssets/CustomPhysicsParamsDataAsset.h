// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Aerodynamics/BodyAerodynamics.h"
#include "Common/PhysConstrains.h"
#include "Common/VelocityDamping.h"
#include "Engine/DataAsset.h"
#include "Libs/InertiaLib.h"
#include "CustomPhysicsParamsDataAsset.generated.h"


struct FPhysRigidBodyParams;

USTRUCT(BlueprintType)
struct FDefaultPhysicsParams
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	bool bOverride = false;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	float MassInKg;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	FVelocityDamping LinearDamping;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	FVelocityDamping AngularDamping;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	bool bEnableGravity = true;
};

UCLASS()
class PHYSICSCALCULATION_API UCustomPhysicsParamsDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
	TEnumAsByte<EShape> Shape = Ball;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
	FBodyAerodynamics Aerodynamics;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
	FPhysConstrains Constrains;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category=Params)
	FPhysRendering Rendering;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Override Defaults")
	FDefaultPhysicsParams DefaultOverride;

public:
	void FillParamsStruct(FPhysRigidBodyParams& P) const;
};
