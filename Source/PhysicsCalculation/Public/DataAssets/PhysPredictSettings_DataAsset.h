// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysPredictSettings.h"
#include "Engine/DataAsset.h"
#include "PhysPredictSettings_DataAsset.generated.h"

struct FPhysPredict;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UPhysPredictSettings_DataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere)
	FPhysPredictSettings Data;

public:
	void FillParamStruct(FPhysPredict& S) const;
	
};
