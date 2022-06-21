// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"

#include "GameModeCustomPhysics.generated.h"

class UCustomPhysicsProcessor;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API AGameModeCustomPhysics : public AGameModeBase
{
	GENERATED_BODY()

public:
	AGameModeCustomPhysics();
	
public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	UCustomPhysicsProcessor* PhysicsProcessor;
	
};
