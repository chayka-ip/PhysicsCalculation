// Fill out your copyright notice in the Description page of Project Settings.


#include "GameModeCustomPhysics.h"
#include "Components/CustomPhysicsProcessor.h"


AGameModeCustomPhysics::AGameModeCustomPhysics()
{
	PhysicsProcessor = CreateDefaultSubobject<UCustomPhysicsProcessor>(TEXT("PhysicsProcessor"));
}
