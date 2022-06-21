// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "constants.h"
#include "Common/FirstTickCheck.h"
#include "Common/PhysTransform.h"
#include "Common/PSI_Data.h"
#include "Components/ActorComponent.h"
#include "CustomPhysicsProcessorBase.generated.h"

struct FPhysRigidBodyParams;
class UAdvancedPhysicsComponent;
class UCustomPhysicsBaseComponent;
class UCustomPhysicsComponent;

/*
 * Resolves and manages custom physics interactions;
 * 
 * At this moment we don't plan to have more than one active object with custom physics and many static objects, so
 * simple collision resolving system is enough.
 * In the future quadtree system can be implemented.
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class PHYSICSCALCULATION_API UCustomPhysicsProcessorBase : public UActorComponent
{
	GENERATED_BODY()
	
private:
	FFirstTickCheck FirstTickCheck;
	
public:	
	UCustomPhysicsProcessorBase();

public:
	bool IsInGameWorld() const;

protected:
	bool IsInitialized() const {return FirstTickCheck.IsTriggered();}
	void TrackSecondTick();
	virtual void OnSecondTick(){}

protected:	
	float SimulationDeltaTime = PHYS_SIM_DT;
	
public:	
	float GetSimDT() const {return  SimulationDeltaTime;}
	static float GetSimulationFrameTime(){return PHYS_FPS;}
	
protected:
	virtual void BeginPlay() override;

public:
	FPSI_Data MakeDefaultDataForPhysicsIteration(UCustomPhysicsComponent* Obj) const;
	
protected:
	void GetPhysObjArrays(TArray<UCustomPhysicsComponent*> &FullObjects, TArray<UCustomPhysicsBaseComponent*> &SimplifiedObjects);

	void UpdateCustomPhysics(float DeltaTime, const TArray<UCustomPhysicsComponent*> &FullObjects, const TArray<UCustomPhysicsBaseComponent*> &SimplifiedObjects) const;
	void UpdateTransformLock(const UCustomPhysicsComponent* Obj, FPhysTransform& InOutT, FVector PrevLocation, FQuat PrevOrientation) const;
	void ProcessPhysicsIteration(const TArray<UCustomPhysicsComponent*> &FullObjects, const TArray<UCustomPhysicsBaseComponent*> &SimplifiedObjects, float DeltaTime) const;

	static FPhysTransform CalculateNextTransformTimeBased(FPSI_Data& Data);
	
	float SplitTimeToSubstepsAndFraction(float Value, int& NumSteps) const;
	float GetSimulationRationalTimeFullStep(float Time) const;

public:
	void PredictTransform(const TArray<UCustomPhysicsBaseComponent*>& StaticBodies, FPSI_Data& Data, FPhysTransform& OutT) const;
	void PredictTransformAnyTime(FPSI_Data& Data, FPhysTransform& OutT) const;
	
public:	
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public:

	UPROPERTY(BlueprintReadOnly)
	TArray<UCustomPhysicsComponent*> Objects;

	UPROPERTY(BlueprintReadOnly)
	TArray<UCustomPhysicsBaseComponent*> SimpleObjects;

	UFUNCTION()
	void SubscribeNewObject(UCustomPhysicsBaseComponent* Obj);

};
