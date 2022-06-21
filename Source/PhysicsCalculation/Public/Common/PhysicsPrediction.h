// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PhysicsPredictionCurve.h"
#include "PhysPredictSettings.h"
#include "PhysTransform.h"
#include "PhysicsPrediction.generated.h"

class UCustomPhysicsComponent;
/**
 * 
 */
UCLASS()
class PHYSICSCALCULATION_API UPhysicsPrediction : public UObject
{
	GENERATED_BODY()

public:
	void Initialize();
	
private:
	
	UPROPERTY()
	UCustomPhysicsComponent* Comp = nullptr;	

	bool bPredict = false;
	FPhysPredictSettings Settings;

protected:
	FPhysicsPredictionCurve PrecisePredictionCurve;
	FPhysicsPredictionCurve RoughPredictionCurve;
	
public:
	
	// How much time left before the next transform will match real object transform
	float TimeLeftToNextPPT = 0;
	float TimeSinceRoughPredictUpdate = 0.0f;

public:
	void SetTimeBeforeNextPredict(float f){TimeLeftToNextPPT = f;}
	float GetTimeBeforeNextPredict() const {return TimeLeftToNextPPT;}
	
public:

	void EnablePrediction();
	void DisablePrediction();
	
	float GetSimulationDeltaTime() const;
	int GetPreciseStepsCount() const;
	float GetRoughSimulationTimeStep() const;
	bool ShouldRecomputeRoughPredict(float TimeSinceLastUpdate) const;
	int GetRoughStepsCount() const;
	void RecomputePrediction();

	bool HasTPredicted() const {return PrecisePredictionCurve.HasKeys();}
	bool HasRoughPredicted() const {return RoughPredictionCurve.HasKeys();}

public:
	/*
	* Should be called when unexpected forces are affected the object (collision, hit, etc)
	*/
	bool IsEnabled() const {return bPredict;}
	void RecomputePrecisePredict();
	void RecomputeRoughPredict();
	void AddStepsToPrecisePrediction(int StepsToAdd);
	void AddStepsToRoughPrediction(int StepsToAdd);
	
public:
	FPhysTransform GetLastTransformOrObjCurrentTransform(const FPhysicsPredictionCurve& Curve) const;
	FPhysTransform GetLastPrecisePredictedTransformPtrOrObjCurrentTransform() const;
	FPhysTransform GetLastRoughPredictedTransformPtrOrLastPreciseTransform() const;

};
