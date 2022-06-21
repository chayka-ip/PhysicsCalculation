#pragma once

#include "CoreMinimal.h"
#include "PhysPredictSettings.h"

#include "PhysTrajectory.h"
#include "PhysTransform.h"
#include "HMStructs/CustomVectorCurve.h"

#include "PhysPredict.generated.h"

/*
 * TODO: rebuild this structure;
 *
 * Use custom structure based on float curves instead of transform arrays for easier interpolation and updates;
 * Value which is obtained from curve at required time should be FPhysTransform; 
 */

/*
* Prediction: Object is moved along obtained trajectory line. Location is changed every substep tick
*/
class UCustomPhysicsComponent;
USTRUCT(BlueprintType)
struct FPhysPredict
{
	GENERATED_BODY()

	UPROPERTY()
	UCustomPhysicsComponent* Comp;	

	bool bPredict = false;
	FPhysPredictSettings Settings;
	
public:
	
	// How much time left before the next transform will match real object transform
	float TimeLeftToNextPPT = 0;
	float TimeSinceRoughPredictUpdate = 0.0f;
	
	UPROPERTY(BlueprintReadOnly)
	TArray<FPhysTransform> PrecisePredictedTransforms;

	UPROPERTY(BlueprintReadOnly)
	TArray<FPhysTransform> RoughPredictedTransforms;


public:

	void Init(UCustomPhysicsComponent* C){Comp = C;}
	int GetPreciseStepsCount() const;
	float GetRoughSimulationTimeStep() const;
	bool ShouldRecomputeRoughPredict(float TimeSinceLastUpdate) const;
	int GetRoughStepsCount() const;
	void RecomputePrediction();

	void EnablePrediction();
	void DisablePrediction();
	float GetSimulationDeltaTime() const;

	bool HasTPredicted() const {return PrecisePredictedTransforms.Num() > 0;}
	bool HasRoughPredicted() const {return RoughPredictedTransforms.Num() > 0;}
	
	/*
	* Should be called when unexpected forces are affected the object (collision, hit, etc)
	*/
	void RecomputePrecisePredict();
	void RecomputeRoughPredict();
	void AddStepsToPrecisePredictArray(const int StepsToAdd);
	void AddStepsToRoughPredictArray(int StepsToAdd);
	void UpdatePrecisePredictionDataBySteps(int StepsToUpdate);
	void AdvanceByTime(float TimeBeforeNextPredict, int StepsToUpdate);
	bool IsEnabled() const {return bPredict;}

	FPhysTransform* GetFirstPrecisePredictedTransformPtr();
	FPhysTransform* GetLastPrecisePredictedTransformPtr();
	FPhysTransform* GetLastPrecisePredictedTransformPtrOrObjCurrentTransform();

	FPhysTransform* GetFirstRoughPredictedTransformPtr();
	FPhysTransform* GetLastRoughPredictedTransformPtr();
	FPhysTransform* GetLastRoughPredictedTransformPtrOrLastPreciseTransform();

	// returns First if NumSteps <= 0; Last if NumSteps >= array size
	FPhysTransform* GetPrecisePredictedTransformPtrAfterStepCount(int NumSubsteps);
	FPhysTransform* GetRoughPredictedTransformPtrAfterStepCount(int NumSubsteps);

	void SetTimeBeforeNextPredict(float f){TimeLeftToNextPPT = f;}
	float GetTimeBeforeNextPredict() const {return TimeLeftToNextPPT;}
	TArray<FVector> GetPredictedLocations();

	FTrajectory GetTrajectory();
	FCustomVectorCurve GetTrajectoryCurve();
};

