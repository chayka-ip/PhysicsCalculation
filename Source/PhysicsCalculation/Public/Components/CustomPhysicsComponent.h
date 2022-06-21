// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "CustomPhysicsBaseComponent.h"
#include "Common/PhysPredict.h"
#include "Common/PhysRigidBodyParams.h"
#include "HMStructs/CustomVectorCurve.h"
#include "Kick/KickImpulseDataStruct.h"
#include "CustomPhysicsComponent.generated.h"

class UPhysPredictSettings_DataAsset;
class UCustomPhysicsParamsDataAsset;
class UCustomPhysicsProcessor;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class PHYSICSCALCULATION_API UCustomPhysicsComponent : public UCustomPhysicsBaseComponent
{
	GENERATED_BODY()

protected:
	virtual void Validate() const;
	
public:	

	UCustomPhysicsComponent();

	UPROPERTY()
	UCustomPhysicsProcessor* PhysicsProcessor;

	UPROPERTY(BlueprintAssignable)
	FCustomPhysicsDelegate OnPredictionRecomputed;

	UPROPERTY(BlueprintAssignable)
	FCustomPhysicsDelegate OnRoughPredictionRecomputed;

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	bool bSimulatePhysics = true;

	UPROPERTY(EditAnywhere)
	bool bEnablePredictionOnStart = true;
	
	UPROPERTY(BlueprintReadOnly)
	FPhysRigidBodyParams PhysicsParams;

	UPROPERTY(EditAnywhere)
	FPhysPredict PhysicsPredict;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	UCustomPhysicsParamsDataAsset* PhysicsData;

	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	UPhysPredictSettings_DataAsset* PhysicsPredictSettings;
	
public:
	FPhysTransform CurrentTransform;

public:
	virtual void SetPrimitiveComponent(UPrimitiveComponent* P) override;
	
	bool IsCustomPhysicsEnabled() const {return bSimulatePhysics;}
	virtual void Initialize() override;

	virtual float GetMass() const override;
	virtual FSimpleMatrix3 GetInertiaTensorInverted() override;
	
protected:
	void CompletePhysRigidBodyParams();
	void RebuildPhysicsRepresentation();
	
public:
	void SubscribeToPredictionRecomputedEvent(UObject* Obj, FName FuncName);
	
public:
	
	/*Radius in Cm*/
	UFUNCTION(BlueprintPure)
	virtual float GetRadius() const	{return  UMathUtils::GetSphereRadiusFromScale(GetScale());}

	UFUNCTION(BlueprintCallable)
	void SetSimulatePhysics(const bool bSimulate);
	UFUNCTION(BlueprintCallable)
	void EnablePrediction() {PhysicsPredict.EnablePrediction();}
	UFUNCTION(BlueprintCallable)
	void DisablePrediction() {PhysicsPredict.DisablePrediction();}

	UFUNCTION(BlueprintCallable)
	void SetMagnusMultiplierBP(float V);
	UFUNCTION(BlueprintPure)
	float GetMagnusBP() const;

	UFUNCTION(BlueprintPure)
	float GetCurrentSideForceVelocityAlpha() const;

public:
	bool IsPredictionEnabled() const {return  PhysicsPredict.IsEnabled();}
	bool IsPredictionRecomputeRequired(const FPhysTransform& TRequired, const FPhysTransform& TPredicted) const;
	FPhysTransform SimulateDeltaMovementPredictMode(float Time);

	void ApplyCurrentTransformToOwner() const;
	
public:
	void SetCurrentTransform(const FPhysTransform& T, bool bRecomputePredict=true);
	void SetCurrentTransformPredictionCheck(const FPhysTransform& TRequired, const FPhysTransform& TPredicted);

	// Returns next supposed transform after motion within given time from current transform without applying it
	FPhysTransform GetPrecisePredictedTransform(float Time);
	FPhysTransform GetPrecisePredictedTransform(const FPhysTransform& TInitial,float Time);

	FPhysTransform GetAnyPredictedTransform(float Time);

	UFUNCTION(BlueprintCallable)
	void SetCurrentTransformBP(FPhysTransform T, bool bRecomputePredict=true) {SetCurrentTransform(T, bRecomputePredict);}
	UFUNCTION(BlueprintCallable)
    void SetCurrentTransform2(FTransform T, FVector LinearVel = FVector::ZeroVector, FVector AngularVel=FVector::ZeroVector, bool bRecomputePredict=true);

	UFUNCTION(BlueprintCallable)
	void SetLocation(FVector Location, bool bRecomputePredict=true);

	virtual void AddWorldLocation(FVector Location, bool bRecomputePredict=true) override;
	virtual void SetLinearVelocity(FVector LinearVelocity, bool bRecomputePredict=true) override;
	virtual void SetWorldOrientation(FQuat Q,bool bRecomputePredict=true) override;
	
	UFUNCTION(BlueprintCallable)
	void SetAngularVelocity(FVector AngularVelocity, bool bRecomputePredict=true);
	UFUNCTION(BlueprintCallable)
	void SetVelocity(FVector LinearVelocity=FVector::ZeroVector, FVector AngularVelocity=FVector::ZeroVector, bool bRecomputePredict=true);
	void RemoveVelocity(bool bRecomputePredict=true);

	UFUNCTION(BlueprintCallable)
	void RecomputePrediction(bool bRecompute=true);

	virtual void AddImpulse(FVector V, bool bRecomputePredict=true) override;
	virtual void AddImpulseAtLocation(const FVector Impulse, const FVector ApplyLocation, bool bRecomputePredict=true) override;
	virtual void AddImpulseToArea(FVector Impulse, const TArray<FVector>& AreaPoints, bool bRecomputePredict=true) override;
	virtual void AddImpulseToAreaForTransform(FPhysTransform &InOutT, FVector Impulse, const TArray<FVector>& AreaPoints);
	
	virtual void AddLinearImpulse(const FVector Force, bool bRecomputePredict=true) override;
	virtual void AddAngularImpulse(const FVector Force, bool bRecomputePredict=true) override;

	virtual void AddLinearImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict=true) override;
	virtual void AddAngularImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict=true) override;

public:

	FPhysTransform CalculateNextPreciseTransform(const FPhysTransform& CurrentT, float SkipTime, bool bCheckCollisions=true);
	FPhysTransform CalculateNextRoughTransform(const FPhysTransform& CurrentT, float SimStep, float SkipTime, bool bCheckCollisions=true);
	
	UFUNCTION(BlueprintCallable)
	FVector GetPredictedLocationAfterTime(float Time);

	UFUNCTION(BlueprintCallable)
	FTransform GetCurrentTransform() const {return CurrentTransform.ToTransform(GetScale());}
	
	UFUNCTION(BlueprintPure)
    FPhysTransform GetCurrentPhysTransform() const {return CurrentTransform;}
	UFUNCTION(BlueprintPure)
	void GetCurrentPhysTransformExpanded(FVector& Location, FVector& LinearVelocity, FVector& AngularVelocity) const;

    virtual FVector GetCurrentLocation() const override;
	virtual FVector GetCurrentLinearVelocity() const override;
	virtual FVector GetCurrentAngularVelocityRadians() const override;

public:

	UFUNCTION(BlueprintCallable)
	void SetPhysParams(FPhysRigidBodyParams P);

	UFUNCTION(BlueprintCallable)
	TArray<FVector> GetPredictedLocations(int SkipStep = 0);

	UFUNCTION(BlueprintCallable)
	FTrajectory GetTrajectory(){return PhysicsPredict.GetTrajectory();}
	UFUNCTION(BlueprintCallable)
	FCustomVectorCurve GetTrajectoryCurve(){return PhysicsPredict.GetTrajectoryCurve();}

	UFUNCTION(BlueprintPure)
	float GetRoughPredictSimStep() const;
	UFUNCTION(BlueprintPure)
	float GetPrecisePredictSimStep() const;
	
	TArray<FPhysTransform> PredictMovementFromTransform(const FPhysTransform& T, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	TArray<FPhysTransform> PredictMovementFromTransformAnyTimeStep(const FPhysTransform& T, float TimeStep, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	TArray<FPhysTransform> PredictMovementFromCurrentTransformAnyTimeStep(float TimeStep, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	TArray<FPhysTransform> PredictMovementFromDirectVelocities(FVector LinearVelocity, FVector AngularVelocity, int NumSteps, bool bCheckCollisions = true, bool bLimitZ=false, float LimitZ=0.0f);
	TArray<FPhysTransform> PredictMovementFromDirectVelocitiesAnyTimeStep(FVector LinearVelocity, FVector AngularVelocity, float TimeStep, int NumSteps, bool bCheckCollisions = true, bool bLimitZ=false, float LimitZ=0.0f);
	TArray<FPhysTransform> PredictMovementAfterImpulseApplied(FVector Impulse, FVector ApplyLocation, FVector InitialBodyLocationOffset, int NumSteps, bool bCheckCollisions=true, bool bRemoveAngularVelocity=false, bool bLimitZ=false, float LimitZ=0.0f);

	UFUNCTION(BlueprintCallable)
	TArray<FVector> PredictMovementFromTransformAnyTimeStepAndGetLocations(const FPhysTransform& T, float TimeStep, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	UFUNCTION(BlueprintCallable)
	TArray<FVector> PredictMovementAfterImpulseAppliedAndGetLocations(FVector Impulse, FVector ApplyLocation, FVector InitialBodyLocationOffset, int NumSteps, bool bCheckCollisions=true, bool bRemoveAngularVelocity=false, bool bLimitZ=false, float LimitZ=0.0f);
	UFUNCTION(BlueprintCallable)
	TArray<FVector> PredictMovementFromDirectVelocitiesAndGetLocations(FVector LinearVelocity,FVector AngularVelocity, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	UFUNCTION(BlueprintCallable)
	TArray<FVector> PredictMovementFromDirectVelocitiesAndGetLocationsAnyTimeStep(FVector LinearVelocity,FVector AngularVelocity, float TimeStep, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	UFUNCTION(BlueprintCallable)
	TArray<FVector> PredictMovementFromCurrentTransformAndGetLocationsAnyTimeStep(float TimeStep, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);

	UFUNCTION(BlueprintCallable)
	FCustomVectorCurve PredictMovementFromDirectVelocitiesAndGetTrajectoryAsCurveAnyTimeStep(FVector LinearVelocity,FVector AngularVelocity, float TimeStep, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	UFUNCTION(BlueprintCallable)
	FCustomVectorCurve PredictMovementFromTransformAndGetTrajectoryAsCurveAnyTimeStep(const FPhysTransform& T, float TimeStep, int NumSteps, bool bCheckCollisions=true, bool bLimitZ=false, float LimitZ=0.0f);
	
public:

	UFUNCTION(BlueprintPure)
	FPhysTransform CalculateImpulseImpactAtLocation(FVector Impulse, FVector ApplyLocation, bool bRemoveVelocity=false, FVector InitialBodyLocationOffset=FVector::ZeroVector);
	FPhysTransform CalculateImpulseImpactAtLocation2(const FKickImpulseDataStruct& ImpulseData, bool bRemoveVelocity=false);

	UFUNCTION(BlueprintPure)
	FPhysTransform CalculateImpulseImpactAtLocationOtherCOM(FVector Impulse, FVector ApplyLocation, FVector COM, FVector InitialBodyLocationOffset=FVector::ZeroVector, bool bRemoveLV=false, bool bRemoveAV=false);

	UFUNCTION(BlueprintPure)
	FPhysTransform CalculateImpulseImpactOnArea(FVector Impulse, const TArray<FVector>& AreaPoints, FVector InitialBodyLocationOffset=FVector::ZeroVector);
};
