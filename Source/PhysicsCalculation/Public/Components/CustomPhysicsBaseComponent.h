// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Common/PhysEnums.h"
#include "Common/SimpleMatrix3.h"
#include "Components/ActorComponent.h"
#include "CustomPhysicsBaseComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FCustomPhysicsDelegate);
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class PHYSICSCALCULATION_API UCustomPhysicsBaseComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	UCustomPhysicsBaseComponent();

protected:

	UPROPERTY()
	AActor* Owner;

	UPROPERTY()
	UPrimitiveComponent* PrimitiveComponent;
		
	virtual void BeginPlay() override;
	virtual void SetDefaultPrimitiveComponent();
	
public:
	// whether this object used when cached physics is computed
	UPROPERTY(EditAnywhere)
	bool bUsedForAdvancedComputation=false;
	
public:
	bool IsDefaultPhysicsEnabled() const;

	UPhysicalMaterial* GetPhysicalMaterial() const;

	FVector GetScale() const {return Owner ? Owner->GetActorScale() : FVector::ZeroVector;}
	
	virtual float GetMass() const;
	float GetMassInv() const;
	float GetRestitution() const;
	float GetFriction() const;

	UFUNCTION(BlueprintPure)
	EPhysicsCombineMode GetFrictionCombineMode() const;
	
	UFUNCTION(BlueprintPure)
	EPhysicsCombineMode GetRestitutionCombineMode() const;

	bool IsLockLocationX() const;
	bool IsLockLocationY() const;
	bool IsLockLocationZ() const;
	
public:
	UFUNCTION(BlueprintCallable)
	virtual void SetPrimitiveComponent(UPrimitiveComponent* P){PrimitiveComponent = P;}

	UFUNCTION(BlueprintPure)
	UPrimitiveComponent* GetPrimitiveComponent() const {return PrimitiveComponent;}

	UFUNCTION(BlueprintCallable)
	void SetUsedForAdvancedPhysicsComputation(bool B){bUsedForAdvancedComputation = B;}
	UFUNCTION(BlueprintPure)
	bool IsEnabledForPhysicsCaching() const {return bUsedForAdvancedComputation;}
	
	UFUNCTION(BlueprintPure)
	virtual FVector GetCurrentLocation() const;
	UFUNCTION(BlueprintPure)
	virtual FVector GetCurrentLinearVelocity() const;
	UFUNCTION(BlueprintPure)
	virtual FVector GetCurrentAngularVelocityRadians() const;
	UFUNCTION(BlueprintPure)
	FVector GetCurrentAngularVelocityAtPointInRadians(FVector P) const;
	UFUNCTION(BlueprintPure)
	FVector GetFullVelocityAtPoint(FVector P) const;

	UFUNCTION(BlueprintPure)
	float GetCurrentSpeed() const {return GetCurrentLinearVelocity().Size();}
	
	UFUNCTION(BlueprintPure)
	float AngleLinearToAngularVelocity() const;

	UFUNCTION(BlueprintPure)
	void GetArrowStartEndFromVector(FVector Dir, float Size, FVector& Start, FVector& End) const;
	UFUNCTION(BlueprintPure)
	void GetCurrentLinearVelocityAxisArrowStartEnd(float Size, FVector& Start, FVector& End) const;
	UFUNCTION(BlueprintPure)
	void GetCurrentAngularVelocityAxisArrowStartEnd(float Size, FVector& Start, FVector& End) const;
	
	UFUNCTION(BlueprintCallable)
	virtual void SetLinearVelocity(FVector LinearVelocity, bool bRecomputePredict=true);

	UFUNCTION(BlueprintCallable)
	virtual void SetWorldOrientation(FQuat Q,bool bRecomputePredict=true);
	
	UFUNCTION(BlueprintCallable)
	virtual  void AddWorldLocation(FVector V, bool bRecomputePredict=true);
	UFUNCTION(BlueprintCallable)
	virtual  void AddImpulse(FVector V, bool bRecomputePredict=true);
	UFUNCTION(BlueprintCallable)
	virtual void AddImpulseAtLocation(FVector Impulse, FVector ApplyLocation, bool bRecomputePredict=true);
	UFUNCTION(BlueprintCallable)
	virtual void AddImpulseToArea(FVector Impulse, const TArray<FVector>& AreaPoints, bool bRecomputePredict=true);
	UFUNCTION(BlueprintCallable)
	virtual void AddLinearImpulse(const FVector Force, bool bRecomputePredict=true);
	UFUNCTION(BlueprintCallable)
	virtual void AddAngularImpulse(const FVector Force, bool bRecomputePredict=true);

	virtual void AddLinearImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict=true);
	virtual void AddAngularImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict=true);

	virtual FSimpleMatrix3 GetInertiaTensorInverted();
	
	virtual void Initialize() {}

		
};
