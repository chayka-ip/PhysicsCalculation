// Fill out your copyright notice in the Description page of Project Settings.

#include "PhysicsCalculation/Public/Components/CustomPhysicsComponent.h"
#include "CommonUtilsLib.h"
#include "Components/CustomPhysicsProcessor.h"
#include "Libs/PhysicsSimulation.h"
#include "DataAssets/CustomPhysicsParamsDataAsset.h"
#include "Libs/PhysicsUtils.h"
#include "Libs/UtilsLib.h"
#include "CommonUtils/Public/Utils.h"
#include "DataAssets/PhysPredictSettings_DataAsset.h"
#include "Libs/MovementPredictionLib.h"

void UCustomPhysicsComponent::Validate() const
{
	check(PhysicsData)
	check(PhysicsPredictSettings)
}

UCustomPhysicsComponent::UCustomPhysicsComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UCustomPhysicsComponent::CompletePhysRigidBodyParams()
{
	const bool IsPhysicsEnabled = PrimitiveComponent->IsSimulatingPhysics();
	const ECollisionEnabled::Type CollisionType = PrimitiveComponent->GetCollisionEnabled();
	PrimitiveComponent->SetSimulatePhysics(true);
	PrimitiveComponent->SetCollisionEnabled(ECollisionEnabled::Type::PhysicsOnly);
	
	PhysicsParams.Radius = GetRadius();
	PhysicsParams.Mass = PrimitiveComponent->GetMass();
	PhysicsParams.LinearDamping.Value = PrimitiveComponent->GetLinearDamping();
	PhysicsParams.AngularDamping.Value = PrimitiveComponent->GetAngularDamping();
	PhysicsParams.bGravityEnabled = PrimitiveComponent->IsGravityEnabled();

	PrimitiveComponent->SetSimulatePhysics(IsPhysicsEnabled);
	PrimitiveComponent->SetCollisionEnabled(CollisionType);

	if(PhysicsData) PhysicsData->FillParamsStruct(PhysicsParams);
	if(PhysicsPredictSettings) PhysicsPredictSettings->FillParamStruct(PhysicsPredict);

	PhysicsParams.Aerodynamics.Init();
}

void UCustomPhysicsComponent::RebuildPhysicsRepresentation()
{
	CompletePhysRigidBodyParams();

	const FPhysTransform T = FPhysTransform(Owner->GetTransform());
	SetCurrentTransform(T, false);
	PhysicsPredict.Init(this);

	if(bSimulatePhysics)
	{
		PrimitiveComponent->SetSimulatePhysics(false);
		if(bEnablePredictionOnStart) EnablePrediction();
	}
}

void UCustomPhysicsComponent::SubscribeToPredictionRecomputedEvent(UObject* Obj, FName FuncName)
{
	UCommonUtilsLib::AddUniqueDelegate(OnPredictionRecomputed, Obj, FuncName);
}

void UCustomPhysicsComponent::SetSimulatePhysics(const bool bSimulate)
{
	const bool Prev = bSimulatePhysics;
	bSimulatePhysics = bSimulate;

	RecomputePrediction(!Prev && bSimulatePhysics);
}

void UCustomPhysicsComponent::SetPrimitiveComponent(UPrimitiveComponent* P)
{
	Super::SetPrimitiveComponent(P);
	PrimitiveComponent->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	RebuildPhysicsRepresentation();
}

void UCustomPhysicsComponent::Initialize()
{
	Super::Initialize();
	PhysicsProcessor = UUtilsLib::GetPhysicsProcessor();
	RebuildPhysicsRepresentation();
	Validate();
}

float UCustomPhysicsComponent::GetMass() const
{
	return IsCustomPhysicsEnabled() ? PhysicsParams.GetMass() : Super::GetMass();
}

FSimpleMatrix3 UCustomPhysicsComponent::GetInertiaTensorInverted()
{
	return IsCustomPhysicsEnabled() ? PhysicsParams.GetInertiaTensorInverted() : Super::GetInertiaTensorInverted();
}

void UCustomPhysicsComponent::SetMagnusMultiplierBP(float V)
{
	PhysicsParams.Aerodynamics.MagnusImpact.Value = V;
	RecomputePrediction(true);
}

float UCustomPhysicsComponent::GetMagnusBP() const
{
	return PhysicsParams.Aerodynamics.MagnusImpact.Value;
}

float UCustomPhysicsComponent::GetCurrentSideForceVelocityAlpha() const
{
	if(PhysicsParams.Aerodynamics.Sideforce.bEnabled)
	{
		const FVector L = CurrentTransform.LinearVelocity;
		const FVector A = CurrentTransform.AngularVelocity;
		return PhysicsParams.Aerodynamics.Sideforce.GetImpactAlphaFromVelocity(L, A);
	}
	return 0.0f;
}

void UCustomPhysicsComponent::ApplyCurrentTransformToOwner() const
{
	if(Owner)
	{
		Owner->SetActorTransform(CurrentTransform.ToTransform(GetScale()));
	}
}

void UCustomPhysicsComponent::SetCurrentTransform2(FTransform T,const FVector LinearVel, const FVector AngularVel, bool bRecomputePredict)
{
	SetCurrentTransform(FPhysTransform(T, LinearVel, AngularVel), bRecomputePredict);
}

void UCustomPhysicsComponent::SetCurrentTransform(const FPhysTransform& T, bool bRecomputePredict)
{
	CurrentTransform = T;
	RecomputePrediction(bRecomputePredict); 
}

bool UCustomPhysicsComponent::IsPredictionRecomputeRequired(const FPhysTransform& TRequired, const FPhysTransform& TPredicted) const
{
	const auto Settings = &PhysicsPredict.Settings;
	const float RequiredVelocity = TRequired.LinearVelocity.Size();
	const float RestVelocityMaxLimit = Settings->GroundVelocityThreshold;
	const float ErrorTolerance = RequiredVelocity < RestVelocityMaxLimit ? Settings->TransformTolerance : Settings->DefaultTolerance;
	const bool IsAcceptable = TRequired.EqualsNoOrientCheck(TPredicted, ErrorTolerance);
	return !IsAcceptable;
}

void UCustomPhysicsComponent::SetCurrentTransformPredictionCheck(const FPhysTransform& TRequired, const FPhysTransform& TPredicted)
{
	const bool bRecompute = IsPredictionRecomputeRequired(TRequired, TPredicted);
	if(bRecompute)
	{
		SetCurrentTransform(TRequired, true);
	}
	else
	{
		SetCurrentTransform(TPredicted, false);
	}
}

FPhysTransform UCustomPhysicsComponent::SimulateDeltaMovementPredictMode(float Time)
{
	const float SimulationDeltaTime = PhysicsProcessor->GetSimDT();
	FPhysTransform T = CurrentTransform;
	if(IsPredictionEnabled())
	{
		UMovementPredictionLib::PredictGetNextPreciseTransformAndUpdateStruct(T, PhysicsPredict, Time, SimulationDeltaTime);
	}
	return T;
}

FPhysTransform UCustomPhysicsComponent::GetPrecisePredictedTransform(float Time)
{
	return UMovementPredictionLib::GetPrecisePredictedTransform(CurrentTransform, PhysicsPredict,  Time, PhysicsProcessor->GetSimDT());
}

FPhysTransform UCustomPhysicsComponent::GetPrecisePredictedTransform(const FPhysTransform& TInitial, float Time)
{
	return UMovementPredictionLib::GetPrecisePredictedTransform(TInitial, PhysicsPredict,  Time, PhysicsProcessor->GetSimDT());
}

FPhysTransform UCustomPhysicsComponent::GetAnyPredictedTransform(float Time)
{
	const float PreciseTime = PhysicsPredict.Settings.PrecisePredictTimeSec;
	if(Time <= PreciseTime) return GetPrecisePredictedTransform( Time);
	Time -= PreciseTime;
	return UMovementPredictionLib::GetRoughPredictedTransform(PhysicsPredict, Time);
}

void UCustomPhysicsComponent::AddImpulseAtLocation(const FVector Impulse, const FVector ApplyLocation, bool bRecomputePredict)
{
	if(IsCustomPhysicsEnabled())
	{
		UPhysicsSimulation::SimulateAddImpulseAtLocationForSphere2(CurrentTransform.LinearVelocity,
														  CurrentTransform.AngularVelocity, Impulse,
														  PhysicsParams.GetMass(),  ApplyLocation,
														  CurrentTransform.Location, GetInertiaTensorInverted());
		RecomputePrediction(bRecomputePredict); 
	}
	else
	{
		Super::AddImpulseAtLocation(Impulse, ApplyLocation, bRecomputePredict);
	}
}

void UCustomPhysicsComponent::AddImpulseToArea(FVector Impulse, const TArray<FVector>& AreaPoints, bool bRecomputePredict)
{
	if(IsCustomPhysicsEnabled())
	{
		AddImpulseToAreaForTransform(CurrentTransform, Impulse, AreaPoints);
		RecomputePrediction(bRecomputePredict);
	}
	else
	{
		Super::AddImpulseToArea(Impulse, AreaPoints, bRecomputePredict);
	}
}

void UCustomPhysicsComponent::AddImpulseToAreaForTransform(FPhysTransform &InOutT, FVector Impulse, const TArray<FVector>& AreaPoints)
{
	UPhysicsSimulation::SimulateAddImpulseToAreaForSphere2(InOutT.LinearVelocity, InOutT.AngularVelocity, Impulse, GetMass(), AreaPoints,
		InOutT.Location, GetInertiaTensorInverted());
}

void UCustomPhysicsComponent::AddLinearImpulse(const FVector Force, bool bRecomputePredict)
{
	const FVector NewVelocity = UPhysicsSimulation::SimulateAddLinearImpulse(Force, GetCurrentLinearVelocity(), GetMass());
	SetLinearVelocity(NewVelocity, bRecomputePredict);
}

void UCustomPhysicsComponent::AddAngularImpulse(const FVector Force, bool bRecomputePredict)
{
	const FVector AV = UPhysicsSimulation::SimulateAddAngularImpulseConvCOM(Force, GetCurrentAngularVelocityRadians(), GetInertiaTensorInverted());
	SetAngularVelocity(AV, bRecomputePredict);
}

void UCustomPhysicsComponent::AddLinearImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict)
{
	// Super::AddLinearImpulseAsApplied(Force, Time, bRecomputePredict);
	AddLinearImpulse(Force, bRecomputePredict);
}

void UCustomPhysicsComponent::AddAngularImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict)
{
	// Super::AddAngularImpulseAsApplied(Force, Time, bRecomputePredict);
	AddAngularImpulse(Force, bRecomputePredict);
}

FPhysTransform UCustomPhysicsComponent::CalculateNextPreciseTransform(const FPhysTransform &CurrentT, float SkipTime,bool bCheckCollisions)
{
	FPhysTransform OutT = CurrentT;
	if(IsCustomPhysicsEnabled() && PhysicsProcessor)
	{
		FPSI_Data Data = PhysicsProcessor->MakeDefaultDataForPhysicsIteration(this);
		Data.SetTransform(CurrentT);
		Data.SetDeltaTime(PhysicsPredict.GetSimulationDeltaTime());
		Data.ToggleCollisions(bCheckCollisions);
		Data.SetSkipTime(SkipTime);
		
		PhysicsProcessor->PredictTransformAnyTime(Data, OutT);
	}
	return OutT;
}

FPhysTransform UCustomPhysicsComponent::CalculateNextRoughTransform(const FPhysTransform& CurrentT, float SimStep, float SkipTime, bool bCheckCollisions)
{
	FPhysTransform OutT = CurrentT;
	if(IsCustomPhysicsEnabled() && PhysicsProcessor)
	{
		FPSI_Data Data = PhysicsProcessor->MakeDefaultDataForPhysicsIteration(this);
		Data.SetTransform(CurrentT);
		Data.SetDeltaTime(SimStep);
		Data.SetSkipTime(SkipTime);
		Data.DisableExtraForces();
		Data.ToggleCollisions(bCheckCollisions);

		PhysicsProcessor->PredictTransformAnyTime(Data, OutT);
	}
	return OutT;
}

FVector UCustomPhysicsComponent::GetPredictedLocationAfterTime(float Time)
{
	if(IsCustomPhysicsEnabled() && PhysicsProcessor)
	{
		return GetPrecisePredictedTransform(Time).Location;
	}
	return  FVector::ZeroVector;
}

void UCustomPhysicsComponent::SetLinearVelocity(FVector LinearVelocity, bool bRecomputePredict)
{
	CurrentTransform.LinearVelocity = LinearVelocity;
	RecomputePrediction(bRecomputePredict); 
}

void UCustomPhysicsComponent::SetWorldOrientation(FQuat Q, bool bRecomputePredict)
{
	if(IsCustomPhysicsEnabled())
	{
		CurrentTransform.Orientation = Q;
		RecomputePrediction(bRecomputePredict);
	}
	else
	{
		Super::SetWorldOrientation(Q, bRecomputePredict);
	}
}

void UCustomPhysicsComponent::SetAngularVelocity(FVector AngularVelocity, bool bRecomputePredict)
{
	CurrentTransform.AngularVelocity = AngularVelocity;
	RecomputePrediction(bRecomputePredict); 
}

void UCustomPhysicsComponent::SetVelocity(FVector LinearVelocity, FVector AngularVelocity, bool bRecomputePredict)
{
	CurrentTransform.LinearVelocity = LinearVelocity;
	CurrentTransform.AngularVelocity = AngularVelocity;
	RecomputePrediction(bRecomputePredict); 
}

void UCustomPhysicsComponent::RemoveVelocity(bool bRecomputePredict)
{
	const FVector VZ = FVector::ZeroVector;
	SetVelocity(VZ, VZ, bRecomputePredict);
}

void UCustomPhysicsComponent::RecomputePrediction(bool bRecompute)
{
	if (PhysicsPredict.IsEnabled() && bRecompute)
	{
		PhysicsPredict.RecomputePrediction();
		OnPredictionRecomputed.Broadcast();
	}
}

void UCustomPhysicsComponent::AddImpulse(FVector V, bool bRecomputePredict)
{
	AddImpulseAtLocation(V, GetCurrentLocation(), bRecomputePredict);	
}

void UCustomPhysicsComponent::SetLocation(FVector Location, bool bRecomputePredict)
{
	CurrentTransform.Location = Location;
	RecomputePrediction(bRecomputePredict); 
}

void UCustomPhysicsComponent::AddWorldLocation(FVector Location, bool bRecomputePredict)
{
	SetLocation(CurrentTransform.Location + Location, bRecomputePredict);
	ApplyCurrentTransformToOwner();
}

void UCustomPhysicsComponent::GetCurrentPhysTransformExpanded(FVector& Location, FVector& LinearVelocity, FVector& AngularVelocity) const
{
	Location = CurrentTransform.Location;
	LinearVelocity = CurrentTransform.LinearVelocity;
	AngularVelocity = CurrentTransform.AngularVelocity;
}

FVector UCustomPhysicsComponent::GetCurrentLocation() const
{
	return IsCustomPhysicsEnabled() ? CurrentTransform.Location : Super::GetCurrentLocation();
}

FVector UCustomPhysicsComponent::GetCurrentLinearVelocity() const
{
	return IsCustomPhysicsEnabled() ?  CurrentTransform.LinearVelocity : Super::GetCurrentLinearVelocity();
}

FVector UCustomPhysicsComponent::GetCurrentAngularVelocityRadians() const
{
	return IsCustomPhysicsEnabled() ?  CurrentTransform.AngularVelocity : Super::GetCurrentAngularVelocityRadians();
}

void UCustomPhysicsComponent::SetPhysParams(FPhysRigidBodyParams P)
{
	PhysicsParams = P;
	RecomputePrediction();
}

TArray<FVector> UCustomPhysicsComponent::GetPredictedLocations(int SkipStep)
{
	TArray<FVector> Data = PhysicsPredict.GetPredictedLocations();
	if(SkipStep == 0) return Data;

	const int Num = Data.Num();
	
	TArray<FVector> OutData;

	for (int i = 0; i < Num; ++i)
	{
		if(i == 0 || i == Num - 1 || i % SkipStep == 0)
		{
			OutData.Add(Data[i]);
		}
	}
	
	return OutData;
}

float UCustomPhysicsComponent::GetRoughPredictSimStep() const
{
	return PhysicsPredict.GetRoughSimulationTimeStep();
}

float UCustomPhysicsComponent::GetPrecisePredictSimStep() const
{
	return PhysicsPredict.GetSimulationDeltaTime();
}

TArray<FPhysTransform> UCustomPhysicsComponent::PredictMovementFromTransform(const FPhysTransform& T, int NumSteps, bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const float TimeStep = GetRoughPredictSimStep();
	return PredictMovementFromTransformAnyTimeStep(T, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
}

TArray<FPhysTransform> UCustomPhysicsComponent::PredictMovementFromTransformAnyTimeStep(const FPhysTransform& T, float TimeStep, int NumSteps,
                                                                                        bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	TArray<FPhysTransform> OutArray = {T};
	for (int i = 1; i < NumSteps; ++i)
	{
		auto NextT = CalculateNextRoughTransform(OutArray.Last(), TimeStep, 0.0f, bCheckCollisions);
		OutArray.Add(NextT);

		if(bLimitZ && NextT.Location.Z <= LimitZ) break;
	}
	return OutArray;
}

TArray<FPhysTransform> UCustomPhysicsComponent::PredictMovementFromCurrentTransformAnyTimeStep(float TimeStep, int NumSteps, bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	return PredictMovementFromTransformAnyTimeStep(CurrentTransform, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
}

TArray<FPhysTransform> UCustomPhysicsComponent::PredictMovementFromDirectVelocities(FVector LinearVelocity, FVector AngularVelocity, int NumSteps,
                                                                                    bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const float TimeStep = GetRoughPredictSimStep();
	return PredictMovementFromDirectVelocitiesAnyTimeStep(LinearVelocity, AngularVelocity, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
}

TArray<FPhysTransform> UCustomPhysicsComponent::PredictMovementFromDirectVelocitiesAnyTimeStep(FVector LinearVelocity, FVector AngularVelocity,
                                                                                               float TimeStep, int NumSteps, bool bCheckCollisions,
                                                                                               bool bLimitZ, float LimitZ)
{
	auto T = CurrentTransform;
	T.LinearVelocity = LinearVelocity;
	T.AngularVelocity = AngularVelocity;
	return PredictMovementFromTransformAnyTimeStep(T, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
}

TArray<FPhysTransform> UCustomPhysicsComponent::PredictMovementAfterImpulseApplied(FVector Impulse, FVector ApplyLocation, FVector InitialBodyLocationOffset, int NumSteps, bool bCheckCollisions, bool bRemoveAngularVelocity, bool bLimitZ, float LimitZ)
{
	auto T = CurrentTransform;

	T.Location += InitialBodyLocationOffset;
	ApplyLocation += InitialBodyLocationOffset;
	
	UPhysicsSimulation::SimulateAddImpulseAtLocationForSphere2(T.LinearVelocity,
													  T.AngularVelocity, Impulse,
													  PhysicsParams.GetMass(),  ApplyLocation,
													  T.Location, GetInertiaTensorInverted());
	
	if(bRemoveAngularVelocity)T.AngularVelocity = FVector::ZeroVector;
	
	return PredictMovementFromTransform(T, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
}

TArray<FVector> UCustomPhysicsComponent::PredictMovementFromTransformAnyTimeStepAndGetLocations(const FPhysTransform& T, float TimeStep, int NumSteps,
	bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const auto P = PredictMovementFromTransformAnyTimeStep(T, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
	return UUtilsLib::LocationsFromPhysTransformArray(P);

}

TArray<FVector> UCustomPhysicsComponent::PredictMovementAfterImpulseAppliedAndGetLocations(FVector Impulse, FVector ApplyLocation, FVector InitialBodyLocationOffset, int NumSteps, bool bCheckCollisions, bool bRemoveAngularVelocity, bool bLimitZ, float LimitZ)
{
	const auto T = PredictMovementAfterImpulseApplied(Impulse, ApplyLocation, InitialBodyLocationOffset, NumSteps, bCheckCollisions, bRemoveAngularVelocity, bLimitZ, LimitZ);
	return UUtilsLib::LocationsFromPhysTransformArray(T);
}

TArray<FVector> UCustomPhysicsComponent::PredictMovementFromDirectVelocitiesAndGetLocations(FVector LinearVelocity, FVector AngularVelocity,
	int NumSteps, bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const auto T = PredictMovementFromDirectVelocities(LinearVelocity, AngularVelocity, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
	return UUtilsLib::LocationsFromPhysTransformArray(T);
}

TArray<FVector> UCustomPhysicsComponent::PredictMovementFromDirectVelocitiesAndGetLocationsAnyTimeStep(FVector LinearVelocity,
	FVector AngularVelocity, float TimeStep, int NumSteps, bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const auto T = PredictMovementFromDirectVelocitiesAnyTimeStep(LinearVelocity, AngularVelocity, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
	return UUtilsLib::LocationsFromPhysTransformArray(T);
}

TArray<FVector> UCustomPhysicsComponent::PredictMovementFromCurrentTransformAndGetLocationsAnyTimeStep(float TimeStep, int NumSteps, bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const auto T = PredictMovementFromCurrentTransformAnyTimeStep(TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
	return UUtilsLib::LocationsFromPhysTransformArray(T);
}

FCustomVectorCurve UCustomPhysicsComponent::PredictMovementFromDirectVelocitiesAndGetTrajectoryAsCurveAnyTimeStep(FVector LinearVelocity,
                                                                                                                  FVector AngularVelocity, float TimeStep, int NumSteps, bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const auto L = PredictMovementFromDirectVelocitiesAndGetLocationsAnyTimeStep(LinearVelocity, AngularVelocity, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
	return FCustomVectorCurve(L, TimeStep);
}

FCustomVectorCurve UCustomPhysicsComponent::PredictMovementFromTransformAndGetTrajectoryAsCurveAnyTimeStep(const FPhysTransform& T, float TimeStep,
	int NumSteps, bool bCheckCollisions, bool bLimitZ, float LimitZ)
{
	const auto L = PredictMovementFromTransformAnyTimeStepAndGetLocations(T, TimeStep, NumSteps, bCheckCollisions, bLimitZ, LimitZ);
	return FCustomVectorCurve(L, TimeStep);
}

FPhysTransform UCustomPhysicsComponent::CalculateImpulseImpactAtLocation(FVector Impulse, FVector ApplyLocation, bool bRemoveVelocity, FVector InitialBodyLocationOffset)
{
	auto T = CurrentTransform;
	T.Location += InitialBodyLocationOffset;
	ApplyLocation += InitialBodyLocationOffset;

	if(bRemoveVelocity)
	{
		T.LinearVelocity = FVector::ZeroVector;
		T.AngularVelocity = FVector::ZeroVector;
	}
	
	UPhysicsSimulation::SimulateAddImpulseAtLocationForSphere2(T.LinearVelocity,
													  T.AngularVelocity, Impulse,
													  PhysicsParams.GetMass(),  ApplyLocation,
													  T.Location, GetInertiaTensorInverted());
	return T;
}

FPhysTransform UCustomPhysicsComponent::CalculateImpulseImpactAtLocation2(const FKickImpulseDataStruct& ImpulseData, bool bRemoveVelocity)
{
	return CalculateImpulseImpactAtLocation(ImpulseData.Impulse, ImpulseData.ApplyLocation,bRemoveVelocity, ImpulseData.InitialBodyLocationOffset);
}

FPhysTransform UCustomPhysicsComponent::CalculateImpulseImpactAtLocationOtherCOM(FVector Impulse, FVector ApplyLocation, FVector COM,
                                                                                 FVector InitialBodyLocationOffset, bool bRemoveLV, bool bRemoveAV)
{
	auto T = CurrentTransform;
	T.Location = COM + InitialBodyLocationOffset;
	ApplyLocation += InitialBodyLocationOffset;
	if(bRemoveLV) T.LinearVelocity = FVector::ZeroVector;
	if(bRemoveAV) T.AngularVelocity = FVector::ZeroVector;
	
	UPhysicsSimulation::SimulateAddImpulseAtLocationForSphere2(T.LinearVelocity,
													  T.AngularVelocity, Impulse,
													  PhysicsParams.GetMass(),  ApplyLocation,
													  T.Location, GetInertiaTensorInverted());
	return T;
}

FPhysTransform UCustomPhysicsComponent::CalculateImpulseImpactOnArea(FVector Impulse, const TArray<FVector>& AreaPoints,
                                                                     FVector InitialBodyLocationOffset) 
{
	auto T = CurrentTransform;
	T.Location += InitialBodyLocationOffset;

	AddImpulseToAreaForTransform(T, Impulse, AreaPoints);
	return T;
}

