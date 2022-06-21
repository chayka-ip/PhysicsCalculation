// Fill out your copyright notice in the Description page of Project Settings.

#include "Components/CustomPhysicsProcessorBase.h"
#include "Collision/CollisionDetection.h"
#include "Collision/CollisionPair.h"
#include "Libs/PhysicsSimulation.h"
#include "Components/CustomPhysicsBaseComponent.h"
#include "Components/CustomPhysicsComponent.h"

UCustomPhysicsProcessorBase::UCustomPhysicsProcessorBase()
{
	PrimaryComponentTick.bCanEverTick = true;
}

bool UCustomPhysicsProcessorBase::IsInGameWorld() const
{
	return GetOuter() && GetWorld();
}

void UCustomPhysicsProcessorBase::TrackSecondTick()
{
	if(IsInitialized() || !IsInGameWorld()) return;
	if(FirstTickCheck.IsNotPassed())
	{
		FirstTickCheck.MarkPassed();
		return;
	}
	if(FirstTickCheck.IsPassed())
	{
		FirstTickCheck.MarkTriggered();
		OnSecondTick();
	}
}

void UCustomPhysicsProcessorBase::BeginPlay()
{
	Super::BeginPlay();
}

void UCustomPhysicsProcessorBase::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	TrackSecondTick();
	
	// this part may be refactored if arrays will be made constant (any changes on objects are synchronized with this component)
	TArray<UCustomPhysicsComponent*> FullObjects;
	TArray<UCustomPhysicsBaseComponent*> SimplifiedObjects;
	GetPhysObjArrays(FullObjects, SimplifiedObjects);

	UpdateCustomPhysics(DeltaTime, FullObjects, SimplifiedObjects);
}

void UCustomPhysicsProcessorBase::GetPhysObjArrays(TArray<UCustomPhysicsComponent*> &FullObjects, TArray<UCustomPhysicsBaseComponent*> &SimplifiedObjects)
{
	SimplifiedObjects.Append(SimpleObjects);
	for (auto Obj : Objects)
	{
		Obj->IsCustomPhysicsEnabled() ? FullObjects.Add(Obj) : SimplifiedObjects.Add(Obj);
	}
}

void UCustomPhysicsProcessorBase::UpdateCustomPhysics(float DeltaTime, const TArray<UCustomPhysicsComponent*> &FullObjects, const TArray<UCustomPhysicsBaseComponent*> &SimplifiedObjects) const
{
	int NumSubsteps = 0;
	const float Fraction = SplitTimeToSubstepsAndFraction(DeltaTime, NumSubsteps);

	for (int i = 0; i < NumSubsteps; ++i)
	{
		ProcessPhysicsIteration(FullObjects, SimplifiedObjects, GetSimDT());
	}

	if(Fraction > 0.0f)
	{
		ProcessPhysicsIteration(FullObjects, SimplifiedObjects, Fraction);
	}

	for (const auto Obj : FullObjects)
	{
		Obj->ApplyCurrentTransformToOwner();
	}
}

void UCustomPhysicsProcessorBase::UpdateTransformLock(const UCustomPhysicsComponent* Obj, FPhysTransform &InOutT, FVector PrevLocation, FQuat PrevOrientation) const
{
	const bool bLockX = Obj->IsLockLocationX();
	const bool bLockY = Obj->IsLockLocationY();
	const bool bLockZ = Obj->IsLockLocationZ();
	const bool HasAnyLock = bLockX || bLockY || bLockZ;

	if(HasAnyLock)
	{
		UPhysicsSimulation::UpdateTransformLock(InOutT, PrevLocation, PrevOrientation, bLockX, bLockY, bLockZ);
	}
}

FPSI_Data UCustomPhysicsProcessorBase::MakeDefaultDataForPhysicsIteration(UCustomPhysicsComponent* Obj) const
{
	FPSI_Data Data;
	
	Data.Obj = Obj;
	Data.SetTransform(Obj->CurrentTransform);
	Data.SetDeltaTime(GetSimDT());
	Data.EnableFullPhysics();
	
	return Data;
}

void UCustomPhysicsProcessorBase::ProcessPhysicsIteration(const TArray<UCustomPhysicsComponent*>& FullObjects,
                                                          const TArray<UCustomPhysicsBaseComponent*>& SimplifiedObjects, float DeltaTime) const
{
	TArray<FPhysTransform> PrevTransforms = {};
	TArray<FPhysTransform> PredictionTransforms = {};
	
	for (const auto Obj : FullObjects)
	{
		auto TPredict = Obj->SimulateDeltaMovementPredictMode(DeltaTime);
		PrevTransforms.Add(Obj->CurrentTransform);
		PredictionTransforms.Add(TPredict);
	}

	TArray<FCollisionPair> CollisionPairs;
	if(UCollisionDetection::FindCollisionsAgainstSphereArray(FullObjects, SimplifiedObjects, CollisionPairs))
	{
		for (auto CollisionPair : CollisionPairs)
		{
			UCollisionDetection::ResolveCustomCollisionAgainstSphere(CollisionPair);
		}
	}
	
	for (int i = 0; i < FullObjects.Num(); ++i)
	{
		const auto Obj = FullObjects[i];
		const bool bPredictMode = Obj->IsPredictionEnabled();

		FPSI_Data Data = MakeDefaultDataForPhysicsIteration(Obj);
		Data.SetDeltaTime(DeltaTime);
		
		FPhysTransform TPhys = CalculateNextTransformTimeBased(Data);
		UpdateTransformLock(Obj, TPhys, PredictionTransforms[i].Location, PredictionTransforms[i].Orientation);
		
		if(bPredictMode)
		{
			FPhysTransform TNext = PredictionTransforms[i];
			Obj->SetCurrentTransformPredictionCheck(TPhys, TNext);
		}
		else
		{
			Obj->SetCurrentTransform(TPhys, false);
		}

		Obj->PhysicsParams.Aerodynamics.Sideforce.Update(DeltaTime);
	}
}

void UCustomPhysicsProcessorBase::PredictTransform(const TArray<UCustomPhysicsBaseComponent*>& StaticBodies, FPSI_Data& Data, FPhysTransform& OutT) const
{
	const auto Obj = Data.Obj;
	const auto InT = Data.GetTransform();
	const bool bCheckCollisions = Data.HasCollisions();
	const FVector InitialLocation = InT.Location;
	const FQuat InitialOrientation = InT.Orientation;

	FPhysTransform TCollisionResolve = InT;

	if(bCheckCollisions)
	{
		TArray<FCollisionPair> CollisionPairs;
		if(UCollisionDetection::FindCollisionAgainstSpherePredictMode(InitialLocation, Obj, StaticBodies, CollisionPairs))
		{
			for (auto CollisionPair : CollisionPairs)
			{
				UCollisionDetection::ResolveCustomCollisionAgainstSpherePredictMode(InT, CollisionPair, TCollisionResolve);
			}
		}
	}

	Data.SetTransform(TCollisionResolve);
	OutT = CalculateNextTransformTimeBased(Data);

	UpdateTransformLock(Obj, OutT, InitialLocation, InitialOrientation);
}

void UCustomPhysicsProcessorBase::PredictTransformAnyTime(FPSI_Data& Data, FPhysTransform& OutT) const
{
	PredictTransform(SimpleObjects, Data, OutT);
}

FPhysTransform UCustomPhysicsProcessorBase::CalculateNextTransformTimeBased(FPSI_Data& Data)
{
	FPhysTransform OutT;
	UPhysicsSimulation::PhysicsSimulateDelta(Data, OutT);
	return OutT;
}

float UCustomPhysicsProcessorBase::SplitTimeToSubstepsAndFraction(float Value, int& NumSteps) const
{
	return UMathUtils::SplitTimeToSubstepsAndFraction(Value, GetSimDT(), NumSteps);
}

float UCustomPhysicsProcessorBase::GetSimulationRationalTimeFullStep(float Time) const
{
	return UMathUtils::GetSimulationRationalTimeFullStep(Time, GetSimDT());
}

void UCustomPhysicsProcessorBase::SubscribeNewObject(UCustomPhysicsBaseComponent* Obj)
{
	const auto ExtComponent = Cast<UCustomPhysicsComponent>(Obj);
	
	if(ExtComponent)
	{
		if(!Objects.Contains(ExtComponent)) Objects.Add(ExtComponent);
	}
	else
	{
		if(!SimpleObjects.Contains(Obj)) SimpleObjects.Add(Obj);
	}
}

