// Fill out your copyright notice in the Description page of Project Settings.


#include "Components/CustomPhysicsBaseComponent.h"

#include "Libs/PhysicsSimulation.h"
#include "Libs/PhysicsUtils.h"
#include "Libs/UtilsLib.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

UCustomPhysicsBaseComponent::UCustomPhysicsBaseComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UCustomPhysicsBaseComponent::BeginPlay()
{
	Super::BeginPlay();
	UUtilsLib::SubscribeToPhysicsProcessor(this);

	Owner = GetOwner();
	if(!PrimitiveComponent) SetDefaultPrimitiveComponent();
	check(PrimitiveComponent)
	
	Initialize();
}

void UCustomPhysicsBaseComponent::SetDefaultPrimitiveComponent()
{
	const auto P = Cast<UPrimitiveComponent>(Owner->GetComponentByClass(UStaticMeshComponent::StaticClass()));
	SetPrimitiveComponent(P);
}

bool UCustomPhysicsBaseComponent::IsDefaultPhysicsEnabled() const
{
	return PrimitiveComponent && PrimitiveComponent->IsSimulatingPhysics();
}

float UCustomPhysicsBaseComponent::GetMass() const
{
	if(IsDefaultPhysicsEnabled())
	{
		return PrimitiveComponent->GetMass();
	}
	return 0.0f;
}

float UCustomPhysicsBaseComponent::GetMassInv() const
{
	const float Mass = GetMass();
	return FMath::IsNearlyZero(Mass) ? 0.0f : 1.0f/Mass;
}

float UCustomPhysicsBaseComponent::GetRestitution() const
{
	if(const auto M = GetPhysicalMaterial()) return  M->Restitution;
	return 0;
}

float UCustomPhysicsBaseComponent::GetFriction() const
{
	if(const auto M = GetPhysicalMaterial()) return  M->Friction;
	return 0;
}

EPhysicsCombineMode UCustomPhysicsBaseComponent::GetFrictionCombineMode() const
{
	if(const auto M = GetPhysicalMaterial())
	{
		if(M->bOverrideFrictionCombineMode)
		{
			return UPhysicsUtils::GetCombineModeFromDefaultPhysics(M->FrictionCombineMode);
		}
	}
	return Average;
}

EPhysicsCombineMode UCustomPhysicsBaseComponent::GetRestitutionCombineMode() const
{
	if(const auto M = GetPhysicalMaterial())
	{
		if(M->bOverrideRestitutionCombineMode)
		{
			return UPhysicsUtils::GetCombineModeFromDefaultPhysics(M->RestitutionCombineMode);
		}
	}
	return Average;
}

bool UCustomPhysicsBaseComponent::IsLockLocationX() const
{
	// const auto BodyInstance = Obj->GetPrimitiveComponent()->GetBodyInstance(NAME_None, false);
	return PrimitiveComponent->BodyInstance.bLockXTranslation > 0;
}

bool UCustomPhysicsBaseComponent::IsLockLocationY() const
{
	return PrimitiveComponent->BodyInstance.bLockYTranslation > 0;
}

bool UCustomPhysicsBaseComponent::IsLockLocationZ() const
{
	return  PrimitiveComponent->BodyInstance.bLockZTranslation > 0;
}

UPhysicalMaterial* UCustomPhysicsBaseComponent::GetPhysicalMaterial() const
{
	return  PrimitiveComponent->BodyInstance.GetSimplePhysicalMaterial();
}

void UCustomPhysicsBaseComponent::SetLinearVelocity(FVector LinearVelocity, bool bRecomputePredict)
{
	if(PrimitiveComponent)
	{
		PrimitiveComponent->SetPhysicsLinearVelocity(LinearVelocity);
	}
}

void UCustomPhysicsBaseComponent::SetWorldOrientation(FQuat Q, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled()) PrimitiveComponent->SetWorldRotation(Q,true);
}

FVector UCustomPhysicsBaseComponent::GetCurrentLocation() const
{
	return PrimitiveComponent ? PrimitiveComponent->GetComponentLocation() : FVector::ZeroVector;
}

FVector UCustomPhysicsBaseComponent::GetCurrentLinearVelocity() const
{
	if(IsDefaultPhysicsEnabled())
	{
		return PrimitiveComponent->GetPhysicsLinearVelocity();
	}
	return  FVector::ZeroVector;
}

FVector UCustomPhysicsBaseComponent::GetCurrentAngularVelocityRadians() const
{
	if(IsDefaultPhysicsEnabled())
	{
		return PrimitiveComponent->GetPhysicsAngularVelocityInRadians();
	}
	return  FVector::ZeroVector;
}

FVector UCustomPhysicsBaseComponent::GetCurrentAngularVelocityAtPointInRadians(FVector P) const
{
	const FVector Angular = GetCurrentAngularVelocityRadians();
	const FVector Arm = P - GetCurrentLocation();
	return  Angular ^ Arm;
}

FVector UCustomPhysicsBaseComponent::GetFullVelocityAtPoint(FVector P) const
{
	return GetCurrentLinearVelocity() + GetCurrentAngularVelocityAtPointInRadians(P);
}

float UCustomPhysicsBaseComponent::AngleLinearToAngularVelocity() const
{
	return UHMV::AngleBetweenVectorsDeg(GetCurrentLinearVelocity(), GetCurrentAngularVelocityRadians());
}

void UCustomPhysicsBaseComponent::GetArrowStartEndFromVector(FVector Dir, float Size, FVector& Start, FVector& End) const
{
	Start = GetCurrentLocation();
	End = Start + Size * Dir.GetSafeNormal();
}

void UCustomPhysicsBaseComponent::GetCurrentLinearVelocityAxisArrowStartEnd(float Size, FVector& Start, FVector& End) const
{
	GetArrowStartEndFromVector(GetCurrentLinearVelocity(), Size, Start, End);
}

void UCustomPhysicsBaseComponent::GetCurrentAngularVelocityAxisArrowStartEnd(float Size, FVector &Start, FVector& End) const
{
	GetArrowStartEndFromVector(GetCurrentAngularVelocityRadians(), Size, Start, End);
}

void UCustomPhysicsBaseComponent::AddWorldLocation(FVector V, bool bRecomputePredict)
{
	if(PrimitiveComponent) PrimitiveComponent->AddWorldOffset(V, true);
}

void UCustomPhysicsBaseComponent::AddImpulse(FVector V, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled()) PrimitiveComponent->AddImpulse(V);
}

void UCustomPhysicsBaseComponent::AddImpulseAtLocation(FVector Impulse, FVector ApplyLocation, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled()) PrimitiveComponent->AddImpulseAtLocation(Impulse, ApplyLocation);
}

void UCustomPhysicsBaseComponent::AddImpulseToArea(FVector Impulse, const TArray<FVector>& AreaPoints, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled())
	{
		const int Num = AreaPoints.Num();
		if(Num == 0) return;

		const FVector Imp = Impulse / Num;
		for (auto P : AreaPoints)
		{
			PrimitiveComponent->AddImpulseAtLocation(Imp, P);
		}
	}
}

void UCustomPhysicsBaseComponent::AddLinearImpulse(const FVector Force, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled())
	{
		const FVector Imp = Force * GetMassInv();
		PrimitiveComponent->AddImpulse(Imp);
	}
}

void UCustomPhysicsBaseComponent::AddAngularImpulse(const FVector Force, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled())
	{
		const FVector Add = GetInertiaTensorInverted().MultiplyByVector(Force);
		const FVector AV = PrimitiveComponent->GetPhysicsAngularVelocityInRadians();
		PrimitiveComponent->SetPhysicsAngularVelocityInRadians(AV + Add);
	}
}

void UCustomPhysicsBaseComponent::AddLinearImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled())
	{
		const float MassInv = GetMassInv();
		const FVector Imp = Force * MassInv;
		const FVector NewLinearVelocity = PrimitiveComponent->GetPhysicsLinearVelocity() + Imp;
		const FVector DeltaLocation = NewLinearVelocity * MassInv;
		const FVector NewLocation = PrimitiveComponent->GetCenterOfMass() + DeltaLocation;

		PrimitiveComponent->SetPhysicsLinearVelocity(NewLinearVelocity);
		PrimitiveComponent->SetWorldLocation(NewLocation);
	}
}

void UCustomPhysicsBaseComponent::AddAngularImpulseAsApplied(const FVector Force, float Time, bool bRecomputePredict)
{
	if(IsDefaultPhysicsEnabled())
	{
		const FVector Add = GetInertiaTensorInverted().MultiplyByVector(Force);
		const FVector AV = PrimitiveComponent->GetPhysicsAngularVelocityInRadians();
		const FVector NewAV = AV + Add;

		FQuat Q = PrimitiveComponent->GetComponentRotation().Quaternion();
		UMathUtils::ApplyAngularVelocityToRotation(NewAV, Time, Q);

		PrimitiveComponent->SetPhysicsAngularVelocityInRadians(NewAV);
		PrimitiveComponent->SetWorldRotation(Q);
	}
}

FSimpleMatrix3 UCustomPhysicsBaseComponent::GetInertiaTensorInverted()
{
	if(IsDefaultPhysicsEnabled())
	{
		const FVector I = PrimitiveComponent->GetInertiaTensor();
		return FSimpleMatrix3::GetInvertedDiagonalMatrixFromVector(I);
	}
	return FSimpleMatrix3();
}
