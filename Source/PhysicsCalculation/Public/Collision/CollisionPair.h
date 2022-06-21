#pragma once

#include "CoreMinimal.h"
#include "Components/CustomPhysicsBaseComponent.h"
#include "Components/CustomPhysicsComponent.h"
#include "CollisionPair.generated.h"


USTRUCT(BlueprintType)
struct FCollisionPair
{
    GENERATED_BODY()

    UPROPERTY()
    UCustomPhysicsBaseComponent* ObjA;

    UPROPERTY()
    UCustomPhysicsComponent* ObjB;

    UPROPERTY()
    FVector CollisionPoint;

    UPROPERTY()
    FVector CollisionNormal;

    UPROPERTY()
    float Penetration;

    bool IsValid() const {return ObjA && ObjB && !GetPenetrationVector().IsNearlyZero();}

    UCustomPhysicsComponent* GetObjACastedToCustom() const {return Cast<UCustomPhysicsComponent>(ObjA);}
    FVector GetPenetrationVector() const {return CollisionNormal * Penetration;}
};