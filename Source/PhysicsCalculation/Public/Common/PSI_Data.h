#pragma once

#include "CoreMinimal.h"
#include "PhysTransform.h"
#include "PSI_Data.generated.h"

class UCustomPhysicsComponent;

USTRUCT(BlueprintType)
struct FPSI_Data
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadWrite)
    UCustomPhysicsComponent* Obj = nullptr;
    
    UPROPERTY(BlueprintReadWrite)
    FPhysTransform T;

    UPROPERTY(BlueprintReadWrite)
    float DeltaTime = 0.000001f;
    UPROPERTY(BlueprintReadWrite)
    float SkipTime = 0.0f;

    UPROPERTY(BlueprintReadWrite)
    bool bApplyExtraForces = false;
    UPROPERTY(BlueprintReadWrite)
    bool bCheckCollisions = false;

public:
    void SetTransform(const FPhysTransform& Transform){T= Transform;}
    FPhysTransform GetTransform() const {return T;}
    
    void EnableFullPhysics()
    {
        EnableCollisions();
        EnableExtraForces();
    }

    void SetupForRoughCheckNoCollisions()
    {
        DisableCollisions();
        DisableExtraForces();
    }

    void ToggleCollisions(bool B){bCheckCollisions = B;}
    void ToggleExtraForces(bool B){bApplyExtraForces = B;}
    
    void EnableCollisions() {ToggleCollisions(true);}
    void DisableCollisions() {ToggleCollisions(false);}
    
    void EnableExtraForces() {ToggleExtraForces(true);}
    void DisableExtraForces() {ToggleExtraForces(false);}

    bool HasExtraForces() const {return  bApplyExtraForces;}
    bool HasCollisions() const {return  bCheckCollisions;}

    void SetDeltaTime(float Time){DeltaTime = Time;}
    float GetDeltaTime() const {return DeltaTime;}
    
    void SetSkipTime(float Time) {SkipTime = Time;}
    float GetSkipTime() const {return SkipTime;}

};
