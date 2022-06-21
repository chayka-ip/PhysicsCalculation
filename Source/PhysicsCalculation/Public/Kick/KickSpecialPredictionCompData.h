#pragma once

#include "CoreMinimal.h"
#include "KickSpecialPredictionCompData.generated.h"

USTRUCT()
struct FKickSpecialPredictionImpulseData
{
    GENERATED_BODY()

    FVector ApplyLocation = FVector::ZeroVector;
    
    UPROPERTY()
    TArray<FQuat> QuatArray = {};

    UPROPERTY()
    TArray<FVector> ImpulseArray = {};

public:
    int GetNum() const {return QuatArray.Num();}
    void Init(const FVector& apply_location, const TArray<FQuat>& quat_array, float impulse_size)
    {
        ApplyLocation = apply_location;
        QuatArray = quat_array;
        for (auto Q : QuatArray) ImpulseArray.Add(Q.GetForwardVector() * impulse_size);
    }
    FVector GetImpulseAtIndex(int Index) const {return ImpulseArray[Index];}
};
