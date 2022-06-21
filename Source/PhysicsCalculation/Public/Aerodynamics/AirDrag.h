#pragma once

#include "CoreMinimal.h"
#include "ConstantsHM.h"
#include "Curves/CurveVector.h"
#include "AirDrag.generated.h"

USTRUCT(BlueprintType)
struct FAirDrag
{
    GENERATED_BODY()

public:
    /*
     * Values are mapped to velocity - cm/s
     * 
     * X - Drag coefficient
     * Y - Lift coefficient
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    UCurveVector* AirDragCurve;

    // kg/m3
    // UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float AirDensity = 1.2f;

public:
    
    FVector GetAirDragVector(float Velocity) const {return  AirDragCurve ? AirDragCurve->GetVectorValue(Velocity) : FVector::ZeroVector;}
    float GetAirDragCoefficient(float Velocity) const {return GetAirDragVector(Velocity).X;}
    float GetLiftCoefficient(float Velocity) const {return GetAirDragVector(Velocity).Y;}
    float GetAirDensityKgCm3() const {return AirDensity * M3ToCm3;}
    
};