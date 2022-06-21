#pragma once

#include "CoreMinimal.h"
#include "HMStructs/FloatMinMax.h"
#include "KickSpinRange.generated.h"

USTRUCT(BlueprintType)
struct FKickSpinRange
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FFloatMinMax FrontSpin;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FFloatMinMax SideSpin;

public:
    static int GetValueInt(const FFloatMinMax& S, bool bMin)
    {
        return bMin ? FMath::CeilToInt(S.GetMin()) : FMath::FloorToInt(S.GetMax());
    }
    int GetMinFrontSpinAngleInt() const {return GetValueInt(FrontSpin, true);}
    int GetMaxFrontSpinAngleInt() const {return GetValueInt(FrontSpin, false);}
    int GetMinSideSpinAngleInt() const {return GetValueInt(SideSpin, true);}
    int GetMaxSideSpinAngleInt() const {return GetValueInt(SideSpin, false);}
    
    float GetMinFrontSpinAngle() const {return FrontSpin.GetMin();}
    float GetMaxFrontSpinAngle() const {return FrontSpin.GetMax();}
    float GetMinSideSpinAngle() const {return SideSpin.GetMin();}
    float GetMaxSideSpinAngle() const {return SideSpin.GetMax();}
};
