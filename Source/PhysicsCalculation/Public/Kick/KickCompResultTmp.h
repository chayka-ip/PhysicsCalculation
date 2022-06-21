#pragma once

#include "CoreMinimal.h"
#include "Common/PhysicsPredictionCurveWithMeta.h"
#include "KickCompResultTmp.generated.h"

USTRUCT()
struct FKickCompResultTmp
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FPhysTransform> ImpactTransformArray = {};

    UPROPERTY()
    TArray<FPhysicsPredictionCurveWithMeta> CurveMetaArray = {};

    bool bOriginalMeta = true;

public:
    void MarkMeta(){bOriginalMeta = false;}
    int GetNum() const {return ImpactTransformArray.Num();}
    bool IsNotEmpty() const {return GetNum() > 0;}
    void AddImpactTransform(const FPhysTransform& T, const FPhysicsPredictionCurveWithMeta& meta)
    {
        ImpactTransformArray.Add(T);
        CurveMetaArray.Add(meta);
    }
    bool GetRandomDataPair(FPhysTransform& TOut, FPhysicsPredictionCurveWithMeta& OutMeta)
    {
        const int N = GetNum();
        if(N < 0) return false;
        int Index = 0;
        if(N > 1) Index = FMath::RandRange(0, N - 1);

        TOut = ImpactTransformArray[Index];
        OutMeta = CurveMetaArray[Index];
        return true;
    }
};
