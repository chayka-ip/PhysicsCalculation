#pragma once

#include "CoreMinimal.h"
#include "HMStructs/FloatMinMax.h"
#include "BallLaunchVerticalDistribution.generated.h"

USTRUCT(BlueprintType)
struct FVerticalDistributionDistanceItem
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    TArray<FFloatMinMax> Data = {};

public:
    void AddData(float MinDistanceXY, float MaxDistanceXY)
    {
        Data.Add(FFloatMinMax(MinDistanceXY, MaxDistanceXY));
    }
    
public:
    bool IsValueInRange(float DistanceXY, float MulCorrection)
    {
        for (const auto Item : Data)
        {
            float Min = Item.Min / MulCorrection;
            float Max = Item.Max * MulCorrection;
            const bool bOK = FMath::IsWithinInclusive(DistanceXY, Min, Max);
            if(bOK) return true;
        }
        return false;
    }
};

USTRUCT(BlueprintType)
struct FBallLaunchVerticalDistribution
{
    GENERATED_BODY()

    // this is mapped to launch hash
    
    /*
     * key is vertical level - represents packed distance Z of target;
     * value is min max distance XY (in cm)
     */
    UPROPERTY()
    TMap<uint8, FVerticalDistributionDistanceItem> Map = {};

public:
    void AddItem(uint8 LevelIndex, float MinDistanceXY, float MaxDistanceXY)
    {
        const auto Item = Map.Find(LevelIndex);
        if(Item)
        {
            Item->AddData(MinDistanceXY, MaxDistanceXY);
        }
        else
        {
            FVerticalDistributionDistanceItem NewItem;
            NewItem.AddData(MinDistanceXY, MaxDistanceXY);
            Map.Add(LevelIndex, NewItem);
        }
    }

    TMap<int, FVerticalDistributionDistanceItem> GetMapIntKey()
    {
        TMap<int, FVerticalDistributionDistanceItem> Out;

        for (const auto Item : Map)
        {
            int Key = Item.Key;
            auto Value = Item.Value;
            Out.Add(Key, Value);
        }
        
        return Out;
    }
    
public:
    bool CanReachTarget(uint8 LevelIndex, float DistanceXY, float MulDistanceXY)
    {
        const auto Level = Map.Find(LevelIndex);
        if(Level)
        {
            return  Level->IsValueInRange(DistanceXY,MulDistanceXY);
        }
        return false;
    }
};