#pragma once

#include "CoreMinimal.h"
#include "BallLaunchParamsItem.h"
#include "BallLaunchVerticalDistribution.h"
#include "HMStructs/HashVector.h"
#include "BallLaunchCache.generated.h"

USTRUCT()
struct FBallLaunchCache_Data
{
    GENERATED_BODY()

public:

    UPROPERTY()
    FHashVector LaunchSpeedHV;
    UPROPERTY()
    FHashVector LaunchAngleHV;
    UPROPERTY()
    FHashVector FrontSpinAngleHV;
    UPROPERTY()
    FHashVector SideSpinAngleHV;
    UPROPERTY()
    float VerticalLevelWidth = 0.0f;
    
    /*
     * launch params item contains hashes instead of values itself
     * value is max distance on XY plane
     * this data can be generated from vertical distribution (if such will be implemented)
     */
    UPROPERTY()
    TMap<FBallLaunchParamsHashed, float> Map = {};

    UPROPERTY()
    TMap<FBallLaunchParamsHashed, FBallLaunchVerticalDistribution> VerticalDistribution = {};
    
public:
    void SetLaunchSpeedVector(const TArray<float>& LaunchSpeedData);
    void SetLaunchAngleVector(const TArray<float>& Data);
    void SetFrontSpinAngleVector(const TArray<float>& Data);
    void SetSideSpinAngleVector(const TArray<float>& Data);
    void Validate() const;
    
public:
    void SetVerticalLevelWidth(float V){VerticalLevelWidth = V;}
    void AddVerticalDistribution(const FBallLaunchParams& Input, const FBallLaunchVerticalDistribution& S);
    void AddDistanceValue(const FBallLaunchParams& Input, float Distance);
    TArray<float> SelectDistancesFromHashArray(const TArray<FBallLaunchParamsHashed>& Data);
    FBallLaunchParamsHashSelection SelectHashedLaunchParams(const FBallLaunchParams& Input);
    FBallLaunchParamsHashSelectionSimple SelectHashedLaunchParamsWithLaunchSpeedRequiredDistanceCheck(const FBallLaunchParams& Input, float MinDistance);
    FBallLaunchParamsHashed HashRealValuesToClosest(const FBallLaunchParams& Input) const;
    FBallLaunchParamsHashed HashRealValuesToClosest(const FBallLaunchParams& Input, bool bFixSideSpin) const;
    FBallLaunchParams GetLaunchParamsFromHash(const FBallLaunchParamsHashed& Hash) const;
    static TArray<FBallLaunchParamsHashed>  SelectHashedLaunchParams(const FIntMinMax& A, const FIntMinMax& B, const FIntMinMax& C, const FIntMinMax& D);
    static TArray<FBallLaunchParamsHashed>  SelectHashedLaunchParams(int A, const FIntMinMax& B, const FIntMinMax& C, const FIntMinMax& D);
    static TArray<FBallLaunchParamsHashed>  SelectHashedLaunchParams(int A, int B, const FIntMinMax& C, const FIntMinMax& D);
    static void SortHashDistanceArrays(FBallLaunchParamsHashSelectionSimple& InOutData);
    uint8 GetVerticalLevelIndex(float DistanceZ) const;
    bool IsLaunchParamsPureHashed(const FBallLaunchParams& Input) const;
};

UCLASS()
class PHYSICSCALCULATION_API UBallLaunchCache : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FBallLaunchCache_Data Data;

public:

    UFUNCTION(BlueprintPure)
    int GetNumItems() const {return Data.Map.Num();}

    UFUNCTION(BlueprintPure)
    TMap<FBallLaunchParamsHashed, float> GetMap(){return Data.Map;}
    
    UFUNCTION(BlueprintCallable)
    float GetDistanceFromInput(FBallLaunchParams Input);

    /*
     * Use MulDistanceXY to expand boundaries for distance range of level
     */
    UFUNCTION(BlueprintCallable)
    bool CanInputReachTarget(FBallLaunchParams Input, float DistanceXY, float DistanceZ, float AbsDerivationZ=0.0f, float MulDistanceXY=1.0f);

    UFUNCTION(BlueprintPure)
    bool IsLaunchParamsPureHashed(const FBallLaunchParams& Input) const;

    UFUNCTION(BlueprintPure)
    bool GetVDistributionForLaunchParams(const FBallLaunchParams& Input, TMap<int, FVerticalDistributionDistanceItem>& Out);
    
    bool ComputeLaunchParamsLaunchSpeedAdjust(FBallLaunchParams InitialParams, float DistanceToTarget, float PosTolerance, FBallLaunchParams& OutParams);
};
