#pragma once

#include "CoreMinimal.h"
#include "HandyMathLibrary.h"
#include "HMStructs/IntMinMax.h"
#include "BallLaunchParamsItem.generated.h"

USTRUCT(BlueprintType)
struct FBallLaunchParamsHashed
{
    GENERATED_BODY()

    FBallLaunchParamsHashed(int launch_speed_hash=0, int launch_angle_hash=0, int front_spin_angle_hash=0, int side_spin_angle_hash=0)
    {
        LaunchSpeedHash = launch_speed_hash;
        LaunchAngleHash = launch_angle_hash;
        FrontSpinAngleHash = front_spin_angle_hash;
        SideSpinAngleHash = side_spin_angle_hash;
    }
    FBallLaunchParamsHashed(const FBallLaunchParamsHashed& Other)
    {
        LaunchSpeedHash = Other.LaunchSpeedHash;
        LaunchAngleHash = Other.LaunchAngleHash;
        FrontSpinAngleHash = Other.FrontSpinAngleHash;
        SideSpinAngleHash = Other.SideSpinAngleHash;
    }

    bool operator==(const FBallLaunchParamsHashed& Other) const
    {
        return Equals(Other);
    }
    bool Equals(const FBallLaunchParamsHashed& Other) const
    {
        const bool B1 = LaunchSpeedHash == Other.LaunchSpeedHash;
        const bool B2 = LaunchAngleHash == Other.LaunchAngleHash;
        const bool B3 = FrontSpinAngleHash == Other.FrontSpinAngleHash;
        const bool B4 = SideSpinAngleHash == Other.SideSpinAngleHash;
        
        return B1 && B2 && B3 && B4;
    }

    FString ToString() const
    {
        const auto Speed = FString::FromInt(LaunchSpeedHash);
        const auto Angle = FString::FromInt(LaunchAngleHash);
        const auto FrontSpin = FString::FromInt(FrontSpinAngleHash);
        const auto SideSpin = FString::FromInt(SideSpinAngleHash);
        return Speed + "| " + Angle +  " | " + FrontSpin + " | " + SideSpin;
    }
    
public:

    UPROPERTY(BlueprintReadWrite)
    int LaunchSpeedHash = 0;
    UPROPERTY(BlueprintReadWrite)
    int LaunchAngleHash = 0;
    UPROPERTY(BlueprintReadWrite)
    int FrontSpinAngleHash = 0;
    UPROPERTY(BlueprintReadWrite)
    int SideSpinAngleHash = 0;
    
};

USTRUCT()
struct FBallLaunchParamsHashSelectionSimple
{
    GENERATED_BODY()
    // Sorted arrays
    
    UPROPERTY()
    TArray<FBallLaunchParamsHashed> HashArray = {};
    UPROPERTY()
    TArray<float> Distances = {};

public:
    bool IsValid() const
    {
        const int n = HashArray.Num();
        const bool B1 = n == Distances.Num();
        const bool B2 = n > 0;
        return B1 && B2;
    }
    void CheckValid() const {check(IsValid());}
    float GetMinDistance(){return Distances[0];}
    float GetMaxDistance(){return Distances.Last();}

    bool IsValueLTE_Min(float V) {return V <= GetMinDistance();}
    bool IsValueGTE_Max(float V) {return V >= GetMaxDistance();}
    bool IsValueInDistanceRange(float V)
    {
        const float Min = GetMinDistance();
        const float Max = GetMaxDistance();
        return V >= Min && V <= Max;
    }
    
public:
    void AddData(const FBallLaunchParamsHashed& Hash, float Distance)
    {
        HashArray.Add(Hash);
        Distances.Add(Distance);
    }
};

USTRUCT()
struct FBallLaunchParamsHashSelection
{
    GENERATED_BODY()

    FBallLaunchParamsHashSelectionSimple SimpleData;
    FIntMinMax LaunchSpeedKeys;
    FIntMinMax LaunchAngleKeys;
    FIntMinMax FrontSpinKeys;
    FIntMinMax SideSpinKeys;

public:
    float GetMinDistance(){return SimpleData.GetMinDistance();}
    float GetMaxDistance(){return SimpleData.GetMaxDistance();}
    bool IsValueLTE_Min(float V) {return V <= GetMinDistance();}
    bool IsValueGTE_Max(float V) {return V >= GetMaxDistance();}
    bool IsValueInDistanceRange(float V)
    {
        const float Min = GetMinDistance();
        const float Max = GetMaxDistance();
        return V >= Min && V <= Max;
    }
};

USTRUCT(BlueprintType)
struct FBallLaunchParams
{
    GENERATED_BODY()

    FBallLaunchParams(){}
    FBallLaunchParams(float launch_speed, float launch_angle, float front_spin_angle, float side_spin_angle)
    {
        Init(launch_speed, launch_angle, front_spin_angle, side_spin_angle);
    }
    
public:
    UPROPERTY(BlueprintReadWrite)
    float LaunchSpeed = 0.0f;
    UPROPERTY(BlueprintReadWrite)
    float LaunchAngle = 0.0f;
    UPROPERTY(BlueprintReadWrite)
    float FrontSpinAngle = 0.0f;
    UPROPERTY(BlueprintReadWrite)
    float SideSpinAngle = 0.0f;

public:
    void Init(float launch_speed, float launch_angle, float front_spin_angle, float side_spin_angle)
    {
        LaunchSpeed = launch_speed;
        LaunchAngle = launch_angle;
        FrontSpinAngle = front_spin_angle;
        SideSpinAngle = side_spin_angle;
    }

    void MakeAbsSideSpin() {SideSpinAngle = FMath::Abs(SideSpinAngle);}
    void SetSideSpinSign(float V) {SideSpinAngle *= UHM::Sign(V);}

    bool Equals(const FBallLaunchParams& Other) const
    {
        constexpr float Tol = 0.01f;
        const bool bLaunchSpeed = FMath::IsNearlyEqual(LaunchSpeed, Other.LaunchSpeed, Tol);
        const bool bLaunchAngle = FMath::IsNearlyEqual(LaunchAngle, Other.LaunchAngle, Tol);
        const bool bFrontSpinAngle = FMath::IsNearlyEqual(FrontSpinAngle, Other.FrontSpinAngle, Tol);
        const bool bSideSpinAngle = FMath::IsNearlyEqual(SideSpinAngle, Other.SideSpinAngle, Tol);
        return bLaunchSpeed && bLaunchAngle && bFrontSpinAngle && bSideSpinAngle;
    }

    bool operator==(const FBallLaunchParams& Other) const
    {
        return Equals(Other);
    }
};


#if UE_BUILD_DEBUG
uint32 GetTypeHash(const FHashMeIfYouCan& Thing);
#else // optimize by inlining in shipping and development builds
FORCEINLINE uint32 GetTypeHash(const FBallLaunchParamsHashed& Thing)
{
    const uint32 Hash = FCrc::MemCrc32(&Thing, sizeof(FBallLaunchParamsHashed));
    return Hash;
}
#endif