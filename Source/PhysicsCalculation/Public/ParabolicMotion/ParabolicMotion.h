#pragma once

#include "CoreMinimal.h"
#include "ParabolicMotion.generated.h"

USTRUCT(BlueprintType)
struct FParabolicCompData
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadWrite)
    float LaunchSpeed = 0.0f;
    UPROPERTY(BlueprintReadWrite)
    float LaunchAngleXY = 0.0f;
    UPROPERTY(BlueprintReadWrite)
    float LaunchAngleXZ = 0.0f;
    
    // initial offset
    UPROPERTY(BlueprintReadWrite)
    float x0 = 0.0f;
    // initial offset
    UPROPERTY(BlueprintReadWrite)
    float z0 = 0.0f;

    //

    UPROPERTY(BlueprintReadWrite)
    float DeltaTime = 0.1f;
    UPROPERTY(BlueprintReadWrite)
    int NumSteps = 0;

    UPROPERTY(BlueprintReadWrite)
    bool bTerminateOnGroundPassed=true;

public:
    void SetNumStepsFromTime(float Time){NumSteps = Time / DeltaTime;}
    void SetOffsetZ(float V){z0 = V;}
};

USTRUCT(BlueprintType)
struct FParabolicMotion
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    float LaunchSpeed = 0.0f;
    UPROPERTY(BlueprintReadOnly)
    float LaunchAngleXZ = 0.0f;
    
    // initial offset
    UPROPERTY(BlueprintReadOnly)
    float x0 = 0.0f;
    // initial offset
    UPROPERTY(BlueprintReadOnly)
    float z0 = 0.0f;

public:
    // computable

    UPROPERTY(BlueprintReadOnly)
    FVector LocationMaxX = FVector::ZeroVector;
    UPROPERTY(BlueprintReadOnly)
    FVector LocationMaxZ = FVector::ZeroVector;

    // time when body reaches the ground
    UPROPERTY(BlueprintReadOnly)
    float TimeMaxX = 0.0f;
    UPROPERTY(BlueprintReadOnly)
    float TimeMaxZ = 0.0f;

public:
    void Init(float launch_speed, float launch_angle_xz, float x_0, float z_0, FVector max_x, FVector max_z, float time_max_x, float time_max_z)
    {
        LaunchSpeed = launch_speed;
        LaunchAngleXZ = launch_angle_xz;
        x0 = x_0;
        z0 = z_0;
        
        LocationMaxX = max_x;
        LocationMaxZ = max_z;
        TimeMaxX = time_max_x;
        TimeMaxZ = time_max_z;
    }
};
