// Fill out your copyright notice in the Description page of Project Settings.

#include "ParabolicMotion/ParabolicMotionLib.h"
#include "HandyMathVectorLibrary.h"
#include "Kismet/KismetMathLibrary.h"
#include "PhysicsEngine/PhysicsSettings.h"

static const float _Gravity = FMath::Abs(UPhysicsSettings::Get()->DefaultGravityZ);

FVector UParabolicMotionLib::ComputeLocationOnTime(float InitialSpeed, float LaunchAngleXZ, float Time, float x_0, float z_0)
{
    const float Sin = UKismetMathLibrary::DegSin(LaunchAngleXZ);
    const float Cos = UKismetMathLibrary::DegCos(LaunchAngleXZ);
    const float x = x_0 + InitialSpeed * Time * Cos;
    const float z = z_0 + InitialSpeed * Time * Sin - 0.5f * _Gravity * Time * Time;
    return FVector(x, 0.0f, z);
}

TArray<FVector> UParabolicMotionLib::ComputeParabolicMotionTrajectory(FParabolicCompData P)
{
    check(P.DeltaTime > 0.0f)

    TArray<FVector> Out;
    
    for (int i = 0; i < P.NumSteps; ++i)
    {
        const float Time = i * P.DeltaTime;
        const FVector RawLocation = ComputeLocationOnTime(P.LaunchSpeed, P.LaunchAngleXZ, Time, P.x0, P.z0);
        FVector Location = UHMV::RotateVectorZAxis(RawLocation, P.LaunchAngleXY);
        Out.Add(Location);

        if(P.bTerminateOnGroundPassed && Location.Z < 0.0f) break;
    }
    
    return Out;
}

void UParabolicMotionLib::ComputeParabolicMotion(float LaunchSpeed, float LaunchAngleXZ, float x_0, float z_0,  FParabolicMotion& Result)
{
    const float FlightTime = ComputeFlightTime(LaunchSpeed, LaunchAngleXZ, z_0);
    const float MaxHeightTime = ComputeMaxHeightTime(LaunchSpeed, LaunchAngleXZ);

    const FVector LocationMaxX = ComputeLocationOnTime(LaunchSpeed, LaunchAngleXZ, FlightTime, x_0, z_0);
    const FVector LocationMaxZ = ComputeLocationOnTime(LaunchSpeed, LaunchAngleXZ, MaxHeightTime, x_0, z_0);
    
    Result = FParabolicMotion();
    Result.Init(LaunchSpeed, LaunchAngleXZ, x_0, z_0, LocationMaxX, LocationMaxZ, FlightTime, MaxHeightTime);
}

float UParabolicMotionLib::ComputeFlightTime(float LaunchSpeed, float LaunchAngleXZ, float z_0)
{
    const float VZ = LaunchSpeed * UKismetMathLibrary::DegSin(LaunchAngleXZ);
    const float M = 2.0f * z_0 * _Gravity;
    const float GravityInv = 1.0f / _Gravity;
    return GravityInv * (VZ + FMath::Sqrt(VZ * VZ + M));
}

float UParabolicMotionLib::ComputeMaxHeightTime(float LaunchSpeed, float LaunchAngleXZ)
{
    const float Sin = UKismetMathLibrary::DegSin(LaunchAngleXZ);
    return LaunchSpeed * Sin / _Gravity;
}

bool UParabolicMotionLib::GetRequiredLaunchSpeedForAngle(float LaunchAngleXZ, float x, float z, float& Speed)
{
    const float Cos = UKismetMathLibrary::DegCos(LaunchAngleXZ);
    const float Tan = UKismetMathLibrary::DegTan(LaunchAngleXZ);
    const float N = _Gravity * x * x;
    const float D =  2.0f * Cos * Cos * (x * Tan - z);

    const bool bSolved = !FMath::IsNearlyZero(D, 0.001f);
    
    if(bSolved)
    {
        Speed = FMath::Sqrt(N / D);
        return true;
    }
    return false;
}

float UParabolicMotionLib::GetRequiredLaunchSpeedForAngle(float LaunchAngleXZ, float x, float z)
{
    float Speed = 0.0f;
    bool bSolved = GetRequiredLaunchSpeedForAngle(LaunchAngleXZ, x, z, Speed);
    if(bSolved) return Speed;

    constexpr float AngleMul = 1.05f;
    const float AngleIncreased = LaunchAngleXZ * AngleMul;

    bSolved = GetRequiredLaunchSpeedForAngle(AngleIncreased, x, z, Speed);
    check(bSolved);

    return Speed;
}

bool UParabolicMotionLib::GetRequiredLaunchAnglesForSpeed(float LaunchSpeed, float x, float z, bool bAccept90Deg, float& MinAngle, float& MaxAngle)
{
    if(FMath::IsNearlyZero(LaunchSpeed)) return false;

    const float xx = x * x;
    const float a = (_Gravity * xx) / (2.0f * LaunchSpeed * LaunchSpeed);
    const float D = xx - 4.0f * a * (a + z);

    if(D < 0.0f) return false;

    const float DRoot = FMath::Sqrt(D);
    const float TwoA = 2.0f * a;
    const float s1 = (x + DRoot) / TwoA;
    const float s2 = (x - DRoot) / TwoA;

    float a1 = UKismetMathLibrary::DegAtan(s1);
    float a2 = UKismetMathLibrary::DegAtan(s2);

    if(!bAccept90Deg)
    {
        ValidateAngle90DegPrevention(a1, a2, a1, a2);
    }
    
    UHM::MapMinMax(a1, a2, MinAngle, MaxAngle);
    return true;
}

bool UParabolicMotionLib::ValidateAngle90DegPrevention(float A1, float A2, float& OutA1, float& OutA2)
{
    constexpr float A90 = 90.0f;
    const bool A1Nearly90 = FMath::IsNearlyEqual(FMath::Abs(A1), A90);
    const bool A2Nearly90 = FMath::IsNearlyEqual(FMath::Abs(A2), A90);

    if(A1Nearly90 && A2Nearly90) return false;
    if(A1Nearly90 || A2Nearly90)
    {
        OutA1 = A1Nearly90 ? A2 : A1;
        OutA2 = OutA1;
        return true;
    }

    OutA1 = A1;
    OutA2 = A2;
    return true;
}
