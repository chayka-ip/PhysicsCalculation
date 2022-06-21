// Fill out your copyright notice in the Description page of Project Settings.


#include "DataAssets/ParabolicCacheParams_DataAsset.h"

TArray<FVector> UParabolicCacheParams_DataAsset::GetLaunchLocations(float OffsetZ) const
{
    TArray<FVector> Out;
    const float Z = OffsetZ + LaunchPositionOffsetZ;
    const int NumSteps = TargetLocation.X / 250.0f;
        
    for (int i = 1; i < NumSteps; ++i)
    {
        const float X = LaunchPositionStepOffsetX * i;
        FVector P = FVector(X, 0.0f, Z);
        Out.Add(P);
    }
        
    return Out;
}

TArray<int> UParabolicCacheParams_DataAsset::GetLaunchAngles() const
{
    TArray<int> Out;
    const int NumSteps = (LaunchAngleMax - LaunchAngleMin) / LaunchAngleStep;
    for (int i = 0; i <= NumSteps; ++i)
    {
        int Angle = LaunchAngleMin + i * LaunchAngleStep; 
        Out.Add(Angle);
    }
    return Out;
}
