// Fill out your copyright notice in the Description page of Project Settings.

#include "DataAssets/PhysicsCacheParams_DataAsset.h"

FVector UPhysicsCacheParams_DataAsset::GetLinearVelocityFromIterations(int SpeedIndex, int XZAngleIndex, int XYAngleIndex) const
{
    return StartLinearVelocity.GetVectorFromIterations(SpeedIndex, XZAngleIndex, XYAngleIndex, KmHourToCmSec);
}

FVector UPhysicsCacheParams_DataAsset::GetAngularVelocityFromIterations(int SpeedIndex, int XZAngleIndex, int XYAngleIndex) const
{
    return  StartLinearVelocity.GetVectorFromIterations(SpeedIndex, XZAngleIndex, XYAngleIndex, RpMToRadSec);
}
