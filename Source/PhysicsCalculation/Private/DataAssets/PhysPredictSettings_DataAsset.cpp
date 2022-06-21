// Fill out your copyright notice in the Description page of Project Settings.

#include "DataAssets/PhysPredictSettings_DataAsset.h"
#include "Common/PhysPredict.h"

void UPhysPredictSettings_DataAsset::FillParamStruct(FPhysPredict& S) const
{
    S.Settings = Data;
}
