// Fill out your copyright notice in the Description page of Project Settings.


#include "DataAssets/CustomPhysicsParamsDataAsset.h"
#include "Common/PhysRigidBodyParams.h"


void UCustomPhysicsParamsDataAsset::FillParamsStruct(FPhysRigidBodyParams& P) const
{
    P.Shape = Shape;
    P.Aerodynamics = Aerodynamics;
    P.Constrains = Constrains;
    P.Rendering = Rendering;

    if(DefaultOverride.bOverride)
    {
        P.Mass = DefaultOverride.MassInKg;
        P.LinearDamping = DefaultOverride.LinearDamping;
        P.AngularDamping = DefaultOverride.AngularDamping;
        P.bGravityEnabled = DefaultOverride.bEnableGravity;
    }
}
