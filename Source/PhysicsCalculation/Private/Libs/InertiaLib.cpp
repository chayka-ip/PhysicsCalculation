// Fill out your copyright notice in the Description page of Project Settings.


#include "Libs/InertiaLib.h"

float UInertiaLib::GetCentralAxisInertiaCoefficient(const EShape Object)
{
    switch (Object)
    {
        case Sphere: return 0.6666666f;
        case Ball: return 0.4f;
    }
    return 1;
}

float UInertiaLib::ComputeInertia(const float Mass, const float R, const float K)
{
    return K * Mass * R * R;
}

float UInertiaLib::GetCentralAxisInertia(const float Mass, const float R, const EShape ObjectType)
{
    const float K = GetCentralAxisInertiaCoefficient(ObjectType);
    return ComputeInertia(Mass, R, K);
}

FSimpleMatrix3 UInertiaLib::GetCentralAxisInertiaTensor(const float Mass, const float R, const EShape Object,
                                                        const bool GetInverted)
{
    const float InertiaCoefficient = GetCentralAxisInertia(Mass, R, Object);
    if (GetInverted)
        return FSimpleMatrix3::GetInvertedDiagonalMatrixFromFloat(InertiaCoefficient);
    return FSimpleMatrix3::GetDiagonalMatrixFromFloat(InertiaCoefficient);
}

