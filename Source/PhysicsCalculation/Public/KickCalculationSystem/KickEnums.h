#pragma once

#include "CoreMinimal.h"
#include "KickEnums.generated.h"

UENUM(BlueprintType)
enum EKickType
{
    EKT_NoSpin,
    EKT_Curved,
    EKT_Lifting,
    EKT_TopSpin,
};

UENUM(BlueprintType)
enum EKickTargetGoalSide
{
    EKTGS_C,
    EKTGS_L,
    EKTGS_R,
};

UENUM(BlueprintType)
enum EKickTypeByPower
{
    EKTP_Short,
    EKTP_Mid,
    EKTP_Long,
};

UENUM(BlueprintType)
enum EKickTargetSelectionType
{
    EKTST_Fixed,
    EKTST_ForwardVectorBased,
};