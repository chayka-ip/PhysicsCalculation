#pragma once
#include "CoreMinimal.h"
#include "Libs/MathUtils.h"
#include "SimpleMatrix3.generated.h"

USTRUCT()
struct FSimpleMatrix3
{
    GENERATED_USTRUCT_BODY()

    FSimpleMatrix3(const FVector InRow1 = FVector::ZeroVector, const FVector InRow2 = FVector::ZeroVector, const FVector InRow3 = FVector::ZeroVector)
    {
        Row1 = InRow1;
        Row2 = InRow2;
        Row3 = InRow3;
    }

    static FSimpleMatrix3 GetDiagonalMatrixFromVector(const FVector V)
    {
        FSimpleMatrix3 M;;

        M.Row1 = FVector(V.X, 0, 0);
        M.Row2 = FVector(0, V.Y, 0);
        M.Row3 = FVector(0, 0, V.Z);

        return M;
    }

    static FSimpleMatrix3 GetInvertedDiagonalMatrixFromVector(const FVector V)
    {
        return  GetDiagonalMatrixFromVector(V.Reciprocal());
    }
    
    static FSimpleMatrix3 GetDiagonalMatrixFromFloat(const float Val)
    {
        FSimpleMatrix3 M;

        M.Row1 = FVector(Val, 0, 0);
        M.Row2 = FVector(0, Val, 0);
        M.Row3 = FVector(0, 0, Val);

        return M;
    }

    static FSimpleMatrix3 GetInvertedDiagonalMatrixFromFloat(const float Val = 1)
    {
        if (Val == 0)
            return GetDiagonalMatrixFromFloat(0);
        return GetDiagonalMatrixFromFloat(1 / Val);
    }

    FVector MultiplyByVector(const FVector &V) const
    {
        if (IsDiagonal())
            return  FVector(Row1.X * V.X, Row2.Y * V.Y, Row3.Z * V.Z);

        const float X = Row1 | V;
        const float Y = Row2 | V;
        const float Z = Row3 | V;
        
        return FVector(X, Y, Z);
    }
    
    UPROPERTY()
    FVector Row1;

    UPROPERTY()
    FVector Row2;

    UPROPERTY()
    FVector Row3;

    bool IsDiagonal() const
    {
        if (UMathUtils::IsRestVectorComponentsZero(Row1, X))
            if (UMathUtils::IsRestVectorComponentsZero(Row2, Y))
                return UMathUtils::IsRestVectorComponentsZero(Row3, Z);
        return false;
    }
};
