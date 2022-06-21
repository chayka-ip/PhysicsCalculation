#include "PhysicsCache/PhysCacheDataStructs.h"

FVector FPhysCacheStartVelocity::GetVectorFromIterations(int SpeedIndex, int XZAngleIndex, int XYAngleIndex, float Scale) const
{
    const float Magnitude = Speed.GetFloatValueOnRangeSegment(SpeedIndex);
    const float Angle_XY = AngleXY.GetFloatValueOnRangeSegment(XYAngleIndex);
    const float Angle_XZ = AngleXZ.GetFloatValueOnRangeSegment(XZAngleIndex);
    const FVector Dir = UHMV::GetWorldForwardRotated_XY_XZ(Angle_XY, Angle_XZ);
    return Scale * Magnitude * Dir;
}

TArray<FVector> FPhysCacheStartVelocity::GetAllVectors(float Scale) const
{
    TArray<FVector> Out;
    const int SpeedIterations = Speed.GetNumSegments();
    const int AngleXZIterations = AngleXZ.GetNumSegments();
    const int AngleXYIterations = AngleXY.GetNumSegments();

    for (int i = 0; i < SpeedIterations; ++i)
    {
        for (int j = 0; j < AngleXZIterations ; ++j)
        {
            for (int k = 0; k < AngleXYIterations; ++k)
            {
                FVector V = GetVectorFromIterations(i, j, k, Scale);
                const bool bUnique = !Out.Contains(V);
                if(bUnique) Out.Add(V);
            }
        }
    }
    return Out;
}

TArray<FDetailedVector> FPhysCacheStartVelocity::GetAllVectorsDetailed(float Scale) const
{
    TArray<FDetailedVector> Out;
    const int SpeedIterations = Speed.GetNumSegments();
    const int AngleXZIterations = AngleXZ.GetNumSegments();
    const int AngleXYIterations = AngleXY.GetNumSegments();

    for (int i = 0; i < SpeedIterations; ++i)
    {
        for (int j = 0; j < AngleXZIterations ; ++j)
        {
            for (int k = 0; k < AngleXYIterations; ++k)
            {
                FVector V = GetVectorFromIterations(i, j, k, Scale);
                
                const bool bUnique = !HasSimilarDetailedVector(Out, V);
                if(bUnique)
                {
                    FDetailedVector S;
                    S.Initialize(V);
                    Out.Add(S);
                }
            }
        }
    }
    return Out;
}

bool FPhysCacheStartVelocity::HasSimilarDetailedVector(const TArray<FDetailedVector>& Array, const FVector& V)
{
    for (auto Item : Array) if(Item.V.Equals(V)) return true;
    return false;
}
