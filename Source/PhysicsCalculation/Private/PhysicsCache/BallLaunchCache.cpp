#include "PhysicsCache/BallLaunchCache.h"

#include "debug.h"
#include "HandyMathArrayLibrary.h"
#include "Libs/SpinMovementLib.h"

void FBallLaunchCache_Data::SetLaunchSpeedVector(const TArray<float>& LaunchSpeedData)
{
    LaunchSpeedHV.MakeCurveFromValues(LaunchSpeedData);
}

void FBallLaunchCache_Data::SetLaunchAngleVector(const TArray<float>& Data)
{
    LaunchAngleHV.MakeCurveFromValues(Data);    
}

void FBallLaunchCache_Data::SetFrontSpinAngleVector(const TArray<float>& Data)
{
    FrontSpinAngleHV.MakeCurveFromValues(Data);
}

void FBallLaunchCache_Data::SetSideSpinAngleVector(const TArray<float>& Data)
{
    SideSpinAngleHV.MakeCurveFromValues(Data);
}

void FBallLaunchCache_Data::Validate() const
{
    check(LaunchSpeedHV.HasKeys())
    check(LaunchAngleHV.HasKeys())
    check(FrontSpinAngleHV.HasKeys())
    check(SideSpinAngleHV.HasKeys())
    check(VerticalLevelWidth > 0.0f)
}

void FBallLaunchCache_Data::AddVerticalDistribution(const FBallLaunchParams& Input, const FBallLaunchVerticalDistribution& S)
{
    const auto Hash = HashRealValuesToClosest(Input);
    check(!VerticalDistribution.Contains(Hash))
    VerticalDistribution.Add(Hash, S);
}

void FBallLaunchCache_Data::AddDistanceValue(const FBallLaunchParams& Input, float Distance)
{
    const auto Hash = HashRealValuesToClosest(Input);
    check(!Map.Contains(Hash))
    Map.Add(Hash, Distance);
}

TArray<float> FBallLaunchCache_Data::SelectDistancesFromHashArray(const TArray<FBallLaunchParamsHashed>& Data)
{
    TArray<float> OutData;

    for (auto Hash : Data)
    {
        const auto Item = Map.Find(Hash);
        check(Item)
        OutData.Add(*Item);
    }
    
    return OutData;
}

FBallLaunchParamsHashSelection FBallLaunchCache_Data::SelectHashedLaunchParams(const FBallLaunchParams& Input)
{
    FBallLaunchParamsHashSelection Out;
    
    Out.LaunchSpeedKeys = LaunchSpeedHV.GetHashValues(Input.LaunchSpeed);
    Out.LaunchAngleKeys = LaunchAngleHV.GetHashValues(Input.LaunchAngle);
    Out.FrontSpinKeys = FrontSpinAngleHV.GetHashValues(Input.FrontSpinAngle);
    Out.SideSpinKeys = SideSpinAngleHV.GetHashValues(Input.SideSpinAngle);
    Out.SimpleData.HashArray = SelectHashedLaunchParams(Out.LaunchSpeedKeys, Out.LaunchAngleKeys, Out.FrontSpinKeys, Out.SideSpinKeys);
    Out.SimpleData.Distances = SelectDistancesFromHashArray(Out.SimpleData.HashArray);
    SortHashDistanceArrays(Out.SimpleData);
    
    return Out;
}

FBallLaunchParamsHashSelectionSimple FBallLaunchCache_Data::SelectHashedLaunchParamsWithLaunchSpeedRequiredDistanceCheck(const FBallLaunchParams& Input, float MinDistance)
{
    TArray<int> LaunchSpeedHashes = LaunchSpeedHV.GetHashesAfterValue(Input.LaunchSpeed);
    const auto LaunchAngleKeys = LaunchAngleHV.GetHashValues(Input.LaunchAngle);
    const auto FrontSpinKeys = FrontSpinAngleHV.GetHashValues(Input.FrontSpinAngle);
    const auto SideSpinKeys = SideSpinAngleHV.GetHashValues(Input.SideSpinAngle);

    FBallLaunchParamsHashSelectionSimple OutData;

    for (const int LaunchSpeed : LaunchSpeedHashes)
    {
        auto HashArray = SelectHashedLaunchParams(LaunchSpeed, LaunchAngleKeys, FrontSpinKeys, SideSpinKeys);
        for (auto Hash : HashArray)
        {
            const auto DistancePtr = Map.Find(Hash);
            check(DistancePtr)
            const float Distance = *DistancePtr;
            if(Distance >= MinDistance)
            {
                OutData.AddData(Hash, Distance);
            }
        }
    }
    return OutData;
}

FBallLaunchParamsHashed FBallLaunchCache_Data::HashRealValuesToClosest(const FBallLaunchParams& Input) const
{
    return HashRealValuesToClosest(Input, true);
}

FBallLaunchParamsHashed FBallLaunchCache_Data::HashRealValuesToClosest(const FBallLaunchParams& Input, bool bFixSideSpin) const
{
    FBallLaunchParamsHashed Out;

    float SideSpinAngle = Input.SideSpinAngle;
    if(bFixSideSpin) SideSpinAngle = FMath::Abs(SideSpinAngle);
    Out.LaunchSpeedHash = LaunchSpeedHV.GetClosestHashValue(Input.LaunchSpeed);
    Out.LaunchAngleHash = LaunchAngleHV.GetClosestHashValue(Input.LaunchAngle);
    Out.FrontSpinAngleHash = FrontSpinAngleHV.GetClosestHashValue(Input.FrontSpinAngle);
    Out.SideSpinAngleHash = SideSpinAngleHV.GetClosestHashValue(SideSpinAngle);
    return Out;
}

FBallLaunchParams FBallLaunchCache_Data::GetLaunchParamsFromHash(const FBallLaunchParamsHashed& Hash) const
{
    // if this hash is not in the map -> we can make fake data; but make exception now
    
    check(Map.Contains(Hash))
    FBallLaunchParams OutData;

    OutData.LaunchSpeed = LaunchSpeedHV.GetSearchValueFromHash(Hash.LaunchSpeedHash);
    OutData.LaunchAngle = LaunchAngleHV.GetSearchValueFromHash(Hash.LaunchAngleHash);
    OutData.FrontSpinAngle = FrontSpinAngleHV.GetSearchValueFromHash(Hash.FrontSpinAngleHash);
    OutData.SideSpinAngle = SideSpinAngleHV.GetSearchValueFromHash(Hash.SideSpinAngleHash);
    
    return OutData;
}

TArray<FBallLaunchParamsHashed> FBallLaunchCache_Data::SelectHashedLaunchParams(const FIntMinMax& A, const FIntMinMax& B, const FIntMinMax& C,
                                                                                const FIntMinMax& D)
{
    auto MinA = SelectHashedLaunchParams(A.Min, B, C, D);
    const auto MaxA = SelectHashedLaunchParams(A.Max, B, C, D);
    MinA.Append(MaxA);
    return MinA;
}

TArray<FBallLaunchParamsHashed> FBallLaunchCache_Data::SelectHashedLaunchParams(int A, const FIntMinMax& B, const FIntMinMax& C, const FIntMinMax& D)
{
    auto Min = SelectHashedLaunchParams(A, B.Min, C, D);
    const auto Max = SelectHashedLaunchParams(A, B.Max, C, D);
    Min.Append(Max);
    return Min;
}

TArray<FBallLaunchParamsHashed> FBallLaunchCache_Data::SelectHashedLaunchParams(int A, int B, const FIntMinMax& C, const FIntMinMax& D)
{
    FBallLaunchParamsHashed P0 = FBallLaunchParamsHashed(A, B, C.Min, D.Min);
    FBallLaunchParamsHashed P1 = FBallLaunchParamsHashed(A, B, C.Min, D.Max);
    FBallLaunchParamsHashed P2 = FBallLaunchParamsHashed(A, B, C.Max, D.Min);
    FBallLaunchParamsHashed P3 = FBallLaunchParamsHashed(A, B, C.Max, D.Max);
    return {P0, P1, P2, P3};
}

void FBallLaunchCache_Data::SortHashDistanceArrays(FBallLaunchParamsHashSelectionSimple& InOutData)
{
    InOutData.CheckValid();
    TArray<FBallLaunchParamsHashed> OutHashes = {};

    TArray<int> SortIndices;
    const TArray<float> OutDistances = UHMA::SortFloatArrayWithIndicesTracking(InOutData.Distances, SortIndices);
    for (const auto SortIndex : SortIndices) OutHashes.Add(InOutData.HashArray[SortIndex]);

    InOutData.Distances = OutDistances;
    InOutData.HashArray = OutHashes;
}

uint8 FBallLaunchCache_Data::GetVerticalLevelIndex(float DistanceZ) const
{
    if(DistanceZ == 0.0f) DistanceZ += 1.0f;
    return USpinMovementLib::GetVerticalLevelIndexFromDistanceZ(DistanceZ, VerticalLevelWidth);
}

bool FBallLaunchCache_Data::IsLaunchParamsPureHashed(const FBallLaunchParams& Input) const
{
    const auto Hash = HashRealValuesToClosest(Input, false);
    const auto Restored = GetLaunchParamsFromHash(Hash);
    return Input.Equals(Restored);
}

float UBallLaunchCache::GetDistanceFromInput(FBallLaunchParams Input)
{
    const auto Hash = Data.HashRealValuesToClosest(Input);
    const auto Item = Data.Map.Find(Hash);
    check(Item)
    return *Item;
}

bool UBallLaunchCache::CanInputReachTarget(FBallLaunchParams Input, float DistanceXY, float DistanceZ, float AbsDerivationZ, float MulDistanceXY)
{
    const auto Hash = Data.HashRealValuesToClosest(Input);
    if(const auto Item = Data.VerticalDistribution.Find(Hash))
    {
        AbsDerivationZ = FMath::Abs(AbsDerivationZ);
        if(MulDistanceXY <= 0.0f) MulDistanceXY = 1.0f;
        const auto PreciseLevelIndex = Data.GetVerticalLevelIndex(DistanceZ);
        const auto MinLevelIndex = Data.GetVerticalLevelIndex(DistanceZ - AbsDerivationZ);
        const auto MaxLevelIndex = Data.GetVerticalLevelIndex(DistanceZ + AbsDerivationZ);
        
        if(Item->CanReachTarget(PreciseLevelIndex, DistanceXY, MulDistanceXY)) return true;
        if(Item->CanReachTarget(MinLevelIndex, DistanceXY, MulDistanceXY)) return true;
        if(Item->CanReachTarget(MaxLevelIndex, DistanceXY, MulDistanceXY)) return true;
    }
    return false;
}

bool UBallLaunchCache::IsLaunchParamsPureHashed(const FBallLaunchParams& Input) const
{
    return Data.IsLaunchParamsPureHashed(Input);
}

bool UBallLaunchCache::GetVDistributionForLaunchParams(const FBallLaunchParams& Input, TMap<int, FVerticalDistributionDistanceItem>& Out)
{
    const auto Hash = Data.HashRealValuesToClosest(Input);
    if(const auto Ptr = Data.VerticalDistribution.Find(Hash))
    {
        Out = Ptr->GetMapIntKey();
        return true;
    }
    return false;
}

bool UBallLaunchCache::ComputeLaunchParamsLaunchSpeedAdjust(FBallLaunchParams InitialParams, float DistanceToTarget, float PosTolerance, FBallLaunchParams& OutParams)
{
    const float SafeDistance = DistanceToTarget + PosTolerance;

    const float SideSpinSigned = InitialParams.SideSpinAngle;
    InitialParams.MakeAbsSideSpin();

    const auto Selection = Data.SelectHashedLaunchParamsWithLaunchSpeedRequiredDistanceCheck(InitialParams, SafeDistance);
    const bool bFoundRequired = Selection.IsValid();
    if(bFoundRequired)
    {
        int Index;
        float ClosestDistance = UHMA::FloatArraySelectClosestValue(SafeDistance, Selection.Distances, Index);
        const auto Hash = Selection.HashArray[Index];
        OutParams = Data.GetLaunchParamsFromHash(Hash);
        OutParams.SetSideSpinSign(SideSpinSigned);
        return true;
    }

    PrintToLog("need to change something");
    
    return false;
}
