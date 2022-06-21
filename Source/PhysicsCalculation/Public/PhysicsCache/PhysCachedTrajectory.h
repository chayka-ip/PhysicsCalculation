#pragma once

#include "CoreMinimal.h"
#include "HMStructs/DetailedVector.h"
#include "HMStructs/FloatValueSearch.h"
#include "PhysCachedTrajectory.generated.h"

USTRUCT()
struct FPhysCachedPoint
{
    GENERATED_BODY()

    UPROPERTY()
    float Time = 0.0f;
    UPROPERTY()
    FVector Location = FVector::ZeroVector;
    UPROPERTY()
    FVector LinVel = FVector::ZeroVector;
    UPROPERTY()
    FVector AngVel = FVector::ZeroVector;
    
public:
    void Init(float time, const FVector& location, const FVector& linVel, const FVector& angVel)
    {
        Time = time;
        Location = location;
        LinVel = linVel;
        AngVel = angVel;
    }

};

USTRUCT()
struct FPhysCachedTrajectory
{
    GENERATED_BODY()
    
    UPROPERTY()
    TArray<FPhysCachedPoint> Points = {};
    UPROPERTY()
    FDetailedVector LinVelInitial;
    UPROPERTY()
    FDetailedVector AngVelInitial;

public:
    void AddPoint(const FPhysCachedPoint& P)
    {
        if(Points.Num() == 0)
        {
            LinVelInitial.Initialize(P.LinVel);
            AngVelInitial.Initialize(P.AngVel);
        }
        
        Points.Add(P);
    }
    void AddPoint(float time, const FVector& location, const FVector& linVel, const FVector& angVel)
    {
        FPhysCachedPoint P;
        P.Init(time, location, linVel, angVel);
        AddPoint(P);
    }
    
public:
    bool IsSimilarTo(const FPhysCachedTrajectory& Other) const
    {
        const bool B1 = LinVelInitial.IsSimilarTo(Other.LinVelInitial);
        const bool B2 = AngVelInitial.IsSimilarTo(Other.AngVelInitial);
        return B1 && B2;
    }

public:
    bool HasAcceptableInitialVelocityValue(const FValueSearchData& SearchData) const
    {
        const auto Ref = SearchData.Ref;
        const auto ValueType = SearchData.GetValueType();
        const auto SearchType = SearchData.GetSearchType();
        const auto Velocity = SearchData.GetVelocityType();

        const auto S = Velocity == ECPVT_Linear ? &LinVelInitial : &AngVelInitial;
        return S->HasAcceptableValue(Ref, ValueType, SearchType);
    }
    
public:
    static TArray<FVector> GetPointLocations(const TArray<FPhysCachedPoint>& P)
    {
        TArray<FVector> Out;
        for (auto p : P) Out.Add(p.Location);
        return Out;
    }
    TArray<FVector> GetPointLocations() const {return GetPointLocations(Points);}
};

USTRUCT()
struct FPhysCachedTrajectoryFull
{
    GENERATED_BODY()

    // trajectory when collision with ground is taken into account
    // FPhysCachedTrajectory WithCollisions;
    // trajectory when there are no collisions; Used to calculate data when ball has offset from ground level
    UPROPERTY()
    FPhysCachedTrajectory NoCollisions;

public:
    bool IsSimilarTo(const FPhysCachedTrajectoryFull& Other) const
    {
        return NoCollisions.IsSimilarTo(Other.NoCollisions);
    }
    
public:
    // todo: search can be switched with bool instead of polluting interface with duplicated methods
    TArray<FVector> GetNoCollisionTrajectoryPoints() const {return NoCollisions.GetPointLocations();}

};

