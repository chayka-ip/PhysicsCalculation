// Fill out your copyright notice in the Description page of Project Settings.


#include "Libs/UtilsLib.h"
#include "Libs/MathUtils.h"
#include "debug.h"
#include "CommonUtils/Public/Utils.h"
#include "Components/CustomPhysicsBaseComponent.h"
#include "Components/CustomPhysicsProcessor.h"
#include "Generators/GeneratorTimeBased.h"

UCustomPhysicsProcessor* UUtilsLib::GetPhysicsProcessor()
{
    UCustomPhysicsProcessor* PhysicsProcessor = nullptr;
    TArray<UObject*> Result = {};
    GetObjectsOfClass(UCustomPhysicsProcessor::StaticClass(), Result, false);

    for (auto Object : Result)
    {
        if(CommonUtils::IsObjectInPlayableWorld(Object))
        {
            if(!PhysicsProcessor)
            {
                PhysicsProcessor = Cast<UCustomPhysicsProcessor>(Object);
            }
            else
            {
                PhysicsProcessor = nullptr;
                break;
            }
        }
    }
    return PhysicsProcessor;
}

void UUtilsLib::SubscribeToPhysicsProcessor(UCustomPhysicsBaseComponent* Obj)
{
    const bool IsPlayableWorld = CommonUtils::IsObjectInPlayableWorld(Obj);
    if(!IsPlayableWorld) return;

    UCustomPhysicsProcessor* PhysicsProcessor = GetPhysicsProcessor();
	
    if(PhysicsProcessor)
    {
        PhysicsProcessor->SubscribeNewObject(Obj);
    }
    else
    {
        PrintToLog("There should be an instance of CustomPhysicsProcessor to work with custom physics");
    }
}

void UUtilsLib::UpdateTimeBasedGenerator(FGeneratorTimeBased& G, float DeltaTime)
{
    if(G.DoUpdate())
    {
        const float CurrentTimeBNP = G.GetTimeBeforeUpdate();
        const float SimStep = G.GetUpdateTimeInterval();
        int NumUpdateSteps, _;
        float NextTimeBeforeUpdate, __;

        UMathUtils::ExtractDataFromTimeForPrediction(DeltaTime, CurrentTimeBNP, SimStep, NumUpdateSteps, _, __, NextTimeBeforeUpdate);

        for (int i = 0; i < NumUpdateSteps; ++i)
        {
            G.UpdateOneStep();
        }
        
        G.SetTimeBeforeNextUpdate(NextTimeBeforeUpdate);
    }
}

FDetailedVector UUtilsLib::MakeDetailedVector(FVector V)
{
    FDetailedVector S;
    S.Initialize(V);
    return S;
}

FDetailedVector UUtilsLib::MakeDetailedVectorFromFloat(float Magnitude, float Angle_XY, float Angle_XZ)
{
    FDetailedVector S;
    S.Initialize(Magnitude, Angle_XY, Angle_XZ);
    return S;
}

TArray<FVector> UUtilsLib::LocationsFromPhysTransformArray(const TArray<FPhysTransform>& Array)
{
    TArray<FVector> Out;
    for (auto T : Array) Out.Add(T.Location);
    return Out;
}
