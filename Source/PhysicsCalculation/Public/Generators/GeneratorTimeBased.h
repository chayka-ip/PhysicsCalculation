#pragma once

#include "CoreMinimal.h"
#include "Libs/UtilsLib.h"
#include "GeneratorTimeBased.generated.h"

USTRUCT(BlueprintType)
struct FGeneratorTimeBased
{
    GENERATED_BODY()
    
public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    bool bEnabled;
    
public:
    UPROPERTY(BlueprintReadOnly)
    float TimeBeforeUpdate = 0.0f;
    
public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float UpdateTimeInterval = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float SimulationLengthSec = 0.0f;

    
public:
    virtual ~FGeneratorTimeBased() = default;
    void SetTimeBeforeNextUpdate(float T){TimeBeforeUpdate = T;}
    float GetTimeBeforeUpdate() const {return TimeBeforeUpdate;}
    float GetTimeAfterLastUpdate() const {return UpdateTimeInterval - TimeBeforeUpdate;}
    float GetUpdateTimeInterval() const {return  UpdateTimeInterval;}
    
    bool DoUpdate() const {return bEnabled && HasValidTime();}
    bool HasValidTime() const {return UpdateTimeInterval > 0.0f && SimulationLengthSec > UpdateTimeInterval;}
    int GetSimulationStepsCount() const {return  HasValidTime() ?  SimulationLengthSec / UpdateTimeInterval : 0;}
    
    void Update(float DeltaTime)
    {
        UUtilsLib::UpdateTimeBasedGenerator(*this, DeltaTime);
    }
    
public:
    virtual void UpdateOneStep(){}
    virtual void Init(){}
};