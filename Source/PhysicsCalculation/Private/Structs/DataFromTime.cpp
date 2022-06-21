#include "Structs/DataFromTime.h"
#include "Libs/MathUtils.h"

FDataFromTime::FDataFromTime(float DeltaTime, float SkipTime, float SimStep, float InitialTimeBeforePredict)
{
    UMathUtils::CalculateTimeExtractionDataForPredictWithSkip(DeltaTime, SkipTime, InitialTimeBeforePredict, SimStep,
                                                    NumStepsToSkip, NumStepsToGet, InitialPartMultiplier, FuturePartMultiplier);
    
    InitialStartIndex = NumStepsToSkip - 1;
    IntegerStartIndex = NumStepsToSkip;
    FutureStartIndex = NumStepsToSkip + NumStepsToGet;
    
    HasInitialPart = InitialPartMultiplier > 0.0f;
    HasIntegerPart = NumStepsToGet > 0;
    HasFuturePart = FuturePartMultiplier > 0.0f;
}
