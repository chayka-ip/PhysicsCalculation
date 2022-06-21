#pragma once

#include "CoreMinimal.h"
#include "FirstTickCheck.generated.h"

USTRUCT()
struct FFirstTickCheck
{
    GENERATED_BODY()

    bool bFirstTickPassed = false;
    bool bAlreadyTriggered = false;

public:
    void MarkPassed(){bFirstTickPassed = true;}
    void MarkTriggered(){bAlreadyTriggered = true;}
public:
    bool IsNotPassed() const {return !bFirstTickPassed;}
    bool IsPassed() const {return bFirstTickPassed;}
    bool IsTriggered() const {return bAlreadyTriggered;}
};
