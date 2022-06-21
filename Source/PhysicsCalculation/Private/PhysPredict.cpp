#include "Common/PhysPredict.h"
#include "CommonUtils/Public/Utils.h"
#include "Components/CustomPhysicsComponent.h"
#include "Components/CustomPhysicsProcessor.h"


int FPhysPredict::GetPreciseStepsCount() const
{
	return  Settings.PrecisePredictTimeSec / GetSimulationDeltaTime();
}

float FPhysPredict::GetRoughSimulationTimeStep() const
{
	return Comp->PhysicsProcessor->GetSimulationFrameTime() * Settings.RoughPredictionTimeScale;
}

bool FPhysPredict::ShouldRecomputeRoughPredict(float TimeSinceLastUpdate) const
{
	return  TimeSinceLastUpdate >= Settings.PrecisePredictTimeSec * Settings.RoughPredictUpdateRate ;
}

int FPhysPredict::GetRoughStepsCount() const
{
	const float dt = GetRoughSimulationTimeStep();
	const float time = FMath::Abs(Settings.RoughPredictTimeSec - Settings.PrecisePredictTimeSec);
	if(FMath::IsNearlyZero(time)) return  0;
	return time / dt;
}

void FPhysPredict::RecomputePrediction()
{
	RecomputePrecisePredict();
	RecomputeRoughPredict();
}

void FPhysPredict::EnablePrediction()
{
	if(Comp)
	{
		bPredict = true;
		RecomputePrediction();
	}
}

void FPhysPredict::DisablePrediction()
{
	bPredict = false;
	PrecisePredictedTransforms.Empty();
	RoughPredictedTransforms.Empty();
}

float FPhysPredict::GetSimulationDeltaTime() const
{
	if(const auto P = Comp->PhysicsProcessor)
	{
		return P->GetSimDT();
	}
	return 0.0f;
}

void FPhysPredict::RecomputePrecisePredict()
{
	TimeLeftToNextPPT = GetSimulationDeltaTime();
	PrecisePredictedTransforms.Empty();

	if(Comp->PhysicsProcessor)
	{
		const int StepsToAdd = GetPreciseStepsCount();
		AddStepsToPrecisePredictArray(StepsToAdd);
	}
	else
	{
		PrecisePredictedTransforms.Add(Comp->CurrentTransform);
	}
}

void FPhysPredict::RecomputeRoughPredict()
{
	TimeSinceRoughPredictUpdate = 0.0f;
	RoughPredictedTransforms.Empty();

	if(Comp->PhysicsProcessor)
	{
		const int StepsToAdd = GetRoughStepsCount();
		AddStepsToRoughPredictArray(StepsToAdd);
	}
	else
	{
		RoughPredictedTransforms.Add(Comp->CurrentTransform);
	}
	
	Comp->OnRoughPredictionRecomputed.Broadcast();
}

void FPhysPredict::AddStepsToPrecisePredictArray(const int StepsToAdd)
{
	if(Comp)
	{
		const float StepTime =  GetSimulationDeltaTime();
		for (int i = 0; i < StepsToAdd; ++i)
		{
			const float SkipTime = PrecisePredictedTransforms.Num() * StepTime;
			auto NewTransform = Comp->CalculateNextPreciseTransform(*GetLastPrecisePredictedTransformPtrOrObjCurrentTransform(), SkipTime, true);
			PrecisePredictedTransforms.Add(NewTransform);
		}
	}
}

void FPhysPredict::AddStepsToRoughPredictArray(const int StepsToAdd)
{
	if(Comp)
	{
		for (int i = 0; i < StepsToAdd; ++i)
		{
			constexpr float SkipTime = 0;
			auto T = *GetLastRoughPredictedTransformPtrOrLastPreciseTransform();
			const float SimStep = GetRoughSimulationTimeStep();
			auto NewTransform = Comp->CalculateNextRoughTransform(T, SimStep, SkipTime, true);
			RoughPredictedTransforms.Add(NewTransform);
		}
	}
}

void FPhysPredict::UpdatePrecisePredictionDataBySteps(int StepsToUpdate)
{
	if(StepsToUpdate < 1) return;
	CommonUtils::RemoveArrayItemsFromEdge(PrecisePredictedTransforms, StepsToUpdate, true);
	AddStepsToPrecisePredictArray(StepsToUpdate);
}

void FPhysPredict::AdvanceByTime(float TimeBeforeNextPredict, int StepsToUpdate)
{
	SetTimeBeforeNextPredict(TimeBeforeNextPredict);
	UpdatePrecisePredictionDataBySteps(StepsToUpdate);
	TimeSinceRoughPredictUpdate += StepsToUpdate * GetSimulationDeltaTime();

	const bool ShouldRecomputeRough = ShouldRecomputeRoughPredict(TimeSinceRoughPredictUpdate);
	if(ShouldRecomputeRough)
	{
		RecomputeRoughPredict();
	}
}

FPhysTransform* FPhysPredict::GetFirstPrecisePredictedTransformPtr()
{
	return HasTPredicted() ? &PrecisePredictedTransforms[0] : nullptr;
}

FPhysTransform* FPhysPredict::GetLastPrecisePredictedTransformPtr()
{
	return HasTPredicted() ? &PrecisePredictedTransforms.Last() : nullptr;
}

FPhysTransform* FPhysPredict::GetLastPrecisePredictedTransformPtrOrObjCurrentTransform()
{
	const auto T = GetLastPrecisePredictedTransformPtr();
	if(T) return T;
	return Comp ?  &Comp->CurrentTransform : nullptr;
}

FPhysTransform* FPhysPredict::GetFirstRoughPredictedTransformPtr()
{
	return HasRoughPredicted() ? &RoughPredictedTransforms[0] : nullptr;
}

FPhysTransform* FPhysPredict::GetLastRoughPredictedTransformPtr()
{
	return HasRoughPredicted() ? &RoughPredictedTransforms.Last() : nullptr;
}

FPhysTransform* FPhysPredict::GetLastRoughPredictedTransformPtrOrLastPreciseTransform()
{
	const auto T = GetLastRoughPredictedTransformPtr();
	if(T) return T;
	return GetLastPrecisePredictedTransformPtrOrObjCurrentTransform();
}

FPhysTransform* FPhysPredict::GetPrecisePredictedTransformPtrAfterStepCount(int NumSubsteps) 
{
	return Native::ArrayGetElementByIndexInBounds(NumSubsteps, PrecisePredictedTransforms);
}

FPhysTransform* FPhysPredict::GetRoughPredictedTransformPtrAfterStepCount(int NumSubsteps) 
{
	return Native::ArrayGetElementByIndexInBounds(NumSubsteps, RoughPredictedTransforms);
}

TArray<FVector> FPhysPredict::GetPredictedLocations()
{
	TArray<FVector> A;
	for (auto Element : PrecisePredictedTransforms) A.Add(Element.Location);
	for (auto Element : RoughPredictedTransforms) A.Add(Element.Location);
	return A;
}

FTrajectory FPhysPredict::GetTrajectory()
{
	FTrajectory T;
	T.Points = GetPredictedLocations();
	return T;
}

FCustomVectorCurve FPhysPredict::GetTrajectoryCurve()
{
	FCustomVectorCurve Out;
	
	const auto FirstPrecisePtr = GetFirstPrecisePredictedTransformPtr();
	const int NumPrecise = PrecisePredictedTransforms.Num();

	const bool bOK = FirstPrecisePtr && NumPrecise > 1;
	if(bOK)
	{
		const FVector FirstLocation = FirstPrecisePtr->Location;
		const FVector SecondLocation = PrecisePredictedTransforms[1].Location;
		Out.InsertKeyToEnd(0.0f, FirstLocation);
		Out.InsertKeyToEnd(GetTimeBeforeNextPredict(), SecondLocation);

		const float TimeBeforeIntegralPredict = GetTimeBeforeNextPredict();
		const float PreciseSimDT = GetSimulationDeltaTime();
		
		for (int i = 2; i < NumPrecise; ++i)
		{
			const float Time = TimeBeforeIntegralPredict + PreciseSimDT * (i - 1);
			FVector Location = PrecisePredictedTransforms[i].Location;
			Out.InsertKeyToEnd(Time, Location);
		}
		
		const float RoughSimDT = GetRoughSimulationTimeStep();
		const float PreciseTimeEnd = Out.GetMaxTime();
		const int NumRough = RoughPredictedTransforms.Num();
		for (int i = 0; i < NumRough; ++i)
		{
			const float Time = PreciseTimeEnd + RoughSimDT * (i + 1);
			FVector Location = RoughPredictedTransforms[i].Location;
			Out.InsertKeyToEnd(Time, Location);
		}
	}
	
	return Out;
}
