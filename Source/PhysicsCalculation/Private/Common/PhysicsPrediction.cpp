// Fill out your copyright notice in the Description page of Project Settings.

#include "Common/PhysicsPrediction.h"
#include "Components/CustomPhysicsComponent.h"
#include "Components/CustomPhysicsProcessor.h"

void UPhysicsPrediction::Initialize()
{
   Comp = Cast<UCustomPhysicsComponent>(GetOuter());
   check(Comp)
}

int UPhysicsPrediction::GetPreciseStepsCount() const
{
   return  Settings.PrecisePredictTimeSec / GetSimulationDeltaTime();
}

float UPhysicsPrediction::GetRoughSimulationTimeStep() const
{
   return Comp->PhysicsProcessor->GetSimulationFrameTime() * Settings.RoughPredictionTimeScale;
}

bool UPhysicsPrediction::ShouldRecomputeRoughPredict(float TimeSinceLastUpdate) const
{
   return  TimeSinceLastUpdate >= Settings.PrecisePredictTimeSec * Settings.RoughPredictUpdateRate ;
}

int UPhysicsPrediction::GetRoughStepsCount() const
{
   const float dt = GetRoughSimulationTimeStep();
   const float time = FMath::Abs(Settings.RoughPredictTimeSec - Settings.PrecisePredictTimeSec);
   if(FMath::IsNearlyZero(time)) return  0;
   return time / dt;
}

void UPhysicsPrediction::RecomputePrediction()
{
   RecomputePrecisePredict();
   RecomputeRoughPredict();
}

void UPhysicsPrediction::RecomputePrecisePredict()
{
   TimeLeftToNextPPT = GetSimulationDeltaTime();
   PrecisePredictionCurve.Reset();

   if(Comp->PhysicsProcessor)
   {
      const int StepsToAdd = GetPreciseStepsCount();
      AddStepsToPrecisePrediction(StepsToAdd);
   }
   else
   {
      PrecisePredictionCurve.InsertKeyToEnd(0.0f, Comp->CurrentTransform);
   }
}

void UPhysicsPrediction::RecomputeRoughPredict()
{
   TimeSinceRoughPredictUpdate = 0.0f;
   RoughPredictionCurve.Reset();

   if(Comp->PhysicsProcessor)
   {
      const int StepsToAdd = GetRoughStepsCount();
      AddStepsToRoughPrediction(StepsToAdd);
   }
   else
   {
      RoughPredictionCurve.InsertKeyToEnd(0.0f, Comp->CurrentTransform);
   }
	
   Comp->OnRoughPredictionRecomputed.Broadcast();
}

void UPhysicsPrediction::AddStepsToPrecisePrediction(int StepsToAdd)
{
   if(Comp)
   {
      const float StepTime =  GetSimulationDeltaTime();
      for (int i = 0; i < StepsToAdd; ++i)
      {
         const int NumExistingKeys = PrecisePredictionCurve.GetNumKeys();
         const float SkipTime = NumExistingKeys * StepTime;
         auto LastKnownTransform = GetLastPrecisePredictedTransformPtrOrObjCurrentTransform();
         auto NewTransform = Comp->CalculateNextPreciseTransform(LastKnownTransform, SkipTime, true);
         PrecisePredictionCurve.InsertKeyToEnd(SkipTime, NewTransform);
      }
   }
}

void UPhysicsPrediction::AddStepsToRoughPrediction(int StepsToAdd)
{
   if(Comp)
   {
      for (int i = 0; i < StepsToAdd; ++i)
      {
         constexpr float SkipTime = 0;
         const float SimStep = GetRoughSimulationTimeStep();
         auto LastKnownTransform = GetLastRoughPredictedTransformPtrOrLastPreciseTransform();
         auto NewTransform = Comp->CalculateNextRoughTransform(LastKnownTransform, SimStep, SkipTime, true);
         RoughPredictionCurve.InsertKeyToEnd(SkipTime, NewTransform);
      }
   }
}

FPhysTransform UPhysicsPrediction::GetLastTransformOrObjCurrentTransform(const FPhysicsPredictionCurve& Curve) const
{
   return Curve.HasKeys() ? Curve.LastKey() : Comp->GetCurrentPhysTransform();
}

FPhysTransform UPhysicsPrediction::GetLastPrecisePredictedTransformPtrOrObjCurrentTransform() const
{
   return GetLastTransformOrObjCurrentTransform(PrecisePredictionCurve);
}

FPhysTransform UPhysicsPrediction::GetLastRoughPredictedTransformPtrOrLastPreciseTransform() const
{
   return GetLastTransformOrObjCurrentTransform(RoughPredictionCurve);
}

void UPhysicsPrediction::EnablePrediction()
{
   if(Comp)
   {
      bPredict = true;
      RecomputePrediction();
   }
}

void UPhysicsPrediction::DisablePrediction()
{
   bPredict = false;
   PrecisePredictionCurve.Reset();
   RoughPredictionCurve.Reset();
}

float UPhysicsPrediction::GetSimulationDeltaTime() const
{
   if(const auto P = Comp->PhysicsProcessor)
   {
      return P->GetSimDT();
   }
   return 0.0f;
}


