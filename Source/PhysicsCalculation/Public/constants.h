#pragma once

#include "CoreMinimal.h"

/*
* USE Cm, Seconds, KG, radians for all computations
*/

//0.0083333f => 120 physics FPS

static float PHYS_FPS = 0.0083333f;
static float PHYS_PRECISION_SCALE = 0.1f; // decrease simulation step to increase predict precision 
static float PHYS_SIM_DT = PHYS_FPS * PHYS_PRECISION_SCALE;
static  float SeparationSphereMultiplier = 1.05f;
static  float TunnelingDetectionSphereRadiusMul = 0.48f; //0.48;

