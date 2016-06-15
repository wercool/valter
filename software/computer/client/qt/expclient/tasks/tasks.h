#ifndef TASKS_H
#define TASKS_H

#include "valter.h"
#include "tasks/delaytask.h"
#include "tasks/setmoduleinitialstatetask.h"

// PLATFORM-MANIPULATOR-AND-IR-BUMPER
#include "tasks/platform-manipulator-and-ir-bumper/setlink1positiontask.h"
#include "tasks/platform-manipulator-and-ir-bumper/setlink2positiontask.h"
#include "tasks/platform-manipulator-and-ir-bumper/setgrippertiltpositiontask.h"
#include "tasks/platform-manipulator-and-ir-bumper/setgripperrotationposition.h"
#include "tasks/platform-manipulator-and-ir-bumper/setgrippergrasperposition.h"
#include "tasks/platform-manipulator-and-ir-bumper/setlink1motordynamics.h"
#include "tasks/platform-manipulator-and-ir-bumper/setlink2motordynamics.h"
#include "tasks/platform-manipulator-and-ir-bumper/setgripperrotationmotordynamics.h"

//PLATFORM-CONTROL-P1
#include "tasks/platform-control-p1/trasnslateplatformlinearlytask.h"
#include "tasks/platform-control-p1/rotateplatformtask.h"

#endif // TASKS_H
