#ifndef TASKS_H
#define TASKS_H

#include "valter.h"

// GENERIC
#include "tasks/generic/delaytask.h"
#include "tasks/generic/setmoduleinitialstatetask.h"
#include "tasks/generic/saytask.h"

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
#include "tasks/platform-control-p1/cmdveltask.h"
#include "tasks/platform-control-p1/rotateplatformtask.h"
#include "tasks/platform-control-p1/rotatebodytask.h"

//ARM-CONTROL-RIGHT
#include "tasks/arm-control-right/setrightforearmpositiontask.h"
#include "tasks/arm-control-right/setrightarmpositiontask.h"
#include "tasks/arm-control-right/setrightlimbpositiontask.h"
#include "tasks/arm-control-right/setrightarmrollpositiontask.h"

//ARM-CONTROL-LEFT
#include "tasks/arm-control-left/setleftforearmpositiontask.h"
#include "tasks/arm-control-left/setleftarmpositiontask.h"
#include "tasks/arm-control-left/setleftlimbpositiontask.h"
#include "tasks/arm-control-left/setleftarmrollpositiontask.h"

//BODY-CONTROL-P1
#include "tasks/body-control-p1/setrightarmyawpositiontask.h"
#include "tasks/body-control-p1/setleftarmyawpositiontask.h"
#include "tasks/body-control-p1/setheadyawpositiontask.h"
#include "tasks/body-control-p1/setheadpitchpositiontask.h"

#endif // TASKS_H
