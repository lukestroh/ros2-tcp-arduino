/*
 * Source file for LimitSwitch class
 * Author: Luke Strohbehn
 * Date: 4/12/23
 * 
 * ISRs have to be static, taking no arguments. However, this means the method is no longer tied to any specific 
 * instance and can only access static variables, defeating the purpose of the class. Therefore, we need to write
 * some "glue" routines. These functions interface between an ISR and a specific class instance. This particular
 * class implements this in the `__init__()` function.
 * For more details, see: http://www.gammon.com.au/forum/?id=12983
 * 
 */

#include "linearstepper.h"


// LinearStepper::LinearStepper() {

// }

// void LinearStepper::calibrate_stepper() {
//     /* Zeros stepper position to limit_switch0 */
// }
