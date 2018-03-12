#ifndef USV16_DYN_H
#define USV16_DYN_H
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "usv16_dyn_types.h"

extern void usv16_dyn(const double tau[3], const double vel[3], double output[3]);
extern void usv16_dyn_initialize();
extern void usv16_dyn_terminate();

#endif
