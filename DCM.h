#ifndef DCM_H
#define DCM_H

#include "vector.h"
#include "math.h"
#include "arduimu.h"
#include "stdlib.h"
#include "matrix.h"
#include "ublox.h"

void Normalize(void);
void Drift_correction(void);
void Accel_adjust(void);
void Matrix_update(void);
void Euler_angles(void);

#endif
