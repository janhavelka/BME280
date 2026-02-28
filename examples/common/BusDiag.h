#pragma once

#include "I2cScanner.h"

namespace bus_diag {
inline void scan() {
  i2c::scan();
}
}