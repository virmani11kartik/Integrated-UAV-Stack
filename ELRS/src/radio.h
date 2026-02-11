// Radio selection wrapper
#pragma once

#include "config.h"

#if ELRS_USE_E28
#include "radio_e28_uart.h"
#else
#include "radio_sx1280.h"
#endif
