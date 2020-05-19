#ifndef _R2CA_H_
#define _R2CA_H_

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "r2ca_lib.h"
#undef true
#undef false
#include "Arduino.h"

#ifdef RCA_USE_SERIAL3
extern Uart Serial3;
#endif /* RCA_USE_SERIAL3 */

#include "kernel_cfg.h"
#include "r2ca_app.h"
#endif /* _RCA_H_ */
