#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


/** Robot wide IPC bus. */
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

bool get_fail_to_score(void);
void set_fail_to_score(bool val);

#ifdef __cplusplus
}
#endif

#endif
