#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns true if the string was a recognized haptic command. */
bool haptic_cmd_handle(const char *cmd);

#ifdef __cplusplus
}
#endif
