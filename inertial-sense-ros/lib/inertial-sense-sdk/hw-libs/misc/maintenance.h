#ifndef MAINTENANCE_H_
#define MAINTENANCE_H_
#ifdef __cplusplus
extern "C" {
#endif

// prototypes
void led_maintenance(void);
void led_sys_heartbeat(void);
void rtos_maintenance(void);
void save_persistent_messages(void);

#ifdef __cplusplus
}
#endif
#endif // MAINTENANCE_H_
