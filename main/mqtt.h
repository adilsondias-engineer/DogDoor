#ifndef _MQTT_H_
#define _MQTT_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
void mqtt_init( void );
void mqtt_do_work( void );
void mqtt_force_update( void );
void mqtt_write_topic(char *topic, uint8_t *p_data, size_t data_len);
#ifdef __cplusplus
}
#endif

#endif