#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "utils.h"
#include "application.h"
#include  "main.h"
//static void _application_task(void *Param);

//-----------------------------------------------------------------------------
/*static void _application_task(void *Param)
{ 
  const uint32_t task_delay_ms = 250;

  while(1)
  {
    delay_ms(task_delay_ms);
    hardware_toggle_led();
  }  
}
*/
//-----------------------------------------------------------------------------
char const * application_get_json( const char *p_custom_header )
{
  static char buffer[2048] = { 0 };
  char *p_buffer = buffer;
  
  if ( p_custom_header )
  {
    p_buffer += sprintf(p_buffer, "%s", p_custom_header);
  }
  
  p_buffer += sprintf(p_buffer, "{");
  p_buffer += sprintf(p_buffer, "\"SystemTime\": \"%s\",", get_system_time_str());
 // p_buffer += sprintf(p_buffer, "Firmware Build: %s %s, Boot Count: %li<br>", __DATE__, __TIME__, nvm_get_param_int32( NVM_PARAM_RESET_COUNTER ));
  p_buffer += sprintf(p_buffer, "\"UpTime\": \"");
  p_buffer += add_formatted_duration_str( p_buffer, system_uptime_s() );
  p_buffer += sprintf(p_buffer, "\",");
  p_buffer += sprintf(p_buffer, "\"DoorStatus\": \"%s\",",  getDoorStatus());
  p_buffer += sprintf(p_buffer, "\"MovementDetected\": \"%d\",", isMovementDetected() );
  p_buffer += sprintf(p_buffer, "\"SensorTriggered\": \"%s\"", getSensorTriggered() );
  //p_buffer += sprintf(p_buffer, "Detected Distance: %0.04f cm<br>", getDetectedDistance() );
   p_buffer += sprintf(p_buffer,"}");


  return buffer;
}

//-----------------------------------------------------------------------------
char const * application_get_html( const char *p_custom_header )
{
  static char buffer[2048] = { 0 };
  char *p_buffer = buffer;
  
  if ( p_custom_header )
  {
    p_buffer += sprintf(p_buffer, "%s", p_custom_header);
  }
  
  p_buffer += sprintf(p_buffer, "<h1>System Info</h1>");
  p_buffer += sprintf(p_buffer, "System Time: %s<br>", get_system_time_str());
 // p_buffer += sprintf(p_buffer, "Firmware Build: %s %s, Boot Count: %li<br>", __DATE__, __TIME__, nvm_get_param_int32( NVM_PARAM_RESET_COUNTER ));
  p_buffer += sprintf(p_buffer, "Up-time: ");
  p_buffer += add_formatted_duration_str( p_buffer, system_uptime_s() );
  p_buffer += sprintf(p_buffer, "<br>");
  p_buffer += sprintf(p_buffer, "Door Status: %s<br>",  getDoorStatus());
  p_buffer += sprintf(p_buffer, "<br>");
  p_buffer += sprintf(p_buffer, "Movement Detected: %d<br>", isMovementDetected() );
  p_buffer += sprintf(p_buffer, "Sensor Triggered: %s<br>", getSensorTriggered() );
  p_buffer += sprintf(p_buffer, "Detected Distance: %0.04f cm<br>", getDetectedDistance() );
  p_buffer += sprintf(p_buffer, "<br>");
  p_buffer += sprintf(p_buffer, "<a href=\"/openDoor\">Open Door</a><br>");
  p_buffer += sprintf(p_buffer, "<a href=\"/closeDoor\">Close Door</a><br>");
  


  return buffer;
}

//-----------------------------------------------------------------------------
char const * application_post_html(const char *p_post_data)
{
  return application_get_html( NULL );
}

//-----------------------------------------------------------------------------
char const * application_get_mqtt_status_msg(void)
{
  static char status_msg[256];
  static uint32_t status_msg_id = 0;
  
  char *p_msg = status_msg;
  status_msg_id++;
  p_msg += sprintf( p_msg, "{ \"message_id\":%li,",   status_msg_id );
  p_msg += sprintf( p_msg, "\"uptime\":%lu,",         (uint32_t)system_uptime_s() ); 
  p_msg += sprintf( p_msg, "\"system_time\":\"%s\",", get_system_time_str() );
  p_msg += sprintf( p_msg, "\"doorStatus\":\"%s\",", getDoorStatus() );
  p_msg += sprintf( p_msg, "\"movementDetected\":\"%d\",", isMovementDetected() );
  p_msg += sprintf( p_msg, "\"sensorTriggered\":\"%s\",", getSensorTriggered() );
  p_msg += sprintf( p_msg, "\"detectedDistance\":\"%0.04f cm\"", getDetectedDistance() );
  p_msg += sprintf( p_msg, "}" );
  
  return status_msg;
}

//-----------------------------------------------------------------------------
void application_handle_mqtt_request_msg( char *p_msg )
{
  printf("MQTT request message data: %s\n", p_msg );
}

//-----------------------------------------------------------------------------
void application_handle_user_button_press(void)
{
}

//-----------------------------------------------------------------------------
void application_init(void)
{
  //xTaskCreate( _application_task, "app_task", 8192, NULL, 5, NULL);
}
  //      ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
  //      ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);