#include <esp_camera.h>
#include <WiFi.h>


bool setup_cam(framesize_t framesize = FRAMESIZE_96X96, int jpeg_quality = 12);

void turn_off_cam();

bool camera_capture_example(); // debug only

/* Use:

camera_fb_t* fb = esp_camera_fb_get();

esp_camera_fb_return(fb);

*/
