#include <Arduino.h>
#include <esp_camera.h>
#include "Config.h"

bool setupCam(framesize_t frameSize = FRAMESIZE_96X96, int jpegQuality = 12);
void turnOffCam();
bool cameraCaptureExample(); // debug only

/* Use:

camera_fb_t* fb = esp_camera_fb_get();

esp_camera_fb_return(fb);

*/
