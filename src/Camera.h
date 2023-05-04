#include <Arduino.h>
#include <esp_camera.h>
#include "Config.h"

bool setupCam(framesize_t frameSize = FRAMESIZE_96X96, int jpegQuality = 12);
void turnOffCam();
bool cameraCaptureExample(); // debug only

#define CAM_FRAMESIZE_DEFAULT FRAMESIZE_VGA
#define CAM_QUALITY_DEFAULT 24
#define CAM_WB_DEFAULT true
#define CAM_AWB_GAIN_DEFAULT true
#define CAM_WB_MODE_DEFAULT 0
#define CAM_EXPOSURE_DEFAULT true
#define CAM_EXPOSURE_VALUE_DEFAULT 600
#define CAM_AEC2_DEFAULT true
#define CAM_RAW_GMA_DEFAULT true

/* Use:

camera_fb_t* fb = esp_camera_fb_get();

esp_camera_fb_return(fb);

*/
