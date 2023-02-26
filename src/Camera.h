#include <esp_camera.h>
#include <WiFi.h>

#define PWDN_GPIO_NUM   -1 //47 // CAM PWR (PMOS)
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   41
#define SIOD_GPIO_NUM    1
#define SIOC_GPIO_NUM    2

#define Y9_GPIO_NUM     42
#define Y8_GPIO_NUM     40
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     37
#define Y5_GPIO_NUM     35
#define Y4_GPIO_NUM     48
#define Y3_GPIO_NUM     45
#define Y2_GPIO_NUM     36
#define VSYNC_GPIO_NUM  43
#define HREF_GPIO_NUM   44
#define PCLK_GPIO_NUM   38

#define CAM_PWR_N_PIN   47


bool setup_cam(framesize_t framesize = FRAMESIZE_96X96, int jpeg_quality = 12);

void turn_off_cam();

bool camera_capture_example(); // debug only

/* Use:

camera_fb_t* fb = esp_camera_fb_get();

esp_camera_fb_return(fb);

*/
