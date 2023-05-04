#include "Camera.h"

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


bool setupCam(framesize_t frameSize, int jpegQuality) {
    // Serial.end();
    // pinMode(VSYNC_GPIO_NUM, INPUT);

    // Cam power on GPIO
    pinMode(CAM_PWR_N_PIN, OUTPUT);
    digitalWrite(CAM_PWR_N_PIN, LOW);  // necessary (default high, measured with voltmeter)

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    // config.xclk_freq_hz = 1000000;
    config.pixel_format = PIXFORMAT_JPEG;
    // config.pixel_format = PIXFORMAT_RGB565;
    // config.pixel_format = PIXFORMAT_GRAYSCALE; // marche pas ?

    // Added by Lio
    // THE KEY TO WORK ON XSTRATO !!!
    config.fb_location = CAMERA_FB_IN_DRAM; /*!< The location where the frame buffer will be allocated */
    config.grab_mode = CAMERA_GRAB_LATEST;  /*!< When buffers should be filled */

    config.frame_size = frameSize; 
    config.jpeg_quality = jpegQuality;
    config.fb_count = 1;  

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        SERIAL_TO_PC.printf("Camera init failed with error 0x%x", err);
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();

    s->set_framesize(s, frameSize);  
    s->set_quality(s, jpegQuality);
    //s->set_brightness(s, 0);     // -2 to 2
    //s->set_contrast(s, 0);       // -2 to 2
    //s->set_saturation(s, 0);     // -2 to 2
    //s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    //s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    //s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    //s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    //s->set_ae_level(s, 0);
    //s->set_gain_ctrl(s, 1);
    //s->set_wpc(s, 1);
    //s->set_aec_value(s, 300);    // 0 to 1200
    //s->set_aec2(s, 1);           // 0 = disable , 1 = enable
    //s->set_lenc(s, 0);           // 0 = disable , 1 = enable
    //s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable

    return true;
}

void turnOffCam() {
    digitalWrite(CAM_PWR_N_PIN, HIGH);
}

bool cameraCaptureExample() {
    // capture a frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        SERIAL_TO_PC.println("Error with frame buffer pointer");
        return false;
    }
    // replace this with your own function
    SERIAL_TO_PC.println("Size: " + String(fb->len) + " Bytes");
    // display_image(fb->width, fb->height, fb->pixformat, fb->buf, fb->len);

    // Save picture in memory

    // return the frame buffer back to be reused
    esp_camera_fb_return(fb);
    return true;
}
