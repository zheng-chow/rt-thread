#ifndef __VISCA_H__
#define __VISCA_H__

#define VISCA_DEV_ADDR      1
#define VISCA_BASIC_CMDLEN  6

typedef enum _VISCA_CategoryCode {
    VISCA_Cat_Basic           = 0x04,
    VISCA_Cat_HYK             = 0x05,
    VISCA_Cat_Gimbal          = 0x06,
    VISCA_Cat_Regs            = 0x30,
    VISCA_Cat_Reserve         = 0xFF,
    
}VISCA_CategoryCode_t;

typedef enum _VISCA_Command{
    VISCA_CMD_PowerOn         = 0x0002,
    VISCA_CMD_PowerOff        = 0x0003,
 
    VISCA_CMD_ZoomStop        = 0x0700,
    VISCA_CMD_ZoomIn          = 0x0702,
    VISCA_CMD_ZoomOut         = 0x0703,
    VISCA_CMD_ZoomInSpeed     = 0x0720, 
    VISCA_CMD_ZoomOutSpeed    = 0x0730,    
    VISCA_CMD_ZoomSet         = 0x47,
    VISCA_CMD_ZoomInquiry     = 0x47,
    VISCA_CMD_FocusStop       = 0x0800,
    VISCA_CMD_FocusFar        = 0x0802,
    VISCA_CMD_FocusNear       = 0x0803,
    VISCA_CMD_FocusAuto       = 0x3802,
    VISCA_CMD_FocusManual     = 0x3803,
    VISCA_CMD_FocusModeSwitch = 0x3810, 
    VISCA_CMD_FocusSet        = 0x48,
    
    VISCA_CMD_PhotoRecord     = 0x0000,
    VISCA_CMD_VideoRecord     = 0x0100,
    
}VISCA_CMD_t;

typedef enum _ZOOM_Ratio {
    ZOOM_Ratio_Optical_20X = 20,
    ZOOM_Ratio_Optical_30X = 30,
    ZOOM_Ratio_Mix_30X_12X = 42
}ZOOM_Ratio_t;

int VISCA_GetZoomPos(float *zoom_pos, ZOOM_Ratio_t ratio, int timeout);

#endif
