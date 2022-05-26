#ifndef  _LCD_IMAGE_
#define _LCD_IMAGE_

#include <stdio.h>
#include <stdint.h>

#define _USING_RGB4444_IMAGE_       0

#if _USING_RGB4444_IMAGE_
extern const uint16_t elnic_map[];
#else
extern const uint8_t elnic_map[];
#endif

#if 1
extern const uint16_t rgb444_image_1[];
#endif

#endif /* LV_COLOR_DEPTH_1 */