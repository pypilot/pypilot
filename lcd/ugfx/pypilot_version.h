static unsigned int width = 28;
static unsigned int height = 13;

#define HEADER_PIXEL(data,pixel) {\
pixel[0] = data[0]; \
pixel[1] = data[0]; \
pixel[2] = data[0]; \
data ++; }

static char header_data[] = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,0,0,0,0,255,255,255,255,255,255,255,255,255,0,0,0,255,255,255,255,0,0,0,0,255,255,255,0,0,0,0,0,255,255,255,255,255,255,255,255,0,0,0,0,255,255,255,0,0,0,0,0,0,255,255,0,0,255,255,0,0,255,255,255,255,255,255,0,0,0,0,0,255,255,255,255,255,255,255,0,0,255,255,0,0,255,255,0,0,255,255,255,255,255,255,0,255,0,0,0,255,255,255,255,255,255,255,0,0,255,0,0,0,255,255,0,0,255,255,255,255,255,255,255,255,0,0,0,255,255,255,255,255,255,0,0,255,255,0,0,0,255,255,0,0,255,255,255,255,255,255,255,255,0,0,0,255,255,255,255,255,0,0,0,255,255,255,0,0,255,255,0,0,255,255,255,255,255,255,255,255,0,0,0,255,255,255,255,0,0,0,255,255,255,255,0,0,255,255,0,0,255,255,255,255,255,255,255,255,0,0,0,255,255,255,255,0,0,255,255,255,255,255,0,0,0,0,0,0,255,255,0,0,255,255,255,255,0,0,0,255,255,255,0,0,0,0,0,0,255,255,255,0,0,0,0,255,255,255,0,0,255,255,255,255,0,0,0,255,255,255,0,0,0,0,0,0,255,};