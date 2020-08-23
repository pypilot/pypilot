#include "ugfx.h"
extern "C" {

void *ugfx_surface_from_surface_c(void *s) { return (void*)new surface((surface*)s); }
void *ugfx_surface_from_data_c(int w, int h, int internal_bypp, const char *data32) { return (void*)new surface(w, h, internal_bypp, data32); }
void *ugfx_surface_from_file_c(const char* filename, int bypp) { return (void*)new surface(filename, bypp); }

void ugfx_surface_c_free(void *s) { delete (surface*)s; }
    
void ugfx_surface_c_info(void *s, int *width, int *height, int *bypp, char **data) {
    surface *surf = (surface*)s;
    *width = surf->width;
    *height = surf->height;
    *bypp = surf->bypp;
    *data = surf->p;
}

//void ugfx_surface_c_file_info(void *dest, const char *filename, int *width, int *height) { ((surface*)dest)->file_info(filename, width, height); }
//void ugfx_surface_c_blit_file(void *dest, const char *filename, int xoff, int yoff) { ((surface*)dest)->blit(filename, xoff, yoff); }

    
void ugfx_surface_c_blit(void *dest, void *src, int xoff, int yoff, int flip) { ((surface*)dest)->blit((surface*)src, xoff, yoff, flip); }
void ugfx_surface_c_line(void *dest, int x1, int y1, int x2, int y2, unsigned int c) { ((surface*)dest)->line(x1, y1, x2, y2, c); }
void ugfx_surface_c_box(void *dest, int x1, int y1, int x2, int y2, unsigned int c)  { ((surface*)dest)->box(x1, y1, x2, y2, c); }
void ugfx_surface_c_invert(void *dest, int x1, int y1, int x2, int y2)  { ((surface*)dest)->invert(x1, y1, x2, y2); }
void ugfx_surface_c_fill(void *dest, unsigned int c)  { ((surface*)dest)->fill(c); }

void *ugfx_spiscreen_c(int driver) { return (void*)new spiscreen(driver); }
void ugfx_spiscreen_c_free(void *screen) { delete (spiscreen*)screen; }
void ugfx_spiscreen_c_refresh(void *screen) { ((spiscreen*)screen)->refresh(); }
}

