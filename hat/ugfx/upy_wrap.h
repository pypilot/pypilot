/* Copyright (C) 2016 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

//unsigned int color(int r, int g, int b);

#ifdef __cplusplus
#extern "C" {
#endif

void *ugfx_surface_from_surface_c(void *s);
void *ugfx_surface_from_data_c(int w, int h, int internal_bypp, const char *data32);
void *ugfx_surface_from_file_c(const char* filename, int bypp);

void ugfx_surface_c_free(void *s);

void ugfx_surface_c_info(void *s, int *width, int *height, int *bypp, char **data);
void ugfx_surface_c_file_info(void *dest, const char *filename, int *width, int *height);
void ugfx_surface_c_blit_file(void *dest, const char *filename, int xoff, int yoff);
void ugfx_surface_c_blit(void *dest, void *src, int xoff, int yoff, bool flip);
void ugfx_surface_c_line(void *dest, int x1, int y1, int x2, int y2, unsigned int c);
void ugfx_surface_c_box(void *dest, int x1, int y1, int x2, int y2, unsigned int c);
void ugfx_surface_c_invert(void *dest, int x1, int y1, int x2, int y2);
void ugfx_surface_c_fill(void *dest, unsigned int c);

void *ugfx_spiscreen_c(int driver);
void ugfx_spiscreen_c_free(void *screen);
void ugfx_spiscreen_c_refresh(void *screen);


#ifdef __cplusplus
}
#endif
