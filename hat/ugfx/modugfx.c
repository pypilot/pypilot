/*
 * This file is part of the MicroPython ESP32 project, https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "sdkconfig.h"

#ifdef CONFIG_MICROPY_USE_UGFX

#include <stdint.h>

#include "py/obj.h"
#include "py/runtime.h"

#include "upy_wrap.h"


#include "tft/tftspi.h"

typedef struct _ugfx_surface_obj_t {
    mp_obj_base_t base;
    void *ptr;
    int contrast, hue;
} ugfx_surface_obj_t;
const mp_obj_type_t ugfx_surface_type;

STATIC mp_obj_t ugfx_surface_blit(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_src, ARG_xoff, ARG_yoff, ARG_flip};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_src, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_xoff, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_yoff, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_flip, MP_ARG_KW_ONLY | MP_ARG_BOOL, { .u_bool = false } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    ugfx_surface_obj_t *src = MP_OBJ_TO_PTR(args[ARG_src].u_obj);
    mp_int_t xoff = args[ARG_xoff].u_int;
    mp_int_t yoff = args[ARG_yoff].u_int;
    uint8_t flip = args[ARG_flip].u_bool;

    ugfx_surface_c_blit(self->ptr, src->ptr, xoff, yoff, flip);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_blit_obj, 0, ugfx_surface_blit);

STATIC mp_obj_t ugfx_surface_line(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_x1, ARG_y1, ARG_x2, ARG_y2, ARG_c};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_c, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x1 = args[ARG_x1].u_int;
    mp_int_t y1 = args[ARG_y1].u_int;
    mp_int_t x2 = args[ARG_x2].u_int;
    mp_int_t y2 = args[ARG_y2].u_int;
    mp_int_t c = args[ARG_c].u_int;

    ugfx_surface_c_line(self->ptr, x1, y1, x2, y2, c);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_line_obj, 0, ugfx_surface_line);

STATIC mp_obj_t ugfx_surface_box(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_x1, ARG_y1, ARG_x2, ARG_y2, ARG_c};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_c, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x1 = args[ARG_x1].u_int;
    mp_int_t y1 = args[ARG_y1].u_int;
    mp_int_t x2 = args[ARG_x2].u_int;
    mp_int_t y2 = args[ARG_y2].u_int;
    mp_int_t c = args[ARG_c].u_int;

    ugfx_surface_c_box(self->ptr, x1, y1, x2, y2, c);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_box_obj, 0, ugfx_surface_box);

STATIC mp_obj_t ugfx_surface_invert(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_x1, ARG_y1, ARG_x2, ARG_y2};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_x2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_y2, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t x1 = args[ARG_x1].u_int;
    mp_int_t y1 = args[ARG_y1].u_int;
    mp_int_t x2 = args[ARG_x2].u_int;
    mp_int_t y2 = args[ARG_y2].u_int;

    ugfx_surface_c_invert(self->ptr, x1, y1, x2, y2);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_invert_obj, 0, ugfx_surface_invert);

STATIC mp_obj_t ugfx_surface_fill(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_c};
    const mp_arg_t allowed_args[] = {
        { MP_QSTR_c, MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
    };

    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_int_t c = args[ARG_c].u_int;

    ugfx_surface_c_fill(self->ptr, c);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_fill_obj, 0, ugfx_surface_fill);


uint8_t minc(uint32_t c) {
    if(c > 255)
        return 255;
    return c;
}

STATIC mp_obj_t ugfx_display_surface_refresh(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    int width, height, bypp;
    char *data;
    ugfx_surface_c_info(self->ptr, &width, &height, &bypp, &data);

    uint32_t c = self->contrast, h = self->hue;
    char palette[256][3];
    uint32_t r, g, b;
    if(h < 85) {
        r = 85 - h;
        g = 85 - r;
        b = 0;
    } else if(h < 170) {
        r = 0;
        g = 170 - h;
        b = 85 - g;
    } else {
        g = 0;
        b = 255 - h;
        r = 85 - b;
    }
    for(uint32_t i=0; i<256; i++) {
        palette[i][0] = minc(i*c*r/8500);
        palette[i][1] = minc(i*c*g/8500);
        palette[i][2] = minc(i*c*b/8500);
    }
    
    disp_select();
#if 1
    // convert 8 bit to 24 bit (which is then converted to 16 bit by send_data before going to lcd)
#define STRIDE 4
    for(int y=0; y<height; y+=STRIDE) {
        uint8_t line[width*STRIDE*3];
        for(int x=0; x<width*STRIDE; x++) {
            uint8_t c = data[y*width+x];
            line[3*x+0] = 0xff - palette[c][0];
            line[3*x+1] = 0xff - palette[c][1];
            line[3*x+2] = 0xff - palette[c][2];
        }
        send_data(52, y+40, 52+width-1, y+40+STRIDE, width*STRIDE, (color_t*)line, 0);
    }
#else
    send_data(52, 40, 52+width-1, 40+height-1, 0, 0, 0); // just setup window
    disp_spi_transfer_cmd(TFT_RAMWR);
    while (disp_spi->handle->host->hw->cmd.usr); // Wait for SPI bus ready

    gpio_set_level(disp_spi->dc, 1); // Set DC to 1 (data mode);
        
    int idx = 0;
    for(int y=0; y<height; y++) {
        for(int x=0; x<width; x+=2) {
            uint8_t c0 = data[y*width+x+0];
            uint8_t c1 = data[y*width+x+1];
            // convert 8bit to 16 bit color 2 pixels at a time
            uint32_t c = 0;
            c |= (uint32_t)(c1 & 0xF8) << 24;
            c |= (uint32_t)(c1 & 0xFC) << 19;
            c |= (uint32_t)(c1 & 0xF8) << 13;
            c |= (uint32_t)(c0 & 0xF8) << 8;
            c |= (uint32_t)(c0 & 0xFC) << 3;
            c |= (uint32_t)(c0 & 0xF8) >> 3;            
            
            disp_spi->handle->host->hw->data_buf[idx++] = c;

            if(idx == 16) {
                _spi_transfer_start(disp_spi, 512, 0);
                idx = 0;
            }
        }
    }
    if(idx)
        _spi_transfer_start(disp_spi, 32*idx, 0);
    
#endif

    
    disp_deselect();

//    disp_select();
//    send_data(0, 0, width, height, width*height, (color_t*)data, 0);
//    wait_trans_finish(1);
//    disp_deselect();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_display_surface_refresh_obj, 0, ugfx_display_surface_refresh);


STATIC mp_obj_t ugfx_surface_free(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    ugfx_surface_c_free(self->ptr);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(ugfx_surface_free_obj, 0, ugfx_surface_free);


STATIC const mp_rom_map_elem_t ugfx_surface_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_blit),                MP_ROM_PTR(&ugfx_surface_blit_obj) },
    { MP_ROM_QSTR(MP_QSTR_line),                MP_ROM_PTR(&ugfx_surface_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_box),                 MP_ROM_PTR(&ugfx_surface_box_obj) },
    { MP_ROM_QSTR(MP_QSTR_invert),              MP_ROM_PTR(&ugfx_surface_invert_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill),                MP_ROM_PTR(&ugfx_surface_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh),             MP_ROM_PTR(&ugfx_display_surface_refresh_obj) },
    { MP_ROM_QSTR(MP_QSTR_free),                MP_ROM_PTR(&ugfx_surface_free_obj) },
};
STATIC MP_DEFINE_CONST_DICT(ugfx_surface_locals_dict, ugfx_surface_locals_dict_table);


//-----------------------------------------------------------------------------------------------
STATIC void ugfx_surface_printinfo(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    ugfx_surface_obj_t *self = self_in;
    int width, height, bypp;
    char *data;
    ugfx_surface_c_info(self->ptr, &width, &height, &bypp, &data);
    mp_printf(print, "surface %dx%dx%d %p", width, height, bypp, data);
}

// constructor(id, ...)
//-----------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t ugfx_surface_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {    
    // this checks the number of arguments (min 1, max 1);
    // on error -> raise python exception
    ugfx_surface_obj_t *self = m_new_obj(ugfx_surface_obj_t);
    self->base.type = &ugfx_surface_type;

    if(n_args == 2) {
        mp_arg_check_num(n_args, n_kw, 1, 2, true);
        char *path = (char *)mp_obj_str_get_str(args[0]);
        self->ptr = ugfx_surface_from_file_c(path, mp_obj_get_int(args[1]));
        return MP_OBJ_FROM_PTR(self);
    }

    mp_arg_check_num(n_args, n_kw, 1, 4, true);

    int w = mp_obj_get_int(args[0]);
    int h = mp_obj_get_int(args[1]);
    int bypp = mp_obj_get_int(args[2]);

    self->contrast = 20;
    self->hue = 0;
    self->ptr = ugfx_surface_from_data_c(w, h, bypp, 0);

    return MP_OBJ_FROM_PTR(self);
}

STATIC void ugfx_surface_attr(mp_obj_t self_in, qstr attribute, mp_obj_t *dest) {
    ugfx_surface_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int width, height, bypp;
    char *data;
    ugfx_surface_c_info(self->ptr, &width, &height, &bypp, &data);
    if(attribute == MP_QSTR_width)
        dest[0] = mp_obj_new_int(width);
    else
    if(attribute == MP_QSTR_height)
        dest[0] = mp_obj_new_int(height);
    else
    if(attribute == MP_QSTR_bypp)
        dest[0] = mp_obj_new_int(bypp);
    else
    if(attribute == MP_QSTR_contrast)
        if(dest[0] == MP_OBJ_SENTINEL) {
            self->contrast = mp_obj_get_int(dest[1]);
            dest[0] = MP_OBJ_NULL;
        } else
            dest[0] = mp_obj_new_int(self->contrast);
    else
    if(attribute == MP_QSTR_hue)
        if(dest[0] == MP_OBJ_SENTINEL) {
            self->hue = mp_obj_get_int(dest[1]);
            dest[0] = MP_OBJ_NULL;
        } else
            dest[0] = mp_obj_new_int(self->hue);
    else {
        // fallback to lookup
        mp_obj_type_t *type = mp_obj_get_type(self_in);
        mp_map_t *locals_map = &type->locals_dict->map;
        mp_map_elem_t *elem = mp_map_lookup(locals_map, MP_OBJ_NEW_QSTR(attribute), MP_MAP_LOOKUP);
        if (elem != NULL)
            mp_convert_member_lookup(self_in, type, elem->value, dest);
    }
}

const mp_obj_type_t ugfx_surface_type = {
    { &mp_type_type },
    .name = MP_QSTR_surface,
    .print = ugfx_surface_printinfo,
    .make_new = ugfx_surface_make_new,
    .attr = ugfx_surface_attr,
    .locals_dict = (mp_obj_t)&ugfx_surface_locals_dict,
};


typedef struct _ugfx_obj_t {
    mp_obj_base_t base;
    
} ugfx_obj_t;

//===============================================================
STATIC const mp_rom_map_elem_t ugfx_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ugfx) },
    { MP_ROM_QSTR(MP_QSTR_surface),                MP_ROM_PTR(&ugfx_surface_type) },
};

//===============================================================================
STATIC MP_DEFINE_CONST_DICT(ugfx_module_globals, ugfx_module_globals_table);

const mp_obj_module_t mp_module_ugfx = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&ugfx_module_globals,
};


#endif
