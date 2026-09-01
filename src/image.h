#ifndef IMAGE_H
#define IMAGE_H
#include <stdint.h>

typedef struct {
        int w, h;
        uint8_t *data;
} image_t;

int  image_load(const char *path, image_t *out);
void image_free(image_t *img);
int  image_save_pgm(const char *path, const image_t *img);

#endif

