#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "image.h"
#include <stdio.h>

int image_load(const char *path, image_t *out) {
        int ch;
        out->data = stbi_load(path, &out->w, &out->h, &ch, 1);  // 1 = force grayscale
        return out->data ? 0 : -1;
}

void image_free(image_t *img) {
        stbi_image_free(img->data);
        img->data = NULL;
}

int image_save_pgm(const char *path, const image_t *img) {
        FILE *f = fopen(path, "wb");
        if (!f) return -1;
        fprintf(f, "P5\n%d %d\n255\n", img->w, img->h);
        fwrite(img->data, 1, (size_t)img->w * img->h, f);
        fclose(f);
        return 0;
}

