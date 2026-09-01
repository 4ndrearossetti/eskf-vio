#include <stdio.h>
#include <stdlib.h>
#include "image.h"
#include "fast.h"

static int nfail = 0;

static void check(const char *name, int cond) {
        printf("%-34s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

int main(void) {
        const char *home = getenv("HOME");
        char path[512];
        snprintf(path, sizeof path,
                 "%s/datasets/euroc/MH_01_easy/mav0/cam0/data/1403636579763555584.png", home);

        image_t img;
        int rc = image_load(path, &img);
        check("image: loads", rc == 0);
        if (rc != 0) { printf("FAIL (1)\n"); return 1; }

        printf("  %d x %d, px(0,0)=%u px(376,240)=%u\n",
               img.w, img.h, img.data[0], img.data[240*img.w + 376]);
        check("image: 752x480", img.w == 752 && img.h == 480);
        check("image: save pgm", image_save_pgm("/tmp/frame0.pgm", &img) == 0);

        corner_t *c = malloc(sizeof(corner_t) * 100000);
        int n = fast_detect(&img, 30, c, 100000);
        printf("  fast on frame 0, t=30: %d corners\n", n);
        check("fast: finds corners on real frame", n > 100);

        FILE *fc = fopen("/tmp/corners.txt", "w");
        for (int i = 0; i < n; i++) fprintf(fc, "%d %d\n", c[i].x, c[i].y);
        fclose(fc);
        free(c);

        image_free(&img);
        check("image: freed", img.data == NULL);

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

