#include <stdio.h>
#include <stdlib.h>
#include "frontend.h"
#include "euroc.h"

static int nfail = 0;
static void check(const char *name, int cond) {
        printf("%-44s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

int main(void) {
        const char *home = getenv("HOME");
        char base[256], path[512];
        snprintf(base, sizeof base, "%s/datasets/euroc/MH_01_easy/mav0/cam0", home);

        snprintf(path, sizeof path, "%s/data.csv", base);
        cam_frame_t *cam;
        size_t nc = euroc_load_cam(path, &cam);
        check("frontend: cam index loads", nc > 0);
        if (!nc) { printf("FAIL (1)\n"); return 1; }

        FILE *out = fopen("/tmp/tracks.txt", "w");

        frontend_t fe;
        frontend_init(&fe);

        const int N = 40;
        int min_pop = 1 << 30;

        for (int k = 0; k < N; k++) {
                snprintf(path, sizeof path, "%s/data/%s", base, cam[k].filename);
                image_t img;
                if (image_load(path, &img) != 0) { check("frontend: frame loads", 0); break; }

                int before = fe.n;
                frontend_process(&fe, &img);

                if (k > 0) {
                        int surv = 0;
                        for (int i = 0; i < fe.n; i++) if (fe.f[i].age > 0) surv++;
                        if (surv < min_pop) min_pop = surv;
                        printf("  frame %2d: %3d in, %3d survived, %3d total\n",
                               k, before, surv, fe.n);
                }

                for (int i = 0; i < fe.n; i++)
                        fprintf(out, "%d %d %.2f %.2f\n", k, fe.f[i].id, fe.f[i].pt.x, fe.f[i].pt.y);

                image_free(&img);
        }
        fclose(out);

        check("frontend: population maintained (>150)", fe.n > 150);
        check("frontend: tracks survive between frames", min_pop > 100);

        int veterans = 0;
        for (int i = 0; i < fe.n; i++) if (fe.f[i].age >= N - 1) veterans++;
        printf("  veterans (alive since frame 0): %d\n", veterans);
        check("frontend: some features live 40 frames", veterans > 20);

        frontend_free(&fe);
        free(cam);
        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

