#ifndef _FILTERS_H
#define _FILTERS_H

// #define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086

#include <string.h>
#include <cmath>

class Filter {

    public:
        Filter(int order, float fCutoff, float TSample, int type);
        Filter(int order, float fCutoff, float TSample, int type, float v0);
        Filter();

        float get_measurement();
        float get_filtered();

        void reset();
        void reset(float state);

        float b0;
        float b1;
        float b2;

        float a1;
        float a2;

        float update(float measurement);

    private:
        int order;
        float fCutoff;
        float TSample;
        int type;

        float m0;
        float m1;
        float m2;

        float f0;
        float f1;
        float f2;

        void LPF();
        void HPF();

};
#endif