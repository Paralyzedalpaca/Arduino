#include <filters.h>

Filter::Filter(int order, float fCutoff, float TSample, int type)
    : order(order), fCutoff(fCutoff), TSample(TSample), type(type) {

        m0 = m1 = m2 = 0;
        f0 = f1 = f2 = 0;

        if (type == 0) {
            LPF();
        } else if (type == 1){
            HPF();
        }
    }

Filter::Filter(int order, float fCutoff, float TSample, int type, float v0)
    : order(order), fCutoff(fCutoff), TSample(TSample), type(type) {

        m0 = m1 = m2 = v0;
        f0 = f1 = f2 = v0;

        if (type == 0) {
            LPF();
        } else if (type == 1){
            HPF();
        }
    }

Filter::Filter() {}

void Filter::reset() {
    m0 = m1 = m2 = 0;
    f0 = f1 = f2 = 0;
}

void Filter::reset(float state) {
    m0 = m1 = m2 = state;
    f0 = f1 = f2 = state;
}

void Filter::LPF() {
    float wn = fCutoff * TSample;
    float ita = 1.0 / tan(3.14*wn);

    if (order == 2) {
        b0 =  1.0 / (1.0 + sqrt(2)*ita + ita*ita);
        b1 = 2 * b0;
        b2 = b0;

        a1 = -2.0 * (ita*ita - 1.0) * b0;
        a2 = (1.0 - sqrt(2)*ita + ita*ita) * b0;
    }
}

void Filter::HPF() {
    float wn = fCutoff * TSample;
    
    if (order == 2) {
        float W = tan(3.14 * wn);
        float K = W*W;
        float Q = 1;

        float alpha = 1 + K / Q + W;

        b0 = 1 / alpha;
        b1 = -2 / alpha;
        b2 = b0;

        a1 = 2 * (W - 1) / alpha;
        a2 = (1 - K + W) / alpha;
    }
    b0 = 0.998668023;
    b1 = -2 * b0;
    b2 = b0;

    a1 = -1.997334272;
    a2 = 0.9973378201;
}

float Filter::update(float m) {
    if (order == 2) {
        m2 = m1;
        m1 = m0;
        m0 = m;

        f2 = f1;
        f1 = f0;
        f0 = b0*m0 + b1*m1 + b2*m2 - a1*f1 - a2*f2;
    }

    return f0;
}

float Filter::get_measurement() {
    return m0;
}

float Filter::get_filtered() {
    return f0;
}