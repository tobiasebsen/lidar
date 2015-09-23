//
//  Quadrilateral.cpp
//
//  Created by Tobias Ebsen on 13/07/15.
//
//

#include "Quadrilateral.h"

Quadrilateral::Quadrilateral()  {
    ofMatrix4x4 A(1,0,0,0,
                  1,1,0,0,
                  1,1,1,1,
                  1,0,1,0);
    AI = A.getInverse();
}

void Quadrilateral::setCorner(int i, float x, float y) {
    corners[i].set(x, y);
    px[i] = x;
    py[i] = y;
    a = AI.postMult(px);
    b = AI.postMult(py);
}

ofVec2f Quadrilateral::getMapped(float x, float y) {

    // Inverse bilinear interpolation
    // https://www.particleincell.com/2012/quad-interpolation/
    // http://www.iquilezles.org/www/articles/ibilinear/ibilinear.htm
    
    float aa = a[3]*b[2] - a[2]*b[3];
    float bb = a[3]*b[0] - a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + x*b[3] - y*a[3];
    float cc = a[1]*b[0] - a[0]*b[1] + x*b[1] - y*a[1];

    // 0 = aa*m*m + bb*m + cc
    float m = (-bb+sqrt(bb*bb - 4*aa*cc))/(2*aa);
    float l = (x-a[0]-a[2]*m)/(a[1]+a[3]*m);
    
    return ofVec2f(m, l);
}
