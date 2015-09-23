//
//  Quadrilateral.h
//
//  Created by Tobias Ebsen on 13/07/15.
//
//

#pragma once

#include "ofMain.h"

class Quadrilateral {
public:
    Quadrilateral();
    
    void setCorner(int i, ofVec2f & corner) {
        setCorner(i, corner.x, corner.y);
    }
    void setCorner(int i, ofVec3f & corner) {
        setCorner(i, corner.x, corner.y);
    }
    void setCorner(int i, float x, float y);
    
    ofVec2f getMapped(ofVec3f & p) {
        return getMapped(p.x, p.y);
    }
    ofVec2f getMapped(ofVec2f & p) {
        return getMapped(p.x, p.y);
    }
    ofVec2f getMapped(float x, float y);

    bool inside(ofVec2f & point) {
        ofVec2f m = getMapped(point.x, point.y);
        return m.x >= 0. && m.x <= 1. && m.y >= 0. && m.y <= 1.;
    }
    
private:
    ofVec2f corners[4];
    ofMatrix4x4 AI;
    ofVec4f px,py;
    ofVec4f a,b;
};
