#pragma once

#include "ofMain.h"
#include "ofxSick.h"
#include "ofxFps.h"
#include "ofxRay.h"
#include "Quadrilateral.h"
#include "ofxUI.h"
#include "ofxTuioServer.h"


class ofApp : public ofBaseApp {
public:
    void setup();
    void exit();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void uiEvent(ofxUIEventArgs & args);
    
    void blobAdded(ofxSick::TrackedBlob & blob);
    void blobUpdated(ofxSick::TrackedBlob & blob);
    void blobRemoved(ofxSick::TrackedBlob & blob);
    
    ofVec3f & screenToWorld(float x, float y);
    
    bool showDebug;
    int view;
    
    ofxSick::Lms100 lidar;
    ofxSick::Plot plot;
    bool flipHoriz;
    bool showLines;
    float pointSize;
    float dataOffset;

    ofEasyCam cam;
    vector<ofVec2f> points;
    vector<ofVec2f> mpoints;
    vector<ofColor> colors;
    bool showPoints;

    ofxSick::Blobs blobs;
    ofxSick::Blobs mblobs;
    ofxSick::Tracker tracker;
    bool showBlobs;
    float blobThreshold;
    float blobMassMin;
    float blobMassMax;
    float blobGroupDist;
    float blobTrackTolerance;
    bool filterOutside;
    float blobOffset;
    
    Quadrilateral quad;
    vector<ofVec2f> area;
    int currentCorner;
    bool showArea;
    bool showDims;
    ofVec3f pointOnPlane;
    ofVec2f pointInArea;
    
    ofxFps fps;
    int scanCount;
    
    ofxUICanvas *uiLidar;
    ofxUICanvas *uiPlot;
    ofxUICanvas *uiBlobs;
    ofxUICanvas *uiMapping;
    //ofxUICanvas *uiBrush;
    vector<ofxUICanvas*> canvases;
    
    ofxUILabel *uiAngleRes;
    ofxUILabel *uiMeasFreq;
    
    /*ofFbo fbo;
    float brushSize;
    float brushOpacity;
    ofTexture texFront, texBack;
    ofShader shaderMask;
    bool showImages;*/
    
    ofxTuioServer tuio;
    map<int, TuioObject*> cursors;
};
