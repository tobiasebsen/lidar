#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    ofLogNotice() << "Connecting to LIDAR ...";
    ofLogVerbose("Connect") << lidar.connect("192.168.0.1", 2111);
    
    string name, version;
    ofLogVerbose("GetVersionString") << lidar.GetVersionString(name, version);
    ofLogVerbose("Name") << name;
    ofLogVerbose("Version") << version;
    
    lidar.start();
    ofAddListener(tracker.addBlob, this, &ofApp::blobAdded);
    ofAddListener(tracker.updateBlob, this, &ofApp::blobUpdated);
    ofAddListener(tracker.removeBlob, this, &ofApp::blobRemoved);
    
    tuio.start((char*)"127.0.0.1", 3333);
    
    cam.setupPerspective(false, 60, 0, 40000);
    cam.setDistance(3000.);
    cam.setGlobalPosition(0, 800, 3000);
    
    uiLidar = new ofxUICanvas(0, 0, 190, 200);
    uiLidar->addLabel("LIDAR");
    uiLidar->addSpacer();
    uiLidar->addLabel("Device: " + name)->setFont(uiLidar->getFontSmall());
    uiLidar->addLabel("Version: " + version)->setFont(uiLidar->getFontSmall());
    uiAngleRes = uiLidar->addLabel("Angle step: -");
    uiAngleRes->setFont(uiLidar->getFontSmall());
    uiMeasFreq = uiLidar->addLabel("Measure freq: -");
    uiMeasFreq->setFont(uiLidar->getFontSmall());
    uiLidar->addSpacer();
    uiLidar->addLabelButton("set config", false);
    uiLidar->addLabelButton("write all", false);
    uiLidar->autoSizeToFitWidgets();
    ofAddListener(uiLidar->newGUIEvent, this, &ofApp::uiEvent);
    canvases.push_back(uiLidar);
    
    uiPlot = new ofxUICanvas(200, 0, 190, 200);
    uiPlot->addLabel("PLOT");
    uiPlot->addSpacer();
    uiPlot->addToggle("flip horizontal", &flipHoriz);
    uiPlot->addSlider("point size", 1., 5., &pointSize);
    uiPlot->addToggle("lines", &showLines);
    uiPlot->addSlider("data offset", -140., 140., &dataOffset);
    uiPlot->loadSettings("plot.xml");
    uiPlot->autoSizeToFitWidgets();
    canvases.push_back(uiPlot);
    
    uiBlobs = new ofxUICanvas(400, 0, 190, 200);
    uiBlobs->addLabel("BLOBS");
    uiBlobs->addSpacer();
    uiBlobs->addToggle("show", &showBlobs);
    uiBlobs->addSlider("blob thres", 0., 100., &blobThreshold); blobThreshold = 60.;
    uiBlobs->addSlider("min mass", 1., 20., &blobMassMin);
    uiBlobs->addSlider("max mass", 1., 1000., &blobMassMax);
    uiBlobs->addSlider("group distance", 0., 200., &blobGroupDist);
    uiBlobs->addToggle("filter outside area", &filterOutside); filterOutside = false;
    uiBlobs->addSlider("tracking tolerance", 1., 200., &blobTrackTolerance);
    uiBlobs->loadSettings("blobs.xml");
    uiBlobs->autoSizeToFitWidgets();
    canvases.push_back(uiBlobs);
    
    uiMapping = new ofxUICanvas(600, 0, 190, 200);
    uiMapping->addLabel("MAPPING");
    uiMapping->addSpacer();
    uiMapping->addToggle("show", &showArea);
    uiMapping->addToggle("dimensions", &showDims);
    uiMapping->loadSettings("mapping.xml");
    uiMapping->autoSizeToFitWidgets();
    canvases.push_back(uiMapping);
    
    for (auto & canvas : canvases) {
        canvas->setColorBack(ofxUIColor(128, 64));
        canvas->setVisible(false);
    }
    
 
    ofxXmlSettings xml("area.xml");
    xml.pushTag("corners");
    int n = xml.getNumTags("corner");
    area.resize(4);
    for (int i=0; i<MIN(n,4); i++) {
        xml.pushTag("corner", i);
        area[i].x = xml.getValue("x", 0.f);
        area[i].y = xml.getValue("y", 0.f);
        quad.setCorner(i, area[i]);
        xml.popTag();
    }
    xml.popTag();
    
    currentCorner = -1;
    view = 0;
    showDebug = false;
    showPoints = true;
}

//--------------------------------------------------------------
void ofApp::exit() {
    ofxXmlSettings xml("area.xml");
    if (!xml.tagExists("corners"))
        xml.addTag("corners");
    xml.pushTag("corners");
    for (int i=0; i<4; i++) {
        if (!xml.tagExists("corner", i))
            xml.addTag("corner");
        xml.pushTag("corner", i);
        xml.setValue("x", area[i].x);
        xml.setValue("y", area[i].y);
        xml.popTag();
    }
    xml.popTag();
    xml.saveFile();
    
    uiPlot->saveSettings("plot.xml");
    uiBlobs->saveSettings("blobs.xml");
    uiMapping->saveSettings("mapping.xml");
}

//--------------------------------------------------------------
void ofApp::update(){

    lidar.update();
    
    if (lidar.isFrameNew()) {
        
        ofxSick::Scan & scan = lidar.getScan();
        
        fps.begin();
        
        plot.update(scan, flipHoriz, false, dataOffset, true);
        
        uiMeasFreq->setLabel("Measure freq: " + ofToString(scan.MeasurementParam1Block.udiScanFreq * 0.01) + "Hz");
        uiAngleRes->setLabel("Angle step: " + ofToString(scan.aDataChannel16.aFlexArrayData[0].DataChannelHdr.uiAngleRes * 0.0001, 2));

        ofVec2f window(ofGetWidth(), ofGetHeight());
        
        blobs.update(plot, blobThreshold);
        blobs.filterByMass(0., 1000.);
        blobs.groupByDistance(blobGroupDist);
        blobs.filterByMass(blobMassMin, blobMassMax);
        
        if (filterOutside) {
            mblobs.clear();
            for (int i=0; i<blobs.size(); i++) {
                if (quad.inside(blobs[i].center)) {
                    mblobs.push_back(blobs[i]);
                }
            }
            tracker.track(mblobs, blobTrackTolerance);
        }
        else
            tracker.track(blobs, blobTrackTolerance);

        vector<int> & labels = blobs.getLabels();
        colors.resize(labels.size());
        for (int i=0; i<labels.size(); i++) {
            ofColor & color = colors[i];
            if (labels[i] == -1)
                color = ofColor::gray;
            else
                color = ofColor::red;
        }
        
        fps.end();
    }

    tuio.run();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    if (view == 0) {
        cam.begin();
        
        ofNoFill();
        ofSetCircleResolution(64);
        
        for (int i=1; i<100; i++) {
            if (i % 10 == 0)
                ofSetColor(128);
            else
                ofSetColor(64);
            ofEllipse(ofPoint(), i*200, i*200);
        }
        ofVec2f line(10000,0);
        ofLine(ofVec2f::zero(), line.rotated(-45));
        ofLine(ofVec2f::zero(), line.rotated(-45+270));
        
        ofSetColor(255, 0, 0);
        glPointSize(pointSize);
        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, colors.data());
        plot.draw(showLines);
        glDisableClientState(GL_COLOR_ARRAY);
        ofSetColor(255);
        
        if (showBlobs) {
            tracker.draw();
            //blobs.draw();
        }
        
        if (showArea) {
            ofSetColor(255, 255, 0);
            glPointSize(10);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(2, GL_FLOAT, 0, area.data());
            glDrawArrays(GL_LINE_LOOP, 0, area.size());
            glDrawArrays(GL_POINTS, 0, area.size());
            glDisableClientState(GL_VERTEX_ARRAY);
            if (showDims) {
                ofVec2f bottom = area[0] - area[3];
                ofDrawBitmapString(ofToString(bottom.length()*0.1,1) + " cm", area[3] + bottom/2 + ofVec2f(0, 0));
                ofVec2f right = area[1] - area[0];
                ofDrawBitmapString(ofToString(right.length()*0.1,1) + " cm", area[0] + right/2 + ofVec2f(0, 0));
                ofVec2f top = area[2] - area[1];
                ofDrawBitmapString(ofToString(top.length()*0.1,1) + " cm", area[1] + top/2 + ofVec2f(0, 0));
                ofVec2f left = area[3] - area[2];
                ofDrawBitmapString(ofToString(left.length()*0.1,1) + " cm", area[2] + left/2 + ofVec2f(0, 0));
            }
            ofSetColor(255);
        }
        
        if (currentCorner != -1 && currentCorner < area.size()) {
            ofDrawBitmapString(ofToString(area[currentCorner]), area[currentCorner]);
        }
        
        cam.end();
    }
    if (view) {
        
        ofVec2f window(ofGetWidth(), ofGetHeight());

        if (showPoints) {
            ofFill();
            //ofDisableAlphaBlending();
            for (int i=0; i<plot.size(); i++) {
                ofVec2f point = window - quad.getMapped(plot[i]) * window;
                ofSetColor(colors[i]);
                ofEllipse(point, 10, 10);
            }
        }

        if (showBlobs) {
            ofNoFill();
            ofSetColor(255);
            for (int i=0; i<tracker.size(); i++) {
                ofxSick::TrackedBlob & blob = tracker[i];
                ofVec2f center = window - quad.getMapped(blob.center) * window;
                ofEllipse(center, blob.mass, blob.mass);
            }
        }
    }
    
    if (showDebug) {
        fps.draw(20, ofGetWindowHeight()-20, "LIDAR");
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    if (key >= '1' && key <= '2') {
        view = key - '1';
    }
    if (key == 'f' || key == 'F') {
        ofToggleFullscreen();
    }
    if (key == '\t') {
        showDebug = !showDebug;
        for (auto & canvas : canvases)
            canvas->setVisible(showDebug);
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

    pointOnPlane = screenToWorld(x, y);

    if (currentCorner != -1 && currentCorner < area.size()) {
        area[currentCorner].set(pointOnPlane);
        quad.setCorner(currentCorner, pointOnPlane);
    }
    if (currentCorner == -1 && area.size() == 4) {
        pointInArea = quad.getMapped(pointOnPlane.x, pointOnPlane.y);
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

    pointOnPlane = screenToWorld(x, y);

    if (currentCorner != -1 && currentCorner < area.size()) {
        area[currentCorner].set(pointOnPlane);
        quad.setCorner(currentCorner, pointOnPlane);
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

    for (int i=0; i<4; i++) {
        if (area[i].distance(pointOnPlane) < 20) {
            currentCorner = i;
            cam.disableMouseInput();
        }
    }
    
    for (auto & canvas : canvases) {
        if (canvas->isHit(x, y))
            cam.disableMouseInput();
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

    currentCorner = -1;
    cam.enableMouseInput();
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::uiEvent(ofxUIEventArgs &args) {
    if (args.getName() == "set config" && args.getButton()->getValue()) {
        lidar.Login(SopasLoginLevel::AUTHORIZEDCLIENT);
        CLms100SopasInterface::EScanConfigError error;
        ofLogVerbose("SetScanConfig") << lidar.SetScanConfig(2500, 2500, -4500, 225000, &error);
        lidar.Logout();
    }
    if (args.getName() == "write all" && args.getButton()->getValue()) {
        lidar.Login(SopasLoginLevel::AUTHORIZEDCLIENT);
        ofLogVerbose("SetScanConfig") << lidar.WriteAll();
        lidar.Logout();
    }
}

//--------------------------------------------------------------
void ofApp::blobAdded(ofxSick::TrackedBlob &blob) {
    ofLog() << "New blob: " << blob.uid;
    ofVec2f window(ofGetWidth(), ofGetHeight());
    ofVec2f center = window - quad.getMapped(blob.center) * window;
    cursors[blob.uid] = tuio.addObject(blob.uid, center.x, center.y, blob.mass);
}
//--------------------------------------------------------------
void ofApp::blobUpdated(ofxSick::TrackedBlob &blob) {
    ofVec2f window(ofGetWidth(), ofGetHeight());
    ofVec2f center = window - quad.getMapped(blob.center) * window;
    tuio.updateObject(cursors[blob.uid], center.x, center.y, blob.mass);
}
//--------------------------------------------------------------
void ofApp::blobRemoved(ofxSick::TrackedBlob &blob) {
    ofLog() << "Deleted blob: " << blob.uid;
    tuio.removeObject(cursors[blob.uid]);
    cursors.erase(cursors.find(blob.uid));
}

//--------------------------------------------------------------
ofVec3f & ofApp::screenToWorld(float x, float y) {
    ofVec3f screenToWorld = cam.screenToWorld(ofVec3f(x, y, 0.));
    ofxRay::Ray ray(cam.getPosition(), screenToWorld - cam.getPosition());
    ofxRay::Plane plane(ofVec3f(0,0,0), ofVec3f(0,0,1));
    plane.intersect(ray,pointOnPlane);
    return pointOnPlane;
}

