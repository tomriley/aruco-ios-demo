//
//  ArucoWrapper.mm
//  ArucoDemo
//
//  Created by Carlo Rapisarda on 20/06/2018.
//  Copyright © 2018 Carlo Rapisarda. All rights reserved.
//

#import "ArucoWrapper.h"
#import "Utilities.h"

#import <opencv2/imgproc/imgproc.hpp>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/core/core.hpp>
#import <opencv2/calib3d.hpp>
#import <aruco.h>
#import <glm/vec3.hpp>
#import <glm/vec4.hpp>
#import <glm/mat4x4.hpp>
#import <glm/gtc/matrix_transform.hpp>
#import <glm/ext.hpp>
#import <glm/gtx/transform.hpp>
#import <glm/gtx/matrix_decompose.hpp>


#pragma mark - Constants

ArucoPreviewRotationType const ArucoPreviewRotationTypeNone = -1;
ArucoPreviewRotationType const ArucoPreviewRotationTypeCw90 = cv::ROTATE_90_CLOCKWISE;
ArucoPreviewRotationType const ArucoPreviewRotationTypeCw180 = cv::ROTATE_180;
ArucoPreviewRotationType const ArucoPreviewRotationTypeCw270 = cv::ROTATE_90_COUNTERCLOCKWISE;

/**
 Given model view matrix, return camera position. only works if matrix doesn't perform scaling operations.
 */
glm::vec3 extractCameraPosNoScale(const glm::mat4& modelview)
{
    glm::mat3 rotation(modelview);
    glm::vec3 d(modelview[3]);
    return -d * rotation;
}


#pragma mark - ArucoMarker

@implementation ArucoMarker

-(id _Nullable) initWithCMarker:(aruco::Marker)cmarker {
    if (!(cmarker.isValid() && cmarker.isPoseValid())) {
        return nil;
    } else if (self = [super init]) {
        self.identifier = cmarker.id;
        self.poseRX = cmarker.Rvec.at<float>(0);
        self.poseRY = cmarker.Rvec.at<float>(1);
        self.poseRZ = cmarker.Rvec.at<float>(2);
        self.poseTX = cmarker.Tvec.at<float>(0);
        self.poseTY = cmarker.Tvec.at<float>(1);
        self.poseTZ = cmarker.Tvec.at<float>(2);
        
        auto rvec = cmarker.Rvec;
        auto tvec = cmarker.Tvec;
        
        // convert to opengl axis layout
        rvec.at<float>(1) = -rvec.at<float>(1);
        rvec.at<float>(2) = -rvec.at<float>(2);
        tvec.at<float>(1) = -tvec.at<float>(1);
        tvec.at<float>(2) = -tvec.at<float>(2);
        
        cv::Mat rm;
        cv::Rodrigues(rvec, rm);
        
        // extract the camera position from the modelview matrix. this is easy because when there is no scaling involved
        glm::mat4 modelview = glm::mat4(rm.at<float>(0), rm.at<float>(3), rm.at<float>(6), 0.0,
                                        rm.at<float>(1), rm.at<float>(4), rm.at<float>(7), 0.0,
                                        rm.at<float>(2), rm.at<float>(5), rm.at<float>(8), 0.0,
                                        tvec.at<float>(0), tvec.at<float>(1), tvec.at<float>(2), 1.0);
        
        // the aruco library gives us the pose at a 180 degree rotation in the X axis as
        // compared with our chessboard pose estimation, so rotate here
        modelview = glm::rotate(modelview, glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        
        glm::mat3 rotation(modelview); // truncate to just the rotation
        glm::mat4 invRotation = glm::transpose(rotation);
        glm::vec3 trans = modelview[3]; // just translation
        
        glm::vec3 eye = glm::vec4(-trans, 1) * rotation;
        glm::vec4 dir = invRotation * glm::vec4(0, 0, -1, 1);
        glm::vec4 up = invRotation * glm::vec4(0, 1, 0, 1);
        
        self.eyeX = eye.x;
        self.eyeY = eye.y;
        self.eyeZ = eye.z;
        
        self.dirX = dir.x;
        self.dirY = dir.y;
        self.dirZ = dir.z;
        
        self.upX = up.x;
        self.upY = up.y;
        self.upZ = up.z;
    }
    return self;
}

@end


#pragma mark - ArucoTracker

@interface ArucoTracker()
@property std::map<uint32_t, aruco::MarkerPoseTracker> *mapOfTrackers;
@property aruco::MarkerPoseTracker *tracker;
@property aruco::MarkerDetector *detector;
@property aruco::CameraParameters *camParams;
@property CIContext *ciContext;
@property BOOL setupDone;
@end

@implementation ArucoTracker

-(id _Nonnull) initWithCalibrationFile:(NSString *_Nonnull)calibrationFilepath delegate:(_Nonnull id<ArucoTrackerDelegate>)delegate {
    if (self = [super init]) {
        self.mapOfTrackers = new std::map<uint32_t, aruco::MarkerPoseTracker>();
        self.tracker = new aruco::MarkerPoseTracker();
        self.detector = new aruco::MarkerDetector();
        self.camParams = new aruco::CameraParameters();
        self.markerSize = 0.0902;
        self.outputImages = true;
        self.previewRotation = -1;
        self.ciContext = [[CIContext alloc] init];
        self.setupDone = false;
        self.delegate = delegate;
        [self readCalibration:calibrationFilepath];
    }
    return self;
}

-(void) readCalibration:(NSString *)filepath {
    self.camParams->readFromXMLFile([filepath UTF8String]);
}

-(void) prepareForOutput:(AVCaptureVideoDataOutput *)videoOutput orientation:(AVCaptureVideoOrientation)orientation {

    dispatch_queue_t queue = dispatch_queue_create("video-sample-buffer", nil);
    [videoOutput setSampleBufferDelegate:self queue:queue];
    [videoOutput setAlwaysDiscardsLateVideoFrames:true];

    [videoOutput setVideoSettings:@{
        (__bridge NSString *)kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_32BGRA)
    }];

    AVCaptureConnection *conn = [videoOutput connectionWithMediaType:AVMediaTypeVideo];

    if (conn.isVideoMirroringSupported && conn.isVideoOrientationSupported) {
        [conn setVideoOrientation:orientation];
    } else {
        [NSException raise:@"DeviceNotSupported" format:@"Device does not support one or more required features"];
    }

    self.setupDone = false;
}

-(void) captureOutput:(AVCaptureOutput *)output didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection {

    cv::Mat bgraImage = BufferToMat(sampleBuffer);

    cv::Mat colorImage, grayImage;
    cv::cvtColor(bgraImage, colorImage, cv::COLOR_BGRA2RGB);
    cv::cvtColor(colorImage, grayImage, cv::COLOR_RGB2GRAY);

    if (!self.setupDone) {
        self.camParams->CamSize = colorImage.size();
        self.setupDone = true;
    }

    auto mapOfTrackers = *self.mapOfTrackers;
    bool hasValidCamParams = self.camParams->isValid();
    std::vector<aruco::Marker> markers = self.detector->detect(grayImage, *self.camParams, self.markerSize);

    NSMutableArray *result = [NSMutableArray new];

    for (auto& m : markers) {
        mapOfTrackers[m.id].estimatePose(m, *self.camParams, self.markerSize);
        ArucoMarker *markerObj = [[ArucoMarker alloc] initWithCMarker:m];
        if (markerObj != nil) [result addObject:markerObj];
    }

    UIImage *preview = nil;

    if (self.outputImages) {

        for (auto& m : markers) {
            m.draw(colorImage, cv::Scalar(0, 0, 255), 2);
            if (hasValidCamParams && m.isPoseValid()) {
                aruco::CvDrawingUtils::draw3dCube(colorImage, m, *self.camParams);
                aruco::CvDrawingUtils::draw3dAxis(colorImage, m, *self.camParams);
            }
        }

        if (_previewRotation >= 0) {
            cv::rotate(colorImage, colorImage, _previewRotation);
        }

        preview = MatToUIImage(colorImage);
    }

    [self.delegate arucoTracker:self didDetectMarkers:result preview:preview];
}

@end


#pragma mark - ArucoGenerator

@interface ArucoGenerator()
@property std::string dictionaryName;
@property aruco::Dictionary dictionary;
@end

@implementation ArucoGenerator

-(id _Nonnull) initWithDictionary:(NSString *_Nonnull)dictionaryName {
    if (self = [super init]) {
        self.dictionaryName = [dictionaryName UTF8String];
        self.dictionary = aruco::Dictionary::load(self.dictionaryName);
        self.enclosingCorners = false;
        self.waterMark = true;
        self.pixelSize = 75;
    }
    return self;
}

-(id _Nonnull) init {
    return [[ArucoGenerator alloc] initWithDictionary:@"ARUCO"];
}

-(UIImage *) generateMarkerImage:(int)markerID {
    cv::Mat markerImage = self.dictionary.getMarkerImage_id(markerID, self.pixelSize, self.waterMark, self.enclosingCorners);
    return MatToUIImage(markerImage);
}

@end


#pragma mark - ArucoDetector

@interface ArucoDetector()
@property aruco::MarkerDetector *detector;
@property aruco::CameraParameters *camParams;
@end

@implementation ArucoDetector

-(id _Nonnull) init {
    if (self = [super init]) {
        self.detector = new aruco::MarkerDetector();
        self.camParams = new aruco::CameraParameters();
        self.markerSize = 0.04;
    }
    return self;
}

-(NSArray<ArucoMarker *> *) detect:(UIImage *)image {

    cv::Mat colorImage, grayImage;
    UIImageToMat(image, colorImage);
    cvtColor(colorImage, grayImage, cv::COLOR_BGR2GRAY);
    std::vector<aruco::Marker> markers = self.detector->detect(grayImage, *self.camParams, self.markerSize);
    NSMutableArray *result = [NSMutableArray new];

    for (auto& m : markers) {
        ArucoMarker *markerObj = [[ArucoMarker alloc] initWithCMarker:m];
        if (markerObj != nil) [result addObject:markerObj];
    }

    return result;
}

-(UIImage *) drawMarkers:(UIImage *)image {

    cv::Mat colorImage, grayImage;
    UIImageToMat(image, colorImage);
    cvtColor(colorImage, grayImage, cv::COLOR_BGR2GRAY);
    std::vector<aruco::Marker> markers = self.detector->detect(grayImage, *self.camParams, self.markerSize);

    for (auto& m : markers) {
        m.draw(colorImage, cv::Scalar(0,0,255), 2);
    }

    return MatToUIImage(colorImage);
}

@end
