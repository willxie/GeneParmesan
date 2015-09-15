#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <vision/Classifier.h>
#include <common/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>
#include <unordered_map>

class BeaconDetector;

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct RunLength;
    struct Blob;
    struct Beacon;
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    Classifier* classifier_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(RobotCalibration);
    void computeRunLength(std::vector<std::vector<RunLength> >& rows);
    RunLength* findRunLengthGrandParent(RunLength* topRunLength);
    void unionFind(std::vector<std::vector<RunLength> >& rows);
    void computeBlobs(std::vector<std::vector<RunLength> >& rows, std::unordered_map<RunLength*, Blob>& blobs);
    bool findBeacon(std::vector<Blob>& blobs, WorldObjectType beacon_type, unsigned char top_color, unsigned char bottom_color, Beacon& beacon);
    double calculateAspectRatio(Blob& blob);
    double calculateDensity(Blob& blob);
    bool findGoal(std::vector<Blob>& blob_list);
    bool findBall(std::vector<Blob>& blob_list);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate*> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
    void detectBall();
    bool findBall(int& imageX, int& imageY);
    bool findGoal(int& imageX, int& imageY, int& numBluePixels);
  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    BallCandidate* ball_candidate_;

    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;

    RobotCalibration* calibration_;
    bool enableCalibration_;
    BeaconDetector* beacon_detector_;
};

#endif
