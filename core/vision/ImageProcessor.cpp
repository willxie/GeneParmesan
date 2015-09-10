#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = new BeaconDetector(DETECTOR_PASS_ARGS);
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_, *abs_parts = vblocks_.body_model->abs_parts_;
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->img_top_;
  return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(RobotCalibration calibration){
  if(calibration_) delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

struct ImageProcessor::RunLength {
	unsigned char color;
	int x_left;
	int x_right;
	RunLength* parent = this;
};

void ImageProcessor::computeRunLength(std::vector<std::vector<RunLength> >& rows) {
	/* Compute Run Length Encoding */
	// Process from left to right
	// Process from top to bottom
	// NOTE: Skip rows because the image coming from getSegImg() is already downsampled
	for (int y = 0; y < 240; y += 2) {
		rows.push_back(std::vector<RunLength>());
		std::vector<RunLength>& row = rows.back();
		RunLength curRunLength;
		curRunLength.x_left = 0;
		curRunLength.color = getSegImg()[y * iparams_.width + 0];
		for (int x = 4; x < 320; x += 4) {
			// Retrieve the segmented color of the pixel at (x,y)
			unsigned char c = getSegImg()[y * iparams_.width + x];
			// End of current run length?
			if (curRunLength.color != c) {
				curRunLength.x_right = (x/4)-1;
				row.push_back(curRunLength);

				// Create new run length
				curRunLength.x_left = x/4;
				curRunLength.color = c;
			}
		}
		curRunLength.x_right = (320 / 4) - 1; // Last row index of downsampled frame
		row.push_back(curRunLength);
	}

	// Print out RLE rows
	int r = 0, row_width = 0, num_rows = 0;
	for (auto& row : rows) {
		printf("===> Row #%d <===\n", r);
		for (auto& runLength : row) {
			auto length = (runLength.x_right - runLength.x_left) + 1;
			printf("L:%dC:%d ", length, runLength.color);
			row_width += length;
		}
		printf("Row Width: %d\n", row_width);
		row_width = 0;
		num_rows++;
		printf("\n");
		r++;
	}
	printf("Num rows: %d\n", num_rows);
}

// Given a candidate RunLength, find the uppermost parent (root of the tree)
ImageProcessor::RunLength* ImageProcessor::findRunLengthGrandParent(RunLength& topRunLength) {
	if (topRunLength.parent == &topRunLength) {
		return &topRunLength;
	}
	return findRunLengthGrandParent(*topRunLength.parent);
}

void ImageProcessor::unionFind(std::vector<std::vector<RunLength> >& rows) {
	// Skip first row
	for (int y = 1; y < rows.size(); ++y) {
		for (int x = 0; x < 80; ++x) {
			int x_top = 0;
			// Center on the bottom row and try to find the parent for each RunLength
			for (RunLength curRunLength : rows[y]) {
				RunLength& topRunLength = rows[y - 1][x_top];
				// Check if the topRunLength is connected to curRunLength
				if (curRunLength.x_left <= topRunLength.x_left  &&
					topRunLength.x_left <= curRunLength.x_right &&
					topRunLength.x_left <= curRunLength.x_left  &&
					curRunLength.x_left <= topRunLength.x_right) {
					// Check for color
					if (curRunLength.color == topRunLength.color) {
						// Check if curRunLength is already set
						if (curRunLength.parent == &curRunLength) {
							curRunLength.parent = findRunLengthGrandParent(topRunLength);
						} else {
							// If the curRunLength.parent is already set, it means
							// that this is the second overlapped region
							findRunLengthGrandParent(topRunLength)->parent = curRunLength.parent;
						}
					}
					x_top++;
				} else {
					// We want to go back to the last connected topRunLength because
					// the next curRunLength might connect with it as well
					x_top--;
				}
			}
		}
	}
}

void ImageProcessor::processFrame(){
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  visionLog(30, "Process Frame camera %i", camera_);

  updateTransform();

  // Horizon calculation
  visionLog(30, "Calculating horizon line");
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 30000);
  vblocks_.robot_vision->horizon = horizon;
  visionLog(30, "Classifying Image", camera_);
  if(!classifier_->classifyImage(color_table_)) return;

  // Compute Run Lengths
  // Note that the dimension of rows is 120 x 80
  std::vector<std::vector<RunLength> > rows;
  computeRunLength(rows);

  // Link RunLengths into regions with the same parent
  unionFind(rows);

  detectBall();
  beacon_detector_->findBeacons();
}

void ImageProcessor::detectBall() {
  int imageX, imageY;

  // Try to find goal every frame
  WorldObject* goal = &vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
  WorldObject* circle = &vblocks_.world_object->objects_[WO_CENTER_CIRCLE];
  if (findGoal(goal->imageCenterX, goal->imageCenterY, circle->imageCenterY)) {
    goal->seen = true;
  }
  ///////////////////////////////

  if(!findBall(imageX, imageY)) return; // function defined elsewhere that fills in imageX, imageY by reference
  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  ball->seen = true;
}

bool ImageProcessor::findBall(int& imageX, int& imageY) {
	int total = 0, totalX = 0, totalY = 0;
	int c_temp;

	// Process from left to right
	for(int x = 0; x < 320; x++) {
	  // Process from top to bottom
	  for(int y = 0; y < 240; y++) {
	    // Retrieve the segmented color of the pixel at (x,y)
	    auto c = getSegImg()[y * iparams_.width + x];
	    c_temp = c;
	    if(c == c_ORANGE) {
	      totalX += x;
	      totalY += y;
	      total++;
	    }
	  }
	}

	if (total > 0) {
		imageX = totalX / total;
		imageY = totalY / total;
	}
//	printf("c_ORANGE = %d,\t c_temp = %d\n", (int)c_ORANGE, (int)c_temp);
//	printf("total orange pixels: %d, \t %d, \t %d, \n", total, imageX, imageY);

	return (total > 10);
}

bool ImageProcessor::findGoal(int& imageX, int& imageY, int& numBluePixels) {
	int total = 0, totalX = 0, totalY = 0;
	int c_temp;

	// Process from left to right
	for(int x = 0; x < 320; x++) {
	  // Process from top to bottom
	  for(int y = 0; y < 240; y++) {
	    // Retrieve the segmented color of the pixel at (x,y)
	    auto c = getSegImg()[y * iparams_.width + x];
	    c_temp = c;
	    if(c == c_BLUE) {
	      totalX += x;
	      totalY += y;
	      total++;
	    }
	  }
	}

	numBluePixels = total;

	if (total > 0) {
		imageX = totalX / total;
		imageY = totalY / total;
	}
//	printf("c_ORANGE = %d,\t c_temp = %d\n", (int)c_ORANGE, (int)c_temp);
	printf("total blue pixels: %d, \t %d, \t %d, \n", total, imageX, imageY);

	return (total > 400);
}

int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
