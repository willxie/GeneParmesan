#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <iostream>


struct ImageProcessor::RunLength {
	unsigned char color = -1;
	int x_left;
	int x_right;
	RunLength* parent = this;
};

struct ImageProcessor::Blob {
	/* Bounding box coordinates */
	unsigned char color;
	int area;
	int top;
	int bottom;
	int left;
	int right;
};

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

void ImageProcessor::computeRunLength(std::vector<std::vector<RunLength> >& rows) {
	/* Compute Run Length Encoding */
	// Process from left to right
	// Process from top to bottom
	// NOTE: Skip rows because the image coming from getSegImg() is already downsampled
	for (int y = 0; y < 240; y += 2) {
		int yy = y / 2;
		// Get the current row
		std::vector<RunLength>& row = rows[yy];
		unsigned char c_prev = -1;

		for (int x = 0; x < 320; x += 4) {
			int xx = x / 4;
			// Retrieve the segmented color of the pixel at (x,y)
			unsigned char c = getSegImg()[y * iparams_.width + x];
			// Color change, create new RunLength
			if (c != c_prev) {
				// Get down sampled index
				// Add new RunLength
//				RunLength tempRunLength;
				row.emplace_back();
				RunLength& curRunLength = row.back();

				curRunLength.parent = &curRunLength;
				curRunLength.x_left = xx;
				curRunLength.x_right = xx;
				curRunLength.color = c;
//				printf("runlength: %p\n", &(rows[yy].back()));
//				printf("runlength->parent: %p\n", rows[yy].back().parent);
//				printf("row size: %d\n", rows[yy].size());
			} else {
				RunLength& curRunLength = row.back();
				// Same color, just update x_right
				curRunLength.x_right = xx;
			}
			c_prev = c;

		}
	}

//	// Print out RLE rows
//	int r = 0, row_width = 0, num_rows = 0;
//	for (auto& row : rows) {
//		printf("===> Row #%d <===\n", r);
//		for (auto& runLength : row) {
//			auto length = (runLength.x_right - runLength.x_left) + 1;
//			printf("L:%dC:%d ", length, runLength.color);
//			row_width += length;
//		}
//		printf("Row Width: %d\n", row_width);
//		row_width = 0;
//		num_rows++;
//		printf("\n");
//		r++;
//	}
//	printf("Num rows: %d\n", num_rows);

	// Link parents
	// TODO this is a very hacky way of doing it. I don't know why it didn't work for setting parents up top
	for (auto& row : rows) {
		for (auto& rl : row) {
			RunLength* rp = &rl;
			rp->parent = rp;
//			printf("----------\n");
//			printf("runlength: %p\n", rp);
//			printf("runlength->parent: %p\n", rp->parent);
//			printf("----------\n");
		}
	}

//	for (int i = 0; i < rows.size(); ++i) {
//		for (int j = 0; j < rows[i].size(); ++j) {
//			RunLength& rp = rows[i][j];
//			printf("----------\n");
//			printf("runlength: %p\n", &rp);
//			printf("runlength->parent: %p\n", rp.parent);
//			printf("----------\n");
//		}
//	}
//
//	while(1);
}

// Given a candidate RunLength, find the uppermost parent (root of the tree)
ImageProcessor::RunLength* ImageProcessor::findRunLengthGrandParent(RunLength* topRunLength) {
//	printf("find 1\n");
	if (topRunLength->parent == topRunLength) {
//		printf("find 2\n");

		return topRunLength;
	}
//	printf("find 3\n");

	return findRunLengthGrandParent(topRunLength->parent);
}

void ImageProcessor::unionFind(std::vector<std::vector<RunLength> >& rows) {
	// Skip first row
	for (int y = 1; y < rows.size(); ++y) {
		int x_top = 0;
//		printf("y = %d\n", y);
		// Center on the bottom row and try to find the parent for each RunLength
		for (RunLength& curRunLength : rows[y]) {
			while (true) {
//				printf("rows[%d - 1].size() = %d\n", y, rows[y - 1].size());
//				printf("x_top = %d\n", x_top);
				RunLength& topRunLength = rows[y - 1][x_top];
				// Check if the topRunLength is connected to curRunLength
//				if ((curRunLength.x_left <= topRunLength.x_left && topRunLength.x_left <= curRunLength.x_right) ||
//						(topRunLength.x_left <= curRunLength.x_left && curRunLength.x_left <= topRunLength.x_right)) {
					// Check for color
					if (curRunLength.color == topRunLength.color) {
						// Is this the first adjacent run length with the same color
						// encountered
						if (curRunLength.parent == &curRunLength) {
							curRunLength.parent = findRunLengthGrandParent(&topRunLength);
						} else {
							// If the curRunLength.parent is already set, it means
							// that this is the second overlapped region
							findRunLengthGrandParent(&topRunLength)->parent = curRunLength.parent;
						}
					}

					// If next top run is guaranteed to be disconnected, move on
					if (topRunLength.x_right >= curRunLength.x_right) {
						break;
					}

					x_top++;
//				}
			}
		}
	}
}


void ImageProcessor::computeBlobs(std::vector<std::vector<RunLength> >& rows, std::unordered_map<RunLength*, Blob>& blobs) {
	// From the run lengths, fill in blobs
	int row = 0;
	for (auto& runLengthRow : rows) {
		for (auto& runLength : runLengthRow) {
			// Is this run length its own parent?
			if (runLength.parent == &runLength) {
				// Create a new blob and add it to the rest
				Blob b;
				b.area = runLength.x_right - runLength.x_left + 1;
				b.top = b.bottom = row;
				b.left = runLength.x_left;
				b.right = runLength.x_right;

				blobs[&runLength] = b;
			} else {

				// This run length has a parent. Find it and update the
				// associated blob
//				RunLength *current = &runLength;
				RunLength *parent = findRunLengthGrandParent(runLength.parent);

//				= runLength.parent;
//				while (current != parent) {
//					current = parent;
//					parent = parent->parent;
//				}

				// Retrieve the blob associated with this parent
				Blob& b = blobs[parent];
				b.area += runLength.x_right - runLength.x_left + 1;
				b.color = runLength.color;
				b.left = min(runLength.x_left, b.left);
				b.right = max(runLength.x_right, b.right);
				b.bottom = max(row, b.bottom);
				b.top = min(row, b.top);
			}
		}
		row++;
	}
}

void ImageProcessor::findBeacon() {

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
  printf("Building RunLengths...\n");
  std::vector<std::vector<RunLength> > rows (120, std::vector<RunLength>());
  computeRunLength(rows);

  // Link RunLengths into regions with the same parent
  printf("Union find...\n");
  unionFind(rows);

  // Detect Blobs
  printf("Building blogs...\n");
  std::unordered_map<RunLength*, Blob> blobs;
  computeBlobs(rows, blobs);

  // Sort blobs base on bounding box area
  printf("Sorting blobs\n");
  std::vector<Blob> blob_list (blobs.size());
  for (auto& pair : blobs) {
	  Blob& blob = pair.second;
	  if (blob.color == c_YELLOW) {
		  blob_list.push_back(blob);
	  }
  }
  // Note that this sorts the list in descending order
  std::sort(blob_list.begin(), blob_list.end(), [] (const Blob& b1, const Blob& b2) {
	  int b1_area = (b1.right - b1.left + 1) * (b1.bottom - b1.top + 1);
	  int b2_area = (b2.right - b2.left + 1) * (b2.bottom - b2.top + 1);
	  return b1_area > b2_area;
  });
//  // Print blob values
//  printf("Blob sizes: \n");
//  for (auto& b1 : blob_list) {
//	  printf("A:%dC:%d  ", (b1.right - b1.left + 1) * (b1.bottom - b1.top + 1), b1.color);
//  }
//  printf("\n");

  printf("Done!\n\n");

  detectBall();

  // Quick test code
  std::vector<WorldObjectType> beacons = {
		  WO_BEACON_BLUE_YELLOW,
		  WO_BEACON_YELLOW_BLUE,
		  WO_BEACON_BLUE_PINK,
		  WO_BEACON_PINK_BLUE,
		  WO_BEACON_PINK_YELLOW,
		  WO_BEACON_YELLOW_PINK,
  };
  auto fid = vblocks_.frame_info->frame_id;
  if(fid >= 6150) {
	  int beacon_count = 0;
	  for (Blob& blob : blob_list) {
		  auto& object = vblocks_.world_object->objects_[beacons[beacon_count]];
		  // TODO Do we need to add one?
		  object.imageCenterX = ((blob.left * 4) + (blob.right * 4)) / 2;
		  object.imageCenterY = ((blob.top * 4) + (blob.bottom * 4)) / 2;
		  float height = (blob.bottom - blob.top + 1) * 4;
		  auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, height);
		  object.visionDistance = cmatrix_.groundDistance(position);
		  object.visionBearing = cmatrix_.bearing(position);
		  object.seen = true;
		  object.fromTopCamera = camera_ == Camera::TOP;
		  visionLog(30, "saw %s at (%"
				  "i,%i) with calculated distance %2.4f", getName(beacons[beacon_count]), object.imageCenterX, object.imageCenterY, object.visionDistance);
		  if (beacon_count < beacons.size() - 1) {
			  beacon_count++;
		  } else {
			  break;
		  }
	  }
  } else {
	  beacon_detector_->findBeacons();
  }
}

void ImageProcessor::detectBall() {
  int imageX, imageY;

//  // Try to find goal every frame
//  WorldObject* goal = &vblocks_.world_object->objects_[WO_UNKNOWN_GOAL];
//  WorldObject* circle = &vblocks_.world_object->objects_[WO_CENTER_CIRCLE];
//  if (findGoal(goal->imageCenterX, goal->imageCenterY, circle->imageCenterY)) {
//    goal->seen = true;
//  }

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
//	printf("total blue pixels: %d, \t %d, \t %d, \n", total, imageX, imageY);

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
