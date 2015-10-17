#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <iostream>


struct ImageProcessor::RunLength {
	unsigned char color = -1;
	int x_left;
	int x_right;
	int y;
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
	int runs; // TODO for testing please remove after
	int parent_x;
	int parent_y;
	bool used;
};


/**
 *  type is of the following:
 *
 *	  WO_BEACON_BLUE_YELLOW
 *	  WO_BEACON_YELLOW_BLUE
 *	  WO_BEACON_BLUE_PINK
 *	  WO_BEACON_PINK_BLUE
 *	  WO_BEACON_PINK_YELLOW
 *	  WO_BEACON_YELLOW_PINK
 */
struct ImageProcessor::Beacon {
	WorldObjectType type;

	// Bounding box
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

  ball_candidate_ = new BallCandidate();
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
	// Compute Run Length Encoding

	// Process from left to right, top to bottom
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
				row.emplace_back();
				RunLength& curRunLength = row.back();

				curRunLength.parent = &curRunLength;
				curRunLength.x_left = xx;
				curRunLength.x_right = xx;
				curRunLength.color = c;
				curRunLength.y = yy;
			} else {
				RunLength& curRunLength = row.back();
				// Same color, just update x_right
				curRunLength.x_right = xx;
			}
			c_prev = c;
		}
	}

	// Link parents
	// TODO this is a very hacky way of doing it. I don't know why it didn't work for setting parents up top
	// TODO actually, this might be the only way to do it since vector memory must be contiguous
	for (auto& row : rows) {
		for (auto& rl : row) {
			RunLength* rp = &rl;
			rp->parent = rp;
		}
	}
}

// Given a candidate RunLength, find the uppermost parent (root of the tree)
ImageProcessor::RunLength* ImageProcessor::findRunLengthGrandParent(RunLength* runLength) {
	if (runLength->parent == runLength) {
		return runLength->parent;
	}
	return findRunLengthGrandParent(runLength->parent);
}

void ImageProcessor::unionFind(std::vector<std::vector<RunLength> >& rows) {
	// Skip first row
	for (int y = 1; y < rows.size(); ++y) {
		int x_top = 0;
		// Center on the bottom row and try to find the parent for each RunLength
		for (RunLength& curRunLength : rows[y]) {
			while (true) {
				RunLength& topRunLength = rows[y - 1][x_top];
				// Check for color
				if (curRunLength.color == topRunLength.color) {
					// Is this the first adjacent run length with the same color
					// encountered
					if (curRunLength.parent == &curRunLength) {
						curRunLength.parent = findRunLengthGrandParent(&topRunLength);
					} else {
						// If the curRunLength.parent is already set, it means
						// that this is the second overlapped region
						findRunLengthGrandParent(&topRunLength)->parent = findRunLengthGrandParent(&curRunLength);
					}
				}

				// Special case when both are right sides align, must advance both
				if (topRunLength.x_right == curRunLength.x_right) {
					x_top++;
					break;
				}
				// If next top run is guaranteed to be disconnected from the bottom, advance bottom
				if (topRunLength.x_right > curRunLength.x_right) {
					break;
				}

				x_top++;
			}
		}
	}
}


void ImageProcessor::computeBlobs(std::vector<std::vector<RunLength> >& rows, std::unordered_map<RunLength*, Blob>& blobs) {
	// From the run lengths, fill in blobs
	int row = 0;
	for (auto& runLengthRow : rows) {
		for (auto& runLength : runLengthRow) {
				RunLength *parent = findRunLengthGrandParent(runLength.parent);
				// Retrieve the blob associated with this parent
				std::unordered_map<RunLength*, Blob>::const_iterator got = blobs.find(parent);
				if (got == blobs.end()) {
					// Creating the blob and initialize
					Blob blob;
					blob.area = runLength.x_right - runLength.x_left + 1;
					blob.top = blob.bottom = row;
					blob.left = runLength.x_left;
					blob.right = runLength.x_right;
					blob.runs = 0; // TODO remove
					blob.parent_x = runLength.x_left;
					blob.parent_y = row;
					blob.used = false;
					blobs[parent] = blob;
				} else {
					// Updating the blob
					Blob& blob = blobs[parent];
					blob.area += runLength.x_right - runLength.x_left + 1;
					blob.color = runLength.color;
					blob.left = min(runLength.x_left, blob.left);
					blob.right = max(runLength.x_right, blob.right);
					blob.bottom = max(row, blob.bottom);
					blob.top = min(row, blob.top);

					blob.runs++; // TODO remove
				}

		}
		row++;
	}
}

/**
 * Input:
 *
 *   blobs - A list of blobs detected by union find
 *   top_color - The top color of the beacon
 *   bottom_color - The bottom color of the beacon
 *   beacon - An empty beacon object to fill in if the beacon is found
 *
 * Output:
 *
 *   true - If the beacon is found. Additionally `beacon` will be filled in
 *   false - If the beacon is not found
 */
bool ImageProcessor::findBeacon(std::vector<Blob>& blobs, WorldObjectType beacon_type,
			unsigned char top_color, unsigned char bottom_color, Beacon& beacon) {
	for (auto& top_blob : blobs) {
		// Color not `top_color`?
		if (top_blob.color != top_color) {
			continue;
		}

		if (top_blob.area < 7)
			continue;

		for (auto& middle_blob : blobs) {
			// Color not `bottom_color`?
			if (middle_blob.color != bottom_color) {
				continue;
			}

			if (middle_blob.area < 7)
				continue;

			// BEGIN HEURISTICS

			// Horizontal Proximity
			int horizontal_diff = std::abs((top_blob.left+top_blob.right)/2 - (middle_blob.left+middle_blob.right)/2);
			if (!((middle_blob.left <= top_blob.left && top_blob.left <= middle_blob.right) ||
					(top_blob.left <= middle_blob.left && middle_blob.left <= top_blob.right))) {
				continue;
			}

			// Vertical Proximity
			const int TOP_BOTTOM_OFFSET = 2;
			if (!(top_blob.bottom + TOP_BOTTOM_OFFSET >= middle_blob.top && middle_blob.bottom > top_blob.bottom + TOP_BOTTOM_OFFSET)) {
				continue;
			}

			// Area
			const float AREA_DIFFERENCE_LIMIT = 0.5;
			int top_area = top_blob.area;
			int bottom_area = middle_blob.area;
			double area_ratio = ((double)top_area)/bottom_area;
			if (std::abs(area_ratio-1) > AREA_DIFFERENCE_LIMIT) {
//				printf("    FAILED AREA TEST!!!\n");
//				printf("    Area Ratio: %f\n", area_ratio);
//				printf("    AREA_DIFFERENCE_LIMIT: %f\n", AREA_DIFFERENCE_LIMIT);
//				printf("    Top blob @ (x=%d, y=%d, area=%d)\n",
//						((top_blob.left * 4) + (top_blob.right * 4)) / 2,
//						((top_blob.top * 2) + (top_blob.bottom * 2)) / 2,
//						top_area);
//				printf("    middle blob @ (x=%d, y=%d, area=%d)\n",
//						((middle_blob.left * 4) + (middle_blob.right * 4)) / 2,
//						((middle_blob.top * 2) + (middle_blob.bottom * 2)) / 2,
//						bottom_area);
				continue;
			}

//			// Aspect Ratio
//			const float ASPECT_RATIO_DIFFERENCE_LIMIT = 1;
			double aspect_ratio = calculateAspectRatio(top_blob) + calculateAspectRatio(middle_blob);
//			if (std::abs(aspect_ratio - 2) > ASPECT_RATIO_DIFFERENCE_LIMIT) {
//				printf("    FAILED ASPECT RATIO TEST!!!\n");
//				printf("    Top blob @ (x=%d, y=%d, aspect_ratio=%f)\n",
//						((top_blob.left * 4) + (top_blob.right * 4)) / 2,
//						((top_blob.top * 2) + (top_blob.bottom * 2)) / 2,
//						aspect_ratio);
//				printf("    middle blob @ (x=%d, y=%d, aspect_ratio=%f)\n",
//						((middle_blob.left * 4) + (middle_blob.right * 4)) / 2,
//						((middle_blob.top * 2) + (middle_blob.bottom * 2)) / 2,
//						aspect_ratio);
//				continue;
//			}

			// Density
			const double DENSITY_DIFFERENCE_LIMIT = 0.5;
			double top_density = calculateDensity(top_blob);
			double bottom_density = calculateDensity(middle_blob);
			double density_ratio = (double) top_density/bottom_density;
			if (std::abs(density_ratio - 1) > DENSITY_DIFFERENCE_LIMIT) {
//				printf("    FAILED DENSITY RATIO TEST!!!\n");
//				printf("    Top blob @ (x=%d, y=%d, density=%f)\n",
//						((top_blob.left * 4) + (top_blob.right * 4)) / 2,
//						((top_blob.top * 2) + (top_blob.bottom * 2)) / 2,
//						top_density);
//				printf("    middle blob @ (x=%d, y=%d, density=%f)\n",
//						((middle_blob.left * 4) + (middle_blob.right * 4)) / 2,
//						((middle_blob.top * 2) + (middle_blob.bottom * 2)) / 2,
//						bottom_density);
//				printf("Density Ratio: %f\n", density_ratio);
				continue;
			}

			// Check and see if the pixel below the middle blob is white
			int white_pixel_x = (middle_blob.left + middle_blob.right) / 2 * 4;
			int white_pixel_y = (middle_blob.bottom + (middle_blob.bottom - middle_blob.top)/2) * 2;
			white_pixel_x = min(white_pixel_x, 320-1);
			white_pixel_y = min(white_pixel_y, 240-1);

			auto c = getSegImg()[white_pixel_y * iparams_.width + white_pixel_x];
			if (c != c_WHITE && c != c_ROBOT_WHITE) {
//				printf("        FAILED THE WHITE PIXEL TEST\n");
//				printf("        (x=%d, y=%d)\n", white_pixel_x, white_pixel_y);
//				printf("        Top blob @ (x=%d, y=%d, area=%d)\n",
//						((top_blob.left * 4) + (top_blob.right * 4)) / 2,
//						((top_blob.top * 2) + (top_blob.bottom * 2)) / 2,
//						top_area);
//				printf("    middle blob @ (x=%d, y=%d, area=%d)\n",
//						((middle_blob.left * 4) + (middle_blob.right * 4)) / 2,
//						((middle_blob.top * 2) + (middle_blob.bottom * 2)) / 2,
//						bottom_area);
				continue;
			}

//			printf("        BEACON DETECTED!\n");
//			printf("        Top blob @ (x=%d, y=%d) (top=%d, bottom=%d, left=%d, right=%d) (area=%d)\n",
//					((top_blob.left * 4) + (top_blob.right * 4)) / 2,
//					((top_blob.top * 2) + (top_blob.bottom * 2)) / 2,
//					top_blob.top * 2, top_blob.bottom * 2, top_blob.left * 4, top_blob.right * 4,
//					top_blob.area);
//			printf("        Middle blob @ (x=%d, y=%d) (top=%d, bottom=%d, left=%d, right=%d) (area=%d)\n",
//					((middle_blob.left * 4) + (middle_blob.right * 4)) / 2,
//					((middle_blob.top * 2) + (middle_blob.bottom * 2)) / 2,
//					middle_blob.top * 2, middle_blob.bottom * 2, middle_blob.left * 4, middle_blob.right * 4,
//					middle_blob.area);
//
//			printf("        Beacon Type: %d\n", beacon_type);
//			printf("        Area Ratio: %f\n", area_ratio);
//			printf("        Aspect Ratio: %f\n", aspect_ratio);
//			printf("        Density Ratio: %f\n", density_ratio);
//			printf("        White pixel color: (x=%d, y=%d, color=%d)\n", white_pixel_x, white_pixel_y, c);

			// The top blob is right below the middle blob and they're the same
			// color. Not only that; there's a robot white blob below the
			// middle one. This is surely a beacon!
			beacon.type = beacon_type;
			beacon.top = top_blob.top;

			// TODO sometime bottom is > top (?)
			beacon.bottom = middle_blob.bottom;

			// Be inclusive and take whichever blob extends furthest left/right
			beacon.left = std::min(top_blob.left, middle_blob.left);
			beacon.right = std::max(top_blob.right, middle_blob.right);

			return true;
		}
	}

	return false; // No beacon found
}

// width / height of bounding box in original image frame. Ratio = 1 means it's a square.
double ImageProcessor::calculateAspectRatio(Blob& blob) {
	return (double)(blob.right - blob.left + 1) * 4 / ((blob.bottom - blob.top + 1) * 2);
}

// Density (area of blob) / (area of bounding box)
double ImageProcessor::calculateDensity(Blob& blob) {
	// This is downsampled space. But the ratio should be the same
	return (double)blob.area / ((blob.right - blob.left + 1) * (blob.bottom - blob.top + 1));
}

// Assume the blob_list is sorted in size
bool ImageProcessor::findGoal(std::vector<Blob>& blob_list) {
	std::vector<BallCandidate> candidate_list;

	for (Blob& blob : blob_list) {
		if (blob.used) {
			continue;
		}
		if (blob.color != c_BLUE) {
			continue;
		}

		const int area_threshold = 40;
//		printf("area = %d\n", blob.area);
		// Take out small blobs, the goal should be BIG relative to other blue things
		if (blob.area < area_threshold) {
			continue;
		}

		const double density_tolerance = 0.50;
		const double ratio_tolerance = 0.50;

		const double density_ref = 1;
		const double ratio_ref = 34.0 / 20.0; // Goal should be 34cm x 20cm  

		// Deviation from the ideal value
		double density = std::abs(calculateDensity(blob) - density_ref);
//		printf("delta density = %f\n", density);
		// Density should be within +- 10% of ideal
		if (!(density < density_tolerance)) {
			continue;
		}

		// Ratio is above a threshold
		double ratio = std::abs(calculateAspectRatio(blob) - ratio_ref);
//		printf("delta ratio = %f\n", ratio);
		if (!(ratio < ratio_tolerance)) {
			continue;
		}

		BallCandidate candidate;
		candidate.centerX = ((blob.left * 4) + (blob.right * 4)) / 2;
		candidate.centerY = ((blob.top * 2) + (blob.bottom * 2)) / 2;
		candidate.radius  = ((blob.right - blob.left + 1) * 4); // Width
		candidate.confidence = (ratio * 1.0)+ (density * 1.0);  // The lower the better
		candidate_list.push_back(candidate);
	}
	
	if (candidate_list.size() == 0) {
		return false;
	}

	// Find best ball, lowest confidence
	std::sort(candidate_list.begin(), candidate_list.end(), [] (const BallCandidate& bc1, const BallCandidate& bc2) {
		// Bounding box area (not pixel area)
		return bc1.confidence < bc2.confidence;
	});

	// This is it!
	BallCandidate& candidate = candidate_list.front();

	WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

	goal->imageCenterX = candidate.centerX;
	goal->imageCenterY = candidate.centerY;
	goal->radius = candidate.radius;

	Position p = cmatrix_.getWorldPosition(goal->imageCenterX, goal->imageCenterY);
	goal->visionBearing = cmatrix_.bearing(p);
	goal->visionElevation = cmatrix_.elevation(p);
	goal->visionDistance = cmatrix_.groundDistance(p);
	goal->seen = true;
	goal->fromTopCamera = camera_ == Camera::TOP;

//	printf("GOAL!\n");

	return true;
}

// Assume the blob_list is sorted in size
bool ImageProcessor::findBall(std::vector<Blob>& blob_list) {
	std::vector<BallCandidate> ball_candidate_list;
	for (Blob& blob : blob_list) {
		if (blob.used) {
			continue;
		}
		// Check color
		if (blob.color != c_ORANGE) {
			continue;
		}

		const int area_threshold = 5;
//		printf("area = %d\n", blob.area);
		// Take out small blobs, the goal should be BIG relative to other blue things
		if (blob.area < area_threshold) {
			continue;
		}

		// Check area to bounding box ratio, it should be close to Pi/4
		const double density_tolerance = 0.20;
		const double ratio_tolerance = 0.20;

		const double density_ref = 3.14159265359 / 4;
		// Deviation from the ideal value
		double density = std::abs(calculateDensity(blob) - density_ref);
//		printf("delta density = %f\n", density);
		// Ball density should be within +- 10% of ideal
		if (!(density < density_tolerance)) {
			continue;
		}

		// Bounding box of ball should be close to square
		double ratio = std::abs(calculateAspectRatio(blob) - 1);
//		printf("delta ratio = %f\n", ratio);
		if (!(ratio < ratio_tolerance)) {
			continue;
		}

		BallCandidate ball_candidate;
		ball_candidate.centerX = ((blob.left * 4) + (blob.right * 4)) / 2;
		ball_candidate.centerY = ((blob.top * 2) + (blob.bottom * 2)) / 2;
		ball_candidate.radius  = ((blob.right - blob.left + 1) * 4) / 2;
		ball_candidate.confidence = (ratio * 1.0)+ (density * 2.0);  // The lower the better
		ball_candidate_list.push_back(ball_candidate);
	}

	if (ball_candidate_list.size() == 0) {
		return false;
	}

	// Find best ball, lowest confidence
	std::sort(ball_candidate_list.begin(), ball_candidate_list.end(), [] (const BallCandidate& bc1, const BallCandidate& bc2) {
		// Bounding box area (not pixel area)
		return bc1.confidence < bc2.confidence;
	});
	// This is it!
	BallCandidate& ball_candidate = ball_candidate_list.front();

	WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

	ball->imageCenterX = ball_candidate.centerX;
	ball->imageCenterY = ball_candidate.centerY;
	ball->radius = ball_candidate.radius;

	Position p = cmatrix_.getWorldPosition(ball->imageCenterX, ball->imageCenterY);
	ball->visionBearing = cmatrix_.bearing(p);
	ball->visionElevation = cmatrix_.elevation(p);
	ball->visionDistance = cmatrix_.groundDistance(p);
	ball->seen = true;
	ball->fromTopCamera = (camera_ == Camera::TOP);

	// Fill in this non-sense extra stuff for drawing when running in core mode
	ball_candidate_->centerX  = ball->imageCenterX;
	ball_candidate_->centerY  = ball->imageCenterY;
	ball_candidate_->radius  = ball->radius;

	// Linear model
	ball->visionDistance = 0.811 * ball->visionDistance + 100.14;
//	printf("Ball distance: %f\n", ball->visionDistance);

	return true;
}

void ImageProcessor::processFrame(){
  // TODO: what is WO_TEAM_COACH?
//  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
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
  std::vector<std::vector<RunLength> > rows (120);
  computeRunLength(rows);

  // Link RunLengths into regions with the same parent
  unionFind(rows);

  // Detect Blobs
  std::unordered_map<RunLength*, Blob> blobs;
  computeBlobs(rows, blobs);

  // Sort blobs base on bounding box area
  std::vector<Blob> blob_list;
  for (auto& pair : blobs) {
	  Blob& blob = pair.second;
	  // Filter out small blobs (pixel area downsampled)
	  blob_list.push_back(blob);
  }
  // Note that this sorts the list in descending order
  std::sort(blob_list.begin(), blob_list.end(), [] (const Blob& b1, const Blob& b2) {
	  // Bounding box area (not pixel area)
	  int b1_area = (b1.right - b1.left + 1) * (b1.bottom - b1.top + 1);
	  int b2_area = (b2.right - b2.left + 1) * (b2.bottom - b2.top + 1);
	  return b1_area > b2_area;
  });

  // Find important objects!
  findBall(blob_list);
  findGoal(blob_list);

  // Beacon types
  static map<WorldObjectType,vector<int>> beacon_configs = {
    { WO_BEACON_YELLOW_BLUE, { c_YELLOW, c_BLUE   } },
	{ WO_BEACON_BLUE_YELLOW, { c_BLUE,   c_YELLOW } },
    { WO_BEACON_YELLOW_PINK, { c_YELLOW, c_PINK   } },
    { WO_BEACON_PINK_YELLOW, { c_PINK,   c_YELLOW } },
    { WO_BEACON_BLUE_PINK,   { c_BLUE,   c_PINK   } },
    { WO_BEACON_PINK_BLUE,   { c_PINK,   c_BLUE   } }
  };

  // Beacon types
  static map<WorldObjectType, int> world_heights = {
    { WO_BEACON_YELLOW_BLUE, 300 },
	{ WO_BEACON_BLUE_YELLOW, 300 },
    { WO_BEACON_YELLOW_PINK, 200 },
    { WO_BEACON_PINK_YELLOW, 200 },
    { WO_BEACON_BLUE_PINK,   200 },
    { WO_BEACON_PINK_BLUE,   200 }
  };

  // Seach for all the beacons
  std::vector<Beacon> beacons;
  for (auto& beacon_config : beacon_configs) {
	  auto type = beacon_config.first;
	  auto colors = beacon_config.second;

	  Beacon beacon;
	  bool beacon_found = findBeacon(blob_list, type, colors[0], colors[1], beacon);
//	  printf("Beacon found (top_color=%d, bottom_color=%d)? %d\n", colors[0], colors[1], beacon_found);
	  if (beacon_found) {
//		  printf("Beacon (Left=%d, Right=%d, Top=%d, Bottom=%d)\n", beacon.left*4, beacon.right*4, beacon.top*2, beacon.bottom*2);
		  beacons.push_back(beacon);
	  } else {
		  // Reset the beacon object
		  auto& object = vblocks_.world_object->objects_[beacon_config.first];
		  object.seen = false;
	  }
  }

  // Display beacons
  for (auto& beacon : beacons) {
	  auto& object = vblocks_.world_object->objects_[beacon.type];
	  object.imageCenterX = ((beacon.left * 4) + (beacon.right * 4)) / 2;
	  object.imageCenterY = ((beacon.top * 2) + (beacon.bottom * 2)) / 2;
//	  printf("(ImageCenterX=%d, ImageCenterY=%d)\n", object.imageCenterX, object.imageCenterY);
	  auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, world_heights[beacon.type]);
	  object.visionDistance = cmatrix_.groundDistance(position);

	  // Linear model
	  object.visionDistance = 0.811 * object.visionDistance + 100.14;
	  printf("Beacon Distance: %f\n", object.visionDistance);

	  object.visionBearing = cmatrix_.bearing(position);
	  object.seen = true;
	  object.fromTopCamera = camera_ == Camera::TOP;
	  visionLog(30, "saw %s at (%"
			  "i,%i) with calculated distance %2.4f", getName(WO_BEACON_YELLOW_BLUE), object.imageCenterX, object.imageCenterY, object.visionDistance);
  }

//  printf("\n");
}

// DEPRECATED
void ImageProcessor::detectBall() {
  int imageX, imageY;

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

// DEPRECATED
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

	return (total > 10);
}

// DEPRECATED
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
	WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
	if (ball->seen) {
		return ball_candidate_;
	} else {
		return NULL;
	}
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}
