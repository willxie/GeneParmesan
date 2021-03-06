#include <localization/LocalizationModule.h>
#include <localization/KalmanFilter.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <cmath>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), pfilter_(new ParticleFilter(cache_, tlogger_)) {
}

LocalizationModule::~LocalizationModule() {
  delete pfilter_;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  pfilter_->init(self.loc, self.orientation);
}



Matrix<double, DIM_X, 1> TransitionFunction(Matrix<double, DIM_X, 1> x, Matrix<double, DIM_U, 1> u) {
    u = u;
    Matrix<double, DIM_X, DIM_X> A;
    A << 1,    0,   dt,    0, pow(dt,2)/2,           0,
        0,    1,    0,   dt,           0, pow(dt,2)/2,
        0,    0,    1,    0,          dt,           0,
        0,    0,    0,    1,           0,          dt,
        0,    0,    0,    0,           1,           0,
        0,    0,    0,    0,           0,           1;
    return (A * x);
}

Matrix<double, DIM_Z, 1> MeasurementFunction(Matrix<double, DIM_X, 1> x) {
    Matrix<double, DIM_Z, DIM_X> C;
    C << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;
    return (C * x);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  pfilter_->init(Point2D(-750,0), 0.0f);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();

  // Kalman Filter Initialization
  // Initial state vector (x, y, x', y', x'', y'')'
  filter.x = Matrix<double, DIM_X, 1>::Zero(DIM_X, 1);

  // Transition matrix
  // Don't need. See KalmanFilter.h for transition and measurement functions
//  filter.A << 1,    0,   dt,    0, pow(dt,2)/2,           0,
//		  	  0,    1,    0,   dt,           0, pow(dt,2)/2,
//			  0,    0,    1,    0,          dt,           0,
//			  0,    0,    0,    1,           0,          dt,
//			  0,    0,    0,    0,           1,           0,
//			  0,    0,    0,    0,           0,           1;
//
//  // No input so it doesn't matter
//  filter.B = Matrix<double, DIM_X, DIM_U>::Zero();
//
//  // Extract predicted x and y values from state vector
//  filter.C << 1, 0, 0, 0, 0, 0,
//		      0, 1, 0, 0, 0, 0;

  filter.transitionFunction = &TransitionFunction;
  filter.measurementFunction = &MeasurementFunction;

  // Sources of error
  filter.pred_err   = Matrix<double, DIM_X, DIM_X>::Identity() * 1; // TODO
  filter.sensor_err = Matrix<double, DIM_Z, DIM_Z>::Identity() * 5; // TODO
  filter.x_err      = Matrix<double, DIM_X, DIM_X>::Identity() * 1000;
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  auto& origin = cache_.world_object->objects_[WO_TEAM_COACH];

  // Process the current frame and retrieve our location/orientation estimate
  // from the particle filter
  pfilter_->processFrame();
  self.loc = pfilter_->pose().translation;
  self.orientation = pfilter_->pose().rotation;
  origin.loc = Point2D(0, 0);
  origin.orientation = Point2D(self.loc.x, self.loc.y).getBearingTo(origin.loc, self.orientation);
  origin.distance = sqrt(pow(self.loc.x-origin.loc.x, 2) + pow(self.loc.y-origin.loc.y, 2));

  log(40, "Localization Update: x=%2.f, y=%2.f, theta=%2.2f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);
    
  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {
	filter.pred_err   = Matrix<double, DIM_X, DIM_X>::Identity() * 1; // TODO
	filter.sensor_err = Matrix<double, DIM_Z, DIM_Z>::Identity() * 5; // TODO

    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;

    // TODO
    if (ball.distance < 500) {
    	filter.pred_err   = Matrix<double, DIM_X, DIM_X>::Identity() * 100; // TODO
    	filter.sensor_err = Matrix<double, DIM_Z, DIM_Z>::Identity() * 1; // TODO
    }

    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window

	Matrix<double, DIM_X, 1> x;
    Matrix<double, DIM_Z, 1> z;
    Matrix<double, DIM_X, DIM_X> covariance;

    // Input control and measurement
    z << ball.loc.x,  ball.loc.y;

	filter.update(Matrix<double, DIM_U, 1>::Zero(), z);

	// Update
	x = filter.getState();
    covariance = filter.getCovariance();

    cache_.localization_mem->state[0] = x(0,0);
    cache_.localization_mem->state[1] = x(1,0);
    //    cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000; // TODO actual cov.
    Matrix<float, 2, 2, Eigen::DontAlign> covariance_small;
    // YOLO
    covariance_small(0, 0) = covariance(0, 0);
    covariance_small(0, 1) = covariance(0, 1);
    covariance_small(1, 0) = covariance(1, 0);
    covariance_small(1, 1) = covariance(1, 1);
    cache_.localization_mem->covariance = covariance_small;

//    cout << "State:" << endl;
//    cout << filter.getState() << endl;

    //
    ball.loc.x    = x(0, 0);
    ball.loc.y    = x(1, 0);
    ball.absVel.x = x(2, 0);
    ball.absVel.y = x(3, 0);
    ball.relVel.x = x(4, 0); // TODO
    ball.relVel.y = x(5, 0); // TODO
  } 
  else {
	// Make filter completely rely on prediction model
	filter.pred_err   = Matrix<double, DIM_X, DIM_X>::Zero();
	filter.sensor_err = Matrix<double, DIM_Z, DIM_Z>::Identity() * 1.8E+307; // TODO

    Matrix<double, DIM_Z, 1> z;
	Matrix<double, DIM_X, 1> x_temp;

//	x_temp = filter.getState();
//
    z <<  0, 0;

//	filter.update(Matrix<double, DIM_U, 1>::Zero(), z);
    filter.updateWithoutAnything();
	Matrix<double, DIM_X, 1> x;
    Matrix<double, DIM_X, DIM_X> covariance;

	x = filter.getState();
    covariance = filter.getCovariance();

	cache_.localization_mem->state[0] = x(0,0);
	cache_.localization_mem->state[1] = x(1,0);
    Matrix<float, 2, 2, Eigen::DontAlign> covariance_small;
    covariance_small(0, 0) = covariance(0, 0);
    covariance_small(0, 1) = covariance(0, 1);
    covariance_small(1, 0) = covariance(1, 0);
    covariance_small(1, 1) = covariance(1, 1);
    cache_.localization_mem->covariance = covariance_small;

//    cout << "State:" << endl;
//    cout << filter.getState() << endl;

//    ball.absVel.x = x(2, 0);
//    ball.absVel.y = x(3, 0);
//    ball.relVel.x = x(4, 0); // TODO
//    ball.relVel.y = x(5, 0); // TODO

    ball.distance = 10000.0f;
    ball.bearing = 0.0f;
  }
}
