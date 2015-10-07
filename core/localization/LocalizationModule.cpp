#include <localization/LocalizationModule.h>
#include <localization/KalmanFilter.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <cmath>

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger) {

}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
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
  cache_.localization_mem->player = self.loc;
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  cache_.localization_mem->player = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();

  // Kalman Filter Initialization
  // This is 1/(sample rate). Assuming 30 hz
  const double dt = 1.0/30;

  // Initial state vector (x, y, x', y', x'', y'')'
  filter.x = Matrix<double, DIM_X, 1>::Zero(DIM_X, 1);

  // Transition matrix
  filter.A << 1,    0,   dt,    0, pow(dt,2)/2,           0,
		  	  0,    1,    0,   dt,           0, pow(dt,2)/2,
			  0,    0,    1,    0,          dt,           0,
			  0,    0,    0,    1,           0,          dt,
			  0,    0,    0,    0,           1,           0,
			  0,    0,    0,    0,           0,           1;

//  filter.A << 1,    0,   dt,    0, 0,           0,
//			  0,    1,    0,   dt,           0, 0,
//			  0,    0,    1,    0,          dt,           0,
//			  0,    0,    0,    1,           0,          dt,
//			  0,    0,    0,    0,           1,           0,
//			  0,    0,    0,    0,           0,           1;

  // No input so it doesn't matter
  filter.B = Matrix<double, DIM_X, DIM_U>::Zero();

  // Extract predicted x and y values from state vector
  filter.C << 1, 0, 0, 0, 0, 0,
		      0, 1, 0, 0, 0, 0;

  // Sources of error
  filter.pred_err   = Matrix<double, DIM_X, DIM_X>::Identity() * 100; // TODO
  filter.sensor_err = Matrix<double, DIM_Z, DIM_Z>::Identity() * 30; // TODO
  filter.x_err      = Matrix<double, DIM_X, DIM_X>::Identity() * 1000;
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  auto sloc = cache_.localization_mem->player;
  self.loc = sloc;
    
  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {
	filter.pred_err   = Matrix<double, DIM_X, DIM_X>::Identity() * 100; // TODO
	filter.sensor_err = Matrix<double, DIM_Z, DIM_Z>::Identity() * 30; // TODO

    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;

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
  //TODO: How do we handle not seeing the ball?
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

    ball.absVel.x = x(2, 0);
    ball.absVel.y = x(3, 0);
    ball.relVel.x = x(4, 0); // TODO
    ball.relVel.y = x(5, 0); // TODO

    ball.distance = 10000.0f;
    ball.bearing = 0.0f;
  }
}
