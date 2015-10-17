#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;

  // Initialize all particles randomly on the field
  srand(time(NULL));
  particles().resize(NUM_PARTICLES);
  for (auto& p : particles()) {
      p.x = (rand() % (2*2500)) - 2500,
      p.y = (rand() % (2*1250)) - 1250,
//      p.x = 0;
//      p.y = 0;
      p.t = rand() % 360;
//      p.t = 0;
      p.w = 1;
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  
  // Step each particle deterministically by the odometry
  for (auto& p : particles()) {
	  p.t += disp.rotation;
      p.x += disp.x * cos(p.t);
      p.y += disp.x * sin(p.t);
  }

  // Add a little noise to each of the new positions
  std::default_random_engine generator;
  std::normal_distribution<double> x_distribution(0, X_STDDEV);
  std::normal_distribution<double> y_distribution(0, Y_STDDEV);
  std::normal_distribution<double> t_distribution(0, T_STDDEV);
  for (auto& p : particles()) {
	  double x_noise = x_distribution(generator);
	  double y_noise = y_distribution(generator);
	  double t_noise = t_distribution(generator);

	  cout << x_noise << " " << y_noise << " " << t_noise << endl;

	  p.x += x_noise;
	  p.y += y_noise;
	  p.t += t_noise;
  }

  static vector<WorldObjectType> beacon_ids = {
		  WO_BEACON_YELLOW_BLUE,
		  WO_BEACON_BLUE_YELLOW,
		  WO_BEACON_YELLOW_PINK,
		  WO_BEACON_PINK_YELLOW,
		  WO_BEACON_BLUE_PINK,
		  WO_BEACON_PINK_BLUE
  };

  // Update particle weights with respect to how far they are from the seen beacon
  //
  // Only take into account the last beacon seen right now
  for (auto& p : particles()) {
	  for (auto beacon_id : beacon_ids) {
		  auto& beacon = cache_.world_object->objects_[beacon_id];
		  if (! beacon.seen)
			  continue;

		  // Beacon distance
		  double p_distance = sqrt(pow(abs(p.x-beacon.loc.x), 2) + pow(abs(p.y-beacon.loc.y), 2));
		  double nao_distance = beacon.visionDistance;
		  double max_distance = sqrt(pow(2500, 2) + pow(5000, 2));
		  double distance_weight = (max_distance - abs(p_distance - nao_distance)) / max_distance;

		  // Orientation
		  double p_bearing = Point2D(p.x, p.y).getBearingTo(Point2D(beacon.loc), p.t);
		  double max_bearing_diff = 2 * M_PI;
		  double bearing_weight = (max_bearing_diff - abs(p_bearing - beacon.visionBearing)) / max_bearing_diff;

		  p.w = distance_weight + bearing_weight;
	  }
  }

//  // View particle weights
//  for (auto& p : particles()) {
//	  cout << p.w << endl;
//  }
//  cout << endl;

  // Get all particle weights
  vector<double> weights;
  for (auto p : particles()) {
	  weights.push_back(p.w);
  }

//  cout << "Weights" << endl;
//  for (auto weight : weights)
//	  cout << weight << endl;

   // Sampling machinery
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());

  // Resampling
  vector<Particle> winners;
  for (int i = 0; i < NUM_PARTICLES; i++) {
	  winners.push_back(particles()[d(gen)]);
  }

  cout << "Winners" << endl;
  for (auto p : winners)
	  cout << p.x << " " << p.y << endl;
  cout << endl;

  particles() = winners;
}

// Pose is the average of all the particles
const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= particles().size();
    dirty_ = false;
  }
  return mean_;
}
