#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::randomParticle(Particle& p) {
    p.x = (rand() % (2*2500)) - 2500;
    p.y = (rand() % (2*1250)) - 1250;
    p.t = rand() % 360;
    p.w = 0;
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;

  // Initialize all particles randomly on the field
  srand(time(NULL));
  particles().resize(NUM_PARTICLES);
  for (auto& p : particles()) {
	  randomParticle(p);
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);

  std::default_random_engine generator;
  std::normal_distribution<double> x_distribution(0, X_STDDEV);
  std::normal_distribution<double> y_distribution(0, Y_STDDEV);
  std::normal_distribution<double> t_distribution(0, T_STDDEV);
  for (auto& p : particles()) {
	  // Step each particle deterministically move by the odometry
	  p.t += disp.rotation;
      p.x += disp.x * cos(p.t);
      p.y += disp.x * sin(p.t);
      p.w = 0;

      // Add a little noise to each of the new positions
      // Noise should be a function of speed
	  double x_noise = x_distribution(generator);
	  double y_noise = y_distribution(generator);
	  double t_noise = t_distribution(generator);
	  if (disp.x != 0) p.x += x_noise;
	  if (disp.x != 0) p.y += y_noise;
//	  if (disp.rotation != 0)
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

  double population_quality_total = 0;
  double population_count = 0;
  bool beacon_seen = false;
  WorldObject* lastBeaconPtr;

  // Update particle weights with respect to how far they are from the seen beacon
  //
  // TODO Only take into account the last beacon seen right now
  for (auto& p : particles()) {
	  for (auto beacon_id : beacon_ids) {
		  auto& beacon = cache_.world_object->objects_[beacon_id];
		  if (!beacon.seen) {
			  continue;
		  }

		  beacon_seen = true;
		  lastBeaconPtr = &beacon;

		  // Beacon distance
		  double p_distance = sqrt(pow(abs(p.x-beacon.loc.x), 2) + pow(abs(p.y-beacon.loc.y), 2));
		  double nao_distance = beacon.visionDistance;
		  double max_distance = sqrt(pow(2500, 2) + pow(5000, 2));
		  double distance_weight = (max_distance - abs(p_distance - nao_distance)) / max_distance;

		  // Orientation
		  double p_bearing = Point2D(p.x, p.y).getBearingTo(beacon.loc, p.t);
		  double bearing_difference = abs(p_bearing - beacon.visionBearing);
		  if (bearing_difference > 2 * M_PI) cout << "ERROR Bearing diff: " << p_bearing << ", " << beacon.visionBearing << endl;
		  if (bearing_difference > M_PI) {
			  // Angle wrap around
			  bearing_difference = (2 * M_PI) - bearing_difference;
		  }
		  double max_bearing_diff = M_PI;
		  double bearing_weight = (max_bearing_diff - bearing_difference) / max_bearing_diff;

//		  cout << "beacon coordinate: " << beacon.loc.x  << ", "<< beacon.loc.y << endl;
//		  cout << "p Bearing: " << p_bearing  << endl;
		  population_quality_total += (distance_weight + bearing_weight) / 2;
		  population_count++;

		  const double weight_ratio = 0.9;
		  p.w += (weight_ratio) * distance_weight + (1 - weight_ratio) * bearing_weight;
	  }
  }

  // If no beacons is seen, don't resample
  if (!beacon_seen) {
	  return;
  }

  // Quality is the best if approaches 1
  double population_quality_avg = population_quality_total / population_count;

//  cout << "num particles: " << particles().size() << endl;
  cout << "Population quality: " << population_quality_avg << endl;


  // Get all particle weights
  vector<double> weights;
  for (auto p : particles()) {
	  weights.push_back(p.w);
  }

  // Sampling machinery
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());

  // Resampling
  vector<Particle> winners;
  for (int i = 0; i < NUM_PARTICLES; i++) {
//	  if (i < NUM_PARTICLES * population_quality_avg) {
	  if (i < NUM_PARTICLES * 0.99) {
		  winners.push_back(particles()[d(gen)]);
	  } else {
		  Particle p;
		  do {
			  // Solve for positions based on the last beacon position
			  float x_sign = -1+2*((float)rand())/RAND_MAX;
			  float y_sign = -1+2*((float)rand())/RAND_MAX;

			  // Fix x solve y
			  int x_offset = rand() % static_cast<int>(lastBeaconPtr->visionDistance);
			  int y_offset = sqrt(pow(lastBeaconPtr->visionDistance, 2) - pow(x_offset, 2));

	//		  cout << lastBeaconPtr->visionDistance << endl;
	//		  cout << x_offset << ", "<< y_offset << endl;
			  // Sign
			  x_sign > 0 ? p.x = lastBeaconPtr->loc.x + x_offset : p.x = lastBeaconPtr->loc.x - x_offset;
			  y_sign > 0 ? p.y = lastBeaconPtr->loc.y + y_offset : p.y = lastBeaconPtr->loc.y - y_offset;
		  } while (!(MIN_FIELD_X < p.x && p.x < MAX_FIELD_X &&
				   MIN_FIELD_Y < p.y && p.y < MAX_FIELD_Y));

		  // Don't need noise because there's already noise in vision
		  // Random angle
		  p.t = rand() % 360;
//		  randomParticle(p);
		  winners.push_back(p);
	  }
  }

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
