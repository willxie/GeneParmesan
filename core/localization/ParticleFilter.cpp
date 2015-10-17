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
//      p.t = rand() % 360;
      p.t = 0;
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

		  double distance = sqrt(pow(abs(p.x-beacon.loc.x), 2) + pow(abs(p.y-beacon.loc.y), 2));
		  p.w = (sqrt(pow(2500, 2) + pow(5000, 2)) - distance) / sqrt(pow(2500, 2) + pow(5000, 2));
	  }
  }

  // View particle weights
  for (auto& p : particles()) {
	  cout << p.w << endl;
  }
  cout << endl;

  // Get all weights
  vector<double> weights;
  for (auto p : particles()) {
	  weights.push_back(p.w);
  }

  cout << "Weights" << endl;
  for (auto weight : weights)
	  cout << weight << endl;

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
