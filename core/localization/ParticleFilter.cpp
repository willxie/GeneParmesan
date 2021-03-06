#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/WorldObjectBlock.h>
#include <set>
#include <map>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::randomParticle(Particle& p) {
    p.x = (rand() % (2*2500)) - 2500;
    p.y = (rand() % (2*1250)) - 1250;
    p.t = (static_cast<double>(rand()) / RAND_MAX) * 2 * M_PI - M_PI;
    p.w = 0;
}

double ParticleFilter::calculateGaussianValue(double m, double s, double x) {
    return ( 1 / ( s * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x-m)/s, 2.0 ) );
}


void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;

  // Initialize all particles randomly on the field
//  srand(time(NULL));
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

  std::random_device rd;
  std::mt19937 generator(rd());
  std::normal_distribution<double> v_distribution(0, V_STDDEV);
  std::normal_distribution<double> x_distribution(0, X_STDDEV);
  std::normal_distribution<double> y_distribution(0, Y_STDDEV);
  std::normal_distribution<double> t_distribution(0, T_STDDEV);

  for (auto& p : particles()) {
	  double t_noise = t_distribution(generator);
	  double v_noise = v_distribution(generator);
	  double x_noise = x_distribution(generator);
	  double y_noise = y_distribution(generator);

	  // Step each particle deterministically move by the odometry
	  if (disp.x == 0) {
		  v_noise = 0;
		  t_noise = 0;
	  }
	  if (disp.rotation == 0) {
		  t_noise = 0;
	  }
	  if (disp.x == 0 && disp.rotation == 0) {
		  x_noise = 0;
		  y_noise = 0;
	  }
	  p.t += disp.rotation + t_noise;
	  p.x += (0.8*disp.x + v_noise) * cos(p.t) + x_noise;
	  p.y += (0.8*disp.x + v_noise) * sin(p.t) + y_noise;
      p.w = 0;

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
		  double v_distance = beacon.visionDistance;
		  // Linear
//		  double max_distance = sqrt(pow(2500, 2) + pow(5000, 2));
//		  double distance_weight = (max_distance - abs(p_distance - v_distance)) / max_distance;
		  // Gaussian
		  double p_gaussian = calculateGaussianValue(p_distance, SDTDEV_POSITION_WEIGHT, v_distance);
		  double v_gaussian = calculateGaussianValue(v_distance, SDTDEV_POSITION_WEIGHT, v_distance);
		  double distance_weight = (v_gaussian - fabs(p_gaussian - v_gaussian)) / v_gaussian;

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

		  population_quality_total += (distance_weight + bearing_weight) / 2;
		  population_count++;

		  const double weight_ratio = 0.5;
		  p.w += (weight_ratio) * distance_weight + (1 - weight_ratio) * bearing_weight;
	  }
  }

  // If no beacons is seen, don't resample
  if (!beacon_seen) {
	  return;
  }

  if (!(disp.x != 0 || disp.rotation != 0)) {
	  return;
  }


  // Quality is the best if approaches 1
  double population_quality_avg = population_quality_total / population_count;

  // Get all particle weights
  vector<double> weights;
  for (auto p : particles()) {
	  weights.push_back(p.w);
  }

  // Sampling machinery
//  std::random_device rd;
//  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());

  // Determine what proportion of the population is random
//  double random_population_ratio = (disp.x == 0) ? 0 : 0.01;
  double random_population_ratio = 0.01;
  double fixed_population_ratio = 1 - random_population_ratio;
  double P_RANDOM_BOUND = lastBeaconPtr->visionDistance; // Max distance random particles can be spawned from the mean

  // Resampling
  vector<Particle> winners;
  for (int i = 0; i < NUM_PARTICLES; i++) {
	  if (i < NUM_PARTICLES * fixed_population_ratio) {
		  winners.push_back(particles()[d(generator)]);
	  } else {
		  Particle p;
		  float coin = -1+2*((float)rand())/RAND_MAX;
		  if (coin > 0.75) {
			  int bound_reject_counter = 0;
			  // Randomly generate particles only around the circumference of the circle around the last seen beacon
			  do {
				  // Solve for positions based on the last beacon position
				  float x_sign = -1+2*((float)rand())/RAND_MAX;
				  float y_sign = -1+2*((float)rand())/RAND_MAX;

				  // Fix x solve y
				  int x_offset = rand() % static_cast<int>(lastBeaconPtr->visionDistance);
				  int y_offset = sqrt(pow(lastBeaconPtr->visionDistance, 2) - pow(x_offset, 2));

				  // Sign
				  x_sign > 0 ? p.x = lastBeaconPtr->loc.x + x_offset : p.x = lastBeaconPtr->loc.x - x_offset;
				  y_sign > 0 ? p.y = lastBeaconPtr->loc.y + y_offset : p.y = lastBeaconPtr->loc.y - y_offset;

				  // Random angle
				  p.t = (static_cast<double>(rand()) / RAND_MAX) * 2 * M_PI - M_PI;


				  // Try to reject particles that are too far from the current mean_
				  // This might not always work so stop after a few tries
				  if (sqrt(pow(p.x-mean_.x, 2) + pow(p.y-mean_.y, 2)) < P_RANDOM_BOUND) {
					  bound_reject_counter++;
					  if (bound_reject_counter < 5) {
						  // Fuck it. Just choose a random particle.
						  randomParticle(p);
						  break;
					  }
				  }
			  } while (!(MIN_FIELD_X < p.x && p.x < MAX_FIELD_X &&
					  MIN_FIELD_Y < p.y && p.y < MAX_FIELD_Y)); // Make sure the point is within bound

		  } else {

			  // Find the point on the circle closest to the mean of the particle blob
			  float target_x = lastBeaconPtr->loc.x;
			  float target_y = lastBeaconPtr->loc.y;
			  float current_x = mean_.x;
			  float current_y = mean_.y;
			  float dx = target_x - current_x;
			  float dy = target_y - current_y;
			  float to_beacon_distance = sqrt(pow(dx, 2) + pow(dy, 2));
			  float to_target_distance = (to_beacon_distance - lastBeaconPtr->visionDistance);
			  float angle = atan(dy / dx) + M_PI;
			  p.x = current_x + to_target_distance * cos(angle);
			  p.y = current_y + to_target_distance * sin(angle);
			  p.t = (static_cast<double>(rand()) / RAND_MAX) * 2 * M_PI - M_PI;

		  }

//		  randomParticle(p);
		  winners.push_back(p);
	  }
  }

  particles() = winners;
}

// Pose is the average of all the particles
const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {

	typedef Point2D Centroid;

	// k means clustering
	//
	// Initialize random centroids
	set<Centroid, centroid_comp_> centroids;
	for (int i = 0; i < NUM_CLUSTERS; i++) {
		Centroid centroid;
	    centroid.x = (rand() % (2*2500)) - 2500;
	    centroid.y = (rand() % (2*1250)) - 1250;

	    centroids.insert(centroid);
	}

//	Centroid centroid;
//	centroid.x = -1000;
//	centroid.y = 0;
//	centroids.insert(centroid);
//
//	centroid.x = 1000;
//	centroid.y = 0;
//	centroids.insert(centroid);

//	cout << "========> BEGIN K-MEANS <========" << endl;

//	cout << "PARTICLES" << endl;
//	for (auto p : particles())
//		cout << p.x << " " << p.y << endl;
//	cout << endl;
////
//	cout << "Random Centroids" << endl;
//	for (auto centroid : centroids)
//		cout << centroid.x << " " << centroid.y << endl;
//	cout << endl;

	// Iterate until convergence
	map<Centroid, set<Particle, particle_comp_>, centroid_comp_> closest_particles;
	set<Point2D, centroid_comp_> new_centroids = centroids;
	map<Centroid, int, centroid_comp_> cluster_size;
	int i = 0;
	do {
//		cout << "PARTICLES" << endl;
//		for (auto p : particles())
//			cout << p.x << " " << p.y << endl;
//		cout << endl;

		centroids = new_centroids;
		new_centroids.clear();
		cluster_size.clear();
		// Compute the closest centroid to each particle
		double closest_centroid_distance = 50000;
		Centroid closest_centroid = NULL;
		for (Particle p : particles()) {
			for (Centroid centroid : centroids) {
//				cout << "Current closest distance: " << closest_centroid_distance << endl;
				double distance = sqrt(pow(p.x-centroid.x, 2) + pow(p.y-centroid.y, 2));
				// Record the closest centroid
//				cout << "Particle " << p.x << " " << p.y << endl;
//				cout << "Comparing centroid: " << centroid.x << " " << centroid.y << endl;
//				cout << "Proposal Distance: " << distance << endl;
				if (distance < closest_centroid_distance) {
//					cout << "Proposed distance is closer!!!" << endl;
					closest_centroid_distance = distance;
					closest_centroid = centroid;
				}
			}

//			cout << "C(" << closest_centroid.x << " " << closest_centroid.y << ") -> P(" << p.x << " " << p.y << ")" << endl;
			closest_particles[closest_centroid].insert(p);

			// Record the closest centroid
//			cout << "Particle " << p.x << " " << p.y << endl;
//			cout << "Closest centroid: " << closest_centroid.x << " " << closest_centroid.y << endl;
//			cout << "Centroid distance: " << closest_centroid_distance << endl << endl;
//			for (auto c : closest_particles) {
//				cout << "CENTROID " << c.first.x << " " << c.first.y << " at this point..." << endl;
//				for (auto pp : c.second) {
//					cout << pp.x << " " << pp.y << endl;
//				}
//				cout << endl;
//			}
//			cout << endl;

			closest_centroid = NULL;
			closest_centroid_distance = 50000;
		}

//		// Print closest particles
//		for (auto elem : closest_particles) {
//			cout << "Centroid: " << elem.first.x << " " << elem.first.y << endl;
//
//			for (auto p : elem.second) {
//				cout << "Particle: " << p.x << " " << p.y << endl;
//			}
//
//			cout << endl;
//		}

		// Recompute the centroids based on the mean of all the closest particles
		for (Centroid centroid : centroids) {
			double sum_x = 0;
			double sum_y = 0;
			for (Particle p : closest_particles[centroid]) {
				sum_x += p.x;
				sum_y += p.y;
			}

			int num_particles = closest_particles[centroid].size();
			if (num_particles > 0) {
				new_centroids.insert(Centroid(sum_x/num_particles, sum_y/num_particles));
				cluster_size[centroid] = num_particles;
			}
		}

//		cout << "Centroids" << endl;
//		for (auto centroid : centroids)
//			cout << centroid.x << " " << centroid.y << " (" << cluster_size[centroid] << ")" << endl;
//		cout << endl;
//
//		cout << "New Centroids" << endl;
//		for (auto centroid : new_centroids)
//			cout << centroid.x << " " << centroid.y << " (" << cluster_size[centroid] << ")" << endl;
//		cout << endl << endl;

		i++;
	} while (i < 3);

//	cout << "Centroids" << endl;
//	for (auto centroid : centroids)
//		cout << centroid.x << " " << centroid.y << " (" << cluster_size[centroid] << ")" << endl;
//	cout << endl;

	Centroid best_cluster;
	double max_particles = -1;
	for (Centroid centroid : centroids) {
		if (cluster_size[centroid] > max_particles) {
			best_cluster = centroid;
			max_particles = cluster_size[centroid];
		}
	}

//	cout << "Best cluster: " << best_cluster.x << " " << best_cluster.y << " (" << cluster_size[best_cluster] << ")" << endl;
//
//	cout << "========> END K-MEANS <========" << endl << endl;

    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= particles().size();

    // k-means update
    mean_.translation = best_cluster;

    dirty_ = false;
  }
  return mean_;
}
