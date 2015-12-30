#pragma once

#include <math/Pose2D.h>
#include <common/Random.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>

const int NUM_PARTICLES = 300;
const double V_STDDEV = 5;
const double X_STDDEV = 3;
const double Y_STDDEV = X_STDDEV;
const double T_STDDEV = 0.05;
const double SDTDEV_POSITION_WEIGHT = 60;

const int MIN_FIELD_X = -2500;
const int MAX_FIELD_X = 2500;
const int MIN_FIELD_Y = -1250;
const int MAX_FIELD_Y = 1250;

const int NUM_CLUSTERS = 1;

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    void init(Point2D loc, float orientation);
    void processFrame();
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

    void randomParticle(Particle& p);
    double calculateGaussianValue(double m, double s, double x);

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    Random rand_;

    mutable Pose2D mean_;
    mutable bool dirty_;

    struct centroid_comp_ {
        bool operator() (const Point2D& a, const Point2D& b) const {
            if (a.x < b.x) return true;
            if (a.x > b.x) return false;

            // Equal x values!

            if (a.y < b.y) return true;
            if (a.y > b.y) return false;

            // Equal x and y values!

            return false;
        }
    };

    struct particle_comp_ {
        bool operator() (const Particle& a, const Particle& b) const {
            if (a.x < b.x) return true;
            if (a.x > b.x) return false;

            // Equal x values!

            if (a.y < b.y) return true;
            if (a.y > b.y) return false;

            // Equal x and y values!

            return false;
        }
    };
};
