#pragma once

#include <math/Pose2D.h>
#include <common/Random.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>

const int NUM_PARTICLES = 200;
const double X_STDDEV = 1;
const double Y_STDDEV = X_STDDEV;
const double T_STDDEV = .01;

const int MIN_FIELD_X = -2500;
const int MAX_FIELD_X = 2500;
const int MIN_FIELD_Y = -1250;
const int MAX_FIELD_Y = 1250;

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

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;
    Random rand_;

    mutable Pose2D mean_;
    mutable bool dirty_;
};
