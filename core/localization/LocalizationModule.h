#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <localization/KalmanFilter.h>

const int DIM = 1;

class LocalizationModule : public Module {
  public:
    LocalizationModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void initFromWorld();
    void reInit();
    void processFrame();

    void loadParams(LocalizationParams params);
  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
    KalmanFilter<DIM> filter;
};
