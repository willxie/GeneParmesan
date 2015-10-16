#pragma once

#include <initializer_list>

#include <common/RobotInfo.h>
#include <common/RobotPositions.h>
#include <VisionCore.h>

#include "CommunicationGenerator.h"
#include "ObservationGenerator.h"
#include "Simulation.h"
#include "SimulationPath.h"

struct AgentError {
  float dist;
  float rot;
  float steps;
  AgentError() : dist(0), rot(0), steps(0) {}
  static AgentError average(std::vector<AgentError> errors) {
    AgentError error;
    for(auto e : errors) {
      error.dist += e.dist;
      error.rot += e.rot;
      error.steps += e.steps;
    }
    error.dist /= errors.size();
    error.rot /= errors.size();
    error.steps /= errors.size();
    return error;
  }
};

struct LocSimAgent {
  enum Type {
    Default = 0
  };
  LocalizationMethod::Type method;
  MemoryCache cache;
  VisionCore* core;
  float distError;
  float rotError;
  int steps;
  Type type;
  std::string name;
  float distRMSE() { return sqrtf(distError / steps); }
  float rotRMSE() { return sqrtf(rotError / steps); }
  LocSimAgent(Type type = Default) : type(type) {
    switch(type) {
      case Default:
        method = LocalizationMethod::Default;
        name = "Default";
        break;
    }
    distError = rotError = 0;
  }
};

class LocalizationSimulation : public Simulation {
  public:
    LocalizationSimulation(int seed = Random::SEED);
    LocalizationSimulation(std::string pathfile);
    LocalizationSimulation(LocSimAgent::Type type);
    ~LocalizationSimulation();
    int defaultPlayer();
    void setPath(const SimulationPath& path);
    void simulationStep();
    
    Memory* getGtMemory(int player = 0);
    Memory* getBeliefMemory(int player = 0);
    MemoryCache getGtMemoryCache(int player = 0);
    MemoryCache getBeliefMemoryCache(int player = 0);
    bool complete();
    std::string getSimInfo();
    std::vector<std::string> getTextDebug(int player = 0);
    AgentError getError(LocSimAgent::Type type);
    void printError();
    void flip();
    void outputBadPaths(float maxDistError, float maxRotError);
  private:
    int seed_;
    int ballmove_;
    std::map<LocSimAgent::Type, LocSimAgent> agents_;
    MemoryCache gtcache_;
    ImageParams iparams_;
    ObservationGenerator og_;
    CommunicationGenerator cg_;

    SimulationPath path_, origPath_;
    float maxDistError_, maxRotError_;
    bool outputBadPaths_;

    void init(std::vector<LocSimAgent::Type> types);
    void startCore(LocSimAgent& agent);

    void stepPose();
    void stepError();
    void generateObservations();
    void generateCommunications();
    void processLocalizationFrame();
    void placeObjects(LocSimAgent& agent);
    void moveBall();
    void moveBall(Point2D position);
    void teleportBall(Point2D position);
    void movePlayer(Point2D position, float orientation, int = 0);
    void teleportPlayer(Point2D position, float orientation, int = 0);
};
