#ifndef BASE_TIME_STEPPER_H
#define BASE_TIME_STEPPER_H

#include "global_definitions.h"
#include <map>
#include <set>
#include <string>

class SoftRobots;
class ElasticRod;
class ElasticJoint;
class BaseController;
class ForceContainer;

class BaseTimeStepper : public std::enable_shared_from_this<BaseTimeStepper>
{
  public:
    BaseTimeStepper(const std::shared_ptr<SoftRobots>& m_soft_robots,
                    const std::shared_ptr<ForceContainer>& m_forces, const SimParams& sim_params);
    virtual ~BaseTimeStepper();

    void addForce(int ind, double p, int limb_idx);

    // Force recording. After a step's solve, the selected forces are recomputed one at a
    // time with addForce redirected into per-force buffers (physical sign, all DOFs
    // including constrained ones). The solver's residual is not touched.
    void setRecordedForces(const std::vector<std::string>& names);
    std::map<std::string, MatX> getRecordedForces(int limb_idx) const;

    virtual void initStepper();
    virtual void prepSystemForIteration();
    virtual void setZero();
    virtual void update();
    virtual void integrator() = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx) = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) = 0;
    virtual void updateSystemForNextTimeStep() = 0;
    virtual double stepForwardInTime() = 0;

    VecX Force;
    VecX DX;

    int freeDOF;
    std::vector<int> offsets;
    int iter = 0;

  protected:
    int mappedInd, mappedInd1, mappedInd2;
    int offset;
    double dt;
    double alpha = 1.0;

    std::vector<std::shared_ptr<ElasticRod>>& limbs;
    std::vector<std::shared_ptr<ElasticJoint>>& joints;
    std::vector<std::shared_ptr<BaseController>>& controllers;
    std::shared_ptr<ForceContainer> forces;

    bool recordingForces() const;
    void recordForces(double dt);

  private:
    std::set<std::string> recorded_names;
    std::map<std::string, std::vector<VecX>> recorded;  // name -> per-limb, size ndof
    std::vector<VecX>* sink = nullptr;
};

#endif  // BASE_TIME_STEPPER_H
