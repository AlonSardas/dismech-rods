#ifndef INERTIAL_FORCE_H
#define INERTIAL_FORCE_H

#include "rod_mechanics/base_force.h"

class BaseTimeStepper;

class InertialForce : public BaseForce
{
  public:
    InertialForce(const std::shared_ptr<SoftRobots>& m_soft_robots);
    ~InertialForce() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;
    std::string getName() const override {
        return "inertia";
    }

  private:
    double f, jac;
};

#endif  // INERTIAL_FORCE_H
