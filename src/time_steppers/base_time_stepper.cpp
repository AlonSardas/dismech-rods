#include "base_time_stepper.h"
#include "rod_mechanics/base_force.h"
#include "rod_mechanics/elastic_joint.h"
#include "rod_mechanics/elastic_rod.h"
#include "rod_mechanics/force_container.h"
#include "rod_mechanics/soft_robots.h"

BaseTimeStepper::BaseTimeStepper(const std::shared_ptr<SoftRobots>& soft_robots,
                                 const std::shared_ptr<ForceContainer>& forces,
                                 const SimParams& sim_params)
    : limbs(soft_robots->limbs), joints(soft_robots->joints), controllers(soft_robots->controllers),
      forces(forces), dt(sim_params.dt) {
    freeDOF = 0;
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }

    Force = VecX::Zero(freeDOF);
    DX = VecX::Zero(freeDOF);
}

void BaseTimeStepper::initStepper() {
    forces->setupForceStepperAccess(shared_from_this());
}

BaseTimeStepper::~BaseTimeStepper() = default;

void BaseTimeStepper::addForce(int ind, double p, int limb_idx) {
    if (sink) {
        (*sink)[limb_idx][ind] -= p;  // residual sign -> physical sign
        return;
    }

    std::shared_ptr<ElasticRod> limb = limbs[limb_idx];

    offset = offsets[limb_idx];

    if (limb->getIfConstrained(ind) == 0)  // free dof
    {
        mappedInd = limb->fullToUnconsMap[ind];
        Force[mappedInd + offset] += p;  // subtracting elastic force
    }
}

void BaseTimeStepper::setRecordedForces(const std::vector<std::string>& names) {
    std::set<std::string> available;
    for (const auto& force : forces->getForces())
        available.insert(force->getName());

    for (const auto& name : names) {
        if (!available.count(name)) {
            std::string msg = "Unknown force '" + name + "'. Available:";
            for (const auto& a : available)
                msg += " " + a;
            throw std::invalid_argument(msg);
        }
    }

    recorded_names = std::set<std::string>(names.begin(), names.end());
    recorded.clear();
    for (const auto& name : recorded_names) {
        for (const auto& limb : limbs)
            recorded[name].push_back(VecX::Zero(limb->ndof));
    }
}

bool BaseTimeStepper::recordingForces() const {
    return !recorded_names.empty();
}

void BaseTimeStepper::recordForces(double dt) {
    for (auto& [name, per_limb] : recorded)
        for (auto& limb_forces : per_limb)
            limb_forces.setZero();

    for (const auto& force : forces->getForces()) {
        auto it = recorded.find(force->getName());
        if (it == recorded.end())
            continue;
        sink = &it->second;
        force->computeForce(dt);
    }
    sink = nullptr;
}

std::map<std::string, MatX> BaseTimeStepper::getRecordedForces(int limb_idx) const {
    if (limb_idx < 0 || limb_idx >= static_cast<int>(limbs.size()))
        throw std::out_of_range("Invalid limb index.");

    int nv = limbs[limb_idx]->nv;
    std::map<std::string, MatX> result;
    for (const auto& [name, per_limb] : recorded) {
        const VecX& f = per_limb[limb_idx];
        MatX nodal(nv, 3);
        for (int i = 0; i < nv; i++)
            for (int k = 0; k < 3; k++)
                nodal(i, k) = f[4 * i + k];
        result[name] = nodal;
    }
    return result;
}

void BaseTimeStepper::setZero() {
    Force.setZero();
}

void BaseTimeStepper::update() {
    freeDOF = 0;
    offsets.clear();
    for (const auto& limb : limbs) {
        offsets.push_back(freeDOF);
        freeDOF += limb->uncons;
    }

    Force.setZero(freeDOF);
    DX.setZero(freeDOF);
}

void BaseTimeStepper::prepSystemForIteration() {
    for (const auto& joint : joints)
        joint->prepLimbs();
    for (const auto& limb : limbs)
        limb->prepareForIteration();
    for (const auto& joint : joints)
        joint->prepareForIteration();
}
