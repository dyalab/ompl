#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/infeasibility/SDCLValidStateSampler.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::placeholders;

bool isStateValid(const ob::State *state)
{
    // a narrow passage 2d environment.
    const auto *state_vector = state->as<ob::RealVectorStateSpace::StateType>();

    // get x and y position
    const double x = state_vector->values[0];
    const double y = state_vector->values[1];
    const auto dist_square = x * x + y * y;

    // narrow passage width
    const double passage_width = 0.05;

    if (dist_square < 1 || dist_square > 4)
        return true;
    else if (x > 0 and abs(y) < passage_width)
        return true;  // on narrow passage
    else
        return false;
}

// SDCL sampler
ob::ValidStateSamplerPtr allocSDCLValidStateSampler(const ob::SpaceInformation *si, const ob::PlannerPtr planner)
{
    auto sampler(std::make_shared<ob::SDCLValidStateSampler>(si, planner));
    return sampler;
}

bool solve(double duration)
{
    int N = 2;

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(N));

    // set the bounds
    ob::RealVectorBounds bounds(N);
    bounds.setLow(-3);
    bounds.setHigh(3);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    si->setup();

    // start state
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = -2.5;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;

    // create a problem instance
    auto pdef_ = std::make_shared<ob::ProblemDefinition>(si);

    // set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    // auto planner_ = std::make_shared<og::LBKPIECE1>(si);
    auto planner_ = std::make_shared<og::PRM>(si);

    // set the problem we are trying to solve for the planner
    planner_->setProblemDefinition(pdef_);

    // We're making one sampler and by god its the only one we're using
    ob::ValidStateSamplerPtr sampler(std::make_shared<ob::SDCLValidStateSampler>(si.get(), planner_));

    auto allocSDCLValidStateSampler_partial = [&](const ob::SpaceInformation *si) { return sampler; };

    // setup SDCL sampler
    si->setValidStateSamplerAllocator(allocSDCLValidStateSampler_partial);

    // perform setup steps for the planner
    planner_->setup();
    std::cout << "Planner setup done! " << std::endl;

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef_->print(std::cout);

    // attempt to solve the problem within the planning time
    ob::PlannerStatus solved = planner_->ob::Planner::solve(duration);

    if (solved && pdef_->hasExactSolution())
    {
        // get the goal representation from the problem definition (not the same as
        // the goal state) and inquire about the found path
        ob::PathPtr path = pdef_->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
        return true;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        return false;
    }
}

void parse(int argc, char **argv, double *duration)
{
    if (argc != 2)
    {
        std::cout << "Please provide a duration(required). " << std::endl;
        exit(1);
    }
    else
    {
        *duration = std::atof(argv[1]);
    }
}

int main(int argc, char **argv)
{
    auto start_time = std::chrono::steady_clock::now();
    std::cout << "PRM with SDCL sampler start!" << std::endl;
    double duration = 0;

    parse(argc, argv, &duration);

    solve(duration);

    std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
    std::cout << "Total time is " << tot.count() << std::endl;

    return 0;
}
