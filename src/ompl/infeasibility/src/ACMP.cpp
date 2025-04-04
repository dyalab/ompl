/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Washington State University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sihui Li */

#include <ompl/infeasibility/ACMP.h>

namespace ompl
{
    namespace magic
    {
        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
    }  // namespace magic
}  // namespace ompl


og::ACMP::ACMP(const base::SpaceInformationPtr &si, double lambda, bool starStrategy)
  : og::PRM(si, starStrategy)
  , lambda_(lambda)
{
    setName("ACMP");
    sampler_ = std::make_shared<ob::SDCLValidStateSampler>(si_.get(), dynamic_cast<ob::Planner*>(this));
    // SDCLSampler_.reset(new ob::SDCLValidStateSampler(si_.get(), this));
    // auto allocSDCLValidStateSampler_partial = [&](const ob::SpaceInformation *si) { return SDCLSampler_; };
    // si_->setValidStateSamplerAllocator(allocSDCLValidStateSampler_partial);
}

og::ACMP::~ACMP()
{
    std::cout << "destroy ACMP" << std::endl;
    // (dynamic_cast<ob::SDCLValidStateSampler*>(sampler_.get()))->endSDCLThread();
}

ompl::base::PlannerStatus og::ACMP::solve(const base::PlannerTerminationCondition &ptc)
{
    auto start_time = std::chrono::steady_clock::now();
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    // Reset addedNewSolution_ member and create solution checking thread
    addedNewSolution_ = false;
    foundInfProof_ = false;
    base::PathPtr sol;

    // construct new planner termination condition that fires when the given ptc is true, or a solution is found
    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] { return ptc || addedNewSolution(); });
    // construct new planner termination condition that fires when the given ptc is true, or an infeasibility proof is found
    base::PlannerTerminationCondition ptcOrInf([this, &ptc] { return ptc || foundInfProof(); });

    std::thread slnThread([this, &ptcOrInf, &sol] { checkForSolution(ptcOrInf, sol); });
    // create inf thread to check the learned manifold in SDCL sampler
    std::thread infThread([this, &ptcOrSolutionFound] {checkManifold(ptcOrSolutionFound); });

    constructRoadmap(ptcOrSolutionFound);

    // Ensure slnThread and infThread is ceased before exiting solve
    slnThread.join();
    infThread.join();

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    OMPL_INFORM("Total training time is %f, total sampling time is %f, total triangulation time is %f, total checking time is %f, \
                 total facet collision checking time is %f, total facet checking time is %f,", 
                stats_.training_time, stats_.sampling_time, stats_.tc_time, stats_.check_time, stats_.fcl_time * 1e-9, \
                stats_.facet_time * 1e-9);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
    auto time_str = oss.str();

    if (sol)
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);
    }
    else if (foundInfProof_)
    {
        OMPL_INFORM("Found infeasibility proof!");

        stats_.inf = 1;
        
    }

    std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
    stats_.total_time += tot.count();

    printStat(time_str);

    if (foundInfProof_) return base::PlannerStatus::EXACT_SOLUTION;
    if (sol) return base::PlannerStatus::EXACT_SOLUTION;

    return base::PlannerStatus::TIMEOUT;
}


// -------------------------------------------------- infeasibility proof part -------------------------------------------------------

void og::ACMP::checkManifold(const base::PlannerTerminationCondition &ptc) 
{

}

bool og::ACMP::foundInfProof() const
{
    return foundInfProof_;
}

void og::ACMP::printStat(std::string time_str) {
    std::ofstream csv_file(time_str + "_stats.csv");
    csv_file << "Solved, INF, Total planning time, SDCL itr, checking itr, training time, sampling time, tc time, \
                 manifold tracing time, save simplices time, checking time, collision checking for decomposed points, \
                 fcl time, facet time, num of facets, free mani-points, \
                 collision points, training points, simplex facets, use Gaussian, use Training, use Amino, use square" << std::endl;
    csv_file << stats_.solved 
             << ", " << stats_.inf
             << ", " << stats_.total_time
             << ", " << stats_.n_SDCL_itr << std::endl;
    csv_file.close();
}