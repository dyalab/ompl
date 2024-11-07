/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, WSU
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
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#include "ompl/infeasibility/SDCLValidStateSampler.h"

namespace ompl
{
    namespace magic
    {
        /** \brief margin outside of configuration space boundaries to space extent fraction.*/
        static const double MARGIN_AS_SPACE_EXTENT_FRACTION = 0.02;

    }  // namespace magic
}  // namespace ompl

ompl::base::SDCLValidStateSampler::SDCLValidStateSampler(const SpaceInformation *si, const PlannerPtr planner)
  : ValidStateSampler(si)
  , sampler_(si->allocStateSampler())
  , planner_(planner)
  , delta_(si->getMaximumExtent() * magic::MARGIN_AS_SPACE_EXTENT_FRACTION)
  , upperBound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high)
  , lowerBound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low)
{
    name_ = "SDCL";
    SDCLPoints_.reset(new std::vector<base::State *>());
    params_.declareParam<double>(
        "virtual_C_margin", [this](double m) { setVirtualCMargin(m); }, [this] { return getVirtualCMargin(); });

    // start the SDCL thread
    SDCLThread_ = std::thread(&ompl::base::SDCLValidStateSampler::generateSDCLSamples, this);

    // start collision points list.
    collisionPoints_.reset(new std::vector<base::State *>());
}

ompl::base::SDCLValidStateSampler::~SDCLValidStateSampler()
{
    endSDCLThread();
    clearStates(SDCLPoints_);
    clearStates(collisionPoints_);
}

void ompl::base::SDCLValidStateSampler::clearStates(std::shared_ptr<std::vector<base::State *>> statelist)
{
    for (std::size_t i = 0; i < statelist->size(); i++)
    {
        si_->freeState((*statelist)[i]);
    }
}

bool ompl::base::SDCLValidStateSampler::sample(State *state)
{
    unsigned int attempts = 0;
    bool valid = false;

    do
    {
        // use SDCL points when available, if not, use uniform sampling
        if (curSDCLPointsCount_ > usedSDCLPointsCount_)
        {
            valid = true;
            SDCLPointsMutex_.lock();
            si_->copyState(state, (*SDCLPoints_)[usedSDCLPointsCount_]);
            SDCLPointsMutex_.unlock();
            usedSDCLPointsCount_++;
        }
        else
        {
            sampleUniformWithMargin(state);
            valid = isValidWithMargin(state);
            if (!valid)
                saveCollisionPoints(state);
        }
        ++attempts;
    } while (!valid && attempts < attempts_);
    return valid;
}

bool ompl::base::SDCLValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    unsigned int attempts = 0;
    bool valid = false;
    do
    {
        sampler_->sampleUniformNear(state, near, distance);
        valid = si_->isValid(state);
        ++attempts;
    } while (!valid && attempts < attempts_);
    return valid;
}

void ompl::base::SDCLValidStateSampler::endSDCLThread()
{
    sdclThreadEnded_ = true;
    SDCLThread_.join();
    // OMPL_INFORM("SDCL sampling thread ended. ");
}

void ompl::base::SDCLValidStateSampler::sampleUniformWithMargin(State *state)
{
    const unsigned int dim = si_->getStateDimension();

    auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
    for (unsigned int i = 0; i < dim; ++i)  // sample with virtual C regions
        rstate->values[i] = rng_.uniformReal(lowerBound_[i] - delta_ * 2, upperBound_[i] + delta_ * 2);
}

bool ompl::base::SDCLValidStateSampler::outOfBounds(State *state)
{
    // whether point is out of joint limits
    const unsigned int dim = si_->getStateDimension();
    auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
    for (std::size_t i = 0; i < dim; i++)
    {
        if (rstate->values[i] < lowerBound_[i] || rstate->values[i] > upperBound_[i])
        {
            return true;
        }
    }
    return false;
}

bool ompl::base::SDCLValidStateSampler::outOfBoundsCollision(State *state)
{
    const unsigned int dim = si_->getStateDimension();
    auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
    for (std::size_t i = 0; i < dim; i++)
    {
        if (rstate->values[i] < lowerBound_[i] - delta_ || rstate->values[i] > upperBound_[i] + delta_)
        {
            return true;
        }
    }
    return false;
}

bool ompl::base::SDCLValidStateSampler::isValidWithMargin(State *state)
{
    if (delta_ > 0)
    {
        if (outOfBoundsCollision(state))
        {
            return true;
        }
        if (outOfBounds(state))
        {
            return false;
        }
    }

    return si_->isValid(state);
}

void ompl::base::SDCLValidStateSampler::generateSDCLSamples()
{
    while (planner_ == NULL || !planner_->isSetup())
    {
    }

    OMPL_INFORM("SDCL sampling thread started ");

    // default is SVM manifold, TODO: set other types of manifold if other learn method also works.
    manifold_.reset(
        new ompl::infeasibility::SVMManifold(planner_->getSpaceInformation(), "SDCL", si_->getStateDimension()));

    while (!sdclThreadEnded_)
    {
        // get planning graph, call manifold function to train the manifold.
        PlannerDataPtr plannerData(std::make_shared<PlannerData>(planner_->getSpaceInformation()));
        planner_->getPlannerData(*plannerData);
        bool success = manifold_->learnManifold(plannerData);

        if (success)
            sampleManifoldPoints();
    }
}

void ompl::base::SDCLValidStateSampler::sampleManifoldPoints()
{
    // need a copy of the collision points for read and write conflicts.
    std::vector<base::State *> collision_copy;
    collisionPointsMutex_.lock();
    int num_collision_points = collisionPoints_->size();
    copy(collisionPoints_->begin(), collisionPoints_->end(), back_inserter(collision_copy));
    collisionPointsMutex_.unlock();

    // start thread pool
    int num_threads = std::thread::hardware_concurrency();
    // OMPL_INFORM("Thread pool for calculating manifold points has %d threads.", num_threads);
    boost::asio::thread_pool threadpool(num_threads);

    // loop to add thread pool
    for (int i = 0; i < num_collision_points; i++)
    {
        boost::asio::post(threadpool, [collision_copy, i, this] { calManifoldPoints(collision_copy[i]); });
    }

    // get Cfree points from the planning data.
    PlannerDataPtr plannerData(std::make_shared<PlannerData>(planner_->getSpaceInformation()));
    planner_->getPlannerData(*plannerData);
    unsigned int num_free_points = plannerData->numVertices();

    for (std::size_t i = 0; i < num_free_points; i++)
    {
        PlannerDataVertex cur_vertex = plannerData->getVertex(i);
        const base::State *s = cur_vertex.getState();
        boost::asio::post(threadpool, [&, i, this] { calManifoldPoints(s); });
    }

    threadpool.join();

    // OMPL_INFORM("There are %d collision points, %d training points", num_collision_points, num_free_points);
}

void ompl::base::SDCLValidStateSampler::calManifoldPoints(const base::State *input_state)
{
    base::State *res_state = si_->allocState();
    manifold_->sampleManifold(input_state, res_state);

    if (isValidWithMargin(res_state))
    {
        SDCLPointsMutex_.lock();
        SDCLPoints_->push_back(res_state);
        SDCLPointsMutex_.unlock();
        curSDCLPointsCount_++;
    }
}

void ompl::base::SDCLValidStateSampler::saveCollisionPoints(base::State *workState)
{
    collisionPointsMutex_.lock();
    collisionPoints_->push_back(si_->cloneState(workState));
    collisionPointsMutex_.unlock();
}
