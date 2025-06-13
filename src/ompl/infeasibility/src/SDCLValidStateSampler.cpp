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
        /** \brief smallest training size to start training. */
        static const unsigned int MIN_TRAINING_SIZE = 50;

        /** \brief margin outside of configuration space boundaries to space extent fraction.*/
        static const double MARGIN_AS_SPACE_EXTENT_FRACTION = 0.02;

    }  // namespace magic
}  // namespace ompl

ompl::base::SDCLValidStateSampler::SDCLValidStateSampler(const SpaceInformation *si, const Planner* planner)
  : ValidStateSampler(si)
  , sampler_(si->allocStateSampler())
  , dim_(si_->getStateDimension())
  , sizeOfSmallestTrainingSet_(magic::MIN_TRAINING_SIZE)
  , delta_(si->getMaximumExtent() * magic::MARGIN_AS_SPACE_EXTENT_FRACTION)
  , upperBound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high)
  , lowerBound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low)
{
    name_ = "SDCL"; 
    planner_ = planner;
    SDCLPoints_.reset(new StateVec());
    manifoldPoints_.reset(new StateVec());
    lastManifoldPoints_.reset(new StateVec());
    virtualCfreePoints_.reset(new StateVec());
    collisionPoints_.reset(new StateVec());
    freePoints_.reset(new StateVec());

    sip_ = planner_->getSpaceInformation();

    plannerData_ = std::make_shared<PlannerData>(sip_);

    params_.declareParam<unsigned int>(
        "size_of_smallest_training_set", [this](unsigned int size) { setSizeSmallestTrainingSet(size); },
        [this] { return getSizeSmallestTrainingSet(); });
    params_.declareParam<double>(
        "virtual_C_margin", [this](double m) { setVirtualCMargin(m); }, [this] { return getVirtualCMargin(); });

    // set manifold type, default is SVM manifold
    setManifoldType("BRF-SVM");
}

ompl::base::SDCLValidStateSampler::~SDCLValidStateSampler()
{
    endSDCLThread();
    clearStateVec(SDCLPoints_);
    clearStateVec(manifoldPoints_);
    clearStateVec(lastManifoldPoints_);
    clearStateVec(collisionPoints_);
    clearStateVec(virtualCfreePoints_);
    clearStateVec(freePoints_);
    if (data_ != NULL) delete[] data_;
    if (classes_ != NULL) delete[] classes_;
    plannerData_->clear();
}

void ompl::base::SDCLValidStateSampler::clearStateVec(std::shared_ptr<StateVec>& vec)
{
    for (std::size_t i = 0; i < vec->size(); i++)
    {
        if ((*vec)[i] != nullptr) si_->freeState((*vec)[i]);
    }
    vec.reset(new StateVec());
}

void ompl::base::SDCLValidStateSampler::setManifoldType(std::string type)
{
    if (type == "BRF-SVM")
    {
        manifold_.reset(new ompl::infeasibility::SVMManifold(si_->getStateDimension()));
        lastManifold_.reset(new ompl::infeasibility::SVMManifold(si_->getStateDimension()));
    }
}

bool ompl::base::SDCLValidStateSampler::sample(State *state)
{
    unsigned int attempts = 0;
    bool valid = false;

    // start sdcl sampling thread if not already started.
    if (!sdclThreadStarted_)
    {
        SDCLThread_ = std::thread(&ompl::base::SDCLValidStateSampler::generateSDCLSamples, this);
        while (!sdclThreadStarted_)
            continue;  // wait until sdcl thread started.
    }

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
            valid = isValidWithInMargin(state);
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
    if (sdclThreadStarted_)
    {
        sdclThreadEnded_ = true;
        sdclThreadStarted_ = false;
        SDCLThread_.join();
        OMPL_INFORM("SDCL sampling ended. \nTotal time to setup training data: %f \nTotal time for training: "
                    "%f \nTotal time for sampling on the manifold: %f \nTotal manifold points used: %d \n", 
                    makeTrainingDataTime, trainingTime, samplingTime, usedSDCLPointsCount_.load());
    }
}

void ompl::base::SDCLValidStateSampler::sampleUniformWithMargin(State *state)
{
    auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
    for (unsigned int i = 0; i < dim_; ++i)  // sample with virtual C regions
        rstate->values[i] = rng_.uniformReal(lowerBound_[i] - delta_ * 2, upperBound_[i] + delta_ * 2);
}

bool ompl::base::SDCLValidStateSampler::outOfBound(const State *state)
{
    // return true if out of bound, return falst if within bound. Also saves the virtual collision points and virtual Cfree points. 
    auto *rstate = state->as<base::RealVectorStateSpace::StateType>();

    // check whether the state is out of bound. 
    for (unsigned int i = 0; i < dim_; i++)
    {
        if (rstate->values[i] < lowerBound_[i] - delta_ || rstate->values[i] > upperBound_[i] + delta_)
        {
            // in virtual Cfree region
            saveVirtualCfreePoints(state);
            return true;
        }
        else if (rstate->values[i] < lowerBound_[i] || rstate->values[i] > upperBound_[i])
        {
            // in virtual collision region
            saveCollisionPoints(state);
            return true;
        }
    }

    return false;
}

bool ompl::base::SDCLValidStateSampler::isValidWithInMargin(const State *state)
{
    // if state is out of bound, return false, if state is within bound, perform collision checking. 
    if (!outOfBound(state))
        return si_->isValid(state);

    return false;
}

void ompl::base::SDCLValidStateSampler::generateSDCLSamples()
{
    sdclThreadStarted_ = true;

    OMPL_INFORM("SDCL sampling thread started ");

    std::chrono::high_resolution_clock timer;

    while (!sdclThreadEnded_)
    {
        if (planner_ == NULL)
            continue;
        if (!planner_->isSetup())
            continue;  // wait unil the planner is setup.

        auto start = timer.now();
        makeTrainingDataFromGraph();
        auto stop = timer.now();
        std::chrono::duration<float> data_time = stop - start;
        makeTrainingDataTime += data_time.count();

        // wait until there is a reasonable number of samples.
        if (numOneClassPoints_ == 0 || numOtherClassPoints_ == 0 ||
            numOneClassPoints_ + numOtherClassPoints_ < sizeOfSmallestTrainingSet_)
            continue;
  
        // train manifold.
        start = timer.now();
        manifoldMutex_.lock();
        bool success = manifold_->learnManifold(data_, classes_, numOneClassPoints_ + numOtherClassPoints_);
        manifoldMutex_.unlock();
        stop = timer.now();
        std::chrono::duration<float> training_time = stop - start;
        trainingTime += training_time.count();

        clearStateVec(manifoldPoints_);

        // sample manifold points
        if (success)
        {
            start = timer.now();
            sampleManifoldPoints();
            stop = timer.now();
            std::chrono::duration<float> sampling_time = stop - start;
            samplingTime += sampling_time.count();
        }
    }
}

void ompl::base::SDCLValidStateSampler::makeTrainingDataFromGraph()
{   
    // get current planner data. 
    planner_->getPlannerData(*plannerData_);
    unsigned int data_size = plannerData_->numVertices();

    // need a copy of the virtual cfree points for read and write conflicts.
    std::shared_ptr<StateVec> virtualCP_copy;
    virtualCP_copy.reset(new StateVec());
    virtualCfreePointsMutex_.lock();
    unsigned int virtual_free_data_size = virtualCfreePoints_->size();
    for (unsigned int i = 0; i < virtual_free_data_size; i++)
    {
        virtualCP_copy->push_back((*virtualCfreePoints_)[i]); // copy pointers 
    }
    virtualCfreePointsMutex_.unlock();

    // prepare for training data container.
    unsigned int start_size = plannerData_->numStartVertices();
    unsigned int goal_size = plannerData_->numGoalVertices();
    int features = dim_;
    if (data_ != NULL) delete[] data_;
    if (classes_ != NULL) delete[] classes_;
    classes_ = new float[data_size + virtual_free_data_size];
    data_ = new float[(data_size + virtual_free_data_size) * features];
    
    // save the cfree points for sampling on the manifold. 
    clearStateVec(freePoints_);
    freePoints_.reset(new StateVec(data_size + virtual_free_data_size, nullptr));  // for sampling on the manifold
    
    // the number of vertices in the goal region and the start region.
    // then use the smaller region's points as one class when training.
    unsigned int n_start_region_points = 0;
    unsigned int n_goal_region_points = 0;
    numOneClassPoints_ = 0;
    numOtherClassPoints_ = 0;
    std::vector<int> start_tags;
    std::vector<int> goal_tags;

    for (unsigned int i = 0; i < start_size; i++)
    {
        start_tags.push_back(plannerData_->getStartVertex(i).getTag());
        PlannerDataPtr subGraph(std::make_shared<PlannerData>(sip_));
        plannerData_->extractReachable(plannerData_->getStartIndex(i), *subGraph);
        n_start_region_points += subGraph->numVertices();
        subGraph->clear();
    }

    for (unsigned int i = 0; i < goal_size; i++)
    {
        goal_tags.push_back(plannerData_->getGoalVertex(i).getTag());
        PlannerDataPtr subGraph(std::make_shared<PlannerData>(sip_));
        plannerData_->extractReachable(plannerData_->getGoalIndex(i), *subGraph);
        n_goal_region_points += subGraph->numVertices();
        subGraph->clear();
    }

    std::vector<int> oneClassTags;

    if (n_goal_region_points > n_start_region_points)
    {
        oneClassTags = start_tags;
    }
    else
    {
        oneClassTags = goal_tags;
    }

    bool inOneClass = false;
    int cur_index = 0;

    for (unsigned int i = 0; i < data_size; i++, cur_index++)
    {
        PlannerDataVertex cur_vertex = plannerData_->getVertex(i);
        int cur_tag = cur_vertex.getTag();
        const State *s = cur_vertex.getState();

        for (int j = 0; j < features; j++)
        {
            data_[features * cur_index + j] = (float)s->as<base::RealVectorStateSpace::StateType>()->values[j];
        }

        (*freePoints_)[cur_index] = si_->allocState();
        si_->copyState((*freePoints_)[cur_index], s);

        // whether current point is in one class.
        if (std::find(oneClassTags.begin(), oneClassTags.end(), cur_tag) != oneClassTags.end())
        {
            inOneClass = true;
        }
        else
        {
            inOneClass = false;
        } 

        if (inOneClass)
        {
            classes_[cur_index] = -1;
            numOneClassPoints_++;
        }
        else
        {
            classes_[cur_index] = 1;
            numOtherClassPoints_++;
        }
    }

    // sample in virtual Cfree regions and add to training set.
    for (unsigned int i = 0; i < virtual_free_data_size; i++, cur_index++)
    {
        for (int j = 0; j < features; j++)
        {
            data_[features * cur_index + j] = (float)(((*virtualCP_copy)[i])->as<base::RealVectorStateSpace::StateType>()->values[j]);
            // (*freePoints_)[cur_index][j] = (double)(*virtualCP_copy)[i][j];
        }
        (*freePoints_)[cur_index] = si_->allocState();
        si_->copyState((*freePoints_)[cur_index], (*virtualCP_copy)[i]);

        classes_[cur_index] = 1;
        numOtherClassPoints_++;
    }
    // OMPL_INFORM("There are %d one class points, %d other class points", numOneClassPoints_, numOtherClassPoints_);
}

void ompl::base::SDCLValidStateSampler::saveCollisionPoints(const State *workState)
{
    if (!collisionPoints_)
        collisionPoints_.reset(new StateVec());

    State *state = si_->allocState();
    si_->copyState(state, workState);

    collisionPointsMutex_.lock();
    collisionPoints_->push_back(state);
    collisionPointsMutex_.unlock();
}

void ompl::base::SDCLValidStateSampler::saveVirtualCfreePoints(const State *workState)
{
    if (!virtualCfreePoints_)
        virtualCfreePoints_.reset(new StateVec());

    State *state = si_->allocState();
    si_->copyState(state, workState);

    virtualCfreePointsMutex_.lock();
    virtualCfreePoints_->push_back(state);
    virtualCfreePointsMutex_.unlock();
}

void ompl::base::SDCLValidStateSampler::sampleManifoldPoints()
{
    // need a copy of the collision points for read and write conflicts.
    std::shared_ptr<StateVec> collision_copy;
    collision_copy.reset(new StateVec());
    // avoid segfault when no collision points are added.
    if (!collisionPoints_)
        collisionPoints_.reset(new StateVec());
    collisionPointsMutex_.lock();
    int num_collision_points = collisionPoints_->size();
    for (int i = 0; i < num_collision_points; i++)
    {
        collision_copy->push_back((*collisionPoints_)[i]);
    }
    collisionPointsMutex_.unlock();

    // clear previous manifold points
    clearStateVec(manifoldPoints_);

    int num_free_points = freePoints_->size();

    // start thread pool
    int num_threads = std::thread::hardware_concurrency();
    // OMPL_INFORM("Thread pool for calculating manifold points has %d threads.", num_threads);
    boost::asio::thread_pool threadpool(num_threads - 4);

    // save current curSDCLPointsCount_
    int prevSDCLPointsCount_ = curSDCLPointsCount_;

    // loop to add thread pool
    for (int i = 0; i < num_collision_points; i++)
    {
        boost::asio::post(threadpool, [collision_copy, i, this] { calManifoldPoints((*collision_copy)[i]); });
    }
    for (int i = 0; i < num_free_points; i++)
    {
        boost::asio::post(threadpool, [&, i, this] { calManifoldPoints((*freePoints_)[i]); });
    }

    threadpool.join();
 
    // if no SDCL points are added, save the current manifold data. 
    if (prevSDCLPointsCount_ == curSDCLPointsCount_)
    {
        (dynamic_cast<ompl::infeasibility::SVMManifold*>(manifold_.get()))->getModelData()->print();
        // if (manifoldPoints_->size() > 10) 
        //     std::cout << "after sampling on manifold " << (*manifoldPoints_)[10]->as<base::RealVectorStateSpace::StateType>()->values[5] << std::endl;
        manifoldPointsAllInCollision_ = true;
        saveManifoldData();
    } else {
        manifoldPointsAllInCollision_ = false;
        clearStateVec(lastManifoldPoints_);
    }

    // OMPL_INFORM("There are %d collision points, %d training points", num_collision_points, num_free_points);
}

void ompl::base::SDCLValidStateSampler::calManifoldPoints(const State* input_state)
{
    State *res_state = si_->allocState();
    bool success = manifold_->sampleManifold(input_state, res_state, 
                                             (si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low,
                                             (si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high);

    if (success)
    {
        if (si_->isValid(res_state))
        {
            SDCLPointsMutex_.lock();
            SDCLPoints_->push_back(res_state);
            SDCLPointsMutex_.unlock();
            curSDCLPointsCount_++;
            return;
        }
        else if (saveManifoldPoints_)
        {
            manifoldPointsMutex_.lock();
            manifoldPoints_->push_back(res_state);
            manifoldPointsMutex_.unlock();
            return;
        }
    }
    si_->freeState(res_state);
}

void ompl::base::SDCLValidStateSampler::saveManifoldData() 
{
    std::lock_guard<std::mutex> _(lastManifoldMutex_);
    // save model data
    lastManifold_->getModelData()->copy(manifold_->getModelData());
    // std::cout << "saved in sampler " << std::endl;
    // (dynamic_cast<ompl::infeasibility::SVMManifold*>(lastManifold_.get()))->getModelData()->print();
    // save manifold points
    clearStateVec(lastManifoldPoints_);
    lastManifoldPoints_.reset(new StateVec(manifoldPoints_->size(), nullptr));  // for sampling on the manifold
    for (std::size_t i = 0; i < manifoldPoints_->size(); i++) 
    {
        (*lastManifoldPoints_)[i] = si_->allocState();
        si_->copyState((*lastManifoldPoints_)[i], (*manifoldPoints_)[i]);
    }
    // if (manifoldPoints_->size() > 10) 
    //     std::cout << "saved in sampler " << (*lastManifoldPoints_)[10]->as<base::RealVectorStateSpace::StateType>()->values[5] << std::endl;
}

bool ompl::base::SDCLValidStateSampler::getLastManifold(std::shared_ptr<ompl::infeasibility::Manifold>& returnManifold, 
    float_tri*& returnManifoldPoints, std::size_t& numManifoldPoints)
{
    std::lock_guard<std::mutex> _(lastManifoldMutex_);

    if (lastManifoldPoints_->size() == 0) return false;

    // get manifold
    returnManifold->getModelData()->copy(lastManifold_->getModelData());
    
    // get manifold points
    // clearStateVec(returnManifoldPoints);
    // returnManifoldPoints.reset(new StateVec(lastManifoldPoints_->size(), nullptr));  // for sampling on the manifold
    numManifoldPoints = lastManifoldPoints_->size();
    returnManifoldPoints = (float_tri*)malloc(sizeof(float_tri) * numManifoldPoints * dim_);

    for (std::size_t i = 0; i < lastManifoldPoints_->size(); i++) 
    {
        for (int j = 0; j < dim_; j++) {
            returnManifoldPoints[i * dim_ + j] = (float_tri)((*lastManifoldPoints_)[i]->as<base::RealVectorStateSpace::StateType>()->values[j]);
        }
        // (*returnManifoldPoints)[i] = si_->allocState();
        // si_->copyState((*returnManifoldPoints)[i], (*lastManifoldPoints_)[i]);
    }

    // if (manifoldPoints_->size() > 10) 
    //     std::cout << "in get manifold " << (*returnManifoldPoints)[10]->as<base::RealVectorStateSpace::StateType>()->values[5] << std::endl;

    return true;
}