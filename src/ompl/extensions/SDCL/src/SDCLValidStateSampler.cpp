/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, Colorado School of Mines
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

/* Author: Ioan Sucan */

#include "ompl/extensions/SDCL/SDCLValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"

double objfunc(unsigned int n, const double *x, double *grad, void *data)
{
    ModelData *d = (ModelData *) data;
    double b = d->b;
    int num_vectors = d->num_vectors;
    double* coef = d->coef;
    double* vectors = d->vectors;
    double gamma = d->gamma;
    double f = 0;
    double dists_square[num_vectors];
    for(int k = 0; k < num_vectors; k++){
        dists_square[k] = 0;
        for(unsigned int i = 0; i < n; i ++){
            dists_square[k] += pow(x[i] - vectors[n*k+i], 2);
        }
        f += coef[k] * exp(-gamma * dists_square[k]);
    }

    if (grad) {
        for(unsigned int i = 0; i < n; i++){
            grad[i] = 0;
            for(int k = 0; k < num_vectors; k++){
                grad[i] += coef[k] * exp(-gamma * dists_square[k]) * (-gamma) * 2 * (x[i] - vectors[n*k+i]);
            }
        }
    }

    for(unsigned int i = 0; i < n; i++) {
        grad[i] = 2 * (f-b) * grad[i];
    }

    return (f-b)*(f-b);
}

int findClosestPoint(double *res, int n, ModelData svm_data, std::vector<double> lower_bound, std::vector<double> upper_bound){
    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LD_SLSQP, n);
    nlopt_set_min_objective(opt, objfunc, &svm_data);
    double maxtime = 0.1;
    nlopt_set_maxtime(opt, maxtime);
    nlopt_set_ftol_rel(opt, 1e-3);

    double lb[n] = {};
    double ub[n] = {};

    for (int i = 0; i < n; i++) 
    {
        lb[i] = lower_bound[i];
        ub[i] = upper_bound[i];
    }
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);

    double minf;
    int result = nlopt_optimize(opt, res, &minf);
    nlopt_destroy(opt);
    // double nul[1];
    return result;
}

ompl::base::SDCLValidStateSampler::SDCLValidStateSampler(const SpaceInformation *si, const PlannerPtr planner)
  : ValidStateSampler(si)
  , sampler_(si->allocStateSampler())
  , planner_(planner)
  , upper_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high)
  , lower_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low)
{
    name_ = "SDCL";
    // planner_ = planner;
    // lower_bound_ = (si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low;
    // upper_bound_ = (si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high

    // setup a seperate thread for generating narrow passage samples. 
    std::thread infThread([this] {generateSDCLSamples(); });
}

bool ompl::base::SDCLValidStateSampler::sample(State *state)
{
    unsigned int attempts = 0;
    bool valid = false;
    do
    {
        // use SDCL points when available, if not, use uniform sampling
        if (curSDCLPointsCount_ > usedSDCLPointsCount_) {
            valid = true;
            const unsigned int dim = si_->getStateDimension();
            auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
            for (unsigned int i = 0; i < dim; ++i)
                rstate->values[i] = (*SDCLPoints_)[usedSDCLPointsCount_][i];
            usedSDCLPointsCount_++;
        } else {
            sampler_->sampleUniform(state);
            valid = si_->isValid(state);
            if (!valid) saveCollisionPoints(state);
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

void ompl::base::SDCLValidStateSampler::generateSDCLSamples() 
{
    trainingSetup();

    while (true)  // TODO: while planner is still running
    {
        makeTrainingDataFromGraph();

        if (numOneClassPoints_ == 0 || numOtherClassPoints_ == 0) continue;

        // train RBF-kernel SVM with thunderSVM
        model_->train(dataset_, param_);
        saveModelData();

        // sample manifold points
        sampleManifoldPoints();
    }
}

void ompl::base::SDCLValidStateSampler::trainingSetup()
{
    model_.reset(new SVC());
    param_.kernel_type = SvmParam::RBF;
    param_.degree = 3;
    param_.gamma = 1.0;
    param_.coef0 = 0;
    param_.nu = 0.5;
    param_.C = 100;
    param_.epsilon = 1e-3;
    param_.p = 0.1;
    param_.probability = 0;
    param_.nr_weight = 0;
    param_.weight_label = NULL;
    param_.weight = NULL;

    // thunderSVM logging config. 
    el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);
    el::Loggers::setLoggingLevel(el::Level::Unknown);
}

void ompl::base::SDCLValidStateSampler::makeTrainingDataFromGraph() 
{
    PlannerDataPtr plannerData;
    planner_->getPlannerData(*plannerData);

    unsigned int data_size = plannerData->numVertices();
    unsigned int start_size = plannerData->numVertices();
    unsigned int goal_size = plannerData->numVertices();
    int features = si_->getStateDimension();
    float* classes = new float[data_size];
    float* data = new float[data_size * features];
    
    freePoints_.reset(new pvec(data_size, pt(features, 0.0)));
    numOneClassPoints_ = 0;
    numOtherClassPoints_ = 0;
    std::vector<int> start_tags;
    std::vector<int> goal_tags;

    // the number of vertices in the goal region and the start region. 
    // then use the smaller region's points as one class when training.
    unsigned int n_start_region_points = 0;
    unsigned int n_goal_region_points = 0; 

    for (unsigned int i = 0; i < start_size; i++) {
        start_tags.push_back(plannerData->getStartVertex(i).getTag());
        PlannerDataPtr subGraph;
        plannerData->extractReachable(plannerData->getStartIndex(i), *subGraph);
        n_start_region_points += subGraph->numVertices();
    }

    for (unsigned int i = 0; i < goal_size; i++) {
        goal_tags.push_back(plannerData->getGoalVertex(i).getTag());
        PlannerDataPtr subGraph;
        plannerData->extractReachable(plannerData->getGoalIndex(i), *subGraph);
        n_goal_region_points += subGraph->numVertices();
    }

    std::vector<int> oneClassTags;

    if (n_goal_region_points > n_start_region_points) {
        oneClassTags = start_tags;
    } else {
        oneClassTags = goal_tags;
    }

    bool inOneClass = false;
    int cur_index = 0;

    for(unsigned int i = 0; i < data_size; i++, cur_index++)
    {
        PlannerDataVertex cur_vertex = plannerData->getVertex(i);
        int cur_tag = cur_vertex.getTag();
        const base::State* s = cur_vertex.getState();

        for (int j = 0; j < features; j++)
        {
            data[features * cur_index + j] = (float)s->as<base::RealVectorStateSpace::StateType>()->values[j];
            (*freePoints_)[cur_index][j] = (double)s->as<base::RealVectorStateSpace::StateType>()->values[j];
        }

        // whether current point is in one class. 
        if(std::find(oneClassTags.begin(), oneClassTags.end(), cur_tag) != oneClassTags.end()) {
            inOneClass = true;
        } else {
            inOneClass = false;
        }

        if (inOneClass) 
        {
            classes[cur_index] = -1;
            numOneClassPoints_++;
        } else
        {
            classes[cur_index] = 1;
            numOtherClassPoints_++;
        }
    }
    
    dataset_.load_from_dense(data_size, features, data, classes);

    OMPL_INFORM("Number of one class points is %d, number of other class points is %d", numOneClassPoints_, numOtherClassPoints_);

    delete [] data;
    delete [] classes;
}

void ompl::base::SDCLValidStateSampler::saveCollisionPoints(base::State *workState) 
{
    if (!collisionPoints_) collisionPoints_.reset(new pvec());

    const unsigned int dim = si_->getStateDimension();
    pt point(dim);
    for (unsigned int i = 0; i < dim; i++) {
        point[i] = (double)workState->as<base::RealVectorStateSpace::StateType>()->values[i];
    }
    collisionPointsMutex_.lock();
    collisionPoints_->push_back(point);
    collisionPointsMutex_.unlock();
}


double ompl::base::SDCLValidStateSampler::evaluate(double* point) {
    if (savedModelData_.num_vectors == 0) saveModelData();

    double f = 0;
    double dists_square[savedModelData_.num_vectors];
    int features = si_->getStateDimension();

    for(int k = 0; k < savedModelData_.num_vectors; k++){
        dists_square[k] = 0;
        for(int i = 0; i < features; i++){
            dists_square[k] += pow(point[i] - savedModelData_.vectors[features*k+i], 2);
        }
        f += savedModelData_.coef[k] * exp(-savedModelData_.gamma * dists_square[k]);
    }

    return f-savedModelData_.b; 
}

void ompl::base::SDCLValidStateSampler::saveModelData() {
    if (!savedModelData_.vectors) delete [] savedModelData_.vectors;

    if (!savedModelData_.coef) delete [] savedModelData_.coef;

    const float_type* rho_data = (model_->get_rho()).host_data();
    DataSet::node2d vectors = model_->svs();
    const float_type *coef_data = (model_->get_coef()).host_data();
    int features = si_->getStateDimension();

    savedModelData_ = ModelData();
    savedModelData_.b = rho_data[0];
    savedModelData_.num_vectors = model_->total_sv();
    savedModelData_.gamma = param_.gamma;
    savedModelData_.coef = new double[savedModelData_.num_vectors];
    savedModelData_.vectors = new double[savedModelData_.num_vectors * features];
    for (int i = 0; i < savedModelData_.num_vectors; i++) {
        for (int j = 0; j < features; j++) {
            savedModelData_.vectors[i * features + j] = vectors[i][j].value;
        }
        savedModelData_.coef[i] = coef_data[i];
    }
}

void ompl::base::SDCLValidStateSampler::sampleManifoldPoints()
{
    // need a copy of the collision points for read and write conflicts.
    pvec collision_copy;
    // avoid segfault when no collision points are added.
    if (!collisionPoints_) collisionPoints_.reset(new pvec()); 
    collisionPointsMutex_.lock();
    int num_collision_points = collisionPoints_->size();
    copy(collisionPoints_->begin(), collisionPoints_->end(), back_inserter(collision_copy));
    collisionPointsMutex_.unlock();

    int num_free_points = freePoints_->size();
    
    // start thread pool
    int num_threads = std::thread::hardware_concurrency();
    OMPL_INFORM("Thread pool for calculating manifold points has %d threads.", num_threads);
    boost::asio::thread_pool threadpool(num_threads);
    
    // loop to add thread pool
    for (int i = 0; i < num_collision_points; i++) {
        boost::asio::post(threadpool, [&, this]{calManifoldPoints(collision_copy[i]);});
    }

    for (int i = 0; i < num_free_points; i++) {
        boost::asio::post(threadpool, [&, this]{calManifoldPoints((*freePoints_)[i]);});
    }

    threadpool.join();

    OMPL_INFORM("There are %d collision points, %d training points", num_collision_points, num_free_points);
}

void ompl::base::SDCLValidStateSampler::calManifoldPoints(pt intput_point){
    const unsigned int dim = si_->getStateDimension();
    double res_data[dim] = {};
    for (unsigned int ii = 0; ii < dim; ii++) {
        res_data[ii] = intput_point[ii];
    }

    // opt to find closest point on manifold
    int success = findClosestPoint(res_data, dim, savedModelData_, lower_bound_, upper_bound_);

    double svm_value = evaluate(res_data);
    base::State *res_state = si_->allocState();
    auto *rstate = static_cast<base::RealVectorStateSpace::StateType *>(res_state);

    if (success > 0 && abs(svm_value) < 0.001) {
        pt res_point(dim, 0); 
        for (unsigned int ii = 0; ii < dim; ii++) {
            res_point[ii] = res_data[ii];
            rstate->values[ii] = res_data[ii];
        }

        if (si_->isValid(rstate)) {
            SDCLPointsMutex_.lock();
            SDCLPoints_->push_back(res_point);
            SDCLPointsMutex_.unlock();
            curSDCLPointsCount_++;
        }
    } 

    si_->freeState(res_state);
}
