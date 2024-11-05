/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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

#include <ompl/infeasibility/SVMManifold.h>

namespace ompl
{
    namespace magic
    {
        /** \brief smallest training size to start training. */
        static const unsigned int MIN_TRAINING_SIZE = 50;
    }  // namespace magic
}  // namespace ompl

double objfunc(unsigned int n, const double *x, double *grad, void *data)
{
    ompl::infeasibility::SVMModelData *d = (ompl::infeasibility::SVMModelData *)data;
    double b = d->b;
    int num_vectors = d->num_vectors;
    double *coef = d->coef;
    double *vectors = d->vectors;
    double gamma = d->gamma;
    double f = 0;
    double dists_square[num_vectors];
    for (int k = 0; k < num_vectors; k++)
    {
        dists_square[k] = 0;
        for (unsigned int i = 0; i < n; i++)
        {
            dists_square[k] += pow(x[i] - vectors[n * k + i], 2);
        }
        f += coef[k] * exp(-gamma * dists_square[k]);
    }

    if (grad)
    {
        for (unsigned int i = 0; i < n; i++)
        {
            grad[i] = 0;
            for (int k = 0; k < num_vectors; k++)
            {
                grad[i] += coef[k] * exp(-gamma * dists_square[k]) * (-gamma) * 2 * (x[i] - vectors[n * k + i]);
            }
        }
        for (unsigned int i = 0; i < n; i++)
        {
            grad[i] = 2 * (f - b) * grad[i];
        }
    }

    return (f - b) * (f - b);
}

int findClosestPoint(double *res, int n, ompl::infeasibility::SVMModelData svm_data, std::vector<double> lower_bound,
                     std::vector<double> upper_bound)
{
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
    double nul[1];
    return result;
}

ompl::infeasibility::SVMManifold::SVMManifold(const base::SpaceInformationPtr si, std::string name, std::size_t ambDim,
                                              std::size_t coDim)
  : Manifold(name, ambDim, coDim), si_(si)
{
    trainingSetup();
}

double ompl::infeasibility::SVMManifold::evalManifold(const base::State *point)
{
    double f = 0;
    double dists_square[modelData_.num_vectors];
    int features = ambDim_;

    auto *rpoint = static_cast<const base::RealVectorStateSpace::StateType *>(point);

    for (int k = 0; k < modelData_.num_vectors; k++)
    {
        dists_square[k] = 0;
        for (int i = 0; i < features; i++)
        {
            dists_square[k] += pow(rpoint->values[i] - modelData_.vectors[k * features + i], 2);
        }
        f += modelData_.coef[k] * exp(-modelData_.gamma * dists_square[k]);
    }

    return f - modelData_.b;
}

bool ompl::infeasibility::SVMManifold::learnManifold(const base::PlannerDataPtr &plannerData)
{
    makeTrainingDataFromGraph(plannerData);

    // wait until there is a reasonable number of samples.
    if (numOneClassPoints_ == 0 || numOtherClassPoints_ == 0 ||
        numOneClassPoints_ + numOtherClassPoints_ < magic::MIN_TRAINING_SIZE)
        return false;

    // train RBF-kernel SVM with thunderSVM.
    model_->train(dataset_, param_);

    saveModelData();

    return true;
}

void ompl::infeasibility::SVMManifold::makeTrainingDataFromGraph(const base::PlannerDataPtr &plannerData)
{
    unsigned int data_size = plannerData->numVertices();
    unsigned int start_size = plannerData->numStartVertices();
    unsigned int goal_size = plannerData->numGoalVertices();
    int features = ambDim_;
    float *classes = new float[data_size];
    float *data = new float[(data_size)*features];
    numOneClassPoints_ = 0;
    numOtherClassPoints_ = 0;
    std::vector<int> start_tags;
    std::vector<int> goal_tags;

    // the number of vertices in the goal region and the start region.
    // then use the smaller region's points as one class when training.
    unsigned int n_start_region_points = 0;
    unsigned int n_goal_region_points = 0;

    for (unsigned int i = 0; i < start_size; i++)
    {
        start_tags.push_back(plannerData->getStartVertex(i).getTag());
        base::PlannerDataPtr subGraph(std::make_shared<base::PlannerData>(si_));
        plannerData->extractReachable(plannerData->getStartIndex(i), *subGraph);
        n_start_region_points += subGraph->numVertices();
    }

    for (unsigned int i = 0; i < goal_size; i++)
    {
        goal_tags.push_back(plannerData->getGoalVertex(i).getTag());
        base::PlannerDataPtr subGraph(std::make_shared<base::PlannerData>(si_));
        plannerData->extractReachable(plannerData->getGoalIndex(i), *subGraph);
        n_goal_region_points += subGraph->numVertices();
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
        base::PlannerDataVertex cur_vertex = plannerData->getVertex(i);
        int cur_tag = cur_vertex.getTag();
        const base::State *s = cur_vertex.getState();

        for (int j = 0; j < features; j++)
        {
            data[features * cur_index + j] = (float)s->as<base::RealVectorStateSpace::StateType>()->values[j];
        }

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
            classes[cur_index] = -1;
            numOneClassPoints_++;
        }
        else
        {
            classes[cur_index] = 1;
            numOtherClassPoints_++;
        }
    }

    dataset_.load_from_dense(data_size, features, data, classes);

    delete[] data;
    delete[] classes;
}

void ompl::infeasibility::SVMManifold::trainingSetup()
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

void ompl::infeasibility::SVMManifold::saveModelData()
{
    if (!modelData_.vectors)
        delete[] modelData_.vectors;

    if (!modelData_.coef)
        delete[] modelData_.coef;

    const double *rho_data = (model_->get_rho()).host_data();
    DataSet::node2d vectors = model_->svs();
    const double *coef_data = (model_->get_coef()).host_data();
    int features = si_->getStateDimension();

    modelData_.b = rho_data[0];
    modelData_.num_vectors = model_->total_sv();
    modelData_.gamma = param_.gamma;
    modelData_.coef = new double[modelData_.num_vectors];
    modelData_.vectors = new double[modelData_.num_vectors * features];
    for (int i = 0; i < modelData_.num_vectors; i++)
    {
        for (int j = 0; j < features; j++)
        {
            modelData_.vectors[i * features + j] = vectors[i][j].value;
        }
        modelData_.coef[i] = coef_data[i];
    }
}

bool ompl::infeasibility::SVMManifold::sampleManifold(const base::State *seed, base::State *res)
{
    double res_data[ambDim_] = {};
    auto *rseed = static_cast<const base::RealVectorStateSpace::StateType *>(seed);

    for (unsigned int ii = 0; ii < ambDim_; ii++)
    {
        res_data[ii] = rseed->values[ii];
    }

    // opt to find closest point on manifold
    int success = findClosestPoint(res_data, ambDim_, modelData_,
                                   (si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low,
                                   (si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high);
    auto *rres = static_cast<base::RealVectorStateSpace::StateType *>(res);

    for (unsigned int ii = 0; ii < ambDim_; ii++)
    {
        rres->values[ii] = res_data[ii];
    }

    double svm_value = evalManifold(res);

    if (success > 0 && abs(svm_value) < 0.001)
    {
        return true;
    }
    return false;
}
