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

#ifndef OMPL_BASE_SAMPLERS_SDCL_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_SDCL_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/PlannerData.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/Planner.h"
#include "ompl/tools/config/MagicConstants.h"
#include <ompl/infeasibility/Manifold.h>
#include <ompl/infeasibility/SVMManifold.h>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/thread_pool.hpp>
#include <thundersvm/svmparam.h>
#include <thundersvm/syncarray.h>
#include <thundersvm/model/svc.h>
#include <thundersvm/util/metric.h>
#include <thundersvm/util/log.h>
#include <algorithm>
#include <mutex>
#include <vector>
#include <atomic>
#include <thread>

struct ModelData
{
    ModelData()
    {
        b = 0;
        num_vectors = 0;
        gamma = 0;
        coef = NULL;
        vectors = NULL;
    };
    double b;
    int num_vectors;
    double gamma;
    double *coef;
    double *vectors;
};

double objfunc(unsigned n, const double *x, double *grad, void *data);
int findClosestPoint(double *p, double *startingP, double *res, int n, ModelData svm_data);

namespace ompl
{
    namespace base
    {
        /** \brief A state sampler that only samples valid states, detail method is in the
         * paper "Sample-Driven Connectivity Learning for Motion Planning
         * in Narrow Passages". */
        class SDCLValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor, base sampler is uniform sampling*/
            SDCLValidStateSampler(const SpaceInformation *si, const PlannerPtr planner);

            ~SDCLValidStateSampler() override;

            bool sample(State *state) override;
            bool sampleNear(State *state, const State *near, double distance) override;

            /** \brief use SDCL proof manifold to generate samples for planning */
            void generateSDCLSamples();

            /** \brief end the SDCL thread */
            void endSDCLThread();

            /** \brief return the number of manifold points added to search */
            unsigned int numSDCLSamplesAdded()
            {
                return usedSDCLPointsCount_;
            }

            /** \brief Get the size of the smallest training set for training the manifold */
            unsigned int getSizeSmallestTrainingSet() const
            {
                return size_of_smallest_training_set_;
            }

            /** \brief Set the size of the smallest training set for training the manifold */
            void setSizeSmallestTrainingSet(unsigned int size)
            {
                size_of_smallest_training_set_ = size;
            }

            /** \brief Get virtual obstacle region and free region margin */
            double getVirtualCMargin() const
            {
                return delta_;
            }

            /** \brief Set the size of the smallest training set for training the manifold */
            void setVirtualCMargin(double m)
            {
                delta_ = m;
            }

        protected:
            using pt = std::vector<double>;

            /** @brief A data structure for storing vector of points in SDCL part */
            using pvec = std::vector<std::vector<double>>;

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief The planner to get training data */
            PlannerPtr planner_;

            /** \brief the sdcl thread */
            std::thread SDCLThread_;

            /** \brief mark termination of the sdcl thread */
            std::atomic<bool> sdclThreadEnded_{false};

            /** \brief mark start of the sdcl thread */
            std::atomic<bool> sdclThreadStarted_{false};

            /** \brief The sampler to build upon */
            std::shared_ptr<std::vector<base::State *>> SDCLPoints_;

            /** \brief valid SDCL points mutex*/
            mutable std::mutex SDCLPointsMutex_;

            /** \brief current number of valid SDCL points*/
            std::atomic<unsigned int> curSDCLPointsCount_{0};

            /** \brief count of used valid SDCL points*/
            std::atomic<unsigned int> usedSDCLPointsCount_{0};

            /** \brief counts the number of times the sampler has been called.*/
            std::atomic<unsigned int> samplingCount_{0};

            /** \brief the size of the smallest allow training set size.*/
            unsigned int size_of_smallest_training_set_;

            /** \brief the margin for virtual obstacle region and virtual free region outside of the boundaries.*/
            double delta_;

            /** \brief An instance of a random number generator */
            RNG rng_;

            /** \brief svm model related data, TODO: use the ompl::base::Constraint class? */
            DataSet dataset_;
            SvmParam param_;
            std::shared_ptr<SvmModel> model_;
            ModelData savedModelData_;

            /** \brief collision points, saved when sampling, used in sampleManifoldPoints*/
            std::shared_ptr<std::vector<base::State *>> collisionPoints_;
            std::shared_ptr<std::vector<base::State *>> freePoints_;
            std::shared_ptr<ompl::infeasibility::Manifold> manifold_;

            /** \brief C free points, saved when getting training data, used in sampleManifoldPoints*/
            // std::shared_ptr<pvec> freePoints_;

            /** \brief virtual C free points, saved when sampling, used for training*/
            // std::shared_ptr<pvec> virtualCfreePoints_;

            /** \brief virtual Cfree points mutex*/
            // mutable std::mutex virtualCfreePointsMutex_;

            /** \brief collision points mutex*/
            mutable std::mutex collisionPointsMutex_;

            /** \brief upper and lower bound used in opt formulation */
            std::vector<double> upper_bound_;
            std::vector<double> lower_bound_;

            /** \brief count the number of goal and start points. */
            // unsigned int numOneClassPoints_{0};
            // unsigned int numOtherClassPoints_{0};

            /** \brief make training data set from graph disjoint set*/
            // void makeTrainingDataFromGraph();

            /** \brief Setup training parameters */
            // void trainingSetup();

            /** \brief sampling points on manifold */
            void sampleManifoldPoints();

            /** \brief save state to collision points */
            void saveCollisionPoints(base::State *workState);

            void getCfreePoints();

            /** \brief save state to virtual cfree points */
            // void saveVirtualCfreePoints(base::State *workState);

            /** \brief calculate manifold points */
            void calManifoldPoints(const base::State *input_state);

            // /* \brief calcualte the value of the manifold function with given point
            // double evaluate(const double *point);

            // /** \brief save model data to data structure */
            // void saveModelData();

            /** \brief whether state is valid with virtual CFree and Cobs */
            bool isValidWithMargin(State *state);

            /** \brief return true if in virtual Cobs */
            bool outOfLimits(State *state);

            /** \brief return true if in virtual CFree */
            bool outOfLimitsCollision(State *state);

            /** \brief sample uniformly with virtual obstacle region and virutal free region */
            void sampleUniformWithMargin(State *state);

            void clearStates(std::shared_ptr<std::vector<base::State *>> statelist);
        };
    }  // namespace base
}  // namespace ompl

#endif
