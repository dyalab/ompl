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

            /** \brief use SDCL learned manifold to generate samples for planning */
            void generateSDCLSamples();

            /** \brief end the SDCL thread */
            void endSDCLThread();

            /** \brief return the number of manifold points used as samples */
            unsigned int numSDCLSamplesAdded()
            {
                return usedSDCLPointsCount_;
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
            bool outOfBounds(State *state);

            /** \brief return true if in virtual CFree */
            bool outOfBoundsCollision(State *state);

            /** \brief sample uniformly with virtual obstacle region and virutal free region */
            void sampleUniformWithMargin(State *state);

            /** \brief helper function to clear states in lists. */
            void clearStates(std::shared_ptr<std::vector<base::State *>> statelist);

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief The planner to get training data */
            PlannerPtr planner_;

            /** \brief the sdcl thread */
            std::thread SDCLThread_;

            /** \brief mark termination of the sdcl thread */
            std::atomic<bool> sdclThreadEnded_{false};

            /** \brief The sampler to build upon */
            std::shared_ptr<std::vector<base::State *>> SDCLPoints_;

            /** \brief valid SDCL points mutex*/
            mutable std::mutex SDCLPointsMutex_;

            /** \brief collision points mutex*/
            mutable std::mutex collisionPointsMutex_;

            /** \brief current number of valid SDCL points*/
            std::atomic<unsigned int> curSDCLPointsCount_{0};

            /** \brief count of used valid SDCL points*/
            std::atomic<unsigned int> usedSDCLPointsCount_{0};

            /** \brief the margin for virtual obstacle region and virtual free region outside of the boundaries.*/
            double delta_;

            /** \brief An instance of a random number generator */
            RNG rng_;

            /** \brief collision points, saved when sampling, used in sampleManifoldPoints.*/
            std::shared_ptr<std::vector<base::State *>> collisionPoints_;

            /** \brief the learned manifold, in manifold class*/
            std::shared_ptr<ompl::infeasibility::Manifold> manifold_;

            /** \brief upper and lower bound used in si*/
            std::vector<double> upperBound_;
            std::vector<double> lowerBound_;
        };
    }  // namespace base
}  // namespace ompl

#endif
