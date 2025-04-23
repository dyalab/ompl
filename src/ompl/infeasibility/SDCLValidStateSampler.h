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
#include <nlopt.h>
#include <ompl/infeasibility/Manifold.h>
#include <ompl/infeasibility/SVMManifold.h>
#include "ompl/infeasibility/triangulation.h"


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
            /** @brief A data structure for storing vector of points in SDCL part */
            using StateVec = std::vector<State*>;
            /** \brief Constructor, base sampler is uniform sampling, TODO: add option to use Gaussian sampling in the
             * future. */
            SDCLValidStateSampler(const SpaceInformation *si, const Planner* planner);

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

            /** \brief Set the learning method for the manifold */
            void setManifoldType(std::string type);

            std::string getManifoldType()
            {
                return type_;
            }
            
            /** \brief Get the lastest manifold that has all manifold points in collision*/
            bool getLastManifold(std::shared_ptr<ompl::infeasibility::Manifold>& returnManifold, float_tri*& returnManifoldPoints, std::size_t& numManifoldPoints);

            /** \brief Set to save manifold point each time sampleManifold is called successfully */
            void setSaveManifoldPoints()
            {
                saveManifoldPoints_ = true;
            }

            /** \brief helper function to clear states in vector of states. */
            void clearStateVec(std::shared_ptr<StateVec>& vec);

        protected:
            // using pt = std::vector<double>;

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            std::string type_;

            /** \brief The planner to get training data */
            const Planner* planner_; // use raw pointer not shared_ptr to prevent call to destructor.

            /** \brief Current planner data */
            PlannerDataPtr plannerData_;

            /** \brief the sdcl thread */
            std::thread SDCLThread_;

            /** \brief mark termination of the sdcl thread */
            std::atomic<bool> sdclThreadEnded_{false};

            /** \brief mark start of the sdcl thread */
            std::atomic<bool> sdclThreadStarted_{false};

            /** \brief current number of valid SDCL points*/
            std::atomic<unsigned int> curSDCLPointsCount_{0};

            /** \brief count of used valid SDCL points*/
            std::atomic<unsigned int> usedSDCLPointsCount_{0};
            
            /** \brief true if all manifold points of the current manifold are in collision*/
            std::atomic<bool> manifoldPointsAllInCollision_;

            /** \brief dimension of vector space */
            unsigned int dim_ = 0;

            /** \brief the size of the smallest allow training set size.*/
            unsigned int size_of_smallest_training_set_;

            /** \brief the margin for virtual obstacle region and virtual free region outside of the boundaries.*/
            double delta_;

            /** \brief An instance of a random number generator */
            RNG rng_;

            // training data
            float *data_;
            float *classes_;

            /** \brief the learned manifold, in manifold class*/
            std::shared_ptr<ompl::infeasibility::Manifold> manifold_;

            /** \brief the last learned manifold that has all manifold point in collision*/
            std::shared_ptr<ompl::infeasibility::Manifold> lastManifold_;

            /** \brief The CFree points on the manifold */
            std::shared_ptr<StateVec> SDCLPoints_;

            /** \brief save manifold points if true.  */
            bool saveManifoldPoints_ = false;

            /** \brief The points on the manifold */
            std::shared_ptr<StateVec> manifoldPoints_;

            /** \brief The points on the last manifold */
            std::shared_ptr<StateVec> lastManifoldPoints_;

            /** \brief collision points, saved when sampling, used in sampleManifoldPoints*/
            std::shared_ptr<StateVec> collisionPoints_;

            /** \brief C free points, saved when getting training data, used in sampleManifoldPoints*/
            std::shared_ptr<StateVec> freePoints_;

            /** \brief virtual C free points, saved when sampling, used for training*/
            std::shared_ptr<StateVec> virtualCfreePoints_;

            /** \brief virtual Cfree points mutex*/
            mutable std::mutex virtualCfreePointsMutex_;

            /** \brief collision points mutex*/
            mutable std::mutex collisionPointsMutex_;

            /** \brief Valid SDCL points mutex*/
            mutable std::mutex SDCLPointsMutex_;

            /** \brief manifold mutex*/
            mutable std::mutex manifoldMutex_;

            /** \brief manifold points mutex for saving to manifold points list from each different thread*/
            mutable std::mutex manifoldPointsMutex_;
            
            /** \brief last saved manifold mutex for saving and getting last manifold and last manifold points*/
            mutable std::mutex lastManifoldMutex_;

            /** \brief upper and lower bound used in opt formulation */
            std::vector<double> upper_bound_;
            std::vector<double> lower_bound_;

            SpaceInformationPtr sip_;

            /** \brief count the number of goal and start points. */
            unsigned int numOneClassPoints_{0};
            unsigned int numOtherClassPoints_{0};

            double makeTrainingDataTime = 0;
            double trainingTime = 0;
            double samplingTime = 0;

            /** \brief make training data set from graph disjoint set*/
            void makeTrainingDataFromGraph();

            /** \brief sampling points on manifold */
            void sampleManifoldPoints();

            /** \brief save state to collision points */
            void saveCollisionPoints(const State *workState);

            /** \brief save state to virtual cfree points */
            void saveVirtualCfreePoints(const State *workState);

            /** \brief calculate manifold points */
            // void calManifoldPoints(const pt input_point);

            void calManifoldPoints(const State* input_state);

            /** \brief whether state is within margin and valid */
            bool isValidWithInMargin(const State *state);

            /** \brief save out of bound state to collision points set or free points set. Return true if out of bound,
             * false otherwise */
            bool outOfBound(const State *state);

            /** \brief sample uniformly with virtual obstacle region and virutal free region */
            void sampleUniformWithMargin(State *state);
            
            /** \brief save the current manifold and manifold points. */
            void saveManifoldData();
        };
    }  // namespace base
}  // namespace ompl

#endif
