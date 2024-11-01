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

#ifndef OMPL_INFEASIBILITY_SVMMANIFOLD
#define OMPL_INFEASIBILITY_SVMMANIFOLD

#include <ompl/infeasibility/Manifold.h>
#include <ompl/infeasibility/basic.h>
#include "ompl/base/PlannerData.h"
#include <ompl/base/State.h>
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/SpaceInformation.h"

#include <thundersvm/model/svc.h>
#include <thundersvm/svmparam.h>
#include <thundersvm/syncarray.h>
#include <thundersvm/model/svc.h>
#include <thundersvm/util/metric.h>
#include <thundersvm/util/log.h>

#include <string>
#include <nlopt.h>

namespace ompl
{
    namespace infeasibility
    {
        struct SVMModelData
        {
            // struct that saves svm model data. 
            SVMModelData()
            {
                b = 0;
                num_vectors = 0;
                gamma = 0;
                coef = NULL;
                vectors = NULL;
            };
            SVMModelData(const SVMModelData &prev)
            {
                b = prev.b;
                num_vectors = prev.num_vectors;
                gamma = prev.gamma;
                coef = prev.coef;
                vectors = prev.vectors;
            };
            float_inf b;
            int num_vectors;
            float_inf gamma;
            float_inf *coef;
            float_inf *vectors;
        };

        class SVMManifold : public ompl::infeasibility::Manifold
        {
        public:
            SVMManifold(base::SpaceInformationPtr si, std::string name, std::size_t ambDim, std::size_t coDim = 1);

            SVMManifold(const SVMManifold &source)
              : Manifold(source.name(), source.getAmbDim(), source.getCoDim())
              , si_(source.getSpaceInformation())
              , modelData_(source.getModelData()){};

            float_inf evalManifold(const base::State *point) override;
            bool learnManifold(const base::PlannerDataPtr &plannerData) override;
            bool sampleManifold(const base::State *seed, base::State *res) override;

            SVMModelData getModelData() const
            {
                return modelData_;
            };

            /** \brief Get the space information this manifold is in */
            const base::SpaceInformationPtr getSpaceInformation() const
            {
                return si_;
            };

            /** \brief make training data set from graph disjoint set*/
            void makeTrainingDataFromGraph(const base::PlannerDataPtr &plannerData);

            /** \brief setup training parameters */
            void trainingSetup();

            /** \brief save model data output from svm library to SVMModelData */
            void saveModelData();

        private:
            base::SpaceInformationPtr si_;

            SVMModelData modelData_; // saved model data
            DataSet dataset_; // dataset for training
            SvmParam param_; // parameter for svm training
            std::shared_ptr<SvmModel> model_; // model in thundersvm lib.

            /** \brief count the number of the two classes of sample points. The class with less number of points is the oneClass.*/
            unsigned int numOneClassPoints_{0};
            unsigned int numOtherClassPoints_{0};
        };
    }  // namespace infeasibility
}  // namespace ompl

#endif