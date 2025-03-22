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

#ifndef OMPL_INFEASIBILITY_SVMMANIFOLD
#define OMPL_INFEASIBILITY_SVMMANIFOLD

#include <ompl/infeasibility/Manifold.h>
#include "ompl/base/PlannerData.h"
#include <ompl/base/State.h>
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/SpaceInformation.h"

#include <string>
#include <nlopt.h>

#if OMPL_HAVE_THUNDERSVM
#include <thundersvm/model/svc.h>
#include <thundersvm/svmparam.h>
#include <thundersvm/syncarray.h>
#include <thundersvm/model/svc.h>
#include <thundersvm/util/metric.h>
#include <thundersvm/util/log.h>
#else
#include "ompl/infeasibility/libsvm/svm.h"
#endif

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
            double b;
            int num_vectors;
            double gamma;
            double *coef;
            double *vectors;
        };

        class SVMManifold : public ompl::infeasibility::Manifold
        {
        public:
            SVMManifold(base::SpaceInformationPtr si, std::size_t ambDim, std::size_t coDim = 1);

            SVMManifold(const SVMManifold &source)
              : Manifold(source.name(), source.getAmbDim(), source.getCoDim())
              , si_(source.getSpaceInformation())
              , modelData_(source.getModelData()){};

            ~SVMManifold();

            double evalManifold(const base::State *point) override;
            bool learnManifold(float* data, float* classes, std::size_t data_size) override;
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

        private:

            /** \brief setup training parameters */
            void trainingSetup();

            /** \brief save model data output from svm library to SVMModelData */
            void saveModelData();
            base::SpaceInformationPtr si_;

            SVMModelData modelData_; // saved model data
            // DataSet dataset_; // dataset for training
            #if OMPL_HAVE_THUNDERSVM
            SvmParam thunderSVMParam_; // parameter for svm training
            std::shared_ptr<SvmModel> thunderSVMModel_; // model in thundersvm lib.
            #else
            svm_parameter libSVMParam_;
            svm_problem prob_;
            svm_model *libSVMModel_; // model in libsvm.
            #endif
            
        };
    }  // namespace infeasibility
}  // namespace ompl

#endif