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
        struct SVMModelData : ModelData
        {
            // struct that saves svm model data. 
            SVMModelData()
            {
                b = 0;
                num_vectors = 0;
                gamma = 0;
                features = 0;
                coef = NULL;
                vectors = NULL;
            };
            SVMModelData(const SVMModelData &prev)
            {
                b = prev.b;
                num_vectors = prev.num_vectors;
                gamma = prev.gamma;
                features = prev.features;
                coef = prev.coef;
                vectors = prev.vectors;
            };
            void copy(const ModelData* resource)
            {
                const SVMModelData* prev = dynamic_cast<const SVMModelData*>(resource);
                b = prev->b;
                num_vectors = prev->num_vectors;
                gamma = prev->gamma;
                features = prev->features;

                if (!vectors)
                    delete[] vectors;
                if (!coef)
                    delete[] coef;

                coef = new float_tri[num_vectors];
                vectors = new float_tri[num_vectors * features];
                for (int i = 0; i < num_vectors; i++)
                {
                    for (int j = 0; j < features; j++)
                    {
                        vectors[i * features + j] = prev->vectors[i * features + j];
                    }
                    coef[i] = prev->coef[i];
                }

            };

            float_tri eval(float_tri* x) override
            {
                float_tri f = 0;
                float_tri dists_square[num_vectors];

                for (int k = 0; k < num_vectors; k++)
                {
                    dists_square[k] = 0;
                    for (int i = 0; i < features; i++)
                    {
                        dists_square[k] += pow(x[i] - vectors[k * features + i], 2);
                    }
                    f += coef[k] * exp(-gamma * dists_square[k]);
                }

                return f - b;
            }

            void clear() 
            {
                if (!vectors)
                    delete[] vectors;

                if (!coef)
                    delete[] coef;
            }

            void print() const 
            {
                std::cout << "Number of support vectors: " << num_vectors << coef[0] << vectors[8] << std::endl;
            }
            float_tri b;
            int num_vectors;
            int features;
            float_tri gamma;
            float_tri *coef;
            float_tri *vectors;
        };

        class SVMManifold : public ompl::infeasibility::Manifold
        {
        public:
            SVMManifold(std::size_t ambDim, std::size_t coDim = 1);

            // SVMManifold(const SVMManifold &source)
            //   : Manifold(source.name(), source.getAmbDim(), source.getCoDim())
            //   , modelData_(source.getModelData()){};

            ~SVMManifold();

            double evalManifold(const base::State *point) override;
             // __host__ __device__ float_tri evalManifold(const float_tri *point) override;
            void copyManifold(std::shared_ptr<ompl::infeasibility::Manifold>& srcManifold) override;
            bool learnManifold(float* data, float* classes, std::size_t data_size) override;
            bool sampleManifold(const base::State *seed, base::State *res, std::vector<double> lower_bounds, std::vector<double> upper_bounds) override;

            ModelData* getModelData() override
            {
                return modelData_;
            };

            std::string name() const override
            {
                return name_;
            }

        private:

            /** \brief setup training parameters */
            void trainingSetup();

            /** \brief save model data output from svm library to SVMModelData */
            void saveModelData();

            SVMModelData* modelData_; // saved model data
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