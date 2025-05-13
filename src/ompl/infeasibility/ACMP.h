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



#ifndef ACMP_H
#define ACMP_H

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/base/ValidStateSampler.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include <ompl/config.h>
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/copy.hpp>
#include <ompl/infeasibility/Manifold.h>
#include <ompl/infeasibility/SVMManifold.h>
#include "ompl/infeasibility/SDCLValidStateSampler.h"
#include "ompl/infeasibility/triangulation.h"


using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oi = ompl::infeasibility;


struct proof_stats {
    proof_stats() : n_valid_mpoints(0){;}
    int solved = 0;
    int inf = 0;
    int n_facets = 0;
    int n_collision_points = 0;
    std::atomic<int> n_valid_mpoints;
    int n_itr = 0;
    int n_SDCL_itr = 0;
    int n_checking_itr = 0;
    int n_simplex_facets = 0;
    double tc_time = 0;
    double mt_time = 0;
    double mt_par_time = 0;
    double mt_par_libcuckoo_time = 0;
    double fcl_time = 0;
    double facet_time = 0;
    double cs_time = 0; // cell complex time
    double cs_old_time = 0;
    double cs_new_time = 0;
    double cs_time_par = 0;
    double cs_time_par_mtcc = 0;
    double saving_time = 0;
    double saving_par_time= 0;
    double mpoints_time = 0;
    double check_time = 0;
    double decompose_time = 0;
    double check_collision_time = 0;
    double total_time = 0;
    double training_time = 0;
    double sampling_time = 0;
    bool useGaussian = false;
    bool useTraining = false;
    bool useSquare = false;
};


namespace ompl
{
    namespace geometric
    {
        /** Asymtoptic complete motion planning */
        class ACMP : public ompl::geometric::PRM
        {
        public:
            /** \brief Constructor */
            ACMP(const base::SpaceInformationPtr &si, double lambda=0.1, bool starStrategy = false);

            ~ACMP();

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

        protected:
            /** \brief algorihtm runing stats recoding data structure */
            proof_stats stats_ = {};

            bool foundInfProof_{false};

            bool foundInfProof() const;

            /** \brief triangulate and check the manifold */
            void checkManifold(const base::PlannerTerminationCondition &ptc);

            /** \brief Save running stats to a csv file */
            void printStat(std::string time_str);

            float lambda_ = 0.0;

            /** \brief the learned manifold from sampler, in manifold class*/
            std::shared_ptr<ompl::infeasibility::Manifold> manifold_;

            /** \brief The points on the manifold */
            // std::shared_ptr<ob::SDCLValidStateSampler::StateVec> manifoldPoints_;
        };
    }
}

#endif
