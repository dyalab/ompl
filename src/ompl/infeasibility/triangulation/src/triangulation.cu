#include "ompl/infeasibility/triangulation/triangulation.h"

template <typename T>
void safeCudaFree(T*& ptr) {
    if (ptr != nullptr) {
        cudaFree(ptr);
        ptr = nullptr;
    }
}

ompl::infeasibility::GPUCoxeterTriangulation::GPUCoxeterTriangulation(const float_tri lambda, const int dim)
: lambda_(lambda)
, dim_(dim)
{
    // match dimension
    assert(dim_ == NN);

    // create the coxeter triangulation elements
    CoxeterTri coxeter_h;
    coxeter_h.matrix_ = lambda_ * root_matrix(dim_);
    coxeter_h.offset_ = VectorXf::Random(dim_);
    coxeter_h.matrixInverse_ = coxeter_h.matrix_.inverse();

    // copy coxeter triangulation to device
    cudaMalloc(&coxeter_d_, sizeof(CoxeterTri));
    cudaMemcpy(coxeter_d_, &coxeter_h, sizeof(CoxeterTri), cudaMemcpyHostToDevice);

}

ompl::infeasibility::GPUCoxeterTriangulation::~GPUCoxeterTriangulation()
{
    safeCudaFree(coxeter_d_);
    safeCudaFree(coef_d_);
    safeCudaFree(vectors_d_);
    safeCudaFree(modelData_d_);
    safeCudaFree(manifoldPoints_d_);
    safeCudaFree(hashset_d_);
}


void ompl::infeasibility::GPUCoxeterTriangulation::copyModelData2Device(const SVMModelData* source)
{
    int num_vectors = source->num_vectors;

    cudaMalloc(&modelData_d_, sizeof(SVMModelData));
    cudaMalloc(&coef_d_, sizeof(float_tri) * num_vectors);
    cudaMalloc(&vectors_d_, sizeof(float_tri) * num_vectors * NN);
    
    cudaMemcpy(modelData_d_, source, sizeof(SVMModelData), cudaMemcpyHostToDevice);
    cudaMemcpy(coef_d_, source->coef, sizeof(float_tri) * num_vectors, cudaMemcpyHostToDevice);
    cudaMemcpy(vectors_d_, source->vectors, sizeof(float_tri) * num_vectors * NN, cudaMemcpyHostToDevice);
    cudaMemcpy(&(modelData_d_->coef), &coef_d_, sizeof(float_tri*), cudaMemcpyHostToDevice);
    cudaMemcpy(&(modelData_d_->vectors), &vectors_d_, sizeof(float_tri*), cudaMemcpyHostToDevice);
}


void ompl::infeasibility::GPUCoxeterTriangulation::triangulate(std::shared_ptr<ompl::infeasibility::Manifold> manifold, 
                                                               float_tri* manifoldPoints_, std::size_t numManifoldPoints,
                                                               const ompl::base::PlannerTerminationCondition &ptc)
{
    // copy seed points to device
    cudaMalloc(&manifoldPoints_d_, sizeof(float_tri) * dim_ * numManifoldPoints);
    cudaMemcpy(manifoldPoints_d_, manifoldPoints_, sizeof(float_tri) * dim_ * numManifoldPoints, cudaMemcpyHostToDevice);
    numManifoldPoints_ = numManifoldPoints;
    // std::cout << "Total number of seeds, " << numManifoldPoints << std::endl;

    // copy model data to device
    ompl::infeasibility::ModelData* modelData = manifold->getModelData();
    // float_tri test[dim_] = {1, 2, 3, 4, 5, 6};
    // std:: cout << "cpu eval " << modelData->eval(test) << std::endl;
    // modelData_->print();
    // if (numManifoldPoints > 10) 
    //     std::cout << "triangulation side" << manifoldPoints_[10 * 6 + 5] << std::endl;
    copyModelData2Device(dynamic_cast<const SVMModelData*>(modelData));
    // print_test<<<1, 1>>>(modelData_d_);

    // ------- manifold tracing ----------
    // allocate hashset
    hashsetSize_ = 100000000;  // TODO: better way to manage this fixed size hashset?
    cudaMalloc(&hashset_d_, sizeof(KeyType) * hashsetSize_);
    cudaMemset(hashset_d_, -1, sizeof(KeyType) * hashsetSize_);

    // locate starting intersection edges from seed points.
    locateEdges(ptc);

    //

    // safeCudaFree(coxeter_d_);
    // safeCudaFree(coef_d_);
    // safeCudaFree(vectors_d_);
    // safeCudaFree(modelData_d_);
    // safeCudaFree(manifoldPoints_d_);
    // safeCudaFree(hashedEdges_);
    // safeCudaFree(hashset_d_);
}

void ompl::infeasibility::GPUCoxeterTriangulation::locateEdges(const ompl::base::PlannerTerminationCondition &ptc)
{
    int mingridsize;
    int threadblocksize;
    float milliseconds;

    // get locate simplices from seeds. //////////////////////////////////////////////////////////////
    FullSimplex* fullSimplices;
    cudaMalloc(&fullSimplices, sizeof(FullSimplex) * numManifoldPoints_);

    launchLocateSimplex(coxeter_d_, manifoldPoints_d_, numManifoldPoints_, fullSimplices);

    // calculate edges of each full simplex. ////////////////////////////////////////////////////////////////
    EdgePoint* eps;
    int edgePerSimplex = (NN+1)*NN/2;
    int numEdges = edgePerSimplex * numManifoldPoints_;
    cudaMalloc(&eps, sizeof(EdgePoint) * numEdges);

    launchGetSimplexEdges(fullSimplices, eps, numManifoldPoints_);

    cudaFree(fullSimplices);

    // compute intersection of edges. ////////////////////////////////////////////////////////////////
    launchGetIntersections(modelData_d_, coxeter_d_, eps, numEdges);

    // hash edges ////////////////////////////////////////////////////////////////
    KeyType* numHashedges_d; // num of non-duplicting edges in hashset
    cudaMalloc(&numHashedges_d, sizeof(KeyType));
    cudaMemset(numHashedges_d, 0, sizeof(KeyType));
    
    launchHashEdges(hashset_d_, eps, numEdges, hashsetSize_, numHashedges_d);

    cudaMemcpy(&numHashedEdges_, numHashedges_d, sizeof(int), cudaMemcpyDeviceToHost);

    // iterate hash table //////////////////////////////////////////////////////////
    cudaMemset(numHashedges_d, 0, sizeof(KeyType));
    cudaMalloc(&hashedEdges_, sizeof(EdgePoint) * numHashedEdges_);

    launchSaveHashEdges(hashset_d_, numHashedges_d, hashsetSize_, eps, hashedEdges_);

    // clean up
    cudaFree(eps);
    cudaFree(numHashedges_d);
}

