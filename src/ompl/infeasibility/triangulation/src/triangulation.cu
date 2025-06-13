#include "ompl/infeasibility/triangulation/triangulation.h"

Matrix root_matrix(unsigned d) {
    Matrix cartan(Matrix::Identity(d, d));
    for (unsigned i = 1; i < d; i++) {
      cartan(i - 1, i) = -0.5;
      cartan(i, i - 1) = -0.5;
    }
    Eigen::SelfAdjointEigenSolver<Matrix> saes(cartan);
    VectorXf sqrt_diag(d);
    for (unsigned i = 0; i < d; ++i) sqrt_diag(i) = std::sqrt(saes.eigenvalues()[i]);

    MatrixXf lower(Matrix::Ones(d, d));
    lower = lower.triangularView<Eigen::Lower>();

    Matrix result = (lower * saes.eigenvectors() * sqrt_diag.asDiagonal()).inverse();
    return result;
}

__device__ float_tri manifold_eval(const ompl::infeasibility::SVMModelData* modelData_d, float_tri* point) {
    int num_vectors = modelData_d->num_vectors;
    float_tri f = 0;
    float_tri dists_square = 0;

    for(int k = 0; k < num_vectors; k++){
        dists_square = 0;
        for(int i = 0; i < NN; i ++){
            dists_square += powf(point[i] - modelData_d->vectors[NN*k+i], 2);
        }
        f += modelData_d->coef[k] * expf(-modelData_d->gamma * dists_square);
    }
    // printf("offset, %f\n", modelData_d->vectors[25]);
    return f - modelData_d->b;
}

__global__ void print_test(const ompl::infeasibility::SVMModelData* modelData_d)
{
    unsigned int threadid = blockIdx.x*blockDim.x + threadIdx.x;
    if (threadid < 1) {
        printf("gpu model data: %f %f %d\n", modelData_d->coef[0], modelData_d->vectors[8], modelData_d->features);
        float_tri test[NN] = {1, 2, 3, 4, 5, 6};
        printf("gpu model eval: %f \n", manifold_eval(modelData_d, test));
    }
}


ompl::infeasibility::GPUCoxeterTriangulation::GPUCoxeterTriangulation(const float_tri lambda, const int dim)
: lambda_(lambda)
, dim_(dim)
{
    // match dimension
    assert(dim_ == NN);

    // create the coxeter triangulation elements
    coxeter_.matrix_ = lambda * root_matrix(dim_);
    coxeter_.offset_ = VectorXf::Random(dim_);
    coxeter_.matrixInverse_ = coxeter_.matrix_.inverse();

    // copy coxeter triangulation to device
    cudaMalloc(&coxeter_d_, sizeof(CoxeterTri));
    cudaMemcpy(coxeter_d_, &coxeter_, sizeof(CoxeterTri), cudaMemcpyHostToDevice);

}

ompl::infeasibility::GPUCoxeterTriangulation::~GPUCoxeterTriangulation()
{
    cudaFree(coxeter_d_);
    cudaFree(coef_d_);
    cudaFree(vectors_d_);
    cudaFree(modelData_d_);
    cudaFree(manifoldPoints_d_);
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
                                                               float_tri* manifoldPoints_, std::size_t numManifoldPoints)
{
    // copy seed points to device
    cudaMalloc(&manifoldPoints_d_, sizeof(float_tri) * dim_ * numManifoldPoints);
    cudaMemcpy(manifoldPoints_d_, manifoldPoints_, sizeof(float_tri) * dim_ * numManifoldPoints, cudaMemcpyHostToDevice);
    // std::cout << "Total number of seeds, " << numManifoldPoints << std::endl;

    // copy model data to device
    ompl::infeasibility::ModelData* modelData = manifold->getModelData();
    float_tri test[dim_] = {1, 2, 3, 4, 5, 6};
    std:: cout << "cpu eval " << modelData->eval(test) << std::endl;
    // modelData_->print();
    // if (numManifoldPoints > 10) 
    //     std::cout << "triangulation side" << manifoldPoints_[10 * 6 + 5] << std::endl;
    copyModelData2Device(dynamic_cast<const SVMModelData*>(modelData));
    print_test<<<1, 1>>>(modelData_d_);

}

