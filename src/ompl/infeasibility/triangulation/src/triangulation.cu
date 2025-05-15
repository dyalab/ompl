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
}



void ompl::infeasibility::GPUCoxeterTriangulation::triangulate(std::shared_ptr<ompl::infeasibility::Manifold> manifold, 
                                                               float_tri* manifoldPoints_, std::size_t numManifoldPoints)
{
    // copy seed points to device
    cudaMalloc(&manifoldPoints_d_, sizeof(float_tri) * dim_ * numManifoldPoints);
    cudaMemcpy(manifoldPoints_d_, manifoldPoints_, sizeof(float_tri) * dim_ * numManifoldPoints, cudaMemcpyHostToDevice);
    std::cout << "Total number of seeds, " << numManifoldPoints << std::endl;

    // copy model data to device
    ModelData* modelData_ = manifold->getModelData();
    float_tri test[dim_] = {0};
    modelData_->eval(test);
    // modelData_->print();
    // if (numManifoldPoints > 10) 
    //     std::cout << "triangulation side" << manifoldPoints_[10 * 6 + 5] << std::endl;
    if (manifold->name() == "RBF-SVM")
    {
        cudaMalloc(&modelData_d_, sizeof(SVMModelData));
        // copy to pointer, use modeldata member pointer to copy. 
    }
}

