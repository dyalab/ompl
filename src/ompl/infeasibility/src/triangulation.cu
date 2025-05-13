#include "triangulation.h"

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


ompl::infeasibility::GPUCoxeterTriangulation::GPUCoxeterTriangulation(float_tri lambda, const int dim)
: lambda_(lambda),
: dim_(dim)
{
    // match dimension
    assert(dim_ == NN);

    // create the coxeter triangulation elements
    coxeter_.matrix_ = lambda * root_matrix(dim_);
    coxeter_.offset_ = VectorXf::Random(dim_);
    coxeter_.matrixInverse_ = coxeter_.matrix_.inverse();

    // copy coxeter triangulation to device
    cudaMalloc(&coxeter_d_, sizeof(oi::CoxeterTri));
    cudaMemcpy(coxeter_d_, &cox_tr, sizeof(oi::CoxeterTri), cudaMemcpyHostToDevice);

}

ompl::infeasibility::GPUCoxeterTriangulation::~GPUCoxeterTriangulation()
{
    cudaFree(coxeter_d_);
}



void ompl::infeasibility::GPUCoxeterTriangulation::triangulate(std::shared_ptr<ompl::infeasibility::Manifold> manifold, 
                                                               float_tri* manifoldPoints_, std::size_t numManifoldPoints, 
                                                               int& num_intersections, int& num_full_simplices)
{
    // copy seed points
    cudaMalloc(&manifoldPoints_d_, sizeof(float_tri) * dim * numManifoldPoints);
    cudaMemcpy(manifoldPoints_d_, manifoldPoints_, sizeof(float_tri) * dim * numManifoldPoints, cudaMemcpyHostToDevice);
    std::cout << "Total number of seeds, " << numManifoldPoints << std::endl;

    // copy model data
    ModelData* modelData_ = manifold->getModelData();
    float_tri test[dim_] = {0};
    modelData_.eval(test);
}

