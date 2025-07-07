#include "ompl/infeasibility/triangulation/triangulation.h"

Matrix root_matrix(unsigned d) 
{
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

__device__ void cartesian_coordinates(const CoxeterTri* d_cox, Vector& values, int* v) 
{
    Eigen::Matrix<float_tri, NN, 1> v_vect;
    for (int j = 0; j < NN; j++) v_vect(j) = v[j];
    values = d_cox->matrix_ * v_vect + d_cox->offset_;
}

__device__ float_tri manifold_eval(const ompl::infeasibility::SVMModelData* modelData_d, float_tri* point) 
{
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

    return f - modelData_d->b;
}

__device__ float_tri manifold_eval(const ompl::infeasibility::SVMModelData* modelData_d, const Vector& point) 
{
    int num_vectors = modelData_d->num_vectors;
    float_tri f = 0;
    float_tri dists_square = 0;

    for(int k = 0; k < num_vectors; k++){
        dists_square = 0;
        for(int i = 0; i < NN; i ++){
            dists_square += powf(point(i) - modelData_d->vectors[NN*k+i], 2);
        }
        f += modelData_d->coef[k] * expf(-modelData_d->gamma * dists_square);
    }
    // printf("offset, %f\n", d_md->vectors[25]);
    return f - modelData_d->b;
}

__device__ void sortByIndex(const float_tri* z, int* indices, int len) 
{
    // print_point(z, len);
    float_tri last_largest = 0;
    int z_min_index = 0;
    int z_max_index = 0;
    float_tri z_min = 1;
    float_tri z_max = 0;
    for (int i = 0; i < len; i++) {
        if (z[i] < z_min) {
            z_min_index = i;
            z_min = z[i];
        }
        if (z[i] > z_max) {
            z_max_index = i;
            z_max = z[i];
        }
    }

    indices[0] = z_max_index;
    last_largest = z_max;

    for (int i = 1; i < len; i++) {
        int cur_index = i;
        float_tri cur_z_max = z_min;
        int cur_max_index = 0;
        for (int j = 0; j < len; j++) {
            if (z[j] >= cur_z_max && z[j] < last_largest) {
                cur_z_max = z[j];
                cur_max_index = j;
            }
        }
        last_largest = cur_z_max;
        indices[i] = cur_max_index;
    }
}

__device__ __host__ void fast_sorting_increasing(int* vec, const int len) 
{
    // vec always contains 0 - NN
    bool temp[NN+1];
    for (int i = 0; i < NN+1; i++){
        temp[i] = false;
    }
    for (int i = 0; i < len; i++){
        temp[vec[i]] = true;
    }
    int cur_vec_index = 0;
    for (int i = 0; i < NN+1; i++) {
        if (temp[i]) {
            vec[cur_vec_index] = i;
            cur_vec_index++;
        }
    }
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

__global__ void locateSimplex(const CoxeterTri* d_cox, const float_tri* d_seeds, const int num_seeds, FullSimplex* full_simplices) 
{
    unsigned int threadid = blockIdx.x*blockDim.x + threadIdx.x;
    if (threadid < num_seeds) {
        FullSimplex* output = full_simplices + threadid;

        float_tri z[NN+1] = {};
        
        Vector p_vect;
        for (int i = 0; i < NN; i++) p_vect(i) = d_seeds[NN * threadid + i];
        Vector x_vect = d_cox->matrixInverse_ * (p_vect - d_cox->offset_);
        for (int i = 0; i < NN; i++) {
            float_tri x_i = x_vect(i);
            float_tri y_i = floorf(x_i);
            output->vertex_[i] = (int)y_i;
            z[i] = x_i - y_i;
        }

        z[NN] = 0;
        int indices[NN+1] = {};
        // int indices_copy[NN+1] = {};
        for (int i = 0; i < NN+1; i++) {
            indices[i] = i;
            // indices_copy[i] = i;
        }

        // thrust::sort(indices, indices+(NN+1), [&z](int i1, int i2) { return z[i1] > z[i2]; }); // todo: replace thrust sort.
        sortByIndex(z, indices, NN+1);
        // print_index(indices, indices_copy, NN+1);

        for (int i = 0; i < NN + 1; i++) {
            output->partitions_[i] = indices[i];
        }
    }
}

__global__ void getSimplexEdges(const FullSimplex* full_simplices, EdgePoint* eps, const int num_seeds) 
{
    unsigned int threadid = blockIdx.x*blockDim.x + threadIdx.x;
    if (threadid < num_seeds) {
        int edgePerSimplex = (NN+1)*NN/2;
        // calculate all edges of simplex
        int C_values[2] = {0, 1};
        for (int edge_idx = 0; edge_idx < edgePerSimplex; edge_idx++) {
            int* cur_partition = eps[edgePerSimplex * threadid + edge_idx].partitions_;
            int* cur_vertex = eps[edgePerSimplex * threadid + edge_idx].vertex_;
            for (int i = 0; i < NN; i++) {
                cur_vertex[i] = full_simplices[threadid].vertex_[i];
            }
            // h = 1
            int num_first_partition = 0;
            for (int i = C_values[0]; i < C_values[1]; i++) {
                cur_partition[num_first_partition] = full_simplices[threadid].partitions_[i];
                num_first_partition++;
            }

            cur_partition[num_first_partition] = -1;
            int num_second_partition = num_first_partition + 1;

            for(int i = C_values[1]; i < NN+1; i++) {
                cur_partition[num_second_partition] = full_simplices[threadid].partitions_[i];
                num_second_partition++;
            }

        
            for (int i = 0; i < C_values[0] ; i++) {
                int j = full_simplices[threadid].partitions_[i];
                if (j != NN){
                    cur_vertex[j]++;
                } else {
                    for (int l = 0; l < NN; l++) cur_vertex[l]--;
                }
                cur_partition[num_second_partition] = j;
                num_second_partition++;
            }

            // increment combination iterator
            if (C_values[1] < NN) {
                C_values[1]++;
            } else {
                if (C_values[0] < NN - 1) {
                    C_values[0]++;
                    C_values[1] = C_values[0] + 1;
                }
            }
            // thrust::sort(cur_partition, cur_partition + num_first_partition, thrust::less<int>());
            // thrust::sort(cur_partition + num_first_partition + 1, cur_partition + NN +2, thrust::less<int>());
            fast_sorting_increasing(cur_partition, num_first_partition);
            fast_sorting_increasing(cur_partition + num_first_partition + 1, NN + 1 - num_first_partition);
            eps[edgePerSimplex * threadid + edge_idx].second_start = num_first_partition + 1;
        }
    }
}

__device__ void getIntersection(const ompl::infeasibility::SVMModelData* d_md, const CoxeterTri* d_cox, EdgePoint* ep) 
{
    // computer two vertices value using manifold function.
    int v1[NN] = {}; // Permutahedral representation of vertices
    int v2[NN] = {};
    // edge_vertices(v1, v2, eps + threadid);
    for (int i = 0; i < NN; i++) {
        v1[i] = ep->vertex_[i];
        v2[i] = ep->vertex_[i];
    }
    for (int i = 0; i < NN + 2; i++) {
        if (ep->partitions_[i] == -1) break;
        if (ep->partitions_[i] != NN) v2[ep->partitions_[i]]++;
        else {
            for (int j = 0; j < NN; j++) v2[j]--;
        }
    }
    
    // print_edgePoint(eps[threadid]);
    // print_v(v1);
    // print_v(v2);
    
    Vector vertex1; // cartesian coordinates of vertices
    cartesian_coordinates(d_cox, vertex1, v1);
    Vector vertex2; // cartesian coordinates of vertices
    cartesian_coordinates(d_cox, vertex2, v2);
    // print_point(vertex1);
    // print_point(vertex2);

    float_tri vertices_value[2] = {};  // manifold value of vertices
    vertices_value[0] = manifold_eval(d_md, vertex1);
    vertices_value[1] = manifold_eval(d_md, vertex2);

    // printf("here %f, %f \n", vertices_value[0], vertices_value[1]);

    if (vertices_value[0] * vertices_value[1] >= 0) {
        for (int i = 0; i < NN; i++) ep->point[i] = -1;
        return;
    }

    Vector2f lambda;
    lambda(0) = -vertices_value[1] / (vertices_value[0] - vertices_value[1]);
    lambda(1) = vertices_value[0] / (vertices_value[0] - vertices_value[1]);

    // printf("here %f %f\n", lambda(0), lambda(1));

    for (std::size_t i = 0; i < (std::size_t)lambda.size(); ++i) {
      assert(fabsf(lambda(i) - 1) > 1e-9 && fabsf(lambda(i) - 0.) > 1e-9);
    }

    Eigen::Matrix<float_tri, 2, NN> vertex_matrix;
    for (int j = 0; j < NN; ++j) {
        vertex_matrix(0, j) = vertex1(j);
        // printf("here %f \n", vertex_matrix(0, j));
        vertex_matrix(1, j) = vertex2(j);
    }

    Vector intersection = lambda.transpose() * vertex_matrix;

    float_tri cur_value = manifold_eval(d_md, intersection);
    float_tri vv0 = vertices_value[0];
    float_tri vv1 = vertices_value[1];
    // Eigen::VectorXd lambda_new(2);
    // Eigen::VectorXd intersection_new(amb_d);

    while(fabsf(cur_value) > 0.001) {
      
        if ((vv0 < 0 && cur_value < 0) || (vv0 > 0 && cur_value > 0)) {
            vv0 = cur_value;
            for (int j = 0; j < NN; ++j) vertex_matrix(0, j) = intersection(j);
        } else {
            vv1 = cur_value;
            for (int j = 0; j < NN; ++j) vertex_matrix(1, j) = intersection(j);
        }

        // lambda = compute_lambda(vv0, vv1);
        lambda(0) = -vv1 / (vv0 - vv1);
        lambda(1) = vv0 / (vv0 - vv1);

        intersection = lambda.transpose() * vertex_matrix;

        cur_value = manifold_eval(d_md, intersection);
    }

    // record results
    for (int i = 0; i < NN; i++) ep->point[i] = intersection(i);

}

__device__ bool isSameEdge(const EdgePoint* ep1, const EdgePoint* ep2) 
{
    for (int i = 0; i < NN; i++) {
        if (ep1->vertex_[i] != ep2->vertex_[i]) return false;
    }

    for (int i = 0; i < NN+2; i++) {
        if (ep1->partitions_[i] != ep2->partitions_[i]) return false;
    }
    return true;
}

__device__ bool noIntersection(const EdgePoint* ep) 
{
    for (int i = 0; i < NN; i++) {
        if (ep->point[i] != -1) return false;
    }
    return true;
}

__device__ int edgeHash(const EdgePoint* ep, const int hashset_size) 
{
    // edge hash function based on vertex and partition
    int hash = 17;
    for (int i = 0; i < NN; ++i)
    {
        hash = hash * 19 + ep->vertex_[i];
        if (ep->partitions_[i] == -1) hash = hash * 19 + NN+3;
        else hash = hash * 19 + ep->partitions_[i];
    }
    hash = hash * 19 + ep->partitions_[NN];
    hash = hash * 19 + ep->partitions_[NN+1];

    return abs(hash % hashset_size);
}

__device__ void copyEdge(const EdgePoint* src, EdgePoint* des) {
    for (int i = 0; i < NN; i++) {
        des->vertex_[i] = src->vertex_[i];
        des->point[i] = src->point[i];
    }
    for (int i = 0; i < NN + 2; i++) {
        des->partitions_[i] = src->partitions_[i];
    }
    des->second_start = src->second_start;
}

__global__ void getIntersections(const ompl::infeasibility::SVMModelData* d_md, const CoxeterTri* d_cox, EdgePoint* eps, const int num_edges) {
    unsigned int threadid = blockIdx.x*blockDim.x + threadIdx.x;
    if (threadid < num_edges) {
        getIntersection(d_md, d_cox, eps + threadid);
    }
}

__global__ void hashEdges(KeyType* hashset, const EdgePoint* eps, const int num_edges, 
                          const int hashset_size, KeyType* d_num_hashedges)
{
    // use linear probing to hash edges. 
    unsigned int threadid = blockIdx.x*blockDim.x + threadIdx.x;
    if (threadid < num_edges)
    {
        if (noIntersection(eps+threadid)) return; // no intersection, discard the edge. 

        KeyType key = threadid;
        KeyType slot = edgeHash(eps + key, hashset_size);

        // printf("hash res is %d \n", slot);
        
        while (true)
        {
            KeyType prev = atomicCAS(hashset+slot, kEmpty_tri, key);  // linear probing
            if (prev == kEmpty_tri || prev == key)
            {
                atomicAdd(d_num_hashedges, 1);
                break;
            }
            if (isSameEdge(eps + hashset[slot], eps+key)) break;

            slot = (slot + 1) % hashset_size;
        }
    }
}

__global__ void iterate_edge_hashset(KeyType* hashset, KeyType* d_num_hashedges, const int hashset_size, 
                                     const EdgePoint* eps, EdgePoint* hash_eps){
    unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
    if (threadid < hashset_size) 
    {
        if (hashset[threadid] != kEmpty_tri) 
        {
            KeyType size = atomicAdd(d_num_hashedges, 1);
            // res[size] = hashset[threadid];
            copyEdge(eps + hashset[threadid], hash_eps + size);
            hashset[threadid] = size;
        }
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
    cudaFree(coxeter_d_);
    cudaFree(coef_d_);
    cudaFree(vectors_d_);
    cudaFree(modelData_d_);
    cudaFree(manifoldPoints_d_);
    cudaFree(hashset_d_);
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

    locateEdges(ptc);


}

void ompl::infeasibility::GPUCoxeterTriangulation::locateEdges(const ompl::base::PlannerTerminationCondition &ptc)
{
    int mingridsize;
    int threadblocksize;
    float milliseconds;

    // Create events for GPU timing
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // get locate simplices from seeds. //////////////////////////////////////////////////////////////
    cudaEventRecord(start);
    cudaEventRecord(start);

    FullSimplex* fullSimplices;
    cudaMalloc(&fullSimplices, sizeof(FullSimplex) * numManifoldPoints_);
    cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, locateSimplex, 0, 0);
    int gridsize = (numManifoldPoints_ + threadblocksize - 1) / threadblocksize;
    locateSimplex<<<gridsize, threadblocksize>>>(coxeter_d_, manifoldPoints_d_, numManifoldPoints_, fullSimplices);

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("GPU located %d seed points' simplices in %f ms\n", numManifoldPoints_, milliseconds);

    // calculate edges of each full simplex. ////////////////////////////////////////////////////////////////
    cudaEventRecord(start);
    EdgePoint* eps;
    int edgePerSimplex = (NN+1)*NN/2;
    int numEdges = edgePerSimplex * numManifoldPoints_;
    cudaMalloc(&eps, sizeof(EdgePoint) * numEdges);
    cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, getSimplexEdges, 0, 0);
    gridsize = (numManifoldPoints_ + threadblocksize - 1) / threadblocksize;

    getSimplexEdges<<<gridsize, threadblocksize>>>(fullSimplices, eps, numManifoldPoints_);

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("GPU found %d edges of the simplices in %f ms\n", numEdges, milliseconds);

    cudaFree(fullSimplices);

    // compute intersection of edges. ////////////////////////////////////////////////////////////////
    cudaEventRecord(start); 
    cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, getIntersections, 0, 0);
    gridsize = (numEdges + threadblocksize - 1) / threadblocksize;

    getIntersections<<<gridsize, threadblocksize>>>(modelData_d_, coxeter_d_, eps, numEdges);

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("GPU found %d intersections in %f ms\n", numEdges, milliseconds);

    // hash edges ////////////////////////////////////////////////////////////////
    cudaEventRecord(start);
    KeyType* numHashedges_d; // num of non-duplicting edges in hashset
    cudaMalloc(&numHashedges_d, sizeof(KeyType));
    cudaMemset(numHashedges_d, 0, sizeof(KeyType));
    cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, hashEdges, 0, 0);
    gridsize = (numEdges + threadblocksize - 1) / threadblocksize;

    hashEdges<<<gridsize, threadblocksize>>>(hashset_d_, eps, numEdges, hashsetSize_, numHashedges_d);

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    int numHashedges_h = 0;
    cudaMemcpy(&numHashedges_h, numHashedges_d, sizeof(int), cudaMemcpyDeviceToHost);
    printf("GPU hashed %d edges in %f ms, total %d edges.\n", numEdges, milliseconds, numHashedges_h);

    // iterate hash table //////////////////////////////////////////////////////////
    cudaEventRecord(start);
    cudaMemset(numHashedges_d, 0, sizeof(KeyType));
    cudaMalloc(&hashedEdges_, sizeof(EdgePoint) * numHashedges_h);
    cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, iterate_edge_hashset, 0, 0);
    gridsize = (hashsetSize_ + threadblocksize - 1) / threadblocksize;

    iterate_edge_hashset<<<gridsize, threadblocksize>>>(hashset_d_, numHashedges_d, hashsetSize_, eps, hashedEdges_);

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    numHashedges_h = 0;
    cudaMemcpy(&numHashedges_h, numHashedges_d, sizeof(int), cudaMemcpyDeviceToHost);
    printf("GPU iterated the hashset in %f ms, a total of %d edges.\n", milliseconds, numHashedges_h);

    cudaFree(eps);
    cudaFree(numHashedges_d);

    numHashedEdges_ = numHashedges_h;
}

