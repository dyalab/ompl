#pragma nv_diag_suppress 177, 20236, 20011, 20013, 20014, 20015
#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <limits.h>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <thrust/sort.h>
#include <cassert>
#include <unordered_set>
#include <queue>
#include <fstream>
#include <ompl/infeasibility/SVMManifold.h>


typedef float float_tri; // float type for triangulation
// typedef unsigned int KeyType;
// const KeyType kEmpty_tri = std::numeric_limits<KeyType>::max();

// struct CoxeterTri{
// 	// CoxeterTri(int d) {
// 	// 	dim = d;
// 	// 	matrix_ = Matrix::Identity(dim, dim);
// 	// 	offset_ = Vector::Zero(dim);
// 	// 	matrix_inverse_ = Matrix::Identity(dim, dim);
// 	// }
//     Matrix matrix_;
//     Vector offset_;
//     Matrix matrix_inverse_;
// };

// // struct Simplex{
// // 	int* vertex_;
// // 	int** partition_;
// // 	int* partitions_size_;
// // 	int n; // n-simplex.
// // 	int num_partitions_;
// // };

// struct FullSimplex {
// 	int vertex_[NN];
// 	int partitions_[NN+1];
// };

// struct FullSimplexPoints {
// 	int vertex_[NN];
// 	int partitions_[NN+1];
// 	int edge_indices_[12]; // in 6 dof at most 12 edges in a simplex intersects witht the manifold (7 vertices in total, 6 x 1, 2 x 5, 3 x 4)
// 	int num_edges_ = 0;
// };

// struct Vertexx {
// 	int vertex_[NN];
// 	Vertexx(int* v) {
// 		bool value = false;
// 		for (int i = 0; i < NN; i++) {
// 	        vertex_[i] = v[i];
// 	    }
// 	}
// 	bool operator==(const Vertexx& v2) const
//     {
//     	const Vertexx* v1 = this;
// 	    for (int i = 0; i < NN; i++) {
// 	        if (v1->vertex_[i] != v2.vertex_[i]) return false;
// 	    }
// 	    return true;
//     }
//     bool operator<(const Vertexx& v2) const
//     {
//     	const Vertexx* v1 = this;
// 	    for (int i = 0; i < NN; i++) {
// 	        if (v1->vertex_[i] == v2.vertex_[i]) continue;
// 	        if (v1->vertex_[i] > v2.vertex_[i]) return false;
// 	        if (v1->vertex_[i] < v2.vertex_[i]) return true;
// 	    }

// 	    return false;
//     }
// };

// struct Edgee {
// 	Edgee(int vv1, int vv2) {
// 		v1 = vv1;
// 		v2 = vv2;
// 	}
// 	int v1; // index of vertex 1 in the vertexx list
// 	int v2; // index of vertex 2 in the vertexx list
// };

// struct DecomposeData {
// 	Edgee* edges;
// 	Vertexx* vertices;
// };

// struct EdgePoint { // intersecing edge and the point of intersection
// 	int vertex_[NN];
// 	int partitions_[NN+2]; // edge has 2 partitions, add an additional -1 as seperation. 
// 	float_tri point[NN] = {0}; // the intersection point. 
// 	int second_start;
// 	bool has_intersection = false;
// 	bool operator==(const EdgePoint& ep2) const
//     {
//     	const EdgePoint* ep1 = this;
// 	    for (int i = 0; i < NN; i++) {
// 	        if (ep1->vertex_[i] != ep2.vertex_[i]) return false;
// 	    }

// 	    for (int i = 0; i < NN+2; i++) {
// 	        if (ep1->partitions_[i] != ep2.partitions_[i]) return false;
// 	    }
// 	    return true;
//     }
// };

// struct EdgeCoface { // 2-simplices
// 	int vertex_[NN];
// 	int partitions_[NN+3]; // has 3 partitions, add two additional -1 as seperation. 
// 	int second_start;
// 	int third_start; // second partition starting index
// };

// struct TriRes {
// 	FullSimplexPoints* fsps;
// 	float_tri* intersections;
// 	CoxeterTri* cox;
// 	ModelData* md;
// 	float_tri* md_vectors;
// 	float_tri* md_coef;
// };

// void freeGPU(FullSimplexPoints* fsps);
// void freeGPU(EdgePoint* eps);
// void freeGPU(Edgee* eps);
// void freeGPU(Vertexx* eps);
// void freeGPU(TriRes* res);

// Matrix root_matrix(unsigned d);

// TriRes* triangulate(const ModelData& md, const float_tri lambda, const std::vector<std::vector<float_tri>>& seeds, 
// 	             const int N, VectorXf& offset, int& num_intersections, int& num_full_simplices);

// TriRes* triangulate(const ModelData& md, const float_tri lambda, const std::vector<Eigen::VectorXd>& seeds, 
// 	             const int N, VectorXf& offset, int& num_intersections, int& num_full_simplices);

// float_tri* tri_intersections(const ModelData& md, const float_tri lambda, const std::vector<std::vector<float_tri>>& seeds, 
//                           const int N, VectorXf& offset, int& num_intersections, int& num_full_simplices);

// float_tri* batch_triangulation(FullSimplexPoints* full_fsps, 
//                                const int start_fs, const int end_fs, int& num_batch_intersections, const int batch_resolution,
//                                const Edgee* decompose_edges, const Vertexx* decompose_vertices, 
//                                const int num_decompose_edges, const int num_decompose_vertices, 
//                                const CoxeterTri* d_cox, const ModelData* d_md, const int num_intersection_multiple);

// DecomposeData* decompose_edge_representation(const int batch_relustion, int& num_decompose_edges, int& num_decompose_vertices);


#endif