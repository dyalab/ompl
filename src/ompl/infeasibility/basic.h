#ifndef BASIC_H
#define BASIC_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <atomic>
#include <fstream>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <cstdint>

typedef float float_inf;
// typedef std::vector<std::vector<float_inf>> pvec;
// typedef std::vector<float_inf> pt;

// constexpr int NN = 6;

// using Matrix = Eigen::Matrix<float_inf, NN, NN>; // need to use fixed size matrix in device code.
// using Vector = Eigen::Matrix<float_inf, NN, 1>;
// using VectorXf = Eigen::Matrix<float_inf, Eigen::Dynamic, 1>;
// using MatrixXf = Eigen::Matrix<float_inf, Eigen::Dynamic, Eigen::Dynamic>;
// using Vector2f = Eigen::Matrix<float_inf, 2, 1>;

#define foreach BOOST_FOREACH

#endif