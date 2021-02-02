#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// LL: Added config header to pass macro that switches Leander's code off and on
#include "detect_3d_cuboid/at3dcv_config.h"

// LL: Added by Leander
#ifdef at3dcv_leander
#include <boost/geometry.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#endif
// LL: Added by Leander

template <class T>
Eigen::Quaternion<T> zyx_euler_to_quat(const T &roll, const T &pitch, const T &yaw);

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw);

template <class T>
void rot_to_euler_zyx(const Eigen::Matrix<T, 3, 3> &R, T &roll, T &pitch, T &yaw);

template <class T>
Eigen::Matrix<T, 3, 3> euler_zyx_to_rot(const T &roll, const T &pitch, const T &yaw);

// input is 3*n (or 2*n)  output is 4*n (or 3*n)
template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in);
template <class T>
void real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_out);
template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> real_to_homo_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_in);

// input is 3*n (or 4*n)  output is 2*n(or 3*n)
template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in);
template <class T>
void homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_out);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> homo_to_real_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_homo_in);

//vertically stack a matrix to a matrix, increase rows, keep columns
void vert_stack_m(const Eigen::MatrixXd &a_in, const Eigen::MatrixXd &b_in, Eigen::MatrixXd &combined_out);
void vert_stack_m_self(Eigen::MatrixXf &a_in, const Eigen::MatrixXf &b_in);

void fast_RemoveRow(Eigen::MatrixXd &matrix, int rowToRemove, int &total_line_number);

// make sure column size is given. not check here. row will be adjusted automatically. if more cols given, will be zero.
template <class T>
bool read_all_number_txt(const std::string txt_file_name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat);

// LL: Added by Leander
#ifdef at3dcv_leander
bool read_inst_segment_vertices(const std::string txt_file_name, std::vector<Eigen::Matrix2Xd> &read_number_mat);
# ifdef at3dcv_leander_no_depth
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> polygon;
typedef boost::geometry::model::d2::point_xy<double> point_type;

void poly_eigen_to_string_rep(Eigen::MatrixXi convex_hull_vertices, std::string &poly_string_rep);
void poly_vec_eigen_to_string_rep(std::vector<Eigen::MatrixXi> raw_all_obj2d_ss_vertix, std::vector<std::string> &geometries);
int poly_string_to_boost_poly(std::string poly_txt_rep, polygon &poly);
double perc_poly2_covered_by_poly1(polygon poly1, polygon poly2);
int visualize_polygons(std::string file_name, std::vector<polygon> polygons);

void sort_2d_cuboid_vertices(double vp_1_position, Eigen::Matrix2Xi all_configs_error_one_objH_int, Eigen::MatrixXi &cub_prop_2di);
void cuboid_2d_vertices_to_2d_surfaces(Eigen::Matrix2Xi cub_prop_2d, std::vector<Eigen::MatrixXi> &eigen_2d_surfaces);
void eigen_2d_cub_surfaces_to_boost_poly_surfaces(std::vector<Eigen::MatrixXi> eigen_2d_surfaces, std::vector<polygon> &boost_poly_surfaces);
# endif
#endif
// LL: Added by Leander

// each line: one string, several numbers. make sure column size is correct.
bool read_obj_detection_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &strings);
// each line several numbers then one string . make sure column size is correct.
bool read_obj_detection2_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &strings);

// (partial) sort a vector, only need top_K element, result is stored in idx   by default increasing
void sort_indexes(const Eigen::VectorXd &vec, std::vector<int> &idx, int top_k);
void sort_indexes(const Eigen::VectorXd &vec, std::vector<int> &idx);

// change [-180 180] to [-90 90] by +-90
template <class T>
T normalize_to_pi(T angle);

template <class T>
void print_vector(const std::vector<T> &vec);

//TODO could be replaced by eigen linespace
template <class T>
void linespace(T starting, T ending, T step, std::vector<T> &res);
