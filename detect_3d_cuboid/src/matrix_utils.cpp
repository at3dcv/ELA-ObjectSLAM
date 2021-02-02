#include "detect_3d_cuboid/matrix_utils.h"

// std c
#include <math.h>
#include <stdio.h>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <ctime>

// LL: Added by Leander
#ifdef at3dcv_leander
#include <vector>
#endif
// LL: Added by Leander

using namespace Eigen;

template <class T>
Eigen::Quaternion<T> zyx_euler_to_quat(const T &roll, const T &pitch, const T &yaw)
{
    T sy = sin(yaw * 0.5);
    T cy = cos(yaw * 0.5);
    T sp = sin(pitch * 0.5);
    T cp = cos(pitch * 0.5);
    T sr = sin(roll * 0.5);
    T cr = cos(roll * 0.5);
    T w = cr * cp * cy + sr * sp * sy;
    T x = sr * cp * cy - cr * sp * sy;
    T y = cr * sp * cy + sr * cp * sy;
    T z = cr * cp * sy - sr * sp * cy;
    return Eigen::Quaternion<T>(w, x, y, z);
}
template Eigen::Quaterniond zyx_euler_to_quat<double>(const double &, const double &, const double &);
template Eigen::Quaternionf zyx_euler_to_quat<float>(const float &, const float &, const float &);

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw)
{
    T qw = q.w();
    T qx = q.x();
    T qy = q.y();
    T qz = q.z();

    roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    pitch = asin(2 * (qw * qy - qz * qx));
    yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}
template void quat_to_euler_zyx<double>(const Eigen::Quaterniond &, double &, double &, double &);
template void quat_to_euler_zyx<float>(const Eigen::Quaternionf &, float &, float &, float &);

template <class T>
void rot_to_euler_zyx(const Eigen::Matrix<T, 3, 3> &R, T &roll, T &pitch, T &yaw)
{
    pitch = asin(-R(2, 0));

    if (abs(pitch - M_PI / 2) < 1.0e-3)
    {
        roll = 0.0;
        yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) + roll;
    }
    else if (abs(pitch + M_PI / 2) < 1.0e-3)
    {
        roll = 0.0;
        yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) - roll;
    }
    else
    {
        roll = atan2(R(2, 1), R(2, 2));
        yaw = atan2(R(1, 0), R(0, 0));
    }
}
template void rot_to_euler_zyx<double>(const Matrix3d &, double &, double &, double &);
template void rot_to_euler_zyx<float>(const Matrix3f &, float &, float &, float &);

template <class T>
Eigen::Matrix<T, 3, 3> euler_zyx_to_rot(const T &roll, const T &pitch, const T &yaw)
{
    T cp = cos(pitch);
    T sp = sin(pitch);
    T sr = sin(roll);
    T cr = cos(roll);
    T sy = sin(yaw);
    T cy = cos(yaw);

    Eigen::Matrix<T, 3, 3> R;
    R << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
        cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
        -sp, sr * cp, cr * cp;
    return R;
}
template Matrix3d euler_zyx_to_rot<double>(const double &, const double &, const double &);
template Matrix3f euler_zyx_to_rot<float>(const float &, const float &, const float &);

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pts_homo_out;
    int raw_rows = pts_in.rows();
    int raw_cols = pts_in.cols();

    pts_homo_out.resize(raw_rows + 1, raw_cols);
    pts_homo_out << pts_in,
        Matrix<T, 1, Dynamic>::Ones(raw_cols);
    return pts_homo_out;
}
template MatrixXd real_to_homo_coord<double>(const MatrixXd &);
template MatrixXf real_to_homo_coord<float>(const MatrixXf &);

template <class T>
void real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_out)
{
    int raw_rows = pts_in.rows();
    int raw_cols = pts_in.cols();

    pts_homo_out.resize(raw_rows + 1, raw_cols);
    pts_homo_out << pts_in,
        Matrix<T, 1, Dynamic>::Ones(raw_cols);
}
template void real_to_homo_coord<double>(const MatrixXd &, MatrixXd &);
template void real_to_homo_coord<float>(const MatrixXf &, MatrixXf &);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> real_to_homo_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, 1> pts_homo_out;
    int raw_rows = pts_in.rows();
    ;

    pts_homo_out.resize(raw_rows + 1);
    pts_homo_out << pts_in,
        1;
    return pts_homo_out;
}
template VectorXd real_to_homo_coord_vec<double>(const VectorXd &);
template VectorXf real_to_homo_coord_vec<float>(const VectorXf &);

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pts_out(pts_homo_in.rows() - 1, pts_homo_in.cols());
    for (int i = 0; i < pts_homo_in.rows() - 1; i++)
        pts_out.row(i) = pts_homo_in.row(i).array() / pts_homo_in.bottomRows(1).array(); //replicate needs actual number, cannot be M or N

    return pts_out;
}
template MatrixXd homo_to_real_coord<double>(const MatrixXd &);
template MatrixXf homo_to_real_coord<float>(const MatrixXf &);

template <class T>
void homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_out)
{
    pts_out.resize(pts_homo_in.rows() - 1, pts_homo_in.cols());
    for (int i = 0; i < pts_homo_in.rows() - 1; i++)
        pts_out.row(i) = pts_homo_in.row(i).array() / pts_homo_in.bottomRows(1).array(); //replicate needs actual number, cannot be M or N
}
template void homo_to_real_coord<double>(const MatrixXd &, MatrixXd &);
template void homo_to_real_coord<float>(const MatrixXf &, MatrixXf &);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> homo_to_real_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_homo_in)
{
    Eigen::Matrix<T, Eigen::Dynamic, 1> pt_out;
    if (pts_homo_in.rows() == 4)
        pt_out = pts_homo_in.head(3) / pts_homo_in(3);
    else if (pts_homo_in.rows() == 3)
        pt_out = pts_homo_in.head(2) / pts_homo_in(2);

    return pt_out;
}
template VectorXd homo_to_real_coord_vec<double>(const VectorXd &);
template VectorXf homo_to_real_coord_vec<float>(const VectorXf &);

void fast_RemoveRow(MatrixXd &matrix, int rowToRemove, int &total_line_number)
{
    matrix.row(rowToRemove) = matrix.row(total_line_number - 1);
    total_line_number--;
}

void vert_stack_m(const MatrixXd &a_in, const MatrixXd &b_in, MatrixXd &combined_out)
{
    assert(a_in.cols() == b_in.cols());
    combined_out.resize(a_in.rows() + b_in.rows(), a_in.cols());
    combined_out << a_in,
        b_in;
}

void vert_stack_m_self(MatrixXf &a_in, const MatrixXf &b_in)
{
    assert(a_in.cols() == b_in.cols());
    MatrixXf combined_out(a_in.rows() + b_in.rows(), a_in.cols());
    combined_out << a_in,
        b_in;
    a_in = combined_out; //TODO why not use convervative resize?
}

// make sure column size is given. no checks here. row will be adjusted automatically. if more cols given, will be zero.
// AC: the offline values of bboxes and segmentation are read here!
template <class T>
bool read_all_number_txt(const std::string txt_file_name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat)
{
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
    std::ifstream filetxt(txt_file_name.c_str());
    int row_counter = 0;
    std::string line;
    if (read_number_mat.rows() == 0)
        read_number_mat.resize(100, 10);

    while (getline(filetxt, line))
    {
        T t;
        if (!line.empty())
        {
            std::stringstream ss(line);
            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
        }
    }
    filetxt.close();

    read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows

    return true;
}
template bool read_all_number_txt(const std::string, MatrixXd &);
template bool read_all_number_txt(const std::string, MatrixXi &);


// LL: Added by Leander 
#ifdef at3dcv_leander
bool read_inst_segment_vertices(const std::string txt_file_name, std::vector<Eigen::Matrix2Xd> &read_inst_segment_vert)
{
    // LL: Check if the file can be read
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
        
    // LL: Read the text file to `filetxt`
    std::ifstream filetxt(txt_file_name.c_str());
    
    std::string line_x;
    std::string line_y;

    // LL: Track the number of columns 
    int row_counter = 0;
	
    // LL: Loop over the current and the next line (x and y value)
    while (getline(filetxt, line_x))
    {
        getline(filetxt, line_y);
    
        // LL: Matrix to hold the vertices of the polygon from the current x and y row  
        // LL: Carful: This configuration expects a polygon to have a maximum of a 100 vertices
        Eigen::Matrix2Xd single_object_is_vertices(2,100);
    
        // LL: Variable to which we pass the x and the y value of the current and the next row
        double t1;
        double t2;
    
        // LL: Retrive the first string (column) in the current line and write it to `classname`
        std::stringstream ss1(line_x);
        std::stringstream ss2(line_y);
    
        // LL: Keep track at what column we are currently at.
        int colu = 0;
    
        // LL: Extracts one by one all remaining columns of the current row and writes them to read_number_mat
        while (ss1 >> t1 && ss2 >> t2 && colu < 99 )
        {
            single_object_is_vertices(0, colu) = t1;
            single_object_is_vertices(1, colu) = t2;
    
            colu++;
        }
        // LL: Since reading two lines at the time increase the row counter by two
        row_counter += 2;
    
        // LL: Reduce the number of columns from a hundred to the actual number
        single_object_is_vertices.conservativeResize(3,colu);
        // LL: Push the vertices matrix of the current object to the vector containing the info. on all the objects in the frame.
        read_inst_segment_vert.push_back(single_object_is_vertices);
    }
    filetxt.close();  
    return true;
}

# ifdef at3dcv_leander_no_depth
void poly_eigen_to_string_rep(Eigen::MatrixXi convex_hull_vertices, std::string &poly_string_rep){
    // LL: Convert polygon to the string format required by boost/geometry.
    // LL: Important: First and last point allways have to be the same to get a correct result.
    // LL: Important: There should not be any taps before and after each ','.  
    poly_string_rep = "POLYGON((";
    for(int j = 0 ; j != convex_hull_vertices.cols(); j++)
    {
        poly_string_rep = poly_string_rep + std::to_string(convex_hull_vertices(0,j)) + " " + std::to_string(convex_hull_vertices(1,j)) + ","; 
    }
    // LL: Add the first vertix to the end to ensure a closed polygon
    poly_string_rep = poly_string_rep + std::to_string(convex_hull_vertices(0,0)) + " " + std::to_string(convex_hull_vertices(1,0)) + "))";
}

void poly_vec_eigen_to_string_rep(std::vector<Eigen::MatrixXi> raw_all_obj2d_ss_vertix, std::vector<std::string> &geometries){
    // LL: Convert all polygons from current file to the string format required by boost/geometry and write them to geometries
    // LL: Important: First and last point allways have to be the same to get a correct result.
    // LL: Important: There should not be any taps before and after every ',' 
    for(int i = 0; i != raw_all_obj2d_ss_vertix.size(); i++) 
    {   
        std::string geo = "POLYGON((";
        for(int j = 0 ; j != raw_all_obj2d_ss_vertix[i].cols(); j++)
        {
            geo = geo + std::to_string(raw_all_obj2d_ss_vertix[i](0,j)) + " " + std::to_string(raw_all_obj2d_ss_vertix[i](1,j)) + ",";
        }
        // LL: Add the first vertix to the end to ensure a closed polygon
        geo = geo + std::to_string(raw_all_obj2d_ss_vertix[i](0,0)) + " " + std::to_string(raw_all_obj2d_ss_vertix[i](1,0)) + "))";
        geometries.push_back(geo);
    }
}

int poly_string_to_boost_poly(std::string poly_txt_rep, polygon &poly){
    
    // LL: Convert txt representation to boost polygon
    std::string reason;
    boost::geometry::read<boost::geometry::format_wkt>(poly, poly_txt_rep);

    // LL: Ensure that the geomtry is valid (e.g. correct the orientation)
    if (!boost::geometry::is_valid(poly))
    {
        boost::geometry::correct(poly);
        if (!boost::geometry::is_valid(poly, reason))
        {   
            std::cout << "The geometry is not a valid geometry for the following reason: " + reason << std::endl;
            return 1;
        }
    }
    return 0;
}

double perc_poly2_covered_by_poly1(polygon poly1, polygon poly2)
{
    // LL: Double-ended queue - an indexed sequence container that allows fast insertion and deletion at both
    std::deque<polygon> output;
    // LL: Given two geometries identify their intersecting geometries a write them to output
    boost::geometry::intersection(poly1, poly2, output);
    // LL: Sum the area of all intersecting geomtries 
    double area = 0;
    for (auto& p : output)
        area += boost::geometry::area(p);

    return (area/boost::geometry::area(poly2));
}


int visualize_polygons(std::string file_name, std::vector<polygon> polygons)
{
    // LL: Map the two polygons to a svg file
    // Declare a stream and an SVG mapper
    std::ofstream svg(file_name + ".svg");
    boost::geometry::svg_mapper<point_type> mapper(svg, 400, 400);
    // Add geometries such that all these geometries fit on the map
    int poly_vec_length = polygons.size();
    for (int i = 0; i != poly_vec_length; ++i)
        {
            mapper.add(polygons[i]);
        }

    // Draw the geometries on the SVG map, using a specific SVG style
    for (int i = 0; i != poly_vec_length; ++i)
        {
            std::string color = std::to_string((255/poly_vec_length)*i);
            std::string color_rgb = "("+color+","+color+","+color+")"; 
            mapper.map(polygons[i], "fill-opacity:0.3;fill:"+color_rgb+";stroke:rgb" +color_rgb+ ";stroke-width:2", 5);
        }

    return 0;
}

void sort_2d_cuboid_vertices(double vp_1_position, Eigen::Matrix2Xi all_configs_error_one_objH_int, Eigen::MatrixXi &cub_prop_2di)
{
    //LL: Retrive the current 2d bb proposal
    //LL: Order the vectors to be: top_front_left, top_front_...
    //LL: Convert vertices to int    
    Eigen::VectorXi cuboid_to_raw_boxstructIds(8);
    if (vp_1_position == 1) // vp1 on left, for all configurations
    	cuboid_to_raw_boxstructIds << 1, 4, 3, 2, 7, 8, 5, 6;
    if (vp_1_position == 2) // vp1 on right, for all configurations
        cuboid_to_raw_boxstructIds << 4, 1, 2, 3, 8, 7, 6, 5;

    for (int i = 0; i < 8; i++)
        cub_prop_2di.col(i) = all_configs_error_one_objH_int.col(cuboid_to_raw_boxstructIds(i) - 1); // minius one to match index

}

void cuboid_2d_vertices_to_2d_surfaces(Eigen::Matrix2Xi cub_prop_2d, std::vector<Eigen::MatrixXi> &eigen_2d_surfaces)
{
    Eigen::MatrixXi sqr_1(2, 4);
    Eigen::MatrixXi sqr_2(2, 4);
    Eigen::MatrixXi sqr_3(2, 4);
    Eigen::MatrixXi sqr_4(2, 4);
    Eigen::MatrixXi sqr_5(2, 4);
    Eigen::MatrixXi sqr_6(2, 4); 
    sqr_1 << cub_prop_2d.col(0), cub_prop_2d.col(1), cub_prop_2d.col(2), cub_prop_2d.col(3);
    sqr_2 << cub_prop_2d.col(0), cub_prop_2d.col(1), cub_prop_2d.col(5), cub_prop_2d.col(4);
    sqr_3 << cub_prop_2d.col(0), cub_prop_2d.col(3), cub_prop_2d.col(7), cub_prop_2d.col(4);
    sqr_4 << cub_prop_2d.col(4), cub_prop_2d.col(5), cub_prop_2d.col(6), cub_prop_2d.col(7);
    sqr_5 << cub_prop_2d.col(6), cub_prop_2d.col(7), cub_prop_2d.col(3), cub_prop_2d.col(2);
    sqr_6 << cub_prop_2d.col(1), cub_prop_2d.col(2), cub_prop_2d.col(6), cub_prop_2d.col(5);
    eigen_2d_surfaces.insert(eigen_2d_surfaces.end(),{sqr_1, sqr_2, sqr_3, sqr_4, sqr_5, sqr_6});
}

void eigen_2d_cub_surfaces_to_boost_poly_surfaces(std::vector<Eigen::MatrixXi> eigen_2d_surfaces, std::vector<polygon> &boost_poly_surfaces)
{
    std::string boost_string_rep;
    for (int i = 0; i != eigen_2d_surfaces.size(); ++i)
    {
        polygon poly;
        poly_eigen_to_string_rep(eigen_2d_surfaces[i], boost_string_rep);
        poly_string_to_boost_poly(boost_string_rep, poly);
        boost_poly_surfaces.push_back(poly);
    }
}
# endif
// LL: Added by Leander 
#endif


bool read_obj_detection_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &all_strings)
{
    // LL: Check if the file can be read
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
    
    // LL: Clear the memory of the all_strings vector that will save the class names
    all_strings.clear();
    
    // LL: Read the text file to `filetxt`
    std::ifstream filetxt(txt_file_name.c_str());
    
    // LL: If the memory was not correctly assigned to `read_number_mat`, set rows to 100 and columns to 10 if rows==0
    if (read_number_mat.rows() == 0)
        read_number_mat.resize(100, 10);
    int row_counter = 0;
    std::string line;
    // LL: Loop over all lines from the file
    while (getline(filetxt, line))
    {
        double t;
        if (!line.empty())
        {
            // LL: Retrive the first string (column) in the current line and write it to `classname`
            std::stringstream ss(line);
            std::string classname;
            ss >> classname;
            all_strings.push_back(classname);

            int colu = 0;
            // LL: Extracts one by one all remaining columns of the current row and writes them to read_number_mat
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
            }
            row_counter++;
            if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
        }
    }
    filetxt.close();
    read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows
}

bool read_obj_detection2_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &all_strings)
{
    if (!std::ifstream(txt_file_name))
    {
        std::cout << "ERROR!!! Cannot read txt file " << txt_file_name << std::endl;
        return false;
    }
    all_strings.clear();
    std::ifstream filetxt(txt_file_name.c_str());
    if (read_number_mat.rows() == 0)
        read_number_mat.resize(100, 10);
    int row_counter = 0;
    std::string line;
    while (getline(filetxt, line))
    {
        double t;
        if (!line.empty())
        {
            std::stringstream ss(line);

            int colu = 0;
            while (ss >> t)
            {
                read_number_mat(row_counter, colu) = t;
                colu++;
                if (colu > read_number_mat.cols() - 1)
                    break;
            }

            std::string classname;
            ss >> classname;
            all_strings.push_back(classname);

            row_counter++;
            if (row_counter >= read_number_mat.rows()) // if matrix row is not enough, make more space.
                read_number_mat.conservativeResize(read_number_mat.rows() * 2, read_number_mat.cols());
        }
    }
    filetxt.close();
    read_number_mat.conservativeResize(row_counter, read_number_mat.cols()); // cut into actual rows
}

void sort_indexes(const Eigen::VectorXd &vec, std::vector<int> &idx, int top_k)
{
    std::partial_sort(idx.begin(), idx.begin() + top_k, idx.end(), [&vec](int i1, int i2) { return vec(i1) < vec(i2); });
}

void sort_indexes(const Eigen::VectorXd &vec, std::vector<int> &idx)
{
    sort(idx.begin(), idx.end(), [&vec](int i1, int i2) { return vec(i1) < vec(i2); });
}

template <class T>
T normalize_to_pi(T angle)
{
    if (angle > M_PI / 2)
        return angle - M_PI; // # change to -90 ~90
    else if (angle < -M_PI / 2)
        return angle + M_PI;
    else
        return angle;
}
template double normalize_to_pi(double);

template <class T>
void print_vector(const std::vector<T> &vec)
{
    for (size_t i = 0; i < vec.size(); i++)
        std::cout << vec[i] << "  ";
    std::cout << std::endl;
}
template void print_vector(const std::vector<double> &);
template void print_vector(const std::vector<float> &);
template void print_vector(const std::vector<int> &);

template <class T>
void linespace(T starting, T ending, T step, std::vector<T> &res)
{
    res.reserve((ending - starting) / step + 2);
    while (starting <= ending)
    {
        res.push_back(starting);
        starting += step; // TODO could recode to better handle rounding errors
        if (res.size() > 1000)
        {
            std::cout << "Linespace too large size!!!!" << std::endl;
            break;
        }
    }
}
template void linespace(int, int, int, std::vector<int> &);
template void linespace(double, double, double, std::vector<double> &);
