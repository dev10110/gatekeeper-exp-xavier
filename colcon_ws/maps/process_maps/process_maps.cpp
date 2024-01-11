

#include <Eigen/Eigen>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;

bool load_cloud(const std::string &filename, PointCloud &cloud) {

  if (pcl::io::loadPLYFile<Point>(filename, cloud) == -1) {
    std::cout << "Couldn't read ply file" << std::endl;
    return false;
  }
  std::cout << "Loaded " << cloud.width * cloud.height << " data points. "
            << std::endl;

  return true;
}

void print_cloud(const PointCloud &cloud, int N = 10) {

  int counter = 0;

  for (const auto &point : cloud) {
    std::cout << "    " << point.x << " " << point.y << " " << point.z << " "
              << point.intensity << std::endl;

    counter++;
    if (counter > N) {
      break;
    }
  }

  return;
}

bool read_matrix(const std::string filename, Eigen::Matrix4d &outputMat) {
  // assumes the data is actually 16 elements, arranged in a row-major format

  // open the file
  std::fstream cin;
  cin.open(filename.c_str());
  if (cin.fail()) {
    std::cout << "Could not read rotations file" << std::endl;
    cin.close();
    return false;
  }

  // file opened, start reading
  std::string s;
  std::vector<double> values;
  // read the line
  while (getline(cin, s)) {
    std::stringstream input(s);
    double temp;
    // read the entries of the line
    while (input >> temp) {
      values.push_back(temp);
    }
  }

  if (values.size() != 16) {
    std::cout << "Error: rotation matrix file does not contain 16 entries"
              << std::endl;
    cin.close();
    return false;
  }

  // save the values vector to a matrix by overwriting
  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      outputMat(i, j) = values[k];
      k = k + 1;
    }
  }

  cin.close();
  return true;
}

// takes the input cloud, builds a kdTree
// for each point in the output PointCloud finds the nearest neighbor
// if the nearest neighbor is within the margin,
// sets the output clouds intensity to this value
// else sets intensity to unknown value.
bool nearest_neighbor_fill(const PointCloud::Ptr cloud_in,
                           PointCloud &cloud_out, double margin = 0.075,
                           double unknownValue = 1000.0) {

  // build a kd tree
  pcl::KdTreeFLANN<Point> kdtree;

  kdtree.setInputCloud(cloud_in);

  constexpr int K = 1;

  for (Point &p : cloud_out) {

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    if (kdtree.nearestKSearch(p, K, pointIdxKNNSearch,
                              pointKNNSquaredDistance) > 0) {
      // found a point
      if (pointKNNSquaredDistance[0] <= margin * margin) {
        p.intensity = (*cloud_in)[pointIdxKNNSearch[0]].intensity;
      } else {
        // too far
        p.intensity = unknownValue;
      }
    } else {
      // could not find a point
      // should not happen
      std::cout << "KNN could not find a nearest point" << std::endl;
      return false;
    }
  }

  return true;
}

bool create_gridded_cloud(PointCloud &cloud, double xmin, double ymin,
                          double zmin, double xmax, double ymax, double zmax,
                          double resolution, double unknownValue = 1000.0) {

  int Nx = int((xmax - xmin) / resolution);
  int Ny = int((ymax - ymin) / resolution);
  int Nz = int((zmax - zmin) / resolution);

  // update cloud dims
  cloud.width = Nx * Ny * Nz;
  cloud.height = 1;
  cloud.points.resize(Nx * Ny * Nz);

  // fill values
  size_t counter = 0;
  for (size_t i = 0; i < Nx; i++) {
    for (size_t j = 0; j < Ny; j++) {
      for (size_t k = 0; k < Nz; k++) {
        cloud[counter].x = xmin + i * resolution;
        cloud[counter].y = ymin + j * resolution;
        cloud[counter].z = zmin + k * resolution;
        cloud[counter].intensity = unknownValue;
        counter++;
      }
    }
  }

  return true;
}

bool filter_by_margin(PointCloud::Ptr cloud, double margin=0.075)
{

	// applies a filter ensuring each cell has an intensity between -margin and +margin
	pcl::PassThrough<Point> filter; 
filter.setInputCloud(cloud);
filter.setFilterFieldName("intensity");
//filter.setFilterLimits(-margin, margin);
filter.setFilterLimits(margin, 100.0);
filter.filter(*cloud);

return true;

}

int main(int argc, char **argv) {

  std::string directory = "/workspaces/isaac_ros-dev/maps/noisy_map";

  // load the rotations
  std::cout << "starting load rotation matrix" << std::endl;
  std::string filename_rotation = directory + "/rototranslation.txt";
  Eigen::Matrix4d T_L_EO_matrix;
  if (!read_matrix(filename_rotation, T_L_EO_matrix)) {
    std::cout << "Failed." << std::endl;
    return -1;
  }
  Eigen::Affine3d T_L_EO(T_L_EO_matrix);

  // note: T_L_EO maps body frame vector TO world frame vector

  // load the esdfs
  std::cout << "starting load esdfs" << std::endl;
  std::string filename_esdf = directory + "/ros2_esdf.ply";
  std::string filename_certified_esdf = directory + "/ros2_certified_esdf.ply";

  PointCloud::Ptr cloud_esdf(new PointCloud);
  PointCloud::Ptr cloud_esdf_rotated(new PointCloud);
  PointCloud::Ptr cloud_certified_esdf(new PointCloud);
  PointCloud::Ptr cloud_certified_esdf_rotated(new PointCloud);

  // load the esdf
  if (!load_cloud(filename_esdf, *cloud_esdf)) {
    std::cout << "Failed." << std::endl;
    return -1;
  }

  // load the certified_esdf
  if (!load_cloud(filename_certified_esdf, *cloud_certified_esdf)) {
    std::cout << "Failed." << std::endl;
    return -1;
  }

  // rotate pointclouds
  std::cout << "starting rotate esdfs" << std::endl;
  pcl::transformPointCloud(*cloud_esdf, *cloud_esdf_rotated, T_L_EO.inverse());
  pcl::transformPointCloud(*cloud_certified_esdf, *cloud_certified_esdf_rotated,
                           T_L_EO.inverse());

  // get min-max axes of the clouds
  Point minPt, maxPt;
  pcl::getMinMax3D(*cloud_esdf, minPt, maxPt);
  std::cout << "  - ESDF minPt: " << minPt << std::endl;
  std::cout << "  - ESDF maxPt: " << maxPt << std::endl;

  pcl::getMinMax3D(*cloud_certified_esdf, minPt, maxPt);
  std::cout << "  - ESDF minPt: " << minPt << std::endl;
  std::cout << "  - ESDF maxPt: " << maxPt << std::endl;

  // create a resampling grid
  std::cout << "starting creating grid" << std::endl;

  double xmin = -7;
  double ymin = -8;
  double zmin = -2;
  double xmax = 9;
  double ymax = 9;
  double zmax = 5;
  double resolution = 0.075;

  PointCloud::Ptr cloud_esdf_gridded(new PointCloud);
  PointCloud::Ptr cloud_certified_esdf_gridded(new PointCloud);
  create_gridded_cloud(*cloud_esdf_gridded, xmin, ymin, zmin, xmax, ymax, zmax,
                       resolution);
  create_gridded_cloud(*cloud_certified_esdf_gridded, xmin, ymin, zmin, xmax,
                       ymax, zmax, resolution);

  // start interpolating
  std::cout << "starting interpolate grid" << std::endl;
  if (!nearest_neighbor_fill(cloud_esdf_rotated, *cloud_esdf_gridded)) {
    std::cout << "FAILED" << std::endl;
    return -1;
  }

  if (!nearest_neighbor_fill(cloud_certified_esdf_rotated,
                             *cloud_certified_esdf_gridded)) {
    std::cout << "FAILED" << std::endl;
    return -1;
  }
  
  // filter points
  std::cout << "starting filtering" << std::endl;
  filter_by_margin(cloud_esdf_gridded, 0.075);
  filter_by_margin(cloud_certified_esdf_gridded, 0.075);

  // write to file
  std::cout << "starting write files" << std::endl;
  std::string filename_esdf_rotated = directory + "/ros2_esdf_rotated.ply";
  std::string filename_certified_esdf_rotated =
      directory + "/ros2_certified_esdf_rotated.ply";
  std::string filename_esdf_gridded =
      directory + "/ros2_esdf_rotated_gridded.ply";
  std::string filename_certified_esdf_gridded =
      directory + "/ros2_certified_esdf_rotated_gridded.ply";
  auto writer = pcl::PLYWriter();

  if (writer.write(filename_esdf_rotated, *cloud_esdf_rotated) < 0) {
    std::cout << "could not write rotated esdf";
    return -1;
  }

  if (writer.write(filename_certified_esdf_rotated,
                   *cloud_certified_esdf_rotated) < 0) {
    std::cout << "could not write rotated certified esdf";
    return -1;
  }

  if (writer.write(filename_esdf_gridded, *cloud_esdf_gridded) < 0) {
    std::cout << "could not write gridded esdf";
    return -1;
  }

  if (writer.write(filename_certified_esdf_gridded,
                   *cloud_certified_esdf_gridded) < 0) {
    std::cout << "could not write gridded certified esdf";
    return -1;
  }

  std::cout << "success" << std::endl;
}
