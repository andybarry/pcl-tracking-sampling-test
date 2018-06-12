#include <iostream>
#include <pcl/point_types.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/tracking.h>
#include <pcl/visualization/cloud_viewer.h>

/**
 * Example demonstrating rotation-space sampling of the PCL tracking.  Works by
 * creating a tracker, asking it to sample a large number of points, and then
 * drawing those points in the following way:
 *
 * 1. Take the rotation of each particle and use it to rotate the unit vector
 * vector (1, 0, 0).
 * 2. Draw the resulting point at the end of the unit vector.
 *
 * This draws the sampled rotations as points on a unit sphere.
 */
int main(int argc, char **argv) {

  boost::shared_ptr<pcl::tracking::ParticleFilterTracker<
      pcl::PointXYZ, pcl::tracking::ParticleXYZRPY> >
      tracker(new pcl::tracking::ParticleFilterTracker<
              pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>());

  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ point;
  point.x = 1;
  point.y = 1;
  point.z = 1;
  ref_cloud->push_back(point);

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  target_cloud->push_back(point);

  std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.25);
  std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

  tracker->setTrans(Eigen::Affine3f::Identity());
  tracker->setInitialNoiseCovariance(initial_noise_covariance);
  tracker->setInitialNoiseMean(default_initial_mean);
  tracker->setIterationNum(1);
  tracker->setParticleNum(100000);
  tracker->setResampleLikelihoodThr(0.00);
  tracker->setUseNormal(false);

  // Setup coherence object for tracking
  pcl::tracking::ApproxNearestPairPointCloudCoherence<
      pcl::PointXYZ>::Ptr coherence =
      pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr(
          new pcl::tracking::ApproxNearestPairPointCloudCoherence<
              pcl::PointXYZ>());

  boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZ> >
      distance_coherence =
          boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZ> >(
              new pcl::tracking::DistanceCoherence<pcl::PointXYZ>());
  coherence->addPointCoherence(distance_coherence);

  boost::shared_ptr<pcl::search::Octree<pcl::PointXYZ> > search(
      new pcl::search::Octree<pcl::PointXYZ>(0.01));
  coherence->setSearchMethod(search);
  coherence->setMaximumDistance(0.01);

  tracker->setCloudCoherence(coherence);

  tracker->setReferenceCloud(ref_cloud);
  tracker->setInputCloud(target_cloud);
  tracker->compute();

  // Draw the particles
  pcl::tracking::ParticleFilterTracker<
      pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>::PointCloudStatePtr
      particles = tracker->getParticles();

  pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = 0; i < particles->points.size(); i++) {
    // We only want to visualize the rotation, so set the point to be on the
    // unit sphere, with its position 1.0 * x_hat.
    Eigen::Affine3f transform = pcl::getTransformation(
        particles->points[i].x, particles->points[i].y, particles->points[i].z,
        particles->points[i].roll, particles->points[i].pitch,
        particles->points[i].yaw);

    Eigen::Vector3f xhat(1, 0, 0);
    Eigen::Vector3f viz_point = transform.rotation() * xhat;

    pcl::PointXYZ point;
    point.x = viz_point.x();
    point.y = viz_point.y();
    point.z = viz_point.z();
    particle_cloud->points.push_back(point);
  }

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(particle_cloud);
  while (!viewer.wasStopped()) {
  }

  return (0);
}
