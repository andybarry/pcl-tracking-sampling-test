pcl-tracking-sampling-test
==========================

A simple example to demonstrate rotation-space sampling in PCL's tracking particle filter.

An example, samping 6000 particles with very high variance (0.25).  The mean transform is the identity, we sample around that rotation (that's why there's only a half-sphere).

Before rotation sampling fix (PCL pull request [#2339](https://github.com/PointCloudLibrary/pcl/pull/2339)):

![Before rotation sampling fix](https://github.com/andybarry/pcl-tracking-sampling-test/raw/master/old_sampling.gif)

After rotation sampling fix:

![After rotation sampling fix](https://github.com/andybarry/pcl-tracking-sampling-test/raw/master/new_sampling.gif)
