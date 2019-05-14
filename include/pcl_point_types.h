// #ifndef PCL_POINT_TYPES_H_
// #define PCL_POINT_TYPES_H_

#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace pcl
{

//PointXYZ and PointXYZI are standard on pcl, so not defined here

//X,Y,Z,intensity,ring
struct EIGEN_ALIGN16 PointXYZIR {
    PCL_ADD_POINT4D;
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
};

//X,Y,Z,intensity,reflectivity
struct EIGEN_ALIGN16 PointXYZIF {
    PCL_ADD_POINT4D;
    uint16_t intensity;
    uint16_t reflectivity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//X,Y,Z,intensity,reflectivity,noise
struct EIGEN_ALIGN16 PointXYZIFN {
    PCL_ADD_POINT4D;
    uint16_t intensity;
    uint16_t reflectivity;
    uint16_t noise;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIF,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint16_t, intensity, intensity)
    (uint16_t, reflectivity, reflectivity)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIFN,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint16_t, intensity, intensity)
    (uint16_t, reflectivity, reflectivity)
    (uint16_t, noise, noise)
)


// #endif // PCL_POINT_TYPES_H_