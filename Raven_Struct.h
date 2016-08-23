#ifndef RAVEN_STRUCT_H
#define RAVEN_STRUCT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


#define LEFT	0
#define RIGHT	1

#define X_CTRL 2 
#define Y_CTRL 0
#define Z_CTRL 1

#define X_READ 0
#define Y_READ 1
#define Z_READ 2


typedef Eigen::Matrix4d tf_t;
typedef Eigen::Matrix3d rot_t;
typedef Eigen::Vector3d vec_t;
typedef Eigen::Vector4d vec4_t;
typedef Eigen::Quaterniond qtr_t;

//#endif

// Structure used for raven positions

#pragma pack(1)

class Raven_Struct
{
public:
	vec_t  pos[2];
	qtr_t  qtr[2];
	double grasp[2];
	bool reached;
	bool manual;
	int left_right;
	vec_t color_3D_location[4];
	vec_t haptic[2];
	
	
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#pragma pack()


// SPECIFIC TO EIGEN: example: std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >
typedef std::vector<pos_struct, Eigen::aligned_allocator<pos_struct>> raven_path_t;

//typedef std::vector<pos_struct> raven_path_t;

/******************************************************************************
 Output operator quaternions.
******************************************************************************/
inline std::ostream& operator<<(std::ostream& os, const qtr_t &q)
{
    return os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
}

#endif
