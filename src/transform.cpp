#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

#define PI 3.1415926535897932346f

int main(int argc,char **argv){
   //3D旋转矩阵 定义为单位阵
   Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
   //定义旋转向量  沿着z轴旋转45度
   Eigen::AngleAxisd rotation_vector(PI/4,Eigen::Vector3d(0,0,1)); 
   //  隐含了 一个变换  angleaxisd
   
   /*
    * 旋转向量  -->  旋转矩阵 
    * 旋转向量  -->  旋转矩阵  --->   欧拉角
    * 旋转向量  -->  四元数
    */
   cout<<"旋转矩阵："<<endl;
   cout<<rotation_vector.matrix()<<endl;   //..............................rotation_vector   to   rotation_matrix(1)
                                                                          //旋转向量--->旋转矩阵
                                                                          
   rotation_matrix = rotation_vector.toRotationMatrix();
   cout<<"旋转矩阵："<<endl;
   cout<<rotation_matrix<<endl;            //...............................rotation_vector to rotation_matrix(2)
                                                                          //旋转向量--->旋转矩阵
   
   Eigen::Quaterniond q = Eigen::Quaterniond (rotation_vector);  //....................rotation_vector  to Quaterniond
   cout<<"四元数："<<endl;
   cout<<q.coeffs()<<endl;  // (x,y,z,w)
                                                                          //旋转向量--->四元数
   
   /*
    * 旋转矩阵  -->  旋转向量
    * 旋转矩阵  --->   欧拉角
    * 旋转矩阵   --->   四元数
    */
   rotation_vector=rotation_matrix;   // ..................................rotation_matrix  to rotation_vector
                                                                          //旋转矩阵--->旋转向量  
   rotation_matrix<<0.986556,0.151367,0.0615994,
         -0.150703,0.98846,-0.0153144,
         -0.0632066,0.00582533,0.997983;                                                                      
   Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);  //.......rotation_matrix  to euler_angles .........attention:  ypr
   cout<<"欧拉角："<<endl;
   cout<<euler_angles.transpose()<<endl;                                   //  ypr
                                                                          //旋转矩阵--->欧拉角
   rotation_matrix<<0.996206,-0.000875961,-0.0870252,
         0.000994554,0.999999,0.0013194,
         0.0870239,-0.00140095,0.996205;  
   q = Eigen::Quaterniond(rotation_matrix);  // .......................................rotation_matrix to Quaterniond
   //q = {1,0,0,0};//Quaternion::IDENTITY(1.0, 0.0, 0.0, 0.0)  单位四元数
   q = {0.996619343758,0.00530286785215,0.0313073396683,-0.0757738277316};
   q = {0.999246239662,0.00210847216658,0.0380004644394,-0.00764782959595};
   cout<<"四元数："<<endl;
   cout<<q.coeffs()<<endl;  // (x,y,z,w)
                                                                           //旋转矩阵--->四元数  
                                                                           
   /*
    * 四元数--->旋转矩阵
    * 四元数--->旋转矩阵 --->欧拉角
    * 四元数--->旋转向量 
    */                
   cout<<"旋转矩阵："<<endl;                                                        
   cout <<q.toRotationMatrix()<<endl;  // .............................................Quaterniond  to rotatin_matrix
                                                                          //四元数--->旋转矩阵   
                 
   cout<<"欧拉角："<<endl;                                                         
   cout<<q.toRotationMatrix().eulerAngles(2,1,0)<<endl; //......................................Quaterniond  to rotatin_matrix  to euler_angles  ....attention:  ypr
   euler_angles = q.toRotationMatrix().eulerAngles(2,1,0);
   cout<<"滚角角度制:" <<euler_angles(2)*180/3.1415926<<endl;
   cout<<"俯角角度制:" <<euler_angles(1)*180/3.1415926<<endl;
   cout<<"偏角角度制:" <<euler_angles(0)*180/3.1415926<<endl;

                                                                          //四元数--->旋转矩阵--->欧拉角
   
   cout<<"旋转向量："<<endl;
   rotation_vector = q ;             // ..................................Quaterniond  to rotation_vector
                                                                          //四元数--->旋转向量     
   /*
    * 都是因为定义的不同，存在隐式转换，
    * Eigen::Matrix3d rotation_matrix, Eigen::AngleAxisd rotation_vector,Eigen::Quaterniond q
    * 欧拉角 --->  旋转向量
    * 欧拉角 --->  旋转矩阵
    * 欧拉角 --->  四元数
    * 
    * 按照ypr旋转
    * tran.rotate(Eigen::AngleAxisf(FLAGS_roll / 180.0 * M_PI, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(FLAGS_pitch / 180.0 * M_PI, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(FLAGS_yaw / 180.0 * M_PI, Eigen::Vector3f::UnitZ()));
    */
 //  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angles(2),Eigen::Vector3d::UnitX()));
 //  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angles(1),Eigen::Vector3d::UnitY()));
 //  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angles(0),Eigen::Vector3d::UnitZ()));
 //  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0.362569/180.0*PI,Eigen::Vector3d::UnitX()));
 //  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(-2.22152/180.0*PI,Eigen::Vector3d::UnitY()));
 //  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(-88.5397/180.0*PI,Eigen::Vector3d::UnitZ()));
 //  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(-0.075884/180.0*PI,Eigen::Vector3d::UnitX()));
 //  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(-4.992493/180.0*PI,Eigen::Vector3d::UnitY()));
 //  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(0.050380/180.0*PI,Eigen::Vector3d::UnitZ()));
   Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(1/180.0*PI,Eigen::Vector3d::UnitX()));
   Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(-4/180.0*PI,Eigen::Vector3d::UnitY()));
   Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(-9/180.0*PI,Eigen::Vector3d::UnitZ()));
//   Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0.075884/180.0*PI,Eigen::Vector3d::UnitX()));
//   Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(4.992493/180.0*PI,Eigen::Vector3d::UnitY()));
//   Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(-0.050380/180.0*PI,Eigen::Vector3d::UnitZ()));
 //  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0/180.0*PI,Eigen::Vector3d::UnitX()));
 //  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(45/180.0*PI,Eigen::Vector3d::UnitY()));
 //  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(0/180.0*PI,Eigen::Vector3d::UnitZ()));
   rotation_matrix = rollAngle * pitchAngle * yawAngle;//................euler_angles to rotation_matrix
   cout<<"旋转矩阵："<<endl;
   cout<< rotation_matrix << endl;
   rotation_vector = rollAngle * pitchAngle * yawAngle;//................euler_angles to rotation_vector
   cout<<"旋转矩阵："<<endl;
   cout<< rotation_vector.matrix() << endl;
   q               = rollAngle * pitchAngle * yawAngle;//................euler_angles to Quaterniond
   cout<<"四元数："<<endl;
   cout<< q.coeffs()<<endl;
  
}
