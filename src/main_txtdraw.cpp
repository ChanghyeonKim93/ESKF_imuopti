#include <ros/ros.h>

#include <iostream>
#include <exception>
#include <vector>
#include <string>
#include <fstream>
#include <ostream>

#include "CvPlot/cvplot.h"

struct DataImu{
  struct AngularVelocity{
    double x;
    double y;
    double z;
    AngularVelocity() {};
    AngularVelocity(double x_, double y_, double z_) : x(x_),y(y_),z(z_) {};
  };
  struct LinearAcceleration{
    double x;
    double y;
    double z;
    LinearAcceleration() {};
    LinearAcceleration(double x_, double y_, double z_) : x(x_),y(y_),z(z_) {};
  };
  struct Orientation{
    double w;
    double x;
    double y;
    double z;
    Orientation() {};
    Orientation(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {};
  };
  double t;
  LinearAcceleration linear_acceleration; 
  AngularVelocity    angular_velocity;
  Orientation        orientation;

  DataImu(){};
  DataImu(double t_, double ax, double ay, double az, double wx, double wy, double wz, double qw, double qx, double qy, double qz)
  : t(t_), angular_velocity(wx,wy,wz), linear_acceleration(ax,ay,az), orientation(qw,qx,qy,qz) {};
  
};

struct DataMag{
  struct MagneticField{
    double x;
    double y;
    double z;
    MagneticField() {};
    MagneticField(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {};
  };
  double t;
  MagneticField magnetic_field;
  
  //
  DataMag() {};
  DataMag(double t_, double mx_, double my_, double mz_) : t(t_), magnetic_field(mx_, my_, mz_) {};
};

struct DataOptitrack{
  struct Position{
    double x;
    double y;
    double z;
    Position() {};
    Position(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {};
  };
  struct Orientation{
    double w;
    double x;
    double y;
    double z;
    Orientation() {};
    Orientation(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {};
  };

  double t;
  Position position;
  Orientation orientation;

  DataOptitrack(){};
  DataOptitrack(double t_, double px, double py, double pz, double qw, double qx, double qy, double qz)
  : t(t_), position(px,py,pz), orientation(qw,qx,qy,qz) {};
};


std::ostream& operator<<(std::ostream& os, const DataImu& msg){
  os <<"["
  << msg.t <<" / " 
  << msg.linear_acceleration.x <<"," 
  << msg.linear_acceleration.y <<"," 
  << msg.linear_acceleration.z <<" / " 
  << msg.angular_velocity.x <<"," 
  << msg.angular_velocity.y <<"," 
  << msg.angular_velocity.z <<" / " 
  << msg.orientation.w <<"," 
  << msg.orientation.x <<"," 
  << msg.orientation.y <<"," 
  << msg.orientation.z << "]"; 

  return os;
};

std::ostream& operator<<(std::ostream& os, const DataOptitrack& msg){
  os <<"["
  << msg.t <<" / " 
  << msg.position.x <<"," 
  << msg.position.y <<"," 
  << msg.position.z <<" / " 
  << msg.orientation.w <<"," 
  << msg.orientation.x <<"," 
  << msg.orientation.y <<"," 
  << msg.orientation.z <<"]"; 
  return os;
};

std::ostream& operator<<(std::ostream& os, const DataMag& msg){
  os <<"["
  << msg.t <<" / " 
  << msg.magnetic_field.x <<"," 
  << msg.magnetic_field.y <<"," 
  << msg.magnetic_field.z << "]"; 
  return os;
};

void getIMUData(const std::string& filename, std::vector<DataImu>& data){
  data.resize(0);
  std::ifstream file(filename, std::ios::in);
  if(file.is_open()){
    std::string s;
    getline(file, s); // data field...
    while(file){
      getline(file, s);
      std::stringstream ss(s);
      
      uint32_t seq;
      DataImu tmp;
      
      ss >> seq 
      >> tmp.t
      >> tmp.linear_acceleration.x
      >> tmp.linear_acceleration.y
      >> tmp.linear_acceleration.z
      >> tmp.angular_velocity.x
      >> tmp.angular_velocity.y
      >> tmp.angular_velocity.z
      >> tmp.orientation.w
      >> tmp.orientation.x
      >> tmp.orientation.y
      >> tmp.orientation.z;

      data.emplace_back(tmp);
    }
    file.close();
  }
  else std::cout << "file open fail\n";
};


void getMagnetometerData(const std::string& filename, std::vector<DataMag>& data){
  data.resize(0);
  std::ifstream file(filename, std::ios::in);
  if(file.is_open()){
    std::string s;
    getline(file, s); // data field...
    while(file){
      getline(file, s);
      std::stringstream ss(s);
      
      uint32_t seq;
      DataMag tmp;
      
      ss >> seq 
      >> tmp.t
      >> tmp.magnetic_field.x
      >> tmp.magnetic_field.y
      >> tmp.magnetic_field.z;

      data.emplace_back(tmp);
    }
    file.close();
  }
  else std::cout << "file open fail\n";
};

void getOptitrackData(const std::string& filename, std::vector<DataOptitrack>& data){
  data.resize(0);
  std::ifstream file(filename, std::ios::in);
  if(file.is_open()){
    std::string s;
    getline(file, s); // data field...
    while(file){
      getline(file, s);
      std::stringstream ss(s);
      
      uint32_t seq;
      DataOptitrack tmp;
      
      ss >> seq 
      >> tmp.t
      >> tmp.position.x
      >> tmp.position.y
      >> tmp.position.z
      >> tmp.orientation.w
      >> tmp.orientation.x
      >> tmp.orientation.y
      >> tmp.orientation.z;

      data.emplace_back(tmp);
    }
    file.close();
  }
  else std::cout << "file open fail\n";
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "txtdraw_node");
  ros::NodeHandle nh("~");

  try {
    // Load txt file and save into the struct
    if(!ros::param::has("~directory")) throw std::runtime_error("'directory' is empty.\n");
    
    std::string dir;
    ros::param::get("~directory", dir);

    std::string filename_imu       = dir + "imu.txt";
    std::string filename_mag       = dir + "magnetometer.txt";
    std::string filename_optitrack = dir + "optitrack.txt";

    std::vector<DataImu> data_imu;
    getIMUData(filename_imu, data_imu);

    std::vector<DataMag> data_mag;
    getMagnetometerData(filename_mag, data_mag);

    std::vector<DataOptitrack> data_optitrack;
    getOptitrackData(filename_optitrack, data_optitrack);


    // draw
    std::vector<double> tx;
    std::vector<double> dt;
    for(int i = 0; i < data_imu.size()-1; ++i) {
      tx.push_back(data_imu[i].t - data_imu[0].t );
      dt.push_back(data_imu[i+1].t-data_imu[i].t);
    } 
    auto axes = CvPlot::makePlotAxes();
    axes.create<CvPlot::Series>(tx, dt, "-.g");
    CvPlot::show("IMU delta_t", axes);

    tx.resize(0);
    dt.resize(0);
    for(int i = 0; i < data_optitrack.size()-1; ++i) {
      tx.push_back(data_optitrack[i].t - data_optitrack[0].t );
      dt.push_back(data_optitrack[i+1].t-data_optitrack[i].t);
    } 
    auto axes2 = CvPlot::makePlotAxes();
    axes2.create<CvPlot::Series>(tx, dt, "-.g");
    CvPlot::show("Optitrack delta_t", axes2);
   
  } 
  catch (const std::exception& e) 
  {

  }
}
