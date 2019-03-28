#ifndef __CONFIGURATION_HPP__
#define __CONFIGURATION_HPP__

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace cvx { namespace util 
{
	
	class CameraRobotConfiguration
	{
	
	public:
		std::string robotFrame;
		std::string toolFrame;
		std::string cameraFrame;
		Eigen::Affine3d initialCameraEstimation;
		double robotToTagX, robotToTagY, robotToTagZ;
		Eigen::Affine3d finalCameraEstimation;
		CameraRobotConfiguration(std::string configurationFile)
		{
			readRobotConfiguration(configurationFile);
			
		}
		
		void setfinalCameraEstimation(Eigen::Affine3d est)
		{
			 finalCameraEstimation = est;
		}
		
		bool writeCameraTranformationToFile(std::string path)
		{
			std::ofstream strm(path) ;
			Eigen::Quaterniond qqt(finalCameraEstimation.rotation());
			strm<<"<?xml version=\"1.0\"?>"<<std::endl;
			strm<<"<launch>"<<std::endl;
			strm<<"<node name=\""<<toolFrame.substr(toolFrame.length()-4)<<"_to_camera\" pkg=\"tf\" type=\"static_transform_publisher\" args=\"";
			strm<<finalCameraEstimation.translation()(0)<<" "<<finalCameraEstimation.translation()(1)<<" "<<finalCameraEstimation.translation()(2);
			strm<<" "<<qqt.x()<<" "<< qqt.y()<<" "<< qqt.z()<<" "<< qqt.w();
			strm<<" "<<toolFrame<<" "<<cameraFrame<<" 100\"/>"<<std::endl;
			strm<<"</launch>"<<std::endl;
			
		}
		bool readRobotConfiguration(const std::string &fname)
		{
			
			std::ifstream file;
			file.open(fname.c_str());

			if (!file) std::cout << "error opening configuration file\n"<<fname << std::endl;
			file.close();
			YAML::Node conf = YAML::LoadFile( fname);
			
			robotFrame = conf["base_frame"].as<std::string>();
			toolFrame = conf["tool_frame"].as<std::string>();
			cameraFrame = conf["camera_frame"].as<std::string>();
			robotToTagX = conf["base_to_tag_x"].as<double>();
			robotToTagY = conf["base_to_tag_y"].as<double>();
			robotToTagZ = conf["base_to_tag_z"].as<double>();
			
			YAML::Node initialEstimation = conf["initial_cam_estimation"];
			
			double posX = initialEstimation["x"].as<double>();
			double posY = initialEstimation["y"].as<double>();
			double posZ = initialEstimation["z"].as<double>();
			
//			double roll = initialEstimation["roll"].as<double>();
//			double pitch = initialEstimation["pitch"].as<double>();
//			double yaw = initialEstimation["yaw"].as<double>();

                        double qx = initialEstimation["qx"].as<double>();
                        double qy = initialEstimation["qy"].as<double>();
                        double qz = initialEstimation["qz"].as<double>();
                        double qw = initialEstimation["qw"].as<double>();
                        Eigen::Quaterniond q(qw, qx, qy, qz);

                        //q.normalized().toRotationMatrix();
			Eigen::Matrix3d m;

//			m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//				* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//				* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
                        m = q.normalized().toRotationMatrix();
			std::cout << "original rotation:" << std::endl;
			std::cout << m << std::endl << std::endl;
			initialCameraEstimation =  Eigen::Translation3d(Eigen::Vector3d(posX, posY, posZ)) * m;
			
			std::cout << "original cam matrix:" << std::endl;
			std::cout << initialCameraEstimation.matrix() << std::endl << std::endl;
			return true;
		}
	};
}}

#endif // __CONFIGURATION_HPP__
