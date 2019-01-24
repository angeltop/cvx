#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>



namespace YAML {
template<>
struct convert< cv::Mat > {
                
	/*
	* Method that encodes cv::Mat in a YAML Node
	* returns a YAML::Node
	*/
	static Node encode(const cv::Mat& mat) {
		Node matrix;
		matrix["rows"] = mat.rows;
		matrix["cols"] = mat.cols;
		Node data;
		
		if(mat.rows>1)
		{
			for(int i=0; i<mat.rows; i++)
				for(int j=0; j<mat.cols; j++)
					data.push_back(mat.at<double>(i,j));
		
		}
		else
			for(int i=0; i<mat.cols; i++)
				data.push_back(mat.at<double>(i));
		matrix["data"] = data;
		return matrix;
	}
	/*
	* Method that decodes a YAML Node in a cv::Mat
	* returns true if it succeeds, otherwise false
	*/
	static bool decode(const Node& node, cv::Mat& mat) {
		int rows = node["rows"].as<int>();
		int cols = node["cols"].as<int>();
		Node dataNode = node["data"];
		
		mat = cv::Mat_<double>::zeros(rows, cols);
		int r=0, c=0;
		for(int i=0; i<dataNode.size(); i++)
		{
			if(rows>1)
			{	
				r= i/rows;
				c= i%rows;
				mat.at<double>(r,c) = dataNode[i].as<double>();
			}
			else
				mat.at<double>(i) = dataNode[i].as<double>();
				
		}
		
		std::cout<<"Matrix: "<<std::endl;
		
		for(int i=0; i<mat.rows; i++)
		{
			for(int j=0; j<mat.cols; j++)
				std::cout<<mat.at<double>(i,j)<<" ";;
			std::cout<<std::endl;
		}
		
		return true;
	}
};
}
namespace cvx { namespace util {

class PinholeCamera
{
public:
    PinholeCamera() {}
    PinholeCamera(double fx, double fy, double cx, double cy, const cv::Size &isz, const cv::Mat &dist = cv::Mat::zeros(5, 1, CV_64F)):
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), sz_(isz), dist_(dist) {}

    cv::Mat getMatrix() const {
        cv::Mat_<double> mat = cv::Mat_<double>::zeros(3, 3) ;
        mat(0, 0) = fx_ ; mat(0, 2) = cx_ ;
        mat(1, 1) = fy_ ; mat(1, 2) = cy_ ;
        mat(2, 2) = 1.0 ;
        return mat ;
    }

    cv::Mat getDistortion() const {
        return dist_ ;
    }

    void setMatrix(const cv::Mat &m) {
        cv::Mat_<double> mat(m) ;
        fx_ = mat(0, 0) ; fy_ = mat(1, 1) ;
        cx_ = mat(0, 2) ; cy_ = mat(1, 2) ;
    }

    void setDistortion(const cv::Mat &m) {
        dist_ = m ;
    }

    void setSize(const cv::Size &sz) {
        sz_ = sz ;
    }

    cv::Point2d project(const cv::Point3d& xyz) const {
        return cv::Point2d(fx_ * xyz.x / xyz.z + cx_, fy_ * xyz.y / xyz.z + cy_) ;
    }

    cv::Point3d backProject(const cv::Point2d& uv) const {
        return cv::Point3d((uv.x - cx_)/fx_, (uv.y - cy_)/fy_, 1.0) ;
    }

    Eigen::Vector3f backProject(float x, float y, float Z) const {
        return Eigen::Vector3f(Z*(x - cx_)/fx_, Z*(y - cy_)/fy_, Z) ;
    }

    cv::Mat rectifyImage(const cv::Mat& raw, int interpolation = cv::INTER_LINEAR) const ;
    cv::Mat unrectifyImage(const cv::Mat& rectified, int interpolation = cv::INTER_LINEAR) const ;

    cv::Point2d rectifyPoint(const cv::Point2d &uv_raw) const ;
    cv::Point2d unrectifyPoint(const cv::Point2d &uv_rect) const ;

    double fx() const { return fx_ ; }
    double fy() const { return fy_ ; }
    double cx() const { return cx_ ; }
    double cy() const { return cy_ ; }

    unsigned int width() const { return sz_.width ; }
    unsigned int height() const { return sz_.height ; }

    cv::Size sz() const { return sz_ ; }

    bool read(const std::string &fname)
    {
		std::ifstream file;
		file.open(fname.c_str());

		if (!file) std::cout << "error opening camera intrinsics file\n"<<fname << std::endl;
		file.close();
		YAML::Node list = YAML::LoadFile( fname);
        cv::Mat intrinsics, distortion;

        sz_.width = list["image_width"].as<int>() ;
        sz_.height = list["image_height"].as<int>();

        intrinsics = list["camera_matrix"].as<cv::Mat>();
        dist_ = list["distortion_coefficients"].as<cv::Mat>();

		
        fx_ = intrinsics.at<double>(0, 0) ;
        fy_ = intrinsics.at<double>(1, 1) ;
        cx_ = intrinsics.at<double>(0, 2) ;
        cy_ = intrinsics.at<double>(1, 2) ;

        return true ;

    }

    bool write(const std::string &fname)
    {
		std::ofstream file(fname.c_str());
		if(!file.is_open())
		{
				std::cerr<<"Unable to save camera intrinsics\n"<<fname<<std::endl;
				return false;
		}
		YAML::Node mainNode;
		mainNode["image_width"] = sz_.width;
		mainNode["image_height"] = sz_.height;
		mainNode["camera_matrix"] = getMatrix();
		mainNode["distortion_coefficients"] = dist_;
		file<<mainNode;
		file.close();
		return true;
    }

protected:

    double fx_, fy_, cx_, cy_ ;
    cv::Mat dist_ ;
    cv::Size sz_ ;

};

}}
#endif
