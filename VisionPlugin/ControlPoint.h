// control point  manipulator

#ifndef DEF_CONTROLPOINT
#define DEF_CONTROLPOINT

#include <initOpticalFlow.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>


class ControlPoint
{
public:
        ControlPoint();
	ControlPoint(
		cv::Point3f initCp,
		cv::Point3f cp,		
		int maxNbNeighbors,
		int nbNeighbors,
		float radius,
		std::vector<int> nnIndices,
		std::vector<float> nnDists,
		std::vector<float> nnQualities
        );
        ControlPoint(
		cv::Point3f initCp,
		int maxNbNeighbors,
		float radius
	);
	~ControlPoint();

	cv::Point3f getInitCp();
	cv::Point3f getCp();
	int getMaxNbNeighbors();
	int getNbNeighbors();
	float getRadius();
	float getGlobalQuality();
	std::vector<int> getNNIndices();
	std::vector<float> getNNDists();
	std::vector<float> getNNQualities();

	void setCp(cv::Point3f pt);
	void setNbNeighbors(int nb);
	void setNNIndices(std::vector<int> nnIndic);
	void setNNDists(std::vector<float> nnDists);
	void setNNQualities(std::vector<float> nnWeight);
	void setNDist(float dis, int indic);
	void setNQuality(float qua, int indic);
	void setGlobalQuality(float gQua);

	void addNIndice(int indic);
	void addNDist(float dist);
	void addNQuality(float qua);	

private:
	cv::Point3f m_initCp;	
	cv::Point3f m_cp;
	int m_maxNbNeighbors;
	int m_nbNeighbors;
	float m_radius;
	float m_globalQuality;
	std::vector<int> m_nnIndices;
	std::vector<float> m_nnDists;
	std::vector<float> m_nnQualities;
};

#endif
