/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ENGINE_ProjectStressOnImage_INL
#define SOFA_COMPONENT_ENGINE_ProjectStressOnImage_INL

#include <ProjectStressOnImage.h>
#include <sofa/helper/rmath.h> //M_PI
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/BasicShapes.h>

#include <sofa/simulation/common/AnimateBeginEvent.h>

namespace sofa
{

namespace component
{

namespace engine
{

template <class DataTypes>
ProjectStressOnImage<DataTypes>::ProjectStressOnImage()
: f_vmCenters( initData (&f_vmCenters, "vmCenters", "input 3D cooridinates") )
, f_vmColors( initData (&f_vmColors, "vmColors", "input vector of colors") )
, f_vmStress( initData (&f_vmStress, "vmStress", "input vector of stress") )
, f_indices( initData (&f_indices, "indices", "input indices to consider") )
, f_imageData( initData (&f_imageData, "imageData", "input image data") )
, f_vidName( initData (&f_vidName, "vidName", "input video name (string)") )
, f_maskName( initData (&f_maskName, "maskName", "input mask name (string)") )
{

	f_imageData.setDisplayed(false);
	this->f_listening.setValue(true);

}

template <class DataTypes>
ProjectStressOnImage<DataTypes>::~ProjectStressOnImage()
{
}


template <class DataTypes>
void ProjectStressOnImage<DataTypes>::init()
{
    addInput(&f_vmCenters);
    addInput(&f_vmColors);
    addInput(&f_vmStress);
    setDirtyValue();

    m_lastTick = cv::getTickCount();

    m_nbFrame = 0;
    m_firstFrameFlag = 0;

    //getIndicesOnMask();
    loadVideo();
}

template <class DataTypes>
void ProjectStressOnImage<DataTypes>::reinit()
{
    update();
}

template<class DataTypes>
void ProjectStressOnImage<DataTypes>::handleEvent(core::objectmodel::Event *event)
{
    // to force update at each time step (no input in engine) 
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
	// compute duration between two "sofa" timestep (dt) to synchronize with "opencv" video stream (fps)
 	double duration = ((double) (cv::getTickCount() - m_lastTick)) /cv::getTickFrequency();	

	// call update
	if(m_fps > 0 && duration > 1/m_fps)
	{
        	setDirtyValue();
        	update();
		m_nbFrame++;

		m_lastTick = cv::getTickCount();
	}
    }

}

template <class DataTypes>
double ProjectStressOnImage<DataTypes>::gaussianNoise()
{

	static double V1, V2, S;
	static int phase = 0;
	double X;

	if(phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
			} while(S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	} else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	return X;
}


template <class DataTypes>
void ProjectStressOnImage<DataTypes>::loadVideo()
{
	m_cap.open(f_vidName.getFullPath());
	if(!m_cap.isOpened()) { 
		std::cout << "\nERROR in opening video file : " << f_vidName.getValue() << std::endl;
	}
	else std::cout << "Loading video File : " << f_vidName.getValue() << std::endl;
	
	m_fps = m_cap.get(cv::CAP_PROP_FPS);
	m_width = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
	m_height = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	m_type = m_cap.get(CV_CAP_PROP_FORMAT);
	

	std::cout << "fps = "<< m_fps << std::endl;
	std::cout << "W x H = "<< m_width << "x" << m_height << std::endl;
	std::cout << "type = "<< m_type << std::endl;

	m_capOut = cv::VideoWriter("out.avi",CV_FOURCC('M','J','P','G'), m_fps, cv::Size(m_width,m_height),true);

}

bool findInVector(helper::ReadAccessor<Data<helper::vector<int> > > v, int value)
{
    for (unsigned int i = 0; i < v.size(); i++)
        if (v[i] == value) return true;
    return false;
}


template <class DataTypes>
void ProjectStressOnImage<DataTypes>::update()
{
	//cleanDirty();


	helper::ReadAccessor<Data<VecCoord> > vmCtrs = f_vmCenters;
	helper::ReadAccessor<Data<helper::vector<defaulttype::Vec3f> > > vmCols = f_vmColors;
	helper::ReadAccessor<Data<helper::vector<Real> > > vmStr = f_vmStress;
	helper::ReadAccessor<Data<helper::vector<int> > > vmInd = f_indices;

        Real minVM = (Real)1e20, maxVM = (Real)-1e20;
	for (size_t i = 0; i < vmStr.size(); i++) {
            minVM = (vmStr[i] < minVM) ? vmStr[i] : minVM;
            maxVM = (vmStr[i] > maxVM) ? vmStr[i] : maxVM;
        }


	// use the mask to filter out bordering point on the first frame only
	if(!m_firstFrameFlag) {
		m_indicesOnMask.resize(vmCtrs.size());
		if (f_maskName.getFullPath().length())
		{
			std::cout << "Mask used." << std::endl;
			cv::Mat mask = cv::imread(f_maskName.getFullPath().c_str(), CV_8UC1);

			//imshow("mask", mask);

			for (int i =0; i < m_indicesOnMask.size(); i++) 
			{	
				cv::Point ctr = cv::Point(vmCtrs[i][0], vmCtrs[i][1]);
				cv::Scalar intensity = mask.at<uchar>(ctr.y, ctr.x);
				//std::cout << intensity.val[0] << std::endl;

				if (intensity.val[0] == 255)		
					m_indicesOnMask[i] = 1;
				else 
					m_indicesOnMask[i] = 0;

			}
		}
		else {
			std::cout << "Mask unused." << std::endl;
			for (int i =0; i < vmCtrs.size(); i++) 
				m_indicesOnMask[i] = 1;
		}

		m_firstFrameFlag = 1;
	}


	double alpha = 0.4;
	float thresh = 0.5;
	int radius = 20;
	

	// capture from camera or from a scene data
	if (f_imageData.getValue().length()) {
		m_image = cv::Mat(m_height, m_width, 16, (uchar*)f_imageData.getValue().c_str(), cv::Mat::AUTO_STEP);
	}
	else
		m_cap >> m_image;

/*
	std::vector<int> normalizedVMStress;
	normalizedVMStress.resize(vmStr.size());
	for (int i = 0; i < normalizedVMStress.size(); i++)
		normalizedVMStress[i] = 255 - (((maxVM - vmStr[i])/maxVM ) * 255);

	cv::Mat gray(m_height, m_width, CV_8U, 255);

	for (int i = 0; i < normalizedVMStress.size(); i++) {
		cv::Point ctr = cv::Point(vmCtrs[i][0], vmCtrs[i][1]);
		for (int j = 0 ; j < 25; j++) {
			gray.at<uchar>(ctr.y,ctr.x) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y,ctr.x + j) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y,ctr.x - j) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y + j,ctr.x - j) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y + j,ctr.x) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y - j,ctr.x) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y + j,ctr.x + j) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y - j,ctr.x - j) = normalizedVMStress[i];
			gray.at<uchar>(ctr.y - j,ctr.x + j) = normalizedVMStress[i];

		}
	}


	cv::imshow("gray", gray);
*/

/*
	double min;
	double max;
	cv::minMaxIdx(gray, &min, &max);
	cv::Mat adjMap;
	// expand your range to 0..255. Similar to histEq();
	gray.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 

	cv::Mat falseColorsMap;
	applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

	cv::imshow("Out", falseColorsMap);		
*/
	//cv::flip(m_image,m_image,1);

	float maxVMX = 40;

	std::vector<cv::Point> vecCtrs;
	vecCtrs.clear();
	if(!m_image.empty())
	{
		cv::Mat overlay, track;
		m_image.copyTo(overlay);
		m_image.copyTo(track);
		for (int i =0; i < vmCtrs.size(); i++)
		{
			cv::Point ctr = cv::Point(vmCtrs[i][0], vmCtrs[i][1]);
			cv::Point cpt1 = cv::Point(vmCtrs[i][0] - radius, vmCtrs[i][1] - radius);
			cv::Point cpt2 = cv::Point(vmCtrs[i][0] + radius, vmCtrs[i][1] + radius);
			cv::Scalar col = cv::Scalar( vmCols[i][2]*255, vmCols[i][1]*255, vmCols[i][0]*255);
/*
			if ( (m_indicesOnMask[i]) && (findInVector(vmInd, i)) )
				{
					//cv::rectangle(overlay, cpt1, cpt2, col, -1, 8, 0);
					if (vmStr[i] >= maxVMX) {
						cv::circle(overlay, ctr, radius, cv::Scalar(10,10,255), -1, 8, 0);			
						vecCtrs.push_back(ctr);
					}
					if (vmStr[i] >= maxVMX*0.5 && vmStr[i] < maxVMX) {
						cv::circle(overlay, ctr, radius, cv::Scalar(10,255,255), -1, 8, 0);
						if (i%2 == 0) {			
						cv::circle(track, ctr, radius*0.2, cv::Scalar(0,255,255), -1, 8, 0);
						cv::circle(track, ctr, radius*0.2, cv::Scalar(0,0,0), 2, 8, 0);
						}		
					}
					if (vmStr[i] < maxVMX*0.5) {
						cv::circle(overlay, ctr, radius, cv::Scalar(10,255,10), -1, 8, 0);
						if (i%2 == 0) {
						cv::circle(track, cv::Point(ctr.x, ctr.y), radius*0.15, cv::Scalar(0,255,0),-1 , 8, 0);
						cv::circle(track, cv::Point(ctr.x, ctr.y), radius*0.15, cv::Scalar(0,0,0), 1, 8, 0);
						cv::circle(track, cv::Point(ctr.x + radius*0.5, ctr.y + radius*0.5), radius*0.15, cv::Scalar(0,255,0),-1,8,0);
						cv::circle(track, cv::Point(ctr.x + radius*0.5, ctr.y + radius*0.5), radius*0.15, cv::Scalar(0,0,0),1.5,8,0);
						cv::circle(track, cv::Point(ctr.x + radius*0.5, ctr.y), radius*0.15, cv::Scalar(0,255,0),-1,8,0);
						cv::circle(track, cv::Point(ctr.x + radius*0.5, ctr.y), radius*0.15, cv::Scalar(0,0,0),1.5,8,0);
						}
											
					}

				}
*/

			if ( (m_indicesOnMask[i]) && (vmStr[i] > maxVM*thresh) && (findInVector(vmInd, i)) ) {
				cv::circle(overlay, ctr, radius, col, -1, 8, 0);
				//cv::rectangle(overlay, cpt1, cpt2, col, -1, 8, 0);
			}
//			else 
//				cv::circle(overlay, ctr, radius, cv::Scalar(10,255,10), -1, 8, 0);	



		}

/*
		// find and draw the convex hull
		cv::Mat imgHull;
		m_image.copyTo(imgHull);
		if (vecCtrs.size()) { 
		        vector<int> hull;

			// Find convex Hull
			cv::convexHull(cv::Mat(vecCtrs), hull, false);

			int hullcount = (int)hull.size();
			cv::Point pt0 = vecCtrs[hull[hullcount-1]];

			for(int i = 0; i < hullcount; i++ )
			{
			    cv::Point pt = vecCtrs[hull[i]];
			    cv::line(imgHull, pt0, pt, cv::Scalar(0, 255, 0), 3 , cv::LINE_AA);
			    pt0 = pt;
			}
			vector<cv::Point> contour;
			for(int i = 0; i < hullcount; i++ )
			{
				contour.push_back(vecCtrs[hull[i]]);
			}
			
			for(int j=0 ; j < imgHull.rows ; j++) 
				for (int i=0; i< imgHull.cols ; i++)
					if ( cv::pointPolygonTest(contour, cv::Point2f(i,j), false ) >= 0)
						imgHull.at<uchar>(j,i) = 255;

		}
		cv::imshow("HULL", imgHull);	
*/
		cv::addWeighted(overlay, alpha, m_image, 1 - alpha, 0, m_image);
		cv::imshow("Image with color stress", m_image);	
		cv::imshow("Tracking", track);	
		m_capOut.write(m_image);
	}


	setDirtyValue();
}



template <class DataTypes>
void ProjectStressOnImage<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/)
{
}



} // namespace engine

} // namespace component

} // namespace sofa

#endif
