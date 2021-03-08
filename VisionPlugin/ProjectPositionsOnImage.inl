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
#ifndef SOFA_COMPONENT_ENGINE_ProjectPositionsOnImage_INL
#define SOFA_COMPONENT_ENGINE_ProjectPositionsOnImage_INL

#include <ProjectPositionsOnImage.h>
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
ProjectPositionsOnImage<DataTypes>::ProjectPositionsOnImage()
: f_positions( initData (&f_positions, "positions", "input 3D cooridinates") )
, f_imageData( initData (&f_imageData, "imageData", "input image data") )
, f_imageDataSTR( initData (&f_imageDataSTR, "imageDataSTR", "input video data (string)") )
, f_vidName( initData (&f_vidName, "vidName", "input video name (string)") )
, f_saveVideo( initData (&f_saveVideo, bool(false), "saveVideo", "save output video") )
{

	f_imageData.setDisplayed(false);
	this->f_listening.setValue(true);

}

template <class DataTypes>
ProjectPositionsOnImage<DataTypes>::~ProjectPositionsOnImage()
{
}


template <class DataTypes>
void ProjectPositionsOnImage<DataTypes>::init()
{
    addInput(&f_positions);
    setDirtyValue();

    m_lastTick = cv::getTickCount();

    m_nbFrame = 0;
    m_firstFrameFlag = 0;

    loadVideo();
}

template <class DataTypes>
void ProjectPositionsOnImage<DataTypes>::reinit()
{
    update();
}

template<class DataTypes>
void ProjectPositionsOnImage<DataTypes>::handleEvent(core::objectmodel::Event *event)
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
		std::cout << m_nbFrame << std::endl;

		m_lastTick = cv::getTickCount();
	}
    }

}

template <class DataTypes>
void ProjectPositionsOnImage<DataTypes>::loadVideo()
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

	m_capOutOri = cv::VideoWriter("outOri.avi",CV_FOURCC('M','J','P','G'), m_fps, cv::Size(m_width,m_height),true);
	m_capOutMask = cv::VideoWriter("outMask.avi",CV_FOURCC('M','J','P','G'), m_fps, cv::Size(m_width,m_height),true);
	m_capOutInpaint = cv::VideoWriter("outInpaint.avi",CV_FOURCC('M','J','P','G'), m_fps, cv::Size(m_width,m_height),true);

}


template <class DataTypes>
void ProjectPositionsOnImage<DataTypes>::update()
{
	//cleanDirty();


	helper::ReadAccessor<Data<VecCoord> > pos = f_positions;

	cv::Mat tmp;
	// capture from camera or from a scene data
	if (f_imageData.getValue().length()) {
		m_image = cv::Mat(m_height, m_width, 16, (uchar*)f_imageData.getValue().c_str(), cv::Mat::AUTO_STEP);
		tmp =  cv::Mat(m_height, m_width, 16, (uchar*)f_imageDataSTR.getValue().c_str(), cv::Mat::AUTO_STEP);
	}
	else
		m_cap >> m_image;


	std::vector<cv::Point> vecCtrs;
	vecCtrs.clear();
	if(!m_image.empty())
	{
		cv::Mat overlay, track;
		m_image.copyTo(overlay);
		m_image.copyTo(track);
		for (int i =0; i < pos.size(); i++)
		{
			vecCtrs.push_back(cv::Point(pos[i][0], pos[i][1]));
		}

		// find and draw the convex hull
		cv::Mat imgHull;
		m_image.copyTo(imgHull);
		std::vector<std::vector<cv::Point> > contours;

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
			std::vector<cv::Point> contour;

			for(int i = 0; i < hullcount; i++ )
			{
				contour.push_back(vecCtrs[hull[i]]);
			}
			contours.push_back(contour);
			
			for(int j=0 ; j < imgHull.rows ; j++) 
				for (int i=0; i< imgHull.cols ; i++)
					if ( cv::pointPolygonTest(contour, cv::Point2f(i,j), false ) >= 0)
						imgHull.at<uchar>(j,i) = 255;

		}
		cv::imshow("HULL", imgHull);	

		cv::Mat imgMask = cv::Mat::zeros(m_image.rows, m_image.cols, CV_8UC1);
		cv::Mat imgInpaint;
		m_image.copyTo(imgInpaint);
		cv::drawContours( imgMask, contours, 0, cv::Scalar( 255, 255, 255 ), CV_FILLED, 8 );
//		cv::inpaint(m_image, imgMask, imgInpaint, 7, cv::INPAINT_NS); //INPAINT_TELEA // INPAINT_NS
		cv::imshow("MASK", imgMask);
		
		for(int j = 0;j < imgMask.rows;j++){
		    for(int i = 0;i < imgMask.cols;i++){
			if (imgMask.at<uchar>(j,i) == 0)
				imgInpaint.at<cv::Vec3b>(j,i) = m_image.at<cv::Vec3b>(j,i);
			else 
				imgInpaint.at<cv::Vec3b>(j,i) = tmp.at<cv::Vec3b>(j,i);
		    }
		}

		cv::imshow("INPAINT", imgInpaint);

		

		if (f_saveVideo.getValue()) {
			{
			m_capOutOri.write(m_image);
			//m_capOutMask.write(imgMask);
			m_capOutInpaint.write(imgInpaint);
			}
		}



	}


	setDirtyValue();
}



template <class DataTypes>
void ProjectPositionsOnImage<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/)
{
}



} // namespace engine

} // namespace component

} // namespace sofa

#endif
