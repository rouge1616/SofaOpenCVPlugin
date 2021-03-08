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
#ifndef SOFA_COMPONENT_ENGINE_InterFrameMatching_INL
#define SOFA_COMPONENT_ENGINE_InterFrameMatching_INL

#include <InterFrameMatching.h>
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
InterFrameMatching<DataTypes>::InterFrameMatching()
: f_outputFeatures( initData (&f_outputFeatures, "outputFeatures", "output 2D features") )
, f_outputStatus( initData (&f_outputStatus, "outputStatus", "output vector of status 1 for tracked feature, 0 for lost feature") )
, f_displayFeatures( initData (&f_displayFeatures, int(1), "displayFeatures", "display or not the tracked features") )
, f_view( initData (&f_view, int(1), "view", "Perpective = 1 | Orthpgraphic = 2 | Hide = 0") )
, f_scaleImg( initData (&f_scaleImg, float(1.0), "scaleImg", "apply scaling factor to the input image") )
, f_detectorThresh( initData (&f_detectorThresh, float(10), "detectorThresh", "threshold for features extraction") )
, f_vidName( initData (&f_vidName, "vidName", "input video name (string)") )
, f_maskName( initData (&f_maskName, "maskName", "input mask name (string)") )
, f_translation( initData (&f_translation, "translation", "3D translation") )
, f_scale( initData (&f_scale, Vec3(1.0,1.0,1.0), "scale", "3D scale") )
{

	this->f_listening.setValue(true);

}

template <class DataTypes>
InterFrameMatching<DataTypes>::~InterFrameMatching()
{

}


template <class DataTypes>
void InterFrameMatching<DataTypes>::init()
{
    addOutput(&f_outputFeatures);
    addOutput(&f_outputStatus);
    setDirtyValue();

    m_lastTick = cv::getTickCount();

    loadVideo();
    extractFeatures();
//    matchFeatures();
}

template <class DataTypes>
void InterFrameMatching<DataTypes>::loadVideo()
{
	m_cap.open(f_vidName.getFullPath());
	if(!m_cap.isOpened()) { 
		std::cout << "\nERROR in opening video file : " << f_vidName.getValue() << std::endl;
	}
	else std::cout << "Loading video File : " << f_vidName.getValue() << std::endl;
	
	m_fps = m_cap.get(cv::CAP_PROP_FPS);
	m_width = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
	m_height = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	std::cout << "fps = "<< m_fps << std::endl;
	std::cout << "W x H = "<< m_width << "x" << m_height << std::endl;
}

template <class DataTypes>
void InterFrameMatching<DataTypes>::extractFeatures()
{

	// capture

	cv::Mat firstFrame;
	m_cap >> firstFrame;
	//cv::flip(firstFrame,firstFrame,1);

	cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(f_detectorThresh.getValue(), true);

	if (f_maskName.getFullPath().length()) {
		cv::Mat mask = cv::imread(f_maskName.getFullPath().c_str(), CV_8UC1);
		fastDetector->detect( firstFrame, m_keypoints, mask);
		std::cout << "Mask used." << std::endl;
	}
	else {
		fastDetector->detect( firstFrame, m_keypoints);
		std::cout << "Mask unused." << std::endl;
	}

	std::cout << "Number of exracted features : "<< m_keypoints.size() << std::endl;	
	if (m_keypoints.size() > 0)
	{
		for (unsigned int i = 0; i < m_keypoints.size(); i++)
			m_points[0].push_back( m_keypoints[i].pt );
	}
	else 
		std::cout << "No features extracted. Check the threshold." << std::endl;

	// compute descriptor

//	cv::SiftDescriptorExtractor siftDescriptor;	
//	cv::SurfDescriptorExtractor surfDescriptor;

	cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    	//cv::Ptr<cv::DescriptorExtractor> descriptor;// = cv::DescriptorExtractor::create( "BRIEF" );	
	descriptor->compute( firstFrame, m_keypoints, m_descriptors);
}


template <class DataTypes>
void InterFrameMatching<DataTypes>::reinit()
{
    update();
}

template<class DataTypes>
void InterFrameMatching<DataTypes>::handleEvent(core::objectmodel::Event *event)
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

		m_lastTick = cv::getTickCount();
	}
    }

}

template <class DataTypes>
void InterFrameMatching<DataTypes>::update()
{
	//cleanDirty();
/*
	if (m_keypoints.size() > 0)
	{
		helper::WriteAccessor<Data<VecCoord> > out = f_outputFeatures;
		out.resize(m_keypoints.size());
		helper::WriteAccessor<Data<helper::vector<int> > >stat = f_outputStatus;
		stat.resize(m_keypoints.size());
		Vec3 trans = f_translation.getValue();
		Vec3 scale = f_scale.getValue();

		// capture
		m_cap >> m_image;
		if(!m_image.empty())
		{
			cv::Mat descriptorsU;
			std::vector<cv::KeyPoint> keypointsU;

			// initialize the matcher
			//FlannBasedMatcher flannmatcher;
			cv::FlannBasedMatcher flannmatcher(new cv::flann::LshIndexParams(20,10,2));

			cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(f_detectorThresh.getValue(), true);
			fastDetector->detect( m_image, keypointsU);
			cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
			descriptor->compute( m_image, keypointsU, descriptorsU);


			flannmatcher.knnMatch( m_descriptors, descriptorsU, m_matches, 2 );

//			cv::drawMatches(m_image, m_keypoints, m_image, keypointsU, _matchesLL, m_matches);
		}
	}
*/

	setDirtyValue();
}


template <class DataTypes>
void InterFrameMatching<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/)
{
	helper::ReadAccessor<Data<helper::vector<int> > >stat = f_outputStatus;

    if (f_displayFeatures.getValue())	
    	for (int i = 0 ; i < m_points[1].size() ; i++) {
		if (stat[i])	
			cv::circle(m_image, m_points[1][i], 3, cv::Scalar(0,255,0), 2, 8, 0);
    		else
			cv::circle(m_image, m_points[1][i], 3, cv::Scalar(255,30,125), 2, 8, 0);			
	}


    std::stringstream imageString;
    imageString.write((const char*)m_image.data, m_image.total()*m_image.elemSize());

    if(m_image.data)
    {
	if(f_view.getValue() == 1)
	{
	// PERSPECTIVE
	
		glEnable(GL_TEXTURE_2D);	// enable the texture
		glDisable(GL_LIGHTING);		// disable the light

		glBindTexture ( GL_TEXTURE_2D, 0 );  // texture bind
		glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, m_image.cols, m_image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imageString.str().c_str() );
		//glTexImage2D (GL_TEXTURE_2D, 0, GL_LUMINANCE, m_imageWidth, m_imageHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_imgData.c_str() );

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering

		float eps = 0.0;
		float z0 = 0.0;

		glBegin(GL_QUADS); //we draw a quad on the entire screen (0,1 - 1,1 - 1,0 - 0,0)
		glColor4f(1.0f,1.0f,1.0f,0.5f);

		float x0 = (float)m_image.cols*f_scaleImg.getValue();
		float y0 = (float)m_image.rows*f_scaleImg.getValue();
		glTexCoord2f(0,1);
/*
		glVertex3f(-x0/2 - eps,-y0/2 - eps, -z0);
		glTexCoord2f(1,1);
		glVertex3f(x0/2 + eps, -y0/2 - eps, -z0);
		glTexCoord2f(1,0);
		glVertex3f(x0/2 + eps, y0/2 + eps, -z0);
		glTexCoord2f(0,0);
		glVertex3f(-x0/2 - eps, y0/2 + eps, -z0);
*/
		glTexCoord2f(0,1);
		glVertex3f(0,y0,0);
		glTexCoord2f(1,1);
		glVertex3f(x0, y0,0);
		glTexCoord2f(1,0);
		glVertex3f(x0, 0,0);
		glTexCoord2f(0,0);
		glVertex3f(0, 0, 0);
		glEnd();

		// glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);		// enable light
		glDisable(GL_TEXTURE_2D);	// disable texture 2D
		//glDepthMask (GL_TRUE);		// enable zBuffer
	}
	else if(f_view.getValue() == 2)
	{
	// ORTHO

		glMatrixMode(GL_PROJECTION);	//init the projection matrix
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0,1,0,1,-1,1);  // orthogonal view		
		glMatrixMode(GL_MODELVIEW);  
		glPushMatrix(); 
		glLoadIdentity(); 

		// BACKGROUND TEXTURING
		//glDepthMask (GL_FALSE);		// disable the writing of zBuffer
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_TEXTURE_2D);	// enable the texture	
		glDisable(GL_LIGHTING);		// disable the light
				
		glBindTexture ( GL_TEXTURE_2D, 0 );  // texture bind	
		glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, m_image.cols, m_image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imageString.str().c_str() ); 	

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering
		 
		// BACKGROUND DRAWING		 
		//glEnable(GL_DEPTH_TEST);		

		glBegin(GL_QUADS); //we draw a quad on the entire screen (0,0 1,0 1,1 0,1)  
			glColor4f(1.0f,1.0f,1.0f,1.0f); 
			glTexCoord2f(0,1);		glVertex2f(0,0);
			glTexCoord2f(1,1);		glVertex2f(1,0);
			glTexCoord2f(1,0);		glVertex2f(1,1);
			glTexCoord2f(0,0);		glVertex2f(0,1);
		glEnd();             

		//glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);		// enable light
		glDisable(GL_TEXTURE_2D);	// disable texture 2D	
		glEnable(GL_DEPTH_TEST);
		//glDepthMask (GL_TRUE);		// enable zBuffer
				
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}
	else if(f_view.getValue() == 0)
	{
		std::cout << "Image hided !" << std::endl;
	}
    }
}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
