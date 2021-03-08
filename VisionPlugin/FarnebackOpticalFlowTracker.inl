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
#ifndef SOFA_COMPONENT_ENGINE_FarnebackOpticalFlowTracker_INL
#define SOFA_COMPONENT_ENGINE_FarnebackOpticalFlowTracker_INL

#include <FarnebackOpticalFlowTracker.h>
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
FarnebackOpticalFlowTracker<DataTypes>::FarnebackOpticalFlowTracker()
: f_outputFeatures( initData (&f_outputFeatures, "outputFeatures", "output 2D features") )
, f_displayFlow( initData (&f_displayFlow, int(1), "displayFlow", "display or not the tracking flow") )
, f_winSize( initData (&f_winSize, int(21), "winSize", "Window size around the pixel (15,15), (20,20) ...") )
, f_view( initData (&f_view, int(1), "view", "Perpective = 1 | Orthpgraphic = 2 | Hide = 0") )
, f_scaleImg( initData (&f_scaleImg, float(1.0), "scaleImg", "apply scaling factor to the input image") )
, f_vidName( initData (&f_vidName, "vidName", "input video name (string)") )
{

	this->f_listening.setValue(true);

}

template <class DataTypes>
FarnebackOpticalFlowTracker<DataTypes>::~FarnebackOpticalFlowTracker()
{

}


template <class DataTypes>
void FarnebackOpticalFlowTracker<DataTypes>::init()
{
    addOutput(&f_outputFeatures);
    setDirtyValue();

    m_lastTick = cv::getTickCount();

    loadVideo();
    initPrevFrame();
}

template <class DataTypes>
void FarnebackOpticalFlowTracker<DataTypes>::loadVideo()
{
	m_cap.open(f_vidName.getValue());
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
void FarnebackOpticalFlowTracker<DataTypes>::initPrevFrame()
{
	cv::Mat firstFrame;
	// capture
	m_cap >> firstFrame;
	//cv::flip(firstFrame,firstFrame,1);

	if(!firstFrame.data) //get one frame form video  
		std::cout << "\nERROR in first frame : " << std::endl;

	cv::resize(firstFrame, m_prevGray, cv::Size(firstFrame.size().width*f_scaleImg.getValue(), firstFrame.size().height*f_scaleImg.getValue()) );  
	cv::cvtColor(m_prevGray, m_prevGray, cv::COLOR_BGR2GRAY);  
}


template <class DataTypes>
void FarnebackOpticalFlowTracker<DataTypes>::reinit()
{
    update();
}

template<class DataTypes>
void FarnebackOpticalFlowTracker<DataTypes>::handleEvent(core::objectmodel::Event *event)
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
void FarnebackOpticalFlowTracker<DataTypes>::update()
{
	//cleanDirty();

	// capture
	m_cap >> m_image;
	//cv::flip(firstFrame,firstFrame,1);

	if(!m_image.data) //get one frame form video  
		std::cout << "\nERROR in reading frames : " << std::endl;

	cv::Mat gray;
	cv::resize(m_image, gray, cv::Size(m_image.size().width*f_scaleImg.getValue(), m_image.size().height*f_scaleImg.getValue()) );  
	cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);  

	float pyr_scale  = 0.5;
	int levels = 3;
	int winsize = f_winSize.getValue();
	int iterations = 3;
	int poly_n = 5; // or 7
	float poly_sigma = 1.2;

	cv::calcOpticalFlowFarneback(m_prevGray, gray, m_flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, 0);  

	helper::WriteAccessor<Data<VecCoord> > out = f_outputFeatures;
	out.resize(m_flow.cols*m_flow.rows);
	int k = 0;
	for(int y = 0; y < m_flow.rows; y++)  
		for(int x = 0; x < m_flow.cols; x++)  
		{  
			cv::Point2f fxy = m_flow.at< cv::Point2f>(y, x);  
			out[k] = Coord(fxy.x, fxy.y, 0);
			k++;
		}  



//	cv::imshow("Previous frame", m_prevGray);  
//	cv::imshow("Current frame", gray);  
	  
	m_prevGray = gray.clone();  

	setDirtyValue();
}


template <class DataTypes>
void FarnebackOpticalFlowTracker<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/)
{
    std::stringstream imageString;
    imageString.write((const char*)m_image.data, m_image.total()*m_image.elemSize());

    if(m_image.data)
    {
	// display flow
	if (f_displayFlow.getValue())
	{
		int step = 10;
		for(int y = 0; y < m_flow.rows; y += step)  
			for(int x = 0; x < m_flow.cols; x += step)  
			{  
				cv::Point2f fxy = m_flow.at< cv::Point2f>(y, x);  
				cv::line(m_image, cv::Point(x,y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), cv::Scalar(0,255,0));  
				cv::circle(m_image, cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), 1, cv::Scalar(0,255,0), -1);  
			}  
	}


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
		glVertex3f(-x0/2 - eps,-y0/2 - eps, -z0);
		glTexCoord2f(1,1);
		glVertex3f(x0/2 + eps, -y0/2 - eps, -z0);
		glTexCoord2f(1,0);
		glVertex3f(x0/2 + eps, y0/2 + eps, -z0);
		glTexCoord2f(0,0);
		glVertex3f(-x0/2 - eps, y0/2 + eps, -z0);
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
