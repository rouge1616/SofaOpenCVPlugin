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
#ifndef SOFA_COMPONENT_ENGINE_LKOpticalFlowTrackerSimple_INL
#define SOFA_COMPONENT_ENGINE_LKOpticalFlowTrackerSimple_INL

#include <LKOpticalFlowTrackerSimple.h>
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
LKOpticalFlowTrackerSimple<DataTypes>::LKOpticalFlowTrackerSimple()
: f_outputFeatures( initData (&f_outputFeatures, "outputFeatures", "output 2D features") )
, f_outputStatus( initData (&f_outputStatus, "outputStatus", "output vector of status 1 for tracked feature, 0 for lost feature") )
, f_displayFeatures( initData (&f_displayFeatures, int(1), "displayFeatures", "display or not the tracked features") )
, f_winSize( initData (&f_winSize, int(21), "winSize", "Window size around the pixel (21,21), (31,31) ...") )
, f_view( initData (&f_view, int(1), "view", "Perpective = 1 | Orthpgraphic = 2 | Hide = 0") )
, f_scaleImg( initData (&f_scaleImg, float(1.0), "scaleImg", "apply scaling factor to the input image") )
, f_detectorThresh( initData (&f_detectorThresh, float(10), "detectorThresh", "threshold for features extraction") )
, f_vidName( initData (&f_vidName, "vidName", "input video name (string)") )
, f_maskName( initData (&f_maskName, "maskName", "input mask name (string)") )
, f_imageData( initData (&f_imageData, "imageData", "output image data") )
{
	f_imageData.setDisplayed(false);
	this->f_listening.setValue(true);

}

template <class DataTypes>
LKOpticalFlowTrackerSimple<DataTypes>::~LKOpticalFlowTrackerSimple()
{
}


template <class DataTypes>
void LKOpticalFlowTrackerSimple<DataTypes>::init()
{
    addOutput(&f_outputFeatures);
    addOutput(&f_outputStatus);
    setDirtyValue();

    m_lastTick = cv::getTickCount();

    m_nbFrame = 0;

    extractFeatures();
    loadVideo();
//    trackFeatures();
}

template <class DataTypes>
void LKOpticalFlowTrackerSimple<DataTypes>::reinit()
{
    update();
}

template<class DataTypes>
void LKOpticalFlowTrackerSimple<DataTypes>::handleEvent(core::objectmodel::Event *event)
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
void LKOpticalFlowTrackerSimple<DataTypes>::loadVideo()
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
void LKOpticalFlowTrackerSimple<DataTypes>::extractFeatures()
{
	// capture
	cv::VideoCapture cap;
	cap.open(f_vidName.getFullPath());

	cv::Mat firstFrame;
	cap >> firstFrame;
	//cv::flip(firstFrame,firstFrame,1);
	
	cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(f_detectorThresh.getValue(), true);
	//cv::FastFeatureDetector fastDetector( f_detectorThresh.getValue(), true);

	if (f_maskName.getFullPath().length())
	{
		cv::Mat mask = cv::imread(f_maskName.getFullPath().c_str(), CV_8UC1);
		//cv::flip(mask,mask,1);
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

	//cv::Mat firstFrameGray;
	//cv::cvtColor(firstFrame, firstFrameGray, CV_BGR2GRAY);
	//computeUncertainty(firstFrameGray);

	cap.release();
}

template <class DataTypes>
void LKOpticalFlowTrackerSimple<DataTypes>::computeUncertainty(cv::Mat img)
{

	cv::Mat H( 2, 2, CV_32FC1 );
	cv::Mat cov( 2, 2, CV_32FC1 );
	cv::Mat evals( 2, 1, CV_32FC1 );
	cv::Mat evecs( 2, 2, CV_32FC1 );

	//float ev1, ev2;

	cv::Mat dXX, dYY, dXY;
	cv::Sobel(img, dXX, CV_32F, 2, 0, 3);
	cv::Sobel(img, dYY, CV_32F, 0, 2, 3);
	cv::Sobel(img, dXY, CV_32F, 1, 1, 3);

	int gauKsize = 11;
	cv::Mat gau = cv::getGaussianKernel(gauKsize, -1, CV_32F);
	cv::sepFilter2D(dXX, dXX, CV_32F, gau.t(), gau);
	cv::sepFilter2D(dYY, dYY, CV_32F, gau.t(), gau);
	cv::sepFilter2D(dXY, dXY, CV_32F, gau.t(), gau);

	cv::imshow("S1", dXX);
	cv::imshow("S2", dXY);
	cv::imshow("S3", dXX);

	for(int i = 0; i < m_points[0].size(); i++ ) {
		int x = m_points[0][i].x;
		int y = m_points[0][i].y;

		// determine hessian at that point and calculate and the covariance matrix, COV = H^-1
		H.at<float>(0,0) = -dXX.at<float>(y, x);
		H.at<float>(1,1) = -dYY.at<float>(y, x);
		H.at<float>(0,1) = -dXY.at<float>(y, x);
		H.at<float>(1,0) = -dXY.at<float>(y, x);

		cov = H.inv(CV_SVD_SYM);

		//std::cout << " Hessian of "<< i << "(" << x << " , " << y << ") = "<< H << std::endl;
		//std::cout << " Cov of "<< i << "(" << x << " , " << y << ") = "<< cov << std::endl;

		cv::Mat eigenvalues, eigenvectors;
		cv::eigen(cov, eigenvalues, eigenvectors);

		//std::cout << eigenvalues << std::endl;
		//std::cout << eigenvectors << std::endl;

		// Calculate the size of the minor and major axes given by eigenvalues
		cv::Size2f axes( fabs(eigenvalues.at<float>(0)*(100)), fabs(eigenvalues.at<float>(1)*(100)) ); // !!!! CHECK FABS
		
		// Calculate the angle between the largest eigenvector and the x-axis
		double angle = atan2(eigenvectors.at<float>(0,1), eigenvectors.at<float>(0,0));

		// Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
		if(angle < 0)
			angle += 6.28318530718;
		angle = 180*angle/3.14159265359; // from radians to degrees

		//std::cout << "axes = " << axes << " || angle = " << angle << std::endl;

		//float vx = eigenvectors.at<double>(0,0);
		//float vy = eigenvectors.at<double>(0,1);
		//angle = sin( vy / sqrt(vx*vx + vy*vy) ) * 180/3.14159265359;
		//std::cout << vx << " | " << vy << " "<< "angle = " << angle << std::endl;

		//The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
		cv::ellipse(img, cv::Point2f(x,y), axes, -angle, 0, 360, cv::Scalar(255,255,0), 1, 8,0);
		cv::imshow("ellipse", img);
	}

}


template <class DataTypes>
void LKOpticalFlowTrackerSimple<DataTypes>::update()
{
	//cleanDirty();

	if (m_points[0].size() > 0)
	{
		helper::WriteAccessor<Data<VecCoord> > out = f_outputFeatures;
		out.resize(m_points[0].size());
		helper::WriteAccessor<Data<helper::vector<int> > > stat = f_outputStatus;
		stat.resize(m_points[0].size());

		cv::Mat gray ;
		std::vector<uchar> status;

		// capture
		m_cap >> m_image;
		//cv::flip(m_image,m_image,1);
		if(!m_image.empty())
		{
			m_image.copyTo(gray);

			if(m_prevGray.empty())
				gray.copyTo(m_prevGray);

			//cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
			cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
			cv::Size winSize(f_winSize.getValue(), f_winSize.getValue());
			std::vector<float> err;	

			cv::calcOpticalFlowPyrLK(m_prevGray, gray, m_points[0], m_points[1], status, err, winSize, 3, termcrit, 0, 0.001);

			// from opencv to sofa coord
			for (unsigned int i = 0; i < m_points[1].size(); i++) {
				out[i] = Coord( m_points[1][i].x , m_points[1][i].y, 1 ); // (x, y , 1)
				stat[i] = (int)status[i];
			}	

			//-- swap buffers
			std::swap(m_points[1], m_points[0]);
			std::swap(m_prevGray, gray);

			std::stringstream imageString;
			imageString.write((const char*)m_image.data, m_image.total()*m_image.elemSize());

			f_imageData = imageString.str();


		}
//		else 			std::cout << "empty" << std::endl;

	}



	setDirtyValue();
}



template <class DataTypes>
void LKOpticalFlowTrackerSimple<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/)
{

    if (f_displayFeatures.getValue())
    {
    	helper::ReadAccessor<Data<helper::vector<int> > >stat = f_outputStatus;
    	for (int i = 0 ; i < m_points[1].size() ; i++) {
		if (stat[i])	
			cv::circle(m_image, m_points[1][i], 3, cv::Scalar(0,255,0), 2, 8, 0);
    		else
			cv::circle(m_image, m_points[1][i], 3, cv::Scalar(255,30,125), 2, 8, 0);			
	}
    }

	
   GLfloat projectionMatrixData[16]; 
   glGetFloatv (GL_PROJECTION_MATRIX, projectionMatrixData);
   GLfloat modelviewMatrixData[16]; 
   glGetFloatv (GL_MODELVIEW_MATRIX, modelviewMatrixData);

    std::stringstream imageString;
    imageString.write((const char*)m_image.data, m_image.total()*m_image.elemSize());
    //f_imageData = imageString.str();

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

//		float eps = 0.0;
//		float z0 = 0.0;

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
		std::cout << "Image hidden !" << std::endl;
	}




    }
}



} // namespace engine

} // namespace component

} // namespace sofa

#endif
