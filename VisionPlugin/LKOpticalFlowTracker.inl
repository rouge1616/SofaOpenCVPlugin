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
#ifndef SOFA_COMPONENT_ENGINE_LKOpticalFlowTracker_INL
#define SOFA_COMPONENT_ENGINE_LKOpticalFlowTracker_INL

#include <LKOpticalFlowTracker.h>
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
LKOpticalFlowTracker<DataTypes>::LKOpticalFlowTracker()
: f_outputFeatures( initData (&f_outputFeatures, "outputFeatures", "output 2D features") )
, f_outputStatus( initData (&f_outputStatus, "outputStatus", "output vector of status 1 for tracked feature, 0 for lost feature") )
, f_displayFeatures( initData (&f_displayFeatures, int(1), "displayFeatures", "display or not the tracked features") )
, f_winSize( initData (&f_winSize, int(21), "winSize", "Window size around the pixel (21,21), (31,31) ...") )
, f_view( initData (&f_view, int(1), "view", "Perpective = 1 | Orthpgraphic = 2 | Hide = 0") )
, f_scaleImg( initData (&f_scaleImg, float(1.0), "scaleImg", "apply scaling factor to the input image") )
, f_detectorThresh( initData (&f_detectorThresh, float(10), "detectorThresh", "threshold for features extraction") )
, f_vidName( initData (&f_vidName, "vidName", "input video name (string)") )
, f_maskName( initData (&f_maskName, "maskName", "input mask name (string)") )
, f_translation( initData (&f_translation, "translation", "3D translation") )
, f_scale( initData (&f_scale, Vec3(1.0,1.0,1.0), "scale", "3D scale") )
, f_cellSize( initData (&f_cellSize, int(0), "cellSize", "cell size for uniform sampling (0 for non uniform)") )
, f_outputControlPoints( initData (&f_outputControlPoints, "outputControlPoints", "output vector of control points") )
, f_outputCpStatus( initData (&f_outputCpStatus, "outputCpStatus", "output vector of status 1 for tracked cp, 0 for lost cp") )
{

	this->f_listening.setValue(true);

}

template <class DataTypes>
LKOpticalFlowTracker<DataTypes>::~LKOpticalFlowTracker()
{

}


template <class DataTypes>
void LKOpticalFlowTracker<DataTypes>::init()
{
    addOutput(&f_outputFeatures);
    addOutput(&f_outputControlPoints);
    addOutput(&f_outputStatus);
    addOutput(&f_outputCpStatus);
    setDirtyValue();

    m_lastTick = cv::getTickCount();

	m_nbFrame = 0;

    extractFeatures();
    loadVideo();
//    trackFeatures();
}

template <class DataTypes>
void LKOpticalFlowTracker<DataTypes>::reinit()
{
    update();
}

template<class DataTypes>
void LKOpticalFlowTracker<DataTypes>::handleEvent(core::objectmodel::Event *event)
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
void LKOpticalFlowTracker<DataTypes>::loadVideo()
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
cv::Point3f LKOpticalFlowTracker<DataTypes>::shepardIDW(std::vector<cv::Point3f> displSet, std::vector<float> distances, float radius)
{ 
	float totalWeight = 0, x = 0, y = 0, z = 0;

	for (int i = 0 ; i < distances.size() ; i++) 
		if (distances.at(i) > 0)	
			totalWeight = totalWeight + ( (radius - distances.at(i))/(radius*distances.at(i)) * (radius - distances.at(i))/(radius*distances.at(i)) );
	   	

	for (int i = 0 ; i < displSet.size() ; i++) {
		if (distances.at(i) > 0) {
			float weight = ( (radius - distances.at(i))/(radius*distances.at(i)) * (radius - distances.at(i))/(radius*distances.at(i)) ) / totalWeight; 
			x +=  displSet.at(i).x * weight;
			y +=  displSet.at(i).y * weight;
			z +=  displSet.at(i).z * weight;
		}
	}
	
	//std::cout << "totalWeight = "<< totalWeight << std::endl;		
	if (totalWeight == 0 )		
		return cv::Point3f( 0, 0, 0 );
	else 
		return cv::Point3f( x, y, z );
}

template <class DataTypes>
float LKOpticalFlowTracker<DataTypes>::euclideanDistance( cv::Point2f pt1, cv::Point2f pt2 )
{
	return sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
}





template <class DataTypes>
void LKOpticalFlowTracker<DataTypes>::initClustering()
{

    	helper::WriteAccessor<Data<helper::vector<int> > > statCp = f_outputCpStatus;
	helper::WriteAccessor<Data<VecCoord> > outCP = f_outputControlPoints;
	outCP.resize(m_controlPoints.size());

	// a KNN search in Radius defined by Mechanical Model
	int kCount; // the effectif number of neighbors	

	// Create the Index on the Matched point only
	cv::flann::Index flannIndex(cv::Mat(m_points[0]).reshape(1), cv::flann::KDTreeIndexParams(32));

	// for each Control Point we  compute the KNN 
	for( int i = 0; i < m_controlPoints.size(); i++ )
	{	 
		m_controlPoints[i].setCp(m_controlPoints[i].getInitCp());
		int k = m_controlPoints[i].getMaxNbNeighbors(); // the maximum number of neighbors
		float radius = m_controlPoints[i].getRadius();  // the radius of search specific on each cp

		kCount = 0;

		std::vector<float> controlPoint = cv::Mat( m_controlPoints[i].getInitCp() ); // convert a 3D point to float matrix
		std::vector<int> knnIndices; // the indices of nearest neighbors
		std::vector<float> knnDists; // the distances of nearest neighbors, 0 for no neighbor found

		int found = flannIndex.radiusSearch(controlPoint, knnIndices, knnDists, radius*radius, k, cv::flann::SearchParams(32));

		// this step is to resize the number of neighbors found, from k to kCount (if found is not null)		
		if (found)
		{ 
			for (int j = 0; j < k; j++)
			   if (knnDists[j]) {
				m_controlPoints[i].addNIndice( knnIndices[j] ); // the indices in the point cloud of each neighbor
				m_controlPoints[i].addNDist( knnDists[j] ); // the distance of each neighbor
				m_controlPoints[i].addNQuality(0); // initialize quality with detector response

				kCount++;
				}
			m_controlPoints[i].setNbNeighbors( kCount );
			statCp.push_back(1);
		}
		else
		{ 
			m_controlPoints[i].setNbNeighbors(-1);
			statCp.push_back(0);
		}

		// control points will be considered as tracked points  
		//m_points[0][i] = cv::Point2f(m_controlPoints[i].getInitCp().x, m_controlPoints[i].getInitCp().y);

		outCP[i] = Coord( m_controlPoints[i].getCp().x, m_controlPoints[i].getCp().y, 0);
	}

}

template <class DataTypes>
void LKOpticalFlowTracker<DataTypes>::updateClustering()
{

	
    	helper::WriteAccessor<Data<helper::vector<int> > > statCp = f_outputCpStatus;
	int cellSize = f_cellSize.getValue();
	float threshold = cellSize*0.8;
	int nbCp = 0;
	for(int i = 0; i < m_controlPoints.size(); i++ )
	{
		if (m_controlPoints[i].getNbNeighbors() > 0)
			{
	    		std::vector<cv::Point3f> cpDisplacement;
			std::vector<int> nnInd = m_controlPoints[i].getNNIndices();
			std::vector<float> nnDist = m_controlPoints[i].getNNDists();
			
			int nbLostNeighbors = 0;
			// compute the displacement beetween t and t-1
			for(int j = 0 ; j < m_controlPoints[i].getNbNeighbors() ; j++ )
			{
			    float displx = m_points[1].at(nnInd[j]).x - m_points[0].at(nnInd[j]).x;
			    float disply = m_points[1].at(nnInd[j]).y - m_points[0].at(nnInd[j]).y;

			    float dist = euclideanDistance( cv::Point2f(m_controlPoints[i].getCp().x, m_controlPoints[i].getCp().y), m_points[1].at(nnInd[j]) );

			    // check if the displacemet is correct
			    if ( dist > threshold) 
			    //if (displx > threshold || disply > threshold)
		   	    {				
				//std::cout << "Dist : " << dist << " > " << cellSize << std::endl;
				//std::cout << "Point with indice : " << nnInd[j] << " has an incorrect displacement." << std::endl;
				nbLostNeighbors++; 
				m_controlPoints[i].setNQuality( -1 , j ); // set a quality to -1 to the incorrect feature
				m_controlPoints[i].setNDist( -1 , j ); // set a distance to -1 to the incorrect feature
				//neighborsStatus[i][j] = 0;
				//std::cout << "Control point " << nbCp << " lost !" << std::endl;
				statCp[i] = 0;					
			    }
			    else 
			    {
				// update the distance  			    
				m_controlPoints[i].setNDist( dist , j ); 
			    }
					
			    cpDisplacement.push_back( cv::Point3f(displx, disply, 0) ); 
			}
	
			if (nbLostNeighbors == m_controlPoints[i].getNbNeighbors())
			{ 
			    //statusToSend[nbCp] = 0;
			    //std::cout << "Control point " << nbCp << " lost !" << std::endl;
			}

			// compute a weighted mean on a set of neighbor's displacement based on shepard inverse distance formula
			cv::Point3f cpDisp = shepardIDW( cpDisplacement, m_controlPoints[i].getNNDists(), cellSize );

			// add the weighted displacement to the actual position
			m_controlPoints[i].setCp( cv::Point3f( m_controlPoints[i].getCp().x + cpDisp.x, m_controlPoints[i].getCp().y + cpDisp.y, m_controlPoints[i].getCp().z + cpDisp.z ) );

			nbCp++;
			}
		    }


//	if (m_nbFrame > 160)
//	for (unsigned int i = 200; i < m_controlPoints.size(); i++)
//		statCp[i] = 0;
}

template <class DataTypes>
int LKOpticalFlowTracker<DataTypes>::consistant(cv::Point2f p0, cv::Point2f p1, float h)
{
	int consist = 1;
	if ( ((p0.x - p1.x) > h) || ((p0.y - p1.y) > h) ) consist = 0;

	return consist;
}

template <class DataTypes>
void LKOpticalFlowTracker<DataTypes>::extractFeatures()
{
	// capture
	cv::VideoCapture cap;
	cap.open(f_vidName.getFullPath());

	cv::Mat firstFrame;
	cap >> firstFrame;
	//cv::flip(firstFrame,firstFrame,1);
	
	int cellSize = f_cellSize.getValue();

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

	cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
	descriptor->compute( firstFrame, m_keypoints, m_descriptors);


	cv::Mat objectRoi;
	cv::Point2f origine;
	std::vector<std::vector<cv::Point> > contours; // 2D container for contours

	if (f_maskName.getFullPath().length())
	{
		cv::Mat mask = cv::imread(f_maskName.getFullPath().c_str(), CV_8UC1);


		//find countours points
		cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
		cv::Mat contoursImg(mask.size(), CV_8U, cv::Scalar(255));
		cv::drawContours(contoursImg, contours, -1, cv::Scalar(0), 2);

		// rotated bounding box
		cv::RotatedRect rot_bbox = cv::minAreaRect(contours[0]);
		cv::Point2f vertices[4];
		rot_bbox.points(vertices);
		for (int i = 0; i < 4; ++i)
			cv::line(contoursImg, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 1, CV_AA);

		// bounding box
		cv::Rect bbox = cv::boundingRect(contours[0]);
		cv::rectangle(contoursImg, bbox, cv::Scalar(0, 0, 255) );

		objectRoi = firstFrame(bbox); // final ROI
		origine = cv::Point2f(bbox.x, bbox.y);
		//cv::imshow("contoursImg", contoursImg);
	}
	else
	{	
		objectRoi = firstFrame; // no ROI
		origine = cv::Point2f(0,0);
	}

	for(int j = 0; j < objectRoi.rows/cellSize + 1; ++j) {
	    for(int i = 0; i < objectRoi.cols/cellSize + 1; ++i) {    
				
		cv::line(objectRoi, cv::Point2f(i*cellSize, 0), cv::Point2f(i*cellSize, objectRoi.rows), cv::Scalar(0, 255, 0), 1, CV_AA);
		cv::line(objectRoi, cv::Point2f(0, j*cellSize), cv::Point2f(objectRoi.cols, j*cellSize), cv::Scalar(0, 255, 0), 1, CV_AA);
		cv::circle(objectRoi, cv::Point2f(i*cellSize + cellSize/2, j*cellSize + cellSize/2), 2, cv::Scalar(255,0,0), 1.5, 8, 0);

		cv::Point2f uniFeat2D = cv::Point2f( (i*cellSize + cellSize/2) + origine.x, (j*cellSize + cellSize/2) + origine.y );
		cv::Point3f uniFeat3D = cv::Point3f( (i*cellSize + cellSize/2) + origine.x, (j*cellSize + cellSize/2) + origine.y, 0 );

		if ( cv::pointPolygonTest( contours[0], uniFeat2D, false ) > 0)
			//m_points[0].push_back( uniFeat2D );
			m_controlPoints.push_back( ControlPoint(uniFeat3D, 2, cellSize/2) );
		}
	}

	initClustering();

	// drawing
	for(int i = 0; i < m_controlPoints.size(); i++ ) {

		std::vector<int> nnInd = m_controlPoints[i].getNNIndices();
		std::vector<float> nnDist = m_controlPoints[i].getNNDists();

		// the control point
		cv::circle(firstFrame, cv::Point2f(m_controlPoints[i].getInitCp().x, m_controlPoints[i].getInitCp().y), 2, cv::Scalar(255,0,0), 1.5, 8, 0);
		// the radius
		cv::circle(firstFrame, cv::Point2f(m_controlPoints[i].getInitCp().x, m_controlPoints[i].getInitCp().y), cellSize, cv::Scalar(255,0,0), 1.5, 8, 0);
		// the connections
		for(int j = 0 ; j < m_controlPoints[i].getNbNeighbors() ; j++ )
			cv::line(firstFrame, 
			cv::Point2f(m_points[0].at(nnInd[j]).x, m_points[0].at(nnInd[j]).y), 
			cv::Point2f(m_controlPoints[i].getInitCp().x, m_controlPoints[i].getInitCp().y), 
			cv::Scalar(0, 255, 0), 1, CV_AA
			);

	}


	//cv::imshow("objectRoi", objectRoi);
	cv::imshow("firstFrame", firstFrame);
	cap.release();
}


template <class DataTypes>
void LKOpticalFlowTracker<DataTypes>::update()
{
	//cleanDirty();

	if (m_points[0].size() > 0)
	{
		helper::WriteAccessor<Data<VecCoord> > out = f_outputFeatures;
		out.resize(m_points[0].size());
		helper::WriteAccessor<Data<VecCoord> > outCP = f_outputControlPoints;
		outCP.resize(m_controlPoints.size());
		helper::WriteAccessor<Data<helper::vector<int> > > stat = f_outputStatus;
		stat.resize(m_points[0].size());
		Vec3 trans = f_translation.getValue();
		Vec3 scale = f_scale.getValue();

		cv::Mat gray ;
		std::vector<uchar> status;

		// capture
		m_cap >> m_image;
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
				out[i] = Coord( m_points[1][i].x*scale[0] + trans[0] , m_points[1][i].y*scale[1] + trans[1], 1*scale[2] + trans[2] ); // (x, y , 1)
				status[i] = 1;
				stat[i] = (int)status[i];								
			}

			updateClustering();

			for (unsigned int i = 0; i < m_controlPoints.size(); i++) {
				outCP[i] = Coord( m_controlPoints[i].getCp().x*scale[0] + trans[0] , m_controlPoints[i].getCp().y*scale[1] + trans[1], 1*scale[2] + trans[2] );
			}



			//-- swap buffers
			std::swap(m_points[1], m_points[0]);
			std::swap(m_prevGray, gray);

		}
	}


	std::cout << m_nbFrame << std::endl;


	setDirtyValue();
}













template <class DataTypes>
void LKOpticalFlowTracker<DataTypes>::draw(const core::visual::VisualParams* /*vparams*/)
{
//std::cout << m_nbFrame << std::endl;

    if (f_displayFeatures.getValue())	{
    helper::ReadAccessor<Data<helper::vector<int> > >stat = f_outputStatus;
    	for (int i = 0 ; i < m_points[1].size() ; i++) {
	//	if (stat[i])	
	//		cv::circle(m_image, m_points[1][i], 3, cv::Scalar(0,255,0), 2, 8, 0);
    	//	else
	//		cv::circle(m_image, m_points[1][i], 3, cv::Scalar(255,30,125), 2, 8, 0);			
	}


    helper::ReadAccessor<Data<helper::vector<int> > > statCp = f_outputCpStatus;

    int cellSize = f_cellSize.getValue();

    for (unsigned int i = 0; i < m_controlPoints.size(); i++) {
	std::vector<int> nnInd = m_controlPoints[i].getNNIndices();
	std::vector<float> nnDist = m_controlPoints[i].getNNDists();

	// the control point
	cv::circle(m_image, cv::Point2f(m_controlPoints[i].getCp().x, m_controlPoints[i].getCp().y), 2, cv::Scalar(255,0,0), 1.5, 8, 0);
	// the radius
	if (statCp[i])
		cv::circle(m_image, cv::Point2f(m_controlPoints[i].getCp().x, m_controlPoints[i].getCp().y), cellSize, cv::Scalar(255,0,0), 1.5, 8, 0);
	else 
		cv::circle(m_image, cv::Point2f(m_controlPoints[i].getCp().x, m_controlPoints[i].getCp().y), cellSize, cv::Scalar(255,255,255), 1.5, 8, 0);
	// the connections
	if (statCp[i])
	for(int j = 0 ; j < m_controlPoints[i].getNbNeighbors() ; j++ )
		cv::line(m_image, 
		cv::Point2f(m_points[0].at(nnInd[j]).x, m_points[0].at(nnInd[j]).y), 
		cv::Point2f(m_controlPoints[i].getCp().x, m_controlPoints[i].getCp().y), 
		cv::Scalar(0, 255, 0), 0.7, CV_AA);
	
    	}

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
		//std::cout << "Image hided !" << std::endl;
	}
    }
}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
