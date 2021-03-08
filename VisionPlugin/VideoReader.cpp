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
#define SOFA_COMPONENT_ENGINE_VideoReader_CPP

#include <VideoReader.inl>
#include <sofa/core/ObjectFactory.h>

#include <VideoReader.h>
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

SOFA_DECL_CLASS(VideoReader)

int VideoReaderClass = core::RegisterObject("Track features automatically extracted from a reference image in a video")
.add< VideoReader >()
;

VideoReader::VideoReader()
    : f_view(initData(&f_view, int(1), "view", "Perpective = 1 | Orthpgraphic = 2 | Hide = 0"))
    , f_scaleImg(initData(&f_scaleImg, float(1.0), "scaleImg", "apply scaling factor to the input image"))
    , f_vidName(initData(&f_vidName, "vidName", "input video name (string)"))
    , f_maskName(initData(&f_maskName, "maskName", "input mask name (string)"))
    , f_imageData(initData(&f_imageData, "imageData", "output image data"))
    , f_imageWidth(initData(&f_imageWidth, "imageWidth", "output image width"))
    , f_imageHeight(initData(&f_imageHeight, "imageHeight", "output image height"))
    , f_imagePixelSize(initData(&f_imagePixelSize, "imagePixelSize", "output image pixel size (in bit, not byte)"))
    , f_imageDataSTR( initData (&f_imageDataSTR, "imageDataSTR", "output image data") )
{
    f_imageData.setDisplayed(false);
    this->f_listening.setValue(true);

    f_imageWidth.setReadOnly(true);
    f_imageHeight.setReadOnly(true);
    f_imagePixelSize.setReadOnly(true);

}

VideoReader::~VideoReader()
{
}


void VideoReader::init()
{
    setDirtyValue();

    m_lastTick = cv::getTickCount();

    m_nbFrame = 0;

    loadVideo();
    update();
}

void VideoReader::reinit()
{
    update();
}

void VideoReader::handleEvent(core::objectmodel::Event *event)
{
    // to force update at each time step (no input in engine) 
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
        // compute duration between two "sofa" timestep (dt) to synchronize with "opencv" video stream (fps)
        double duration = ((double)(cv::getTickCount() - m_lastTick)) / cv::getTickFrequency();

        // call update
        if (m_fps > 0 && duration > 1 / m_fps)
        {
            setDirtyValue();
            update();
            m_nbFrame++;
            //std::cout << m_nbFrame << std::endl;

            m_lastTick = cv::getTickCount();
        }
    }

}


void VideoReader::loadVideo()
{
    m_cap.open(f_vidName.getFullPath());
    if (!m_cap.isOpened()) {
        std::cout << "\nERROR in opening video file : " << f_vidName.getValue() << std::endl;
    }
    else std::cout << "Loading video File : " << f_vidName.getValue() << std::endl;

    m_fps = m_cap.get(cv::CAP_PROP_FPS);
    m_width = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    m_height = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    //m_format = m_cap.get(cv::CV_CAP_PROP_FORMAT); //TODO: test all possible format to get number of bytes

    f_imageWidth.setValue(m_width);
    f_imageHeight.setValue(m_height);
    f_imagePixelSize.setValue(8 * 3);//same

    std::cout << "fps = " << m_fps << std::endl;
    std::cout << "W x H = " << m_width << "x" << m_height << std::endl;
}

void VideoReader::update()
{
    // capture
    m_cap >> m_image;
    //cv::flip(m_image,m_image,1);
    //	if(!m_image.empty())
//    cv::Mat m_image2;
//    cv::resize(m_image, m_image2, cv::Size(1024, 1024), 0, 0, CV_INTER_LINEAR);

    int nbpp = m_image.elemSize();
    f_imagePixelSize.setValue(nbpp*8);
    helper::vector<unsigned char> &imageData = *f_imageData.beginEdit();
    imageData.resize(m_image.total()*m_image.elemSize());
    for (unsigned int i = 0; i < imageData.size(); i++)
        imageData[i] = m_image.data[i];
    
    if(!m_image.empty()) {
	std::stringstream imageString;
	imageString.write((const char*)m_image.data, m_image.total()*m_image.elemSize());
	f_imageDataSTR = imageString.str();
    }

    f_imageData.endEdit();
    setDirtyValue();
}

void VideoReader::draw(const core::visual::VisualParams* /*vparams*/)
{

    GLfloat projectionMatrixData[16];
    glGetFloatv(GL_PROJECTION_MATRIX, projectionMatrixData);
    GLfloat modelviewMatrixData[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, modelviewMatrixData);

    std::stringstream imageString;
    imageString.write((const char*)m_image.data, m_image.total()*m_image.elemSize());
    //f_imageData = imageString.str();

    if (m_image.data)
    {
        if (f_view.getValue() == 1)
        {
            // PERSPECTIVE

            glEnable(GL_TEXTURE_2D);	// enable the texture
            glDisable(GL_LIGHTING);		// disable the light

            glBindTexture(GL_TEXTURE_2D, 0);  // texture bind
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_image.cols, m_image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imageString.str().c_str());
            //glTexImage2D (GL_TEXTURE_2D, 0, GL_LUMINANCE, m_imageWidth, m_imageHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m_imgData.c_str() );

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering

                                                                                //		float eps = 0.0;
                                                                                //		float z0 = 0.0;

            glBegin(GL_QUADS); //we draw a quad on the entire screen (0,1 - 1,1 - 1,0 - 0,0)
            glColor4f(1.0f, 1.0f, 1.0f, 0.5f);

            float x0 = (float)m_image.cols*f_scaleImg.getValue();
            float y0 = (float)m_image.rows*f_scaleImg.getValue();
            glTexCoord2f(0, 1);
            /*
            glVertex3f(-x0/2 - eps,-y0/2 - eps, -z0);
            glTexCoord2f(1,1);
            glVertex3f(x0/2 + eps, -y0/2 - eps, -z0);
            glTexCoord2f(1,0);
            glVertex3f(x0/2 + eps, y0/2 + eps, -z0);
            glTexCoord2f(0,0);
            glVertex3f(-x0/2 - eps, y0/2 + eps, -z0);
            */
            glTexCoord2f(0, 1);
            glVertex3f(0, y0, 0);
            glTexCoord2f(1, 1);
            glVertex3f(x0, y0, 0);
            glTexCoord2f(1, 0);
            glVertex3f(x0, 0, 0);
            glTexCoord2f(0, 0);
            glVertex3f(0, 0, 0);
            glEnd();

            // glEnable(GL_DEPTH_TEST);
            glEnable(GL_LIGHTING);		// enable light
            glDisable(GL_TEXTURE_2D);	// disable texture 2D
                                        //glDepthMask (GL_TRUE);		// enable zBuffer
        }
        else if (f_view.getValue() == 2)
        {
            // ORTHO

            glMatrixMode(GL_PROJECTION);	//init the projection matrix
            glPushMatrix();
            glLoadIdentity();
            glOrtho(0, 1, 0, 1, -1, 1);  // orthogonal view		
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();

            // BACKGROUND TEXTURING
            //glDepthMask (GL_FALSE);		// disable the writing of zBuffer
            glDisable(GL_DEPTH_TEST);
            glEnable(GL_TEXTURE_2D);	// enable the texture	
            glDisable(GL_LIGHTING);		// disable the light

            glBindTexture(GL_TEXTURE_2D, 0);  // texture bind	
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_image.cols, m_image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, imageString.str().c_str());

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// Linear Filtering

                                                                                // BACKGROUND DRAWING		 
                                                                                //glEnable(GL_DEPTH_TEST);		

            glBegin(GL_QUADS); //we draw a quad on the entire screen (0,0 1,0 1,1 0,1)  
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            glTexCoord2f(0, 1);		glVertex2f(0, 0);
            glTexCoord2f(1, 1);		glVertex2f(1, 0);
            glTexCoord2f(1, 0);		glVertex2f(1, 1);
            glTexCoord2f(0, 0);		glVertex2f(0, 1);
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
        else if (f_view.getValue() == 0)
        {
            //std::cout << "Image hidden !" << std::endl;
        }
        
    }
}

} // namespace constraint

} // namespace component

} // namespace sofa
