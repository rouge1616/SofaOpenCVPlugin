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
#ifndef SOFA_COMPONENT_ENGINE_VideoReader_H
#define SOFA_COMPONENT_ENGINE_VideoReader_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <iostream>
#include <fstream>

#include "initOpticalFlow.h"
//#include "ControlPoint.h"

// sofa
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/accessor.h>
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>
#include <sofa/helper/gl/RAII.h>
#include <sofa/core/objectmodel/DataFileName.h>

// opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/features2d.hpp"

namespace sofa
{

namespace component
{

namespace engine
{

/** 
 * This class tracks features on video using lucas-kanade optical flow technique
 */
class SOFA_OPTICALFLOWPLUGIN_API VideoReader : public core::DataEngine
{
public:
    SOFA_CLASS(VideoReader,core::DataEngine);

    typedef typename sofa::defaulttype::Vector3 Vec3;
    typedef sofa::defaulttype::Mat<3,3,double> Matrix3;

public:

    VideoReader();

    ~VideoReader();

    void init();

    void reinit();

    void update();

    void handleEvent(core::objectmodel::Event *event);

    void draw(const core::visual::VisualParams *vparams);

    void loadVideo();

protected:
    Data<int> f_view; ///< perspective / orthographic view
    Data<float> f_scaleImg; ///< scale the image 
    sofa::core::objectmodel::DataFileName f_vidName; ///< input video name
    sofa::core::objectmodel::DataFileName f_maskName; ///< input mask name
    Data<helper::vector<unsigned char> > f_imageData;  
    Data<short> f_imageWidth;
    Data<short> f_imageHeight;
    Data<short> f_imagePixelSize;
    Data<std::string > f_imageDataSTR;

    cv::VideoCapture m_cap;
    float m_fps;
    int m_width;
    int m_height;
    cv::Mat m_image;
    int m_nbFrame;

    long m_lastTick;
};

} // namespace engine

} // namespace component

} // namespace sofa

#endif
