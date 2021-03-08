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
#ifndef SOFA_COMPONENT_ENGINE_InterFrameMatching_H
#define SOFA_COMPONENT_ENGINE_InterFrameMatching_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <iostream>
#include <fstream>

#include "initOpticalFlow.h"

// sofa
#include <sofa/component/component.h>
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
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/features2d/features2d.hpp>

namespace sofa
{

namespace component
{

namespace engine
{

/** 
 * This class tracks features on video using lucas-kanade optical flow technique
 */
template <class DataTypes>
class SOFA_OPTICALFLOWPLUGIN_API InterFrameMatching : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(InterFrameMatching,DataTypes),core::DataEngine);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename sofa::defaulttype::Vector3 Vec3;
    typedef sofa::defaulttype::Mat<3,3,double> Matrix3;

public:

    InterFrameMatching();

    ~InterFrameMatching();

    void init();

    void reinit();

    void update();

    void handleEvent(core::objectmodel::Event *event);

    void draw(const core::visual::VisualParams *vparams);

    void loadVideo();
    void extractFeatures();

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const InterFrameMatching<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


protected:
    Data<VecCoord> f_outputFeatures; ///< ouput features (in vec3d)
    Data<helper::vector<int> > f_outputStatus; ///< output feature status 
    Data<int> f_displayFeatures; ///< perspective / orthographic view
    Data<int> f_view; ///< perspective / orthographic view
    Data<float> f_scaleImg; ///< scale the image 
    Data<float> f_detectorThresh; ///< threshold for the detector
    sofa::core::objectmodel::DataFileName f_vidName; ///< input video name
    sofa::core::objectmodel::DataFileName f_maskName; ///< input mask name
    Data<Vec3> f_translation; ///< translation
    Data<Vec3> f_scale; ///< scale


    std::vector<cv::KeyPoint> m_keypoints;
    std::vector<cv::Point2f> m_points[2];
    cv::VideoCapture m_cap;
    float m_fps;
    int m_width;
    int m_height;
    cv::Mat m_image;
    cv::Mat m_descriptors;
    std::vector<std::vector<cv::DMatch > > m_matches;

    long m_lastTick;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_InterFrameMatching_CPP)

#ifndef SOFA_FLOAT
extern template class SOFA_ENGINE_API InterFrameMatching<defaulttype::Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_ENGINE_API InterFrameMatching<defaulttype::Vec3fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
