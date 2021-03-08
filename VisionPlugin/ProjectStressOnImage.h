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
#ifndef SOFA_COMPONENT_ENGINE_ProjectStressOnImage_H
#define SOFA_COMPONENT_ENGINE_ProjectStressOnImage_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <iostream>
#include <fstream>

#include "initOpticalFlow.h"
#include "ControlPoint.h"

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
#include <SofaBaseVisual/BaseCamera.h>

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
template <class DataTypes>
class SOFA_OPTICALFLOWPLUGIN_API ProjectStressOnImage : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ProjectStressOnImage,DataTypes),core::DataEngine);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename sofa::defaulttype::Vector3 Vec3;
    typedef sofa::defaulttype::Mat<3,3,double> Matrix3;

public:

    ProjectStressOnImage();

    ~ProjectStressOnImage();

    void init();

    void reinit();

    void update();

    void handleEvent(core::objectmodel::Event *event);

    void draw(const core::visual::VisualParams *vparams);

    void loadVideo();

    double gaussianNoise();


    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const ProjectStressOnImage<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


protected:
    Data<VecCoord> f_vmCenters; ///< input elements' centers
    Data<helper::vector<defaulttype::Vec3f> > f_vmColors; ///< input elements' colors
    Data<helper::vector<Real> > f_vmStress; ///< input elements' stresses
    Data<helper::vector<int> > f_indices; ///< input indices to consider
    Data<std::string > f_imageData; ///< input indices to consider
    sofa::core::objectmodel::DataFileName f_vidName; ///< input video name
    sofa::core::objectmodel::DataFileName f_maskName; ///< input mask name

    cv::VideoCapture m_cap;
    cv::VideoWriter m_capOut;

    float m_fps;
    int m_width;
    int m_height;
    int m_type;
    cv::Mat m_image;
    int m_nbFrame;
    long m_lastTick;
    std::vector<int> m_indicesOnMask;
    bool m_firstFrameFlag;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_ProjectStressOnImage_CPP)

#ifndef SOFA_FLOAT
extern template class SOFA_ENGINE_API ProjectStressOnImage<defaulttype::Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_ENGINE_API ProjectStressOnImage<defaulttype::Vec3fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
