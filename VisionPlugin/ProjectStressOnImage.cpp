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
#define SOFA_COMPONENT_ENGINE_ProjectStressOnImage_CPP
#include <ProjectStressOnImage.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace engine
{

SOFA_DECL_CLASS(ProjectStressOnImage)

int ProjectStressOnImageClass = core::RegisterObject("Project Stress On Image")
#ifndef SOFA_FLOAT
.add< ProjectStressOnImage<defaulttype::Vec3dTypes> >(true) // default template
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
.add< ProjectStressOnImage<defaulttype::Vec3fTypes> >()
#endif //SOFA_DOUBLE
;

#ifndef SOFA_FLOAT
template class SOFA_OPTICALFLOWPLUGIN_API ProjectStressOnImage<defaulttype::Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
template class SOFA_OPTICALFLOWPLUGIN_API ProjectStressOnImage<defaulttype::Vec3fTypes>;
#endif //SOFA_DOUBLE

} // namespace constraint

} // namespace component

} // namespace sofa
