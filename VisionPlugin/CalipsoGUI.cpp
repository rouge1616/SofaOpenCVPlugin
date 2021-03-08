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
#define SOFA_COMPONENT_ENGINE_CalipsoGUI_CPP

#include <CalipsoGUI.inl>
#include <sofa/core/ObjectFactory.h>

#include <CalipsoGUI.h>
#include <sofa/helper/rmath.h> //M_PI
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/BasicShapes.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>

int gravity = 10;
int mass = 100;
int stifness = 25;
int damping = 0;
int forces = 0;

namespace sofa
{

namespace component
{

namespace engine
{

SOFA_DECL_CLASS(CalipsoGUI)

int CalipsoGUIClass = core::RegisterObject("GUI for CALIPSO")
.add< CalipsoGUI >()
;

CalipsoGUI::CalipsoGUI()
    : f_view(initData(&f_view, int(1), "view", "Perpective = 1 | Orthpgraphic = 2 | Hide = 0"))
{
    this->f_listening.setValue(true);
}

CalipsoGUI::~CalipsoGUI()
{
}


void CalipsoGUI::init()
{
    setDirtyValue();

    update();
}

void CalipsoGUI::reinit()
{
    update();
}

void CalipsoGUI::handleEvent(core::objectmodel::Event *event)
{
    // to force update at each time step (no input in engine) 
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {

    }

}

void on_trackbar( int, void* )
{

}

void CalipsoGUI::update()
{

    setDirtyValue();
}

void CalipsoGUI::draw(const core::visual::VisualParams* /*vparams*/)
{
    	cv::namedWindow( "Physical Properties", cv::WINDOW_OPENGL );
	cv::createTrackbar( "Gravity", "Physical Properties", &gravity, 100, on_trackbar );
	cv::createTrackbar( "Mass", "Physical Properties", &mass, 100, on_trackbar );
	cv::createTrackbar( "Stifness", "Physical Properties", &stifness, 100, on_trackbar );
	cv::createTrackbar( "Damping", "Physical Properties", &damping, 10, on_trackbar );
	cv::createTrackbar( "Forces", "Physical Properties", &forces, 10, on_trackbar );
    	on_trackbar(gravity, 0);
    	on_trackbar(mass, 0);
    	on_trackbar(stifness, 0);
    	on_trackbar(damping, 0);
    	on_trackbar(forces, 0);

}

} // namespace constraint

} // namespace component

} // namespace sofa
