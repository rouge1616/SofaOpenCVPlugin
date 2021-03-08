include(${SOFA_CMAKE_DIR}/preProject.cmake)

find_package(OpenCV REQUIRED COMPONENTS core highgui video features2d videoio imgproc flann photo)

set(HEADER_FILES

    initOpticalFlow.h 
    LKOpticalFlowTracker.h 
    LKOpticalFlowTracker.inl  
    LKOpticalFlowTrackerSimple.h 
    LKOpticalFlowTrackerSimple.inl 
    FarnebackOpticalFlowTracker.h 
    FarnebackOpticalFlowTracker.inl 
    InterFrameMatching.h
    InterFrameMatching.inl
    ControlPoint.h
    ProjectStressOnImage.inl
    ProjectStressOnImage.h
    ProjectPositionsOnImage.inl
    ProjectPositionsOnImage.h
    VideoReader.inl
    VideoReader.h
    CalipsoGUI.inl
    CalipsoGUI.h


	)

set(SOURCE_FILES

    initOpticalFlow.cpp 
    LKOpticalFlowTracker.cpp 
    LKOpticalFlowTrackerSimple.cpp 
    FarnebackOpticalFlowTracker.cpp 
    InterFrameMatching.cpp
    ControlPoint.cpp
    ProjectStressOnImage.cpp
    ProjectPositionsOnImage.cpp
    VideoReader.cpp
    CalipsoGUI.cpp
	)


include_directories(${OpenCV_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

AddCompilerDefinitions("SOFA_BUILD_OPTICALFLOW")
AddLinkerDependencies(${OpenCV_LIBRARIES})

include(${SOFA_CMAKE_DIR}/postProject.cmake)

