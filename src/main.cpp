#include <OpenNI.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>

// (1) Include Header
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

int main( int argc, char **argv )
{
    // (2) Set Optimized
    cv::setUseOptimized( true );

    // (3) Open Video Capture
    //cv::VideoCapture capture( cv::VideoCaptureAPIs::CAP_INTELPERC );
    //if( !capture.isOpened() ){
    //    return -1;
    //}

    openni::OpenNI::initialize();
    openni::Device device;
    int ret = device.open( openni::ANY_DEVICE );
    if ( ret != openni::STATUS_OK ) {
        return -1;
    }
    device.setDepthColorSyncEnabled(true);
    device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    
    openni::VideoStream colorStream;
    colorStream.create( device, openni::SENSOR_COLOR);
    colorStream.start();

    openni::VideoStream depthStream;
    depthStream.create( device, openni::SENSOR_DEPTH );
    depthStream.start();

    std::vector<openni::VideoStream*> streams;
    streams.push_back( &colorStream );
    streams.push_back( &depthStream );



    // (4) Retrieve Camera Parameters
    const uint32_t width  = static_cast<uint32_t>(depthStream.getVideoMode().getResolutionX());
    const uint32_t height = static_cast<uint32_t>(depthStream.getVideoMode().getResolutionY());
    const float fx = 0.5*width/(0.5*static_cast<float>(depthStream.getHorizontalFieldOfView()));
    const float fy = 0.5*height/(0.5*static_cast<float>(depthStream.getHorizontalFieldOfView()));
    const float cx = width  / 2.0f - 0.5f;
    const float cy = height / 2.0f - 0.5f;
    std::cout << fx <<  " " << fy << " " << std::endl;
    const cv::Matx33f camera_matrix = cv::Matx33f( fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f );


    // (5) Initialize KinFu Parameters
    cv::Ptr<cv::kinfu::Params> params;
    params = cv::kinfu::Params::defaultParams(); // Default Parameters
    //params = cv::kinfu::Params::coarseParams(); // Coarse Parameters

    params->frameSize   = cv::Size( width, height ); // Frame Size
    params->intr        = camera_matrix;             // Camera Intrinsics
    params->depthFactor = 1000.0f;                   // Depth Factor (1000/meter)

    // (6) Create KinFu
    cv::Ptr<cv::kinfu::KinFu> kinfu;
    kinfu = cv::kinfu::KinFu::create( params );

    cv::Mat depthImage, colorImage;
    while( true ){
        // (7) Grab All Frames and Retrieve Depth Frame        
        int changedIndex;
        openni::OpenNI::waitForAnyStream( &streams[1], streams.size(), &changedIndex );
        if (changedIndex == 0) {
            openni::VideoFrameRef depthFrame;
            depthStream.readFrame( &depthFrame );
            if ( depthFrame.isValid() ) {
                depthImage = cv::Mat( depthStream.getVideoMode().getResolutionY(), 
                    depthStream.getVideoMode().getResolutionX(),
                    CV_16UC1, (char*)depthFrame.getData() );
                cv::UMat frame;
                depthImage.copyTo(frame);

                // (8) Flip Image
                cv::flip( frame, frame, 1 );
                // (9) Update Frame
                if( !kinfu->update( frame ) ){
                    std::cout << "reset" << std::endl;
                    kinfu->reset();
                    continue;
                }
                // (10) Rendering
                cv::UMat render;
                kinfu->render( render );
                // (11) Show Image
                cv::imshow( "Kinect Fusion", render );
            }
            
        }
        openni::VideoFrameRef colorFrame;
        colorStream.readFrame( &colorFrame );
        if ( colorFrame.isValid() ) {

            colorImage = cv::Mat( colorStream.getVideoMode().getResolutionY(), 
                colorStream.getVideoMode().getResolutionX(),
                CV_8UC3, (char*)colorFrame.getData() );
            cv::flip( colorImage, colorImage, 1 );
            cv::cvtColor(colorImage, colorImage,cv::COLOR_BGR2RGB);
            cv::imshow( "color", colorImage );
        }

        const int32_t key = cv::waitKey( 1 );
        if( key == 'r' ){
            kinfu->reset();
        }
        if( key == 'q' ){
            break;
        }

    }

    cv::destroyAllWindows();

    return 0;
}