#include "PointCloudReader.h"
#include "PointCloudVisualizer.h"
#include "FeatureExtraction.h"
#include "LaserOdom.h"
#include "OdoRefinement.h"

#include <chrono>
#include <thread>

#include "tic_toc.h"


int main(int argc, char** argv) {


    double target_frame_duration = 99.5; //99.5 ms

    // Define a string for the folder path
    std::string directoryPath = "/Users/yuranliang/Downloads/dataset/sequences/00/velodyne_pcd";

    if(argc >= 2)
    {
        directoryPath = argv[1];
    }

    // if(argc < 2)
    // {
    //     std::cerr << "Please specify the directory path as an argument!" << std::endl;
    //     return -1;
    // }

    std::shared_ptr<PointCloudReader> reader = std::make_shared<PointCloudReader>(directoryPath);
    std::shared_ptr<FeatureExtraction> scanregister = std::make_shared<FeatureExtraction>();
    std::shared_ptr<LaserOdometry> laserodometry = std::make_shared<LaserOdometry>();
    std::shared_ptr<OdoRefinement> odorefinement = std::make_shared<OdoRefinement>();

    PointCloudVisualizer visualizer("Lidar SLAM", PointCloudVisualizer::CAR);


    while (true) {
        // Get the current timestamp (or any other required operations) before reading the file
        TicToc t_whole;
                
        if (!reader->loadNext()) {
            break;  // Exit the loop if there are no more files to read
        }

        scanregister->registerCloud(reader->GetCloud());
        reader->ClearCloud();
        laserodometry->process(scanregister);

        // cout << "Current odometry position: " << laserodometry->GetTwCurr() << endl;

        odorefinement->updateodomhighfreq(laserodometry->GetTwCurr(), laserodometry->GetQwCurr());

        // cout << "Current Map position high frequency: " << odorefinement->GetTwCurrHighFreq() << endl;

        if (laserodometry->GetFrameCount() % laserodometry->GetSkipFrameNum() == 0)
            {
                laserodometry->SetFrameCount(0);
                odorefinement->process(laserodometry);
                cout << "Current Map position x = " << odorefinement->GetTwCurr()[0] << " m    y = " << odorefinement->GetTwCurr()[1] << " m    z = " << odorefinement->GetTwCurr()[2]<<  " m \n";

                visualizer.updateCloud(odorefinement->GetCloudSurround());
                visualizer.updateCarPose(odorefinement->GetTwCurr(),odorefinement->GetQwCurr());
                visualizer.updateTrajectory(odorefinement->GetTwPath());
                
                
            }

            

        

        // visualizer.updateCloud(laserodometry->GetSurfPointsFlatLast());
        visualizer.spinOnce();      
        
        if (visualizer.wasStopped()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            break;  // Exit the loop if the visualizer window has been closed
        }

        // visualizer_surf_lessflat.updateCloud(scanregister.getSurfPointsLessFlat());

        // visualizer_surf_lessflat.spinOnce();

        // if (visualizer_surf_lessflat.wasStopped()) {
        //     std::this_thread::sleep_for(std::chrono::microseconds(100));
        //     break;  // Exit the loop if all of the visualizer windows have been closed
        // }


        // scanregister->clearCloud();

        // If the time for this frame is too small (i.e. frame_duration is
        // smaller than the target ms_per_frame), delay the loop to
        // achieve the correct frame rate.
        cout << "frame processing time " << t_whole.toc() << " ms *************\n";
        if(t_whole.toc() < target_frame_duration) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(target_frame_duration - t_whole.toc())));
        }
    }
    

    return 0;
}

