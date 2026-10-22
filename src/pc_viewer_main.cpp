#include "PointCloudReader.h"
#include "PointCloudVisualizer.h"
#include "tic_toc.h"
#include <chrono>
#include <iostream>
#include <thread>



int main(int argc, char** argv) {


    double target_frame_duration = 99.5; //99.5 ms


    // Define a string for the folder path
    std::string directoryPath = "../velodyne_pcd";

    if(argc >= 2)
    {
        directoryPath = argv[1];
    }

    // if(argc < 2)
    // {
    //     std::cerr << "Please specify the directory path as an argument!" << std::endl;
    //     return -1;
    // }

    // PointCloudReader reader(directoryPath);
    std::shared_ptr<PointCloudReader> reader = std::make_shared<PointCloudReader>(directoryPath);
    PointCloudVisualizer visualizer("Pointcloud Viewer");

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while (true) {
        // Get the current timestamp (or any other required operations) before reading the file
        TicToc t_whole;
                
        if (!reader->loadNext()) {
            break;  // Exit the loop if there are no more files to read
        }


        visualizer.updateCloud(reader->GetCloud());

        visualizer.spinOnce();
        
        if (visualizer.wasStopped()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            break;  // Exit the loop if the visualizer window has been closed
        }

        // If the time for this frame is too small (i.e. frame_duration is
        // smaller than the target ms_per_frame), delay the loop to
        // achieve the correct frame rate.
        std::cout << "frame processing time " << t_whole.toc() << " ms *************\n";
        if(t_whole.toc() < target_frame_duration) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(target_frame_duration - t_whole.toc())));
        }
    }

    return 0;
}

