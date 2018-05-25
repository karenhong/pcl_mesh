#include <iostream>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>


void extractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints) {

}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormal (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
    return cloud_normals;

}

int pcd_read(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/Users/karenhong/Desktop/Meshes/SpatialMapping_meier_low3.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }
    return (0);
}

int writeToFile(pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs) {
    std::ofstream myfile ("hist1.txt");
    if (myfile.is_open())
    {
        for (size_t i = 0; i < pfhs->points.size (); ++i)
        {
            for (size_t j = 0; j < 125; ++j)
            {
                myfile << pfhs->points[i].histogram[j];
                myfile << " ";
            }
            myfile << "\n";
        }
        myfile.close();
    }
    else std::cout << "Unable to open file";
    return 0;
}

int
main (int argc, char** argv)
{

//    ... read, pass in or create a point cloud with normals ...
//    ... (note: you can create a single PointCloud<PointNormal> if you want) ...

//    const std::string & filename = "/Users/karenhong/Desktop/Meshes/SpatialMapping_meier_low1.obj";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());

    pcd_read(cloud);
//    extractKeypoints(cloud, model_keypoints);
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormal(cloud);

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (cloud);
    pfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (0.05);

    // Compute the features
    pfh.compute (*pfhs);

    writeToFile(pfhs);
    // pfhs->points.size () should have the same size as the input cloud->points.size ()*
}
