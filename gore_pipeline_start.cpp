#include <boost/random.hpp>
#include <fstream>

#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>

/////////////////////////////////////////////////////////////////////
// User defined parameters
/////////////////////////////////////////////////////////////////////
std::string inputFilename = "PATH_TO_INPUT_FILE";
std::string outputname = "NAME_OF_OUTPUT_FILE";
double featureSearchRadius = 0.5; // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
double normalSearchRadius = 0.3;
bool noise = true;
float noise_percent = 4; // add noise to x0% of points
double noise_std = 0.001; //1mm


/////////////////////////////////////////////////////////////////////

void readobj2pc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Input stream
    std::ifstream is(inputFilename.c_str());

    // Read line by line
    for(std::string line; std::getline(is, line); )
    {
        std::istringstream in(line);

        std::string v;
        in >> v;
        if (v == "v")
        {
            // Read x y z
            float x, y, z;
            in >> x >> y >> z;
            cloud->push_back(pcl::PointXYZ(x, y, z));
        }
    }
    is.close();
}

// Function that adds noise to a user defined percentage of points
void add_noise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    struct timeval start;
    gettimeofday (&start, NULL);
    boost::mt19937 rng;
    rng.seed (start.tv_usec);
    boost::normal_distribution<> nd (0.0, noise_std);
    boost::variate_generator<boost::mt19937&,
            boost::normal_distribution<> > var_nor (rng, nd);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        float f = rand()*1.0f/RAND_MAX;
        float vv= noise_percent / 10.0f;
        if (f < vv) {
            std::cout << var_nor() << "\n";
            cloud->points[i].x += var_nor ();
            cloud->points[i].y += var_nor ();
            cloud->points[i].z += var_nor ();
        }
    }

}

double compute_cloud_resolution (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i) {
        if (! pcl_isfinite ((*cloud)[i].x)) {
            continue;
        }
        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2) {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        res /= n_points;
    }
    return res;
}

int iss3d( pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr kpts) {
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>() );

    double cloud_resolution = compute_cloud_resolution(cloud);

    iss_detector.setSearchMethod (tree);
    iss_detector.setNormals(normals);
    iss_detector.setSalientRadius (6 * cloud_resolution);
    iss_detector.setNonMaxRadius (4 * cloud_resolution);

    iss_detector.setThreshold21 (0.8);
    iss_detector.setThreshold32 (0.8);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (1);
    iss_detector.setInputCloud (cloud);
    iss_detector.compute(*kpts);

    return 0;
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

    // Use all neighbors in a sphere of a user defined radius
    ne.setRadiusSearch (normalSearchRadius);

    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
    return cloud_normals;

}

int writeToFile(pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs) {
    std::ofstream myfile (outputname + "_hist.csv");
    if (myfile.is_open())
    {
        for (size_t i = 0; i < pfhs->points.size (); ++i)
        {
            for (size_t j = 0; j < 125; ++j)
            {
                myfile << pfhs->points[i].histogram[j];
                myfile << ", ";
            }
            myfile << "\n";
        }
        myfile.close();
    }
    else std::cout << "Unable to open file";
    return 0;
}

int writeKeyPointsToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::ofstream myfile (outputname + "_keypoints.csv");
    if (myfile.is_open())
    {
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                myfile << cloud->points[i].data[j];
                myfile << ", ";
            }
            myfile << "\n";
        }
        myfile.close();
    }
    else std::cout << "Unable to open file";
    return 0;

}

int writePCToFile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::ofstream myfile (outputname + "_pointcloud.csv");
    if (myfile.is_open())
    {
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                myfile << cloud->points[i].data[j];
                myfile << ", ";
            }
            myfile << "\n";
        }
        myfile.close();
    }
    else std::cout << "Unable to open file";
    return 0;

}

int main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ> ());

    readobj2pc(cloud);
    if (noise) {
        add_noise(cloud);
    }

    pcl::PointCloud<pcl::Normal>::Ptr estimated_normals = estimateNormal(cloud);
    iss3d(cloud, estimated_normals, keypoints);

    writePCToFile(cloud);
    writeKeyPointsToFile(keypoints);

    estimated_normals = estimateNormal(keypoints);
    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (keypoints);
    pfh.setInputNormals (estimated_normals);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // Use all neighbors in a sphere of a user defined radius
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (featureSearchRadius);

    // Compute the features
    pfh.compute (*pfhs);

    writeToFile(pfhs);
}
