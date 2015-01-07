/*************************************************************************
	> File Name: plane_segentation.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Tue 30 Dec 2014 03:32:49 PM CST
 ************************************************************************/

#include<iostream>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv ;

class SimpleOpenNIViewer 
{ 
    public: 
        SimpleOpenNIViewer (); 

        void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud); 
        void image_cb_ (const boost::shared_ptr<openni_wrapper::Image>& image) ;

        void run (); 

        pcl::visualization::CloudViewer viewer; 
    private:
        Mat RGBImage ;

}; 


SimpleOpenNIViewer::SimpleOpenNIViewer(): viewer ("PCL OpenNI Viewer") { } 

void SimpleOpenNIViewer::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) 
{ 
    if (!viewer.wasStopped()) 
    { 
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>); 
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGBA>); 
        
        // Step 1: Filter out NaNs from data
        pcl::PassThrough<pcl::PointXYZRGBA> pass; 
        pass.setInputCloud (cloud); 
        pass.setFilterFieldName ("z"); 
        pass.setFilterLimits (0, 6.0); 
        pass.filter (*cloud_filtered); 
        /*
        pass.setInputCloud (cloud_filtered); 
        pass.setFilterFieldName ("x"); 
        pass.setFilterLimits (-4.0, 4.0); 
        //pass.setFilterLimitsNegative (true); 
        pass.filter (*cloud_filtered); 

        pass.setInputCloud (cloud_filtered); 
        pass.setFilterFieldName ("y"); 
        pass.setFilterLimits (-4.0, 4.0); 
        pass.filter (*cloud_filtered);
        */

        // Step 2: Filter out statistical outliers*****************influence the speed
        /*
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor ;
        sor.setInputCloud(cloud_filtered) ;
        sor.setMeanK(50) ;
        sor.setStddevMulThresh(1.0) ;
        sor.filter(*cloud_filtered) ;
        */

        // Step 3: Downsample the point cloud (to save time in the next step)
        pcl::VoxelGrid<pcl::PointXYZRGBA> downSampler; 
        downSampler.setInputCloud (cloud_filtered); 
        downSampler.setLeafSize (0.01f, 0.01f, 0.01f); 
        downSampler.filter (*cloud_filtered2); 
        
        // Step 4: Remove the ground plane using RANSAC 
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); 
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 
        pcl::SACSegmentation<pcl::PointXYZRGBA> seg; 
        seg.setOptimizeCoefficients (true); // Optional 
        //seg.setMaxIteractions(100) ; //Optional,maybe can be lower
        // Mandatory 
        seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        //seg.setMethodType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE) ;
        seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
        seg.setEpsAngle (pcl::deg2rad (10.0));
        //seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC); 
        seg.setDistanceThreshold (0.01); //1cm
        seg.setInputCloud (cloud_filtered2->makeShared()); 
        seg.segment (*inliers, *coefficients);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ()) ;
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud (cloud_filtered2);
        // Step 4.1: Extract the points that lie in the ground plane
        extract.setIndices (inliers);
        if( inliers->indices.size() > 4000 ){
            cout << "indices size =  " << inliers->indices.size() << endl ;
            extract.setNegative (false);
            extract.filter (*cloud_plane);
        }
        // Step 4.2: Extract the points that are objects(i.e. are not in the ground plane)
        extract.setNegative (true);
        extract.filter ( *cloud_filtered2 ) ;
        
        if (inliers->indices.size () == 0) 
        { 
                PCL_ERROR ("Could not estimate a planar model for the given dataset."); 
            //exit(0); 
        } 
        // PAINT SURFACE
        for (unsigned int i = 0; i < inliers->indices.size(); i++) 
        { 
            int idx = inliers->indices[i]; 
            cloud_filtered2->points[idx].r = 255; 
            cloud_filtered2->points[idx].g = 0; 
            cloud_filtered2->points[idx].b = 0; 
        } 
        
        // Step 5: EuclideanCluster Extract the moving objects
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud (cloud_filtered2);

        std::vector<pcl::PointIndices> cluster_indices ;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec ;
        ec.setClusterTolerance (0.02) ; // 2cm
        ec.setMinClusterSize (1000) ;
        ec.setMaxClusterSize (25000) ;
        ec.setSearchMethod (tree) ;
        ec.setInputCloud (cloud_filtered2) ;
        ec.extract (cluster_indices) ;

        //Extract cluster using indices
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back (cloud_filtered2->points[*pit]); 
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            break ;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        }
        

        //viewer.showCloud(cloud_cluster); 
        viewer.showCloud(cloud_plane) ;
        
    } 
} 

void SimpleOpenNIViewer::image_cb_(const boost::shared_ptr<openni_wrapper::Image>& image){
    //boost::shared_ptr<openni_wrapper::Image> image_w ;
    //image_w = image ;
    
    cv::Mat frameRGB=cv::Mat(image->getHeight(), image->getWidth(), CV_8UC3);
	cv::Mat frameBGR=cv::Mat(image->getHeight(), image->getWidth(), CV_8UC3);
	image->fillRGB(frameRGB.cols, frameRGB.rows, frameRGB.data, frameRGB.step);
	cv::cvtColor(frameRGB,frameBGR,CV_RGB2BGR);
	cv::namedWindow("rgb");
	cv::imshow("rgb",frameBGR);
	cv::waitKey(30);

    //cout << "height= " << image->getHeight() << " ,width " << image->getWidth << endl ;
    //RGBImage( image_w->getHeight(), image_w->getWidth, CV_8UC3) ;
    //image_w->fileRGB( image_w->getWidth(), image_w->getHeight(), (unsigned char *)RGBImage.data, RGBImage.step ) ;
    /*
    image->fillRGB( 640, 480, RGBImage.data ) ;
    cvtColor( RGBImage, RGBImage, CV_BGR2RGB ) ;
    imshow( "RGB Show", RGBImage ) ;
    cvWaitKey(1);*/
    
}

void SimpleOpenNIViewer::run() 
{ 
    //pcl::Grabber* interface = new pcl::OpenNIGrabber(); 
    //cv::namedWindow("RBG Show", CV_WINDOW_AUTOSIZE) ;
    pcl::Grabber* grabber = new pcl::OpenNIGrabber() ;

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind ( &SimpleOpenNIViewer::cloud_cb_,this, _1 ) ; 
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> renderRGB = boost::bind( &SimpleOpenNIViewer::image_cb_,this, _1 ) ;

    //interface->registerCallback (f) ; 
    //grabber->registerCallback(f) ;
    boost::signals2::connection connect_ = grabber->registerCallback( f ) ;
    boost::signals2::connection image_image_ = grabber->registerCallback( renderRGB ) ;

    //interface->start () ; 
    grabber->start() ;

    while (!viewer.wasStopped()) 
    { 
        boost::this_thread::sleep (boost::posix_time::seconds (1)) ; 
    } 

    //interface->stop ();
    grabber->stop() ;
} 
int main () 
{ 
    SimpleOpenNIViewer v; 
    v.run (); 
    return 0; 
} 
