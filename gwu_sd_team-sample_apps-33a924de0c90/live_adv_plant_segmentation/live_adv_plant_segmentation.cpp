/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <algorithm>    // std::min

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

void
printHelp (int, char **argv)
{
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_error ("Syntax is: %s [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]\n", argv [0]);
  print_info ("%s -h | --help : shows this help\n", argv [0]);
  //print_info ("%s -xyz : use only XYZ values and ignore RGB components (this flag is required for use with ASUS Xtion Pro) \n", argv [0]);
  print_info ("%s -l : list all available devices\n", argv [0]);
  print_info ("%s -l <device-id> :list all available modes for specified device\n", argv [0]);
  print_info ("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
  print_info ("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
  print_info ("\t\t                   <serial-number>\n");
#endif
  print_info ("\n\nexamples:\n");
  print_info ("%s \"#1\"\n", argv [0]);
  print_info ("\t\t uses the first device.\n");
  print_info ("%s  \"./temp/test.oni\"\n", argv [0]);
  print_info ("\t\t uses the oni-player device to play back oni file given by path.\n");
  print_info ("%s -l\n", argv [0]);
  print_info ("\t\t list all available devices.\n");
  print_info ("%s -l \"#2\"\n", argv [0]);
  print_info ("\t\t list all available modes for the second device.\n");
  #ifndef _WIN32
  print_info ("%s A00361800903049A\n", argv [0]);
  print_info ("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
  print_info ("%s 1@16\n", argv [0]);
  print_info ("\t\t uses the device on address 16 at USB bus 1.\n");
  #endif
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class OpenNIViewer
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename Cloud::Ptr CloudPtr;

    OpenNIViewer (pcl::Grabber& grabber)
      : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI cloud"))
      , seg_viewer_ (new pcl::visualization::PCLVisualizer ("Seg cloud"))
      , image_viewer_ ()
      , grabber_ (grabber)
      , rgb_data_ (0), rgb_data_size_ (0)
      , pcd_write_number_(0)
      , write_blocked_ (false)
      , doSegmentation_ (false)
    {
    }

    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void
    image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
    {
      FPS_CALC ("image callback");
      boost::mutex::scoped_lock lock (image_mutex_);
      image_ = image;
      
      if (image->getEncoding () != openni_wrapper::Image::RGB)
      {
        if (rgb_data_size_ < image->getWidth () * image->getHeight ())
        {
          if (rgb_data_)
            delete [] rgb_data_;
          rgb_data_size_ = image->getWidth () * image->getHeight ();
          rgb_data_ = new unsigned char [rgb_data_size_ * 3];
        }
        image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
      }
    }
    
    void 
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if(event.getKeyCode() == 27)
      {
        cout << "\nEscape detected, exit condition set to true.\n" << endl;
      }
      else if (event.getKeyCode() == 32 && !write_blocked_) //spacebar
      {
        cout << " SAVING CLOUD" << endl;
        std::stringstream ss;
        ss << "frame-" << pcd_write_number_++ << ".pcd";
        boost::mutex::scoped_lock lock (cloud_mutex_);
        writer_.writeBinaryCompressed (ss.str (), *colored_cloud);
        write_blocked_ = true;
      }
      else if (event.getKeyCode() == 97 && event.keyDown())
      {
          doSegmentation_ = true;
          cout << " RUN SEGMENTATION" << endl;
      }
      else if (event.getKeyCode() == 's' && event.keyDown()) // save png screen shot
      {
          std::stringstream ss;
          ss << "frame-" << pcd_write_number_ << ".png";
          seg_viewer_->saveScreenshot(ss.str());
          cout << " SAVE PNG" << endl;
      }


      if (event.getKeyCode ())
        cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
      else
        cout << "the special key \'" << event.getKeySym() << "\' was";
      if (event.keyDown())
        cout << " pressed" << endl;
      else{
        cout << " released" << endl;
        write_blocked_ = false;
      }
    }
    
    void 
    mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    void
    RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
    {
        r /= 255;
        g /= 255;
        b /= 255;
        float min, max, delta;
        min = std::min(r, g);
        min = std::min(min, b);
        max = std::max(r, g);
        max = std::max(max, b);
        *v = max;				// v
        delta = max - min;
        if( max != 0 )
            *s = delta / max;		// s
        else {
            // r = g = b = 0		// s = 0, v is undefined
            *s = 0;
            *h = -1;
            return;
        }
        if( r == max )
            *h = ( g - b ) / delta;		// between yellow & magenta
        else if( g == max )
            *h = 2 + ( b - r ) / delta;	// between cyan & yellow
        else
            *h = 4 + ( r - g ) / delta;	// between magenta & cyan
        *h *= 60;				// degrees
        if( *h < 0 )
            *h += 360;
    }

    pcl::PointCloud <pcl::PointXYZRGBA>::Ptr
    plant_segmentation(CloudPtr& cloud)
    {
        int cfgSegAlgo = 0;     // 0 - color based region grwoing (organized)
                                // 1 - normals based region growing
                                // 2 - euclidian distance clustering
                                // 3 - supervoxels

        int cfgFilterOutliers = 1; // 0 - OFF; 1 - ON

        pcl::PointCloud <pcl::PointXYZRGBA>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        colored_cloud->height = 1;
        colored_cloud->width = 0;
        colored_cloud->is_dense = 0;

        // Define color range
        int minHue = 60;    int maxHue = 150;
        int minSat = 0.3;    int maxSat = 1;
        int minVal = 0.4;    int maxVal = 1;

        // Parse cloud and segment
        if (!(*cloud).isOrganized())
        {
            cout << "Cloud not organized. Use other segmentation method" << endl;
        }
        else
        {
            //cout << "  Segment cloud " << (*cloud).width << "," << (*cloud).height << endl;
            for (int r = 0; r < (*cloud).height; r++)
                for (int c = 0; c < (*cloud).width; c++)
                {
                    pcl::PointXYZRGBA point = (*cloud)(c, r);

                    if (!pcl::isFinite(point)) continue;

                    float c_r = (float)point.r;
                    float c_g = (float)point.g;
                    float c_b = (float)point.b;
                    float h, s, v;
                    RGBtoHSV(c_r, c_g, c_b, &h, &s, &v);

                    if (h < minHue || h > maxHue ||
                        s < minSat || v < minVal)
                    { // delete not-a-plant point
                        (*cloud)(c, r).z = NAN;
                    }
                    else
                    { // add to the new cloud
                        colored_cloud->push_back(point);
                    }
                }
        }


        // Apply segmentation
        if (cfgSegAlgo == 0) // apply color based region growing
        {
            //cout << "New size " << colored_cloud->width << " , " << colored_cloud->height << " organized: " << colored_cloud->isOrganized() << endl;
            pcl::search::Search <pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
            pcl::IndicesPtr indices (new std::vector <int>);
            pcl::PassThrough<pcl::PointXYZRGBA> pass;
            pass.setInputCloud (colored_cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.5, 1.5);
            pass.filter (*indices);

            pcl::RegionGrowingRGB<pcl::PointXYZRGBA> reg;
            reg.setInputCloud (colored_cloud);
            reg.setIndices (indices);
            reg.setSearchMethod (tree);
            reg.setDistanceThreshold (0.01);
            reg.setPointColorThreshold (30);
            reg.setRegionColorThreshold (5);
            reg.setMinClusterSize (200);

            std::vector <pcl::PointIndices> clusters;
            reg.extract (clusters);
            pcl::PointCloud <pcl::PointXYZRGB>::Ptr tmp = reg.getColoredCloud ();

            colored_cloud->clear();
            for (int r = 0; r < (*tmp).height; r++)
                for (int c = 0; c < (*tmp).width; c++)
                {
                    pcl::PointXYZRGB point = (*tmp)(c, r);
                    if (!pcl::isFinite(point)) continue;

                    pcl::PointXYZRGBA pp;
                    if (point.r == 255 && point.g == 0 && point.b == 0)
                    {
                        continue;
                    }
                    pp.x = point.x;
                    pp.y = point.y;
                    pp.z = point.z;
                    pp.r = point.r;
                    pp.g = point.g;
                    pp.b = point.b;
                    colored_cloud->push_back( pp );
                }

            // Filter the noise [optional]
            if (cfgFilterOutliers)
            {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
                sor.setInputCloud (colored_cloud);
                sor.setMeanK (50);
                sor.setStddevMulThresh (0.5);
                sor.filter (*colored_cloud);
            }

            return colored_cloud;
        }
        else if (cfgSegAlgo == 1) // apply region growing based on normals
        {
            pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
            pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normalEstimator;
            normalEstimator.setSearchMethod (tree);
            normalEstimator.setInputCloud (colored_cloud);
            normalEstimator.setKSearch (50);
            normalEstimator.compute (*normals);

            pcl::IndicesPtr indices (new std::vector <int>);
            pcl::PassThrough<pcl::PointXYZRGBA> pass;
            pass.setInputCloud (colored_cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.5, 1.5);
            pass.filter (*indices);

            pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
            reg.setMinClusterSize (50);
            reg.setMaxClusterSize (100000);
            reg.setSearchMethod (tree);
            reg.setNumberOfNeighbours (30);
            reg.setInputCloud (colored_cloud);
            reg.setIndices (indices);
            reg.setInputNormals (normals);
            reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
            reg.setCurvatureThreshold (1.0);

            std::vector <pcl::PointIndices> clusters;
            reg.extract (clusters);
            pcl::PointCloud <pcl::PointXYZRGB>::Ptr tmp = reg.getColoredCloud ();

            for (int r = 0; r < (*tmp).height; r++)
                for (int c = 0; c < (*tmp).width; c++)
                {
                    pcl::PointXYZRGB point = (*tmp)(c, r);
                    if (!pcl::isFinite(point)) continue;

                    pcl::PointXYZRGBA pp;
                    pp.x = point.x;
                    pp.y = point.y;
                    pp.z = point.z;
                    pp.r = point.r;
                    pp.g = point.g;
                    pp.b = point.b;
                    colored_cloud->push_back( pp );
                }

            return colored_cloud;
        }
        else if (cfgSegAlgo == 2) // apply euclidian clustering
        {
            //cout << "New size " << colored_cloud->width << " , " << colored_cloud->height << " organized: " << colored_cloud->isOrganized() << endl;
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
            tree->setInputCloud (colored_cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
            ec.setClusterTolerance (0.005); // 1cm
            ec.setMinClusterSize (200);
            //qec.setMaxClusterSize (25000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (colored_cloud);
            ec.extract (cluster_indices);

            int j = 0;
            int maxJ = cluster_indices.size();
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                {
                    colored_cloud->points[*pit].r = (char)(255.0 * j / maxJ);
                    colored_cloud->points[*pit].g = (char)(255.0 * j / (2*maxJ));
                    colored_cloud->points[*pit].b = (char)(255.0 * (maxJ - j) / maxJ);
                }
                j++;
            }

            return colored_cloud;

        }
        else if (cfgSegAlgo == 3)// apply supervoxels
        {
            //cout << "New size " << colored_cloud->width << " , " << colored_cloud->height << " organized: " << colored_cloud->isOrganized() << endl;
            bool use_transform = true;
            float voxel_resolution = 0.008f;
            float seed_resolution = 0.1f;
            float color_importance = 0.2f;
            float spatial_importance = 0.4f;
            float normal_importance = 1.0f;

            pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution, use_transform);
            super.setInputCloud (colored_cloud);
            super.setColorImportance (color_importance);
            super.setSpatialImportance (spatial_importance);
            super.setNormalImportance (normal_importance);

            std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
            super.extract (supervoxel_clusters);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = super.getColoredVoxelCloud ();

            return voxel_centroid_cloud;
        }

    }

    /**
     * @brief starts the main loop
     */
    void
    run ()
    {
      cloud_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);
      cloud_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
      seg_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);
      seg_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNIViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);
      
      boost::signals2::connection image_connection;
      if (grabber_.providesCallback<void (const boost::shared_ptr<openni_wrapper::Image>&)>())
      {
        image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));
        image_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);
        image_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
        boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&OpenNIViewer::image_callback, this, _1);
        image_connection = grabber_.registerCallback (image_cb);
      }
      
      bool image_init = false, cloud_init = false;
      
      grabber_.start ();

      while (!cloud_viewer_->wasStopped () && (image_viewer_ && !image_viewer_->wasStopped ()) && !seg_viewer_->wasStopped ())
      {
        boost::shared_ptr<openni_wrapper::Image> image;
        CloudConstPtr cloud;
        seg_viewer_->spinOnce();
        cloud_viewer_->spinOnce ();

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          FPS_CALC ("drawing cloud");
          CloudPtr newCloud;
          newCloud = cloud->makeShared();

          // Process the cloud
          if (doSegmentation_)
          {
              colored_cloud = plant_segmentation(newCloud);
          }

          // Display the cloud
          if (!cloud_init)
          {
            cloud_viewer_->setPosition (0, 0);
            cloud_viewer_->setSize (newCloud->width, newCloud->height);
            seg_viewer_->setPosition (0, 0);
            seg_viewer_->setSize (newCloud->width, newCloud->height);
            cloud_init = !cloud_init;
          }

          if (!cloud_viewer_->updatePointCloud (newCloud, "OpenNICloud"))
          {
            cloud_viewer_->addPointCloud (newCloud, "OpenNICloud");
            cloud_viewer_->resetCameraViewpoint ("OpenNICloud");
          }

          if (doSegmentation_)
          {
              if (!seg_viewer_->updatePointCloud (colored_cloud, "SegCloud"))
              {
                seg_viewer_->addPointCloud (colored_cloud, "SegCloud");
                seg_viewer_->resetCameraViewpoint ("SegCloud");
              }
          }
        }

        // See if we can get an image
        if (image_mutex_.try_lock ())
        {
          image_.swap (image);
          image_mutex_.unlock ();
        }

        if (image)
        {
          if (!image_init && cloud && cloud->width != 0)
          {
            image_viewer_->setPosition (cloud->width, 0);
            image_viewer_->setSize (cloud->width, cloud->height);
            image_init = !image_init;
          }

          if (image->getEncoding() == openni_wrapper::Image::RGB)
            image_viewer_->addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
          else
            image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
          image_viewer_->spinOnce ();
        }
        
      }

      grabber_.stop ();
      
      cloud_connection.disconnect ();
      image_connection.disconnect ();
      if (rgb_data_)
        delete[] rgb_data_;
    }
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> seg_viewer_;
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    
    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;
    
    int pcd_write_number_;
    pcl::PCDWriter writer_;
    bool write_blocked_;
    CloudConstPtr cloud_;
    CloudPtr colored_cloud;
    boost::shared_ptr<openni_wrapper::Image> image_;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;

    // Filters to apply
    bool doSegmentation_;
};

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
boost::shared_ptr<pcl::visualization::ImageViewer> img;

/* ---[ */
int
main (int argc, char** argv)
{
  std::string device_id("");
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  bool xyz = false;
  
  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      printHelp(argc, argv);
      return 0;
    }
    else if (device_id == "-l")
    {
      if (argc >= 3)
      {
        pcl::OpenNIGrabber grabber(argv[2]);
        boost::shared_ptr<openni_wrapper::OpenNIDevice> device = grabber.getDevice();
        cout << "Supported depth modes for device: " << device->getVendorName() << " , " << device->getProductName() << endl;
        std::vector<std::pair<int, XnMapOutputMode > > modes = grabber.getAvailableDepthModes();
        for (std::vector<std::pair<int, XnMapOutputMode > >::const_iterator it = modes.begin(); it != modes.end(); ++it)
        {
          cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
        }

        if (device->hasImageStream ())
        {
          cout << endl << "Supported image modes for device: " << device->getVendorName() << " , " << device->getProductName() << endl;
          modes = grabber.getAvailableImageModes();
          for (std::vector<std::pair<int, XnMapOutputMode > >::const_iterator it = modes.begin(); it != modes.end(); ++it)
          {
            cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
          }
        }
      }
      else
      {
        openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
        if (driver.getNumberDevices() > 0)
        {
          for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx)
          {
            cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName(deviceIdx) << ", product: " << driver.getProductName(deviceIdx)
              << ", connected: " << driver.getBus(deviceIdx) << " @ " << driver.getAddress(deviceIdx) << ", serial number: \'" << driver.getSerialNumber(deviceIdx) << "\'" << endl;
          }

        }
        else
          cout << "No devices connected." << endl;
        
        cout <<"Virtual Devices available: ONI player" << endl;
      }
      return 0;
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() > 0)
      cout << "Device Id not set, using first device." << endl;
  }
  
  unsigned mode;
  if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
    depth_mode = pcl::OpenNIGrabber::Mode (mode);

  if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
    image_mode = pcl::OpenNIGrabber::Mode (mode);
  
  //if (pcl::console::find_argument (argc, argv, "-xyz") != -1)
  //  xyz = true;
  
  pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);
  /*
  if (xyz || !grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    OpenNIViewer<pcl::PointXYZ> openni_viewer (grabber);
    cout << "XYZ -> Cloud space" << endl;
    openni_viewer.run ();
  }
  else
  {
    cout << "XYZRGBA -> Cloud space" << endl;
    OpenNIViewer<pcl::PointXYZRGBA> openni_viewer (grabber);
    openni_viewer.run ();
  }
  */
  cout << "XYZRGBA -> Cloud space" << endl;
  OpenNIViewer<pcl::PointXYZRGBA> openni_viewer (grabber);
  openni_viewer.run ();
  
  return (0);
}
/* ]--- */
