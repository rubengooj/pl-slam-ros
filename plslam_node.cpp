#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <sceneRepresentation.h>
#include <mrpt/utils/CTicTac.h>
#include <stereoFrame.h>
#include <stereoFrameHandler.h>
#include <yaml-cpp/yaml.h>

#include <mapHandler.h>
#include <slamScene.h>

#include <X11/Xlib.h>


class PLSLAMNode
{

private:

  ros::NodeHandle nh_;

  // The synchronization policy used by the interface to sync stereo images
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

  // Subscribers
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> img_sub_l_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> img_sub_r_;
  std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;

  image_transport::ImageTransport it_;

  PinholeStereoCamera* cam_pin;
  bool rectify;

  StereoFrameHandler* StVO;
  PLSLAM::MapHandler* map;
  int frame_counter;

  PLSLAM::slamScene scene;

public:

  PLSLAMNode( std::string params_file, std::string config_file )
    : it_(nh_)
  {

      cout << endl << "Initializing PL-SLAM..." << flush ;

      // Initialize camera object
      //------------------------------------------------------
      // read content of the .yaml dataset configuration file
      YAML::Node dset_config  = YAML::LoadFile( params_file );
      // setup camera
      YAML::Node cam_config = dset_config["cam0"];
      string camera_model = cam_config["cam_model"].as<string>();
      rectify = false;
      if( camera_model == "Pinhole" )
      {
          // if EuRoC or Falcon yaml file
          if( cam_config["Kl"].IsDefined() )
          {
              rectify = true;
              Mat Kl, Kr, Dl, Dr, R, t;
              vector<double> Kl_ = cam_config["Kl"].as<vector<double>>();
              vector<double> Kr_ = cam_config["Kr"].as<vector<double>>();
              vector<double> Dl_ = cam_config["Dl"].as<vector<double>>();
              vector<double> Dr_ = cam_config["Dr"].as<vector<double>>();
              Kl = ( Mat_<float>(3,3) << Kl_[0], 0.0, Kl_[2], 0.0, Kl_[1], Kl_[3], 0.0, 0.0, 1.0 );
              Kr = ( Mat_<float>(3,3) << Kr_[0], 0.0, Kr_[2], 0.0, Kr_[1], Kr_[3], 0.0, 0.0, 1.0 );
              // load rotation and translation
              vector<double> R_ = cam_config["R"].as<vector<double>>();
              vector<double> t_ = cam_config["t"].as<vector<double>>();
              R = Mat::eye(3,3,CV_64F);
              t = Mat::eye(3,1,CV_64F);
              int k = 0;
              for( int i = 0; i < 3; i++ )
              {
                  t.at<double>(i,0) = t_[i];
                  for( int j = 0; j < 3; j++, k++ )
                      R.at<double>(i,j) = R_[k];
              }
              // load distortion parameters
              int Nd = Dl_.size();
              Dl = Mat::eye(1,Nd,CV_64F);
              Dr = Mat::eye(1,Nd,CV_64F);
              for( int i = 0; i < Nd; i++ )
              {
                  Dl.at<double>(0,i) = Dl_[i];
                  Dr.at<double>(0,i) = Dr_[i];
              }
              // if dtype is equidistant (now it is default)
              if( cam_config["dtype"].IsDefined() )
              {
                  cam_pin = new PinholeStereoCamera(
                      cam_config["cam_width"].as<double>(),
                      cam_config["cam_height"].as<double>(),
                      cam_config["cam_bl"].as<double>(),
                      Kl, Kr, R, t, Dl, Dr, true);

              }
              else
              // create camera object for EuRoC
                  cam_pin = new PinholeStereoCamera(
                      cam_config["cam_width"].as<double>(),
                      cam_config["cam_height"].as<double>(),
                      cam_config["cam_bl"].as<double>(),
                      Kl, Kr, R, t, Dl, Dr,false);
          }
          // else
          else
              cam_pin = new PinholeStereoCamera(
                  cam_config["cam_width"].as<double>(),
                  cam_config["cam_height"].as<double>(),
                  fabs(cam_config["cam_fx"].as<double>()),
                  fabs(cam_config["cam_fy"].as<double>()),
                  cam_config["cam_cx"].as<double>(),
                  cam_config["cam_cy"].as<double>(),
                  cam_config["cam_bl"].as<double>(),
                  cam_config["cam_d0"].as<double>(),
                  cam_config["cam_d1"].as<double>(),
                  cam_config["cam_d2"].as<double>(),
                  cam_config["cam_d3"].as<double>()  );
      }
      else

      // Initialize VO object
      //------------------------------------------------------
      frame_counter = 0;
      StVO = new StereoFrameHandler(cam_pin);
      if (!config_file.empty()) Config::loadFromFile(config_file);

      // Initialize SLAM object
      //------------------------------------------------------
      map  = new PLSLAM::MapHandler(cam_pin);
      if (!config_file.empty()) SlamConfig::loadFromFile(config_file);

      // Initialize scene object
      //------------------------------------------------------
      Matrix4d Tcw, Tfw = Matrix4d::Identity(), Tfw_prev = Matrix4d::Identity(), T_inc;
      Tcw = Matrix4d::Identity();
      XInitThreads();
      scene = PLSLAM::slamScene( "/home/ruben/code/pl-slam-dev/config/scene_config_indoor.ini" );
      scene.setStereoCalibration( cam_pin->getK(), cam_pin->getB() );
      scene.initializeScene(Tcw);
      mrpt::utils::CTicTac clock;

      // Subscribe to input video feed and publish output video feed
      img_sub_l_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
                   new message_filters::Subscriber<sensor_msgs::Image>(
                   nh_, "image_left", 1) );

      img_sub_r_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
                   new message_filters::Subscriber<sensor_msgs::Image>(
                   nh_, "image_right", 1) );

      // Creating a synchronizer
      sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
              new message_filters::Synchronizer<sync_pol>(sync_pol(10), *img_sub_l_,
                                                      *img_sub_r_));

      // Registering the synchronized image callback
      sync_->registerCallback(
          boost::bind( &PLSLAMNode::PLSLAMCb, this, _1, _2) );

      cout << " ... done." << endl << endl;

  }

  ~PLSLAMNode(){}

  void PLSLAMCb( const sensor_msgs::ImageConstPtr& msg_l, const sensor_msgs::ImageConstPtr& msg_r )
  {

      // grab left image
      cv_bridge::CvImageConstPtr cv_ptr_l;
      try
      {
        cv_ptr_l = cv_bridge::toCvShare( msg_l, "mono8" );
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // grab right image
      cv_bridge::CvImageConstPtr cv_ptr_r;
      try
      {
        cv_ptr_r = cv_bridge::toCvShare( msg_r, "mono8" );
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // rectify images (if distorted)
      cv::Mat img_l_rec, img_r_rec;
      if( rectify )
          cam_pin->rectifyImagesLR( cv_ptr_l->image, img_l_rec, cv_ptr_r->image, img_r_rec );
      else
      {
          img_l_rec = cv_ptr_l->image;
          img_r_rec = cv_ptr_r->image;
      }

      // initialize VO if frame #0
      mrpt::utils::CTicTac clock;
      if( frame_counter == 0 )
      {
          StVO->initialize( img_l_rec, img_r_rec, frame_counter );
          PLSLAM::KeyFrame* kf = new PLSLAM::KeyFrame( StVO->prev_frame, 0 );
          map->initialize( kf );
          scene.initViewports( img_l_rec.cols, img_l_rec.rows );
          scene.setImage(StVO->curr_frame->plotStereoFrame());
          scene.updateScene( map );
          frame_counter++;
      }
      // run VO otherwise
      else
      {
          clock.Tic();
          // PL-StVO
          StVO->insertStereoPair( img_l_rec, img_r_rec, frame_counter );
          // solve with robust kernel and IRLS
          StVO->optimizePose();
          double t = 1000 * clock.Tac(); //ms
          // console output
          cout.setf(ios::fixed,ios::floatfield); cout.precision(6);
          cout << "Frame: " << frame_counter << "\tRes.: " << StVO->curr_frame->err_norm;
          cout.setf(ios::fixed,ios::floatfield); cout.precision(3);
          cout << " \t Proc. time: " << t  << " ms\t ";
          if( Config::adaptativeFAST() )  cout << "\t FAST: "   << StVO->orb_fast_th;
          if( Config::hasPoints())        cout << "\t Points: " << StVO->matched_pt.size() << " (" << StVO->n_inliers_pt << ") " ;
          if( Config::hasLines() )        cout << "\t Lines:  " << StVO->matched_ls.size() << " (" << StVO->n_inliers_ls << ") " ;
          cout << endl;

          // 1. check if a new keyframe is needed
          if( StVO->needNewKF() )
          {
              cout <<         "#KeyFrame:     " << map->max_kf_idx + 1;
              cout << endl << "#Points:       " << map->map_points.size();
              cout << endl << "#Segments:     " << map->map_lines.size();
              cout << endl << endl;
              // grab StF and update KF in StVO (the StVO thread can continue after this point)
              PLSLAM::KeyFrame* curr_kf = new PLSLAM::KeyFrame( StVO->curr_frame );
              // update KF in StVO
              StVO->currFrameIsKF();
              // ----------------------------------------------
              // ANOTHER THREAD, THE ODOMETRY RUNS CONTINUOUSLY
              // ----------------------------------------------
              map->addKeyFrame( curr_kf );
              cout << endl << "..end" << endl;
              // update scene
              scene.setImage(StVO->curr_frame->plotStereoFrame());
              scene.updateScene( map );
          }
          else
          {
              scene.setImage(StVO->curr_frame->plotStereoFrame());
              scene.setPose( StVO->curr_frame->DT );
              scene.updateScene();
          }

          // update StVO
          StVO->updateFrame();
          frame_counter++;

      }

  }

};

using namespace std;

int main(int argc, char** argv)
{

  // Usage: ./slam  <params_file>  <config_file>
  // <params_file> - file with the camera configuration
  // <config_file> - file with the VO/SLAM configuration (optional)

  ros::init(argc, argv, "plslam");

  std::string params_file = argv[1];
  std::string config_file ;
  if( argc == 3 )
      config_file = argv[2];

  PLSLAMNode plslam( params_file, config_file );

  if( true )
      ros::spin();
  else
  {
      ros::Rate r(10);
      while( ros::ok() )
      {
        ros::spinOnce();
        r.sleep();
      }
  }

  return 0;

}
