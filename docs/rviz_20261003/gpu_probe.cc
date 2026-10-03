#include <QApplication>
#include <QTimer>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <rviz/visualization_frame.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <OgreRenderWindow.h>
#include <iostream>
int main(int argc, char** argv) {
  if(argc<3) return 2;
  QApplication app(argc,argv);
  ros::init(argc,argv,"rviz_gpu_probe",ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  auto pub=nh.advertise<tf2_msgs::TFMessage>("/tf_static",1,true);
  tf2_msgs::TFMessage tf;tf.transforms.resize(1);
  tf.transforms[0].header.frame_id="world";
  tf.transforms[0].header.stamp=ros::Time::now();
  tf.transforms[0].child_frame_id="body";
  tf.transforms[0].transform.rotation.w=1;pub.publish(tf);
  rviz::VisualizationFrame frame;
  frame.setSplashPath("");frame.initialize(argv[1]);frame.show();
  int frames=0;
  QObject::connect(frame.getManager(),&rviz::VisualizationManager::preUpdate,[&frames](){++frames;});
  QTimer::singleShot(5000,[&](){
    auto window=frame.getManager()->getRenderPanel()->getRenderWindow();
    window->writeContentsToFile(argv[2]);
    std::cout<<"GPU_CAPTURE frames="<<frames<<" size="<<window->getWidth()<<"x"<<window->getHeight()<<std::endl;
    app.quit();
  });
  int result=app.exec();ros::shutdown();return result;
}
