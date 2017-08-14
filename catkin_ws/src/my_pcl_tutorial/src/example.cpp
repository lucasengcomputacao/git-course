//Bibliotecas de inclusão ros
#include <ros/ros.h>
/*Bilbiotecas a primeira e para passar mensagens
*Pcl_conversions para conversão de pontos em std_msg
*pcl point cloud para nuvem de pontos
*iostream e pcl io e para entrada e saida
*/
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
/*Openni grabber framework de entrda e saida
*Biblioteca de visualization
*Biblioteca de filtro usando Voxel grid que diminui os numeros de pontos
*/
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
//Publica um nó ROS
ros::Publisher pub;

//pcl::visualization::PCLVisualizer viewer("PCL Viewer");


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  
  /*
  *Objeto Cloud do tipo PointCloud2
  * Objeto ponteiro cloudPTR (Definido como struct)
  */
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  //Converte uma nuvem de pontos em uma mensagem
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  /*
  *Filtro VoxelGrid
  */
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
//Função que salva a nuvem de pontos como um arquivo pcd
 pcl::io::savePCDFile ("test_pcd.pcd",cloud_filtered);

  /*
  *Mensagem do tipo PointCloud2 output
  *Conversão da mensagem ara pcl
  */
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);



  // Publicação
  pub.publish (output);
}

int main (int argc, char** argv)
{
  /*
  *Função inicializa o ros argc e argv são necessarias
  *mais o nome do no logo embaixo no construtor e tambem destrututor
  *
  */
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //Função para impressão da matriz no terminal da nuvem de pontos

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;

  //Loop do terminal para finalizar o processo com ctrl c
  ros::spin ();
}
