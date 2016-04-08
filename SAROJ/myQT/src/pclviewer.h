/* This file is the header for the class PCLViewer */


#ifndef PCLVIEWER_H
#define PCLVIEWER_H

//QT libraries 
#include <QMainWindow>     // This contains UI elements
#include <QFileDialog>

//PCL 
// go to pcl and get these headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

//Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK) for qvtkWidget
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud <Point> PointCloudT;

namespace Ui
{
	class PCLViewer;
}

class PCLViewer : public QMainWindow
{
	//  inform the compiler that this object contains UI elements; 
	//  this imply that this file will be 
	//  processed through the Meta-Object Compiler (moc)
	Q_OBJECT 

	public:
	explicit PCLViewer(QWidget *parent = 0); //Constructor
	~PCLViewer();	//Destructor

	public slots:
	// Triggered whenever the "Save file" button is clicked 
	void saveFileButtonPressed ();
	//// Triggered whenever the "Save file" button is clicked 
	void loadFileButtonPressed ();

	protected:
		//The PCL Visualizer object
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

		// Point cloud displayed
		PointCloud::Ptr cloud_;
		void colorCloudDistances ();

	private:
		//ui pointer
		Ui::PCLViewer *ui;

};

#endif //PCLVIEWER_H