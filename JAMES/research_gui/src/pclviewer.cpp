#include "pclviewer.h"
#include "../include/ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);

  // The default color
  red   = 128;
  green = 128;
  blue  = 128;

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  viewer->addPointCloud (cloud, "cloud");
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
}


PCLViewer::~PCLViewer ()
{
  delete ui;
}

void PCLViewer::on_open_pcd_clicked()
{
    printf ("Opening file dialog\n");
    QString fileName = QFileDialog::getOpenFileName(this,
            tr("Open PCD File"), "/home", tr("PCD Files (*.pcd)"));

    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (fileName.toStdString(), *cloud);

    viewer->updatePointCloud (cloud, "cloud");
    ui->qvtkWidget->update ();
}
