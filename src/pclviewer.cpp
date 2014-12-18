#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("Desktracking");

  // Setup the cloud pointer
  cloud_.reset (new PointCloudT);

  // Set up the QVTK window
  viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer_->setBackgroundColor (0.1, 0.1, 0.1);
  ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "Load" and "Save" buttons and their functions
  connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
  connect (ui->pushButton_save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));

  connect (ui->StartReceiverButton, SIGNAL(clicked()) , this, SLOT(on_StartReceiverButton_clicked()));

  //viewer_->addPointCloud (nullptr, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}

void
PCLViewer::loadFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "%UserProfile%", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }

  //colorCloudDistances ();
  viewer_->removeAllPointClouds();
  viewer_->addPointCloud (cloud_, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

void
PCLViewer::saveFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "%UserProfile%", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}

void PCLViewer::on_StartReceiverButton_clicked()
{
    receiver.receiveFrames();
}
