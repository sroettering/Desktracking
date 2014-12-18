#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QMainWindow>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "KinectFrameReceiver.h"

typedef pcl::PointXYZ PointT; //RGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

  public:
    /** @brief Constructor */
    explicit
    PCLViewer (QWidget *parent = 0);

    /** @brief Destructor */
    ~PCLViewer ();

  public slots:
    /** @brief Triggered whenever the "Save file" button is clicked */
    void
    saveFileButtonPressed ();

    /** @brief Triggered whenever the "Load file" button is clicked */
    void
    loadFileButtonPressed ();

  protected:
    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    /** @brief The point cloud displayed */
    PointCloudT::Ptr cloud_;

private slots:
    void on_StartReceiverButton_clicked();

private:
    /** @brief ui pointer */
    Ui::PCLViewer *ui;
    KinectFrameReceiver receiver;
};

#endif // PCLVIEWER_H
