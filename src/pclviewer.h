#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QListWidgetItem>

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

#include <map>
#include <fstream>
#include <iostream>

using namespace std;

typedef pcl::PointXYZ PointT; //RGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pair<QString, PointCloudT::Ptr> StringCloudPair;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

  static const int cDepthWidth  = 512;
  static const int cDepthHeight = 424;

  public:
    /** @brief Constructor */
    explicit
    PCLViewer (QWidget *parent = 0);

    /** @brief Destructor */
    ~PCLViewer ();

  public slots:
    /** @brief Triggered whenever the "Save file" button is clicked */
    void
    saveButtonPressed ();

    /** @brief Triggered whenever the "Load file" button is clicked */
    void
    loadButtonPressed ();

  protected:
    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

private slots:
    void streamButtonPressed();
    void toggleCloudSelection(QListWidgetItem*);
    void exitApplication();

private:
    /** @brief ui pointer */
    Ui::PCLViewer *ui;
    KinectFrameReceiver receiver;

    map<QString, PointCloudT::Ptr> pointClouds;

    bool isStreaming;

    int loadDFRFile(string, PointCloudT&);
    int loadTXTFile(string, PointCloudT&);
};

#endif // PCLVIEWER_H
