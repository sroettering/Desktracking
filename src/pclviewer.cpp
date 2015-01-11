#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

using namespace std;

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer),
    isStreaming(false)
{
  ui->setupUi (this);
  this->setWindowTitle ("Desktracking");

  // Setup the cloud pointer
  PointCloudT::Ptr tmpCloud;
  tmpCloud.reset (new PointCloudT);
  pointClouds.insert(StringCloudPair("", tmpCloud));

  // Set up the QVTK window
  viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer_->setBackgroundColor (0.1, 0.1, 0.1);
  ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "Load" and "Save" buttons and their functions
  connect (ui->loadButton, SIGNAL(clicked ()), this, SLOT(loadButtonPressed ()));
  connect (ui->saveButton, SIGNAL(clicked ()), this, SLOT(saveButtonPressed ()));
  connect (ui->streamButton, SIGNAL(clicked()) , this, SLOT(streamButtonPressed()));

  // Connect Menu items to their functions
  connect (ui->actionOpen_PCD_file, SIGNAL(triggered()), this, SLOT(loadButtonPressed()));
  connect (ui->actionSave_As, SIGNAL(triggered()), this, SLOT(saveButtonPressed()));
  connect (ui->actionOpen_Kinect_Stream, SIGNAL(triggered()), this, SLOT(streamButtonPressed()));
  connect (ui->actionExit, SIGNAL(triggered()), this, SLOT(exitApplication()));

  // Connect Items from cloud listWidget
  connect (ui->listWidget, SIGNAL(itemPressed(QListWidgetItem*)), this, SLOT(toggleCloudSelection(QListWidgetItem*)));

  //viewer_->addPointCloud (nullptr, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}

void
PCLViewer::loadButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "%UserProfile%", tr ("Point cloud data (*.pcd *.ply *.txt *.dfr)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else if(filename.endsWith(".ply", Qt::CaseInsensitive))
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);
  else if(filename.endsWith(".dfr", Qt::CaseInsensitive))
    return_status = loadDFRFile(filename.toStdString(), *cloud_tmp);
  else if(filename.endsWith(".txt", Qt::CaseInsensitive))
    return_status = loadTXTFile(filename.toStdString(), *cloud_tmp);
  else
    return_status = 1;

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  PointCloudT::Ptr cloud_ (new PointCloudT);
  if (cloud_tmp->is_dense) {
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
    pointClouds.insert(StringCloudPair(filename, cloud_));
  } else {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }

  viewer_->addPointCloud (cloud_, filename.toStdString ());
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
  cout << "test1" << endl;
  QListWidgetItem* item = new QListWidgetItem(filename, ui->listWidget);
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
  item->setCheckState(Qt::Checked);
  cout << "test2" << endl,
  ui->listWidget->addItem(item);
}

void
PCLViewer::saveButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "%UserProfile%", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = 0;//pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = 0;//pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    //return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}

void PCLViewer::streamButtonPressed() {
    isStreaming = !isStreaming;
    if(isStreaming) ui->streamButton->setText("Stop Live Stream");
    if(!isStreaming) ui->streamButton->setText("Start Live Stream");
    receiver.receiveFrames();
}

void PCLViewer::toggleCloudSelection(QListWidgetItem* item) {
    PointCloudT::Ptr cloud_ = pointClouds.find(item->text())->second;
    if(item->checkState() == Qt::Unchecked) {
        item->setCheckState(Qt::Checked);
        viewer_->addPointCloud (cloud_, item->text().toStdString());
    } else {
        item->setCheckState(Qt::Unchecked);
        viewer_->removePointCloud(item->text().toStdString());
    }
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();
}

int PCLViewer::loadDFRFile(std::string fileName, PointCloudT& cloud_) {
    string line;
    std::vector<std::string> splitLine;
    float x;
    float y;
    float z;
    ifstream clouddata;
    clouddata.open (fileName, ios::in);

    if(clouddata.is_open()) {
        while(getline(clouddata,line)) {
            if(line.find("INF") == string::npos) {
                boost::split(splitLine, line, boost::is_any_of(";"));
                x = stof(splitLine[0]);
                y = stof(splitLine[1]);
                z = stof(splitLine[2]);
                cloud_.push_back(pcl::PointXYZ(x * -200,y * -200,z * 200));
                splitLine.clear();
            }
        }
        clouddata.close();
        return 0;
    }
    return 1;
}

int PCLViewer::loadTXTFile(string fileName, PointCloudT& cloud_) {
    float maxZ = 0;
    float cubeSize = 500;
    string line;
    ifstream clouddata;
    clouddata.open (fileName, ios::in);
    std::vector<float> depthData;

    if (clouddata.is_open()) {

        int rowCount = 0;
        int colCount = 0;

        while ( getline (clouddata,line) ) {
            float z = std::stof(line);
            depthData.push_back(z);
            if(maxZ <= z) maxZ = z;
        }
        std::cout << maxZ << std::endl;
        for each (float z in depthData) {
            float x = colCount * cubeSize / cDepthWidth;
            float y = rowCount * cubeSize / cDepthHeight;
            float newZ = z * cubeSize / maxZ;

            colCount++;

            if (colCount == cDepthWidth) {
                colCount = 0;
                rowCount++;
            }

            if(z != 0) cloud_.push_back(pcl::PointXYZ(x,y,z));
        }
        clouddata.close();
        return 0;
    }
    return 1;
}

void PCLViewer::exitApplication() {
    qApp->exit();
}
