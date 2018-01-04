
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
//    r1("/vrpn_client_node/RigidBody1/pose"),
//    x(1),y(1)
      ,x(10),y(10),curve_data(100)
{
     ui->setupUi(this);
     this->setWindowTitle("plottest");

    drone_curve =  new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis);
    getRosTopics(topic_list);
    for(ros::master::V_TopicInfo::iterator it=topic_list.begin();it!=topic_list.end();it++)
    {
      ros::master::TopicInfo& info = *it;
      if(info.name.compare(0,5,"/vrpn")==0)  //record rigidbodypose
      {
        rigidbody_list.push_back(info.name);
      }
    }
    //add rigidbody
    for(int i=0;i<rigidbody_list.size();i++)
    {
      rigidbody_group.push_back(new rigidbody(rigidbody_list[i]));
      curve_list.push_back(new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis));
      QCPCurveData *path_data;
     // path_data_list.push_back(*path_data);
    }

    ui->customplot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


    ui->customplot->xAxis->setLabel("x");
    ui->customplot->yAxis->setLabel("y");

    // set axes ranges, so we see all data:
    ui->customplot->xAxis->setRange(-3, 3);
    ui->customplot->yAxis->setRange(-3, 3);

    for(int i=0;i<50;i++)
    {
      ros::spinOnce();
    }

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this, SLOT(plot_loop()));
    timer->start(10);
    //looprate

    drone_curve->setPen(QPen(Qt::red));



}

MainWindow::~MainWindow()
{
    delete ui;
  delete timer;

}
void MainWindow::plot_loop()
{

  if(ros::ok())
  {

     ros::spinOnce();

     //plot rigid body curve
    for(unsigned int j = 0; j< rigidbody_group.size();j++){

      for (unsigned int i=0; i<100; ++i)
      {
        curve_data[i] = QCPCurveData(i,  rigidbody_group[j]->data.x+0.15*cos(0.1*i), rigidbody_group[j]->data.y+0.15*sin(0.1*i));
      }
        curve_list[j]->setPen(QPen(Qt::red));
        curve_list[j]->data()->set(curve_data,true);




    }

    ui->customplot->xAxis->setLabel("x");
    ui->customplot->yAxis->setLabel("y");
    ui->customplot->replot();
  }




  else
  {
     QApplication::quit();
  }
}

bool MainWindow::getRosTopics(ros::master::V_TopicInfo& topics){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    std::string str;
    if (!ros::master::execute("getTopicTypes", args, result, payload, true)){
        std::cout << "Failed!" << std::endl;
        return false;
    }

    topics.clear();
    for (int i = 0; i < payload.size(); ++i)
        topics.push_back(ros::master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
    return true;
}
