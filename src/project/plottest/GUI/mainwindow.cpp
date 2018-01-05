
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)

      ,x(10),y(10),curve_data(100)
{
     ui->setupUi(this);
     this->setWindowTitle("sis-2017-project");


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
      path_curve_list.push_back(new QCPCurve(ui->customplot->xAxis, ui->customplot->yAxis));
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
    //timer->start(10);




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
      QPen mypen;
      mypen.setWidth(2);
      if(j==1){
        mypen.setColor(Qt::blue);
      }else{
        mypen.setColor(Qt::red);
      }
        curve_list[j]->setPen(mypen);
        curve_list[j]->data()->set(curve_data,true);
    }

    for(unsigned int j=0;j< rigidbody_group.size();j++){
      path_data.resize(rigidbody_group[j]->record_data.size());

      for(unsigned int i=0;i<rigidbody_group[j]->record_data.size();i++){

        path_data[i] = QCPCurveData(i,rigidbody_group[j]->record_data[i].x,rigidbody_group[j]->record_data[i].y );

      }
      QPen mypen;
      mypen.setWidth(2);
      if(j==1){
        mypen.setColor(Qt::blue);
      }else{
        mypen.setColor(Qt::red);
      }

      path_curve_list[j]->setPen(mypen);
      path_curve_list[j]->data()->set(path_data,true);

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

void MainWindow::on_pushButton_clicked()
{
  if(timer->isActive()){
    timer->stop();
    ui->pushButton->setText("Stop");
  }else{
    ui->pushButton->setText("Start");
     timer->start(10);
  }
}
