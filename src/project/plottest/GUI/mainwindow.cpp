
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)

      ,x(10),y(10),curve_data(100)
{
     ui->setupUi(this);
     this->setWindowTitle("sis-2017-project");
    ui->progressBar->setTextVisible(true);
    battery_state_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",10,&MainWindow::get_battery_state,this);
    mavros_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,&MainWindow::get_mavros_state,this);
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
    state_timer = new QTimer(this);
    connect(state_timer, SIGNAL(timeout()),this, SLOT(state_loop()));
    state_timer->start(10);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this, SLOT(plot_loop()));
    //timer->start(10);




}

MainWindow::~MainWindow()
{
    delete ui;
  delete timer;

}
void MainWindow::state_loop(){
  ros::spinOnce();
  float percentagge;

  QPalette palette = ui->label->palette();
  // palette.setColor(ui->label->backgroundRole(), Qt::yellow);



  percentagge =  battery_state.percentage * 100;
  ui->progressBar->setValue(percentagge);
  if(percentagge > 80){
    ui->label->setText(QString("Battery Well"));
    palette.setColor(ui->label->foregroundRole(), Qt::green);
    ui->label->setPalette(palette);
  }else if(percentagge > 60){
     palette.setColor(ui->label->foregroundRole(), Qt::green);
    ui->label->setText(QString("Battery Medium"));
    ui->label->setPalette(palette);
  }else if(percentagge > 40){
    ui->label->setText(QString("Battery Warnning"));
     palette.setColor(ui->label->foregroundRole(), Qt::darkYellow);
     ui->label->setPalette(palette);
  }else if(percentagge < 20){
   palette.setColor(ui->label->foregroundRole(), Qt::red);
    ui->label->setText(QString("Battery Fatal"));
    ui->label->setPalette(palette);
  }



  std::string str = mavros_state.mode;
  ui->checkBox->setChecked(mavros_state.armed);
  ui->checkBox_2->setChecked(mavros_state.connected);

  ui->label_2->setText(QString("Mode : %1").arg(str.c_str()));

}

void MainWindow::plot_loop()
{

  if(ros::ok())
  {
   // ros::spinOnce();
    //ros::spinOnce();
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

void MainWindow::get_mavros_state(const mavros_msgs::State::ConstPtr& state){
  mavros_state = * state;
}

void MainWindow::get_battery_state(const sensor_msgs::BatteryState::ConstPtr& state){

     battery_state = *state;

}
