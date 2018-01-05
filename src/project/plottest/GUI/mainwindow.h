#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"
#include "include/rigidbody.h"
#include "qgraphicsview.h"
#include"stdio.h"
#include "math.h"
#include <QApplication>
#include<QTimer>
#include "ui_mainwindow.h"
//#include "sensor_msgs/BatteryState"
#include "sensor_msgs/BatteryState.h"
#include "mavros_msgs/State.h"
#include "QString"
#include "string.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void plot_loop();
    void state_loop();
    void on_pushButton_clicked();

private:
    QCPCurveData plot;
    QVector<QCPCurveData> curve_data;
    std::vector<QCPCurve*> curve_list;
    std::vector<QCPCurve*> path_curve_list;
    QVector<QCPCurveData> path_data;
    void get_mavros_state(const mavros_msgs::State::ConstPtr& state);
    void get_battery_state(const sensor_msgs::BatteryState::ConstPtr& state);
    sensor_msgs::BatteryState  battery_state;
    mavros_msgs::State  mavros_state;
    QTimer *timer;
    QTimer *state_timer;
    //QVector<double> x,y;
    std::vector<QVector<double>> x,y;
    Ui::MainWindow *ui;
    int count=0;
    std::vector<std::string> rigidbody_list;
    std::vector<rigidbody*> rigidbody_group;
    ros::NodeHandle nh;
    ros::Subscriber battery_state_sub;
    ros::Subscriber mavros_state_sub;
    ros::master::V_TopicInfo topic_list;
    bool getRosTopics(ros::master::V_TopicInfo& topics);
};

#endif // MAINWINDOW_H
