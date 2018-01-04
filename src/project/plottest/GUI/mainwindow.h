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

private:

    QVector<QCPCurveData> curve_data;
    QCPCurve *drone_curve;
    std::vector<QCPCurve*> curve_list;
    std::vector<QCPCurve*> path_curve_list;
    QVector<QCPCurveData> path_data_list;
    std::vector<geometry_msgs::PoseStamped> path_data;

//    QVector<> path_data;
//    std::vector<QVector<double>> path_data_list;

    QTimer *timer;
    //QVector<double> x,y;
    std::vector<QVector<double>> x,y;
    Ui::MainWindow *ui;
    int count=0;
    std::vector<std::string> rigidbody_list;
    std::vector<rigidbody*> rigidbody_group;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::master::V_TopicInfo topic_list;
    bool getRosTopics(ros::master::V_TopicInfo& topics);
};

#endif // MAINWINDOW_H
