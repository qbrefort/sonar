#ifndef REPERE_H
#define REPERE_H

#include <QObject>
#include <QtCore>
#include <QtGui>

#include <cmath>
using namespace std;
#include <iostream>
#include <vector>


class repere : public QObject
{
    Q_OBJECT
public:
    QGraphicsScene* Scene;
    double xmin,xmax,ymin,ymax;

    explicit repere(QObject *parent = 0);
    repere(QObject *parent = 0,QGraphicsView* G=0,double xmin=0,double xmax=10,double ymin=0,double ymax=10);
    double xToPix(double x);
    double yToPix(double y);
    void Clean();
    void DrawArrow(double x1,double y1,double dx,double dy,double r=0.1, QPen pen1=QPen(Qt::black));
    void DrawAxis(QPen pen1=QPen(Qt::black));
    void DrawBox(double xmin,double xmax,double ymin,double ymax, QPen pen1, QBrush brush1);
    void DrawData(vector<double> T, vector<double> Ymin, vector<double> Ymax,QPen pen1=QPen(Qt::red));
    void DrawEllipse(double cx, double cy, double r, QPen pen1, QBrush brush1);
    void DrawLine(double x1,double y1,double x2,double y2, QPen pen1);
    void DrawPolygone(double x,double y,double theta,vector<double> X, vector<double> Y, QPen pen1, QBrush brush1);
    void DrawRobot(double x,double y,double theta,double s=1.0);
    void DrawText(QString s, int n);
    void Save(QString nom);





signals:

public slots:

};

#endif // REPERE_H
