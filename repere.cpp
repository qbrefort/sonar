#include "repere.h"
#include <QDebug>


repere::repere(QObject *parent) :
    QObject(parent)
{
}
//--------------------------------------------------------------------------------------------------
repere::repere(QObject *parent,QGraphicsView* G,double xmin1, double xmax1,double ymin1, double ymax1) :
    QObject(parent)
{    Scene=new QGraphicsScene(this);
     G->setScene(Scene);
     xmin=xmin1; xmax=xmax1; ymin=ymin1; ymax=ymax1;
     Scene->setSceneRect(0,0,G->geometry().width()-3,G->geometry().height()-3);
     DrawBox(xmin,xmax,ymin,ymax,QPen(Qt::black),QBrush(Qt::NoBrush));
}
//--------------------------------------------------------------------------------------------------
double repere::xToPix(double x)
{   double echx = Scene->width()/(xmax-xmin);
    return (x-xmin)*echx;
}
//--------------------------------------------------------------------------------------------------
double repere::yToPix(double y)
{
    double echy = Scene->height()/(ymax-ymin);
    return Scene->height()-(y-ymin)*echy;
}
//--------------------------------------------------------------------------------------------------
void repere::Clean()
{
    Scene->clear();
}
//--------------------------------------------------------------------------------------------------
void repere::DrawBox(double xmin,double xmax,double ymin,double ymax, QPen pen1, QBrush brush1)
{       QPolygonF box1;
        box1 << QPointF(xToPix(xmin),yToPix(ymin))<< QPointF(xToPix(xmax),yToPix(ymin))
             << QPointF(xToPix(xmax),yToPix(ymax))<< QPointF(xToPix(xmin),yToPix(ymax));
        QGraphicsPolygonItem *P;
        pen1.setWidth(2);
        P=Scene->addPolygon(box1,pen1,brush1);
        Scene->update();
 }
//--------------------------------------------------------------------------------------------------
void repere::DrawArrow(double x1,double y1,double dx,double dy, double r, QPen pen1)
{       double x2=x1+dx;
        double y2=y1+dy;
        double a=3.14-(3.14/4);
        double pixx1=xToPix(x1);
        double pixy1=yToPix(y1);
        double pixx2=xToPix(x2);
        double pixy2=yToPix(y2);
        double pixdx=pixx2-pixx1;
        double pixdy=pixy2-pixy1;
        double px=pixx2+r*(cos(a)*pixdx-sin(a)*pixdy);    // cot� de pointe
        double py=pixy2+r*(sin(a)*pixdx+cos(a)*pixdy);
        double qx=pixx2+r*(cos(-a)*pixdx-sin(-a)*pixdy);  // autre cot� de pointe
        double qy=pixy2+r*(sin(-a)*pixdx+cos(-a)*pixdy);
        Scene->addLine(pixx1,pixy1,pixx2,pixy2,pen1);
        Scene->addLine(pixx2,pixy2,px,py,pen1);
        Scene->addLine(pixx2,pixy2,qx,qy,pen1);
 }
//--------------------------------------------------------------------------------------------------
void repere::DrawAxis(QPen pen1)
{   double x1=xmin;
    double y1=ymin;
    double dx=xmax-xmin;
    double dy=ymax-ymin;
    DrawArrow(x1,0,dx,0,0.02, pen1);
    DrawArrow(0,y1,0,dy,0.02, pen1);
}
//--------------------------------------------------------------------------------------------------
void repere::DrawData(vector<double> T, vector<double> Ymin, vector<double> Ymax,QPen pen1)
{       pen1.setWidth(3);
        for (uint k=0;k<T.size();k++)
           DrawLine(T[k],Ymin[k],T[k],Ymax[k], pen1);
}
//--------------------------------------------------------------------------------------------------
void repere::DrawEllipse(double cx, double cy, double r, QPen pen1, QBrush brush1)
{
     Scene->addEllipse(xToPix(cx-r),yToPix(cy+r),xToPix(cx+r)-xToPix(cx-r),yToPix(cy-r)-yToPix(cy+r),pen1,brush1);
}
//--------------------------------------------------------------------------------------------------
void repere::DrawLine(double x1,double y1,double x2,double y2, QPen pen1)
{       Scene->addLine(xToPix(x1),yToPix(y1),xToPix(x2),yToPix(y2),pen1);
}
//--------------------------------------------------------------------------------------------------
void repere::DrawPolygone(double x,double y,double theta,vector<double> X, vector<double> Y, QPen pen1, QBrush brush1)
{       QPolygonF poly1;
        for (uint k=0;k<X.size();k++)
        {
            double x1=x+cos(theta)*X[k]-sin(theta)*Y[k];
            double y1=y+sin(theta)*X[k]+cos(theta)*Y[k];
            poly1 << QPointF(xToPix(x1),yToPix(y1));
        }
        QGraphicsPolygonItem *P;
        P=Scene->addPolygon(poly1,pen1,brush1);
 }
//--------------------------------------------------------------------------------------------------
void repere::DrawRobot(double x,double y,double theta, double s)
{   vector<double> X,Y;
    X.push_back(0*s); Y.push_back(-1*s);
    X.push_back(3*s); Y.push_back(0*s);
    X.push_back(0*s); Y.push_back(1*s);
    DrawPolygone(x,y,theta,X,Y,QPen(Qt::blue),QBrush(Qt::NoBrush));
}
//--------------------------------------------------------------------------------------------------
void repere::DrawText(QString s, int n)
{       for (int k=1;k<=n;k++)
        s="\r\n"+s;
        //Pour la syntaxe:  int a; double p; s=QString("a=%1,\r\n p=%2").arg(k).arg(p);
        Scene->addText(s);
}
//--------------------------------------------------------------------------------------------------
void repere::Save(QString nom)
{
    QImage *image = new QImage(QSize(1000,1000),QImage::Format_ARGB32);
    image->fill(QColor(Qt::white).rgb());
    QPainter *pngPainter = new QPainter(image);
    pngPainter->setRenderHint(QPainter::Antialiasing);
    Scene->render(pngPainter);
    pngPainter->end();
    image->save(nom+".png","PNG",100);
}
//--------------------------------------------------------------------------------------------------

