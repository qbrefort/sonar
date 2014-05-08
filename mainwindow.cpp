#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ibex.h"
#include "sivia.h"

double epsilon;

MainWindow::MainWindow(QWidget *parent) :  QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
}

void MainWindow::Init() {
    epsilon=ui->EpsilonSpinBox->value();
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_ButtonStart_clicked()
{

    Init();

    // Build the frame
    double xmin=-10;
    double xmax=10;
    double ymin=-10;
    double ymax=10;

    // run SIVIA
    double th1=0;
    double ry=9;
    for(int i=0;i<20 ;i++){
        repere* R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
        Sivia sivia(*R,epsilon,th1,ry);
        th1+=3.14/4;
        ry-=1;
        QTime dieTime= QTime::currentTime().addMSecs(100);
        while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    }

}
