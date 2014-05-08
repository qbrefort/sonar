#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ibex.h"
#include "sivia.h"

double epsilon;
sivia_struct *par = new sivia_struct();

MainWindow::MainWindow(QWidget *parent) :  QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
}

void MainWindow::Init() {
    epsilon=ui->EpsilonSpinBox->value();
    par->epsilon = epsilon;
    par->xb1=-4;
    par->xb2=-6;
    par->xb3=5;
    par->yb1=5;
    par->yb2=-7;
    par->yb3=0;
    par->isinside=0;par->isinside1=0;par->isinside2=0;par->isinside3=0;
    par->sonar_radius = 7;
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

    par->th1= 1 * M_PI/180;
    par->th2= 20 * M_PI/180;
    par->th2= -40 * M_PI/180;
    par->yr=5;

    // run SIVIA

    for(int i=0;i<20 ;i++){
        repere* R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
        Sivia sivia(*R,par);
        if(par->state.compare("found")!=0){
            par->th1+=3.14/4;
            par->th2+=3.14/4;
            par->th3+=3.14/4;
        }
        else cout<<"fouund"<<endl;

        par->yr-=1;
        QTime dieTime= QTime::currentTime().addMSecs(300);
        while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        if (par->isinside1==1){
            cout<<"found1:"<<par->xin<<";"<<par->yin<<endl;
            par->isinside1=0;
        }
        if (par->isinside2==1){
            cout<<"found2:"<<par->xin<<";"<<par->yin<<endl;
            par->isinside2=0;
        }
        if (par->isinside3==1){
            cout<<"found3:"<<par->xin<<";"<<par->yin<<endl;
            par->isinside3=0;
        }

    }

}
