#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ibex.h"
#include "sivia.h"

double epsilon;
bool motion,artifact;
sivia_struct *par = new sivia_struct();

MainWindow::MainWindow(QWidget *parent) :  QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->xb1_SpinBox->setValue(-5);
    ui->yb1_SpinBox->setValue(-5);
    ui->xb2_SpinBox->setValue(-3);
    ui->yb2_SpinBox->setValue(6);
    ui->xb3_SpinBox->setValue(4);
    ui->yb3_SpinBox->setValue(2);
    ui->eiSpinBox->setValue(0.1);
    motion = ui->checkBox->isChecked();
    artifact = ui->artifact_checkBox->isChecked();
}

void MainWindow::Init() {
    epsilon=ui->EpsilonSpinBox->value();
    par->kp = ui->KpSpinBox->value();
    par->epsilon = epsilon;
    par->isinside=0;par->isinside1=0;par->isinside2=0;par->isinside3=0;
    par->sonar_radius = 7;
    par->sonar_arc = M_PI/8;
    par->sonar_speed = M_PI/8;
    par->th = new double[3];
    par->wr = 1;
    par->lr = 4;
    par->thick = 0.10;
    par->ya = -12;
    par->xa = -2;
    par->ra = 0.3;
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
    double *xrpos,*yrpos;

    par->th[0]= -70 * M_PI/180;
    par->th[1]= -120 * M_PI/180;
    par->th[2]= -90 * M_PI/180;
    par->yr=-0-par->lr/2;
    par->xr=0;
    if (motion){
        par->yr=-10-par->lr/2;
        par->xr=0;
    }
    // run SIVIA
    xrpos = new double[30];
    yrpos = new double[30];
    double areax[30];
    double areay[30];
    for(int i=0;i<30 ;i++){
        repere* R = new repere(this,ui->graphicsView,xmin,xmax,ymin,ymax);
        Sivia sivia(*R,par);
        if(par->state.compare("found")!=0){
            for(int i=0;i<3;i++){
                par->th[i]+=par->sonar_speed;
            }
        }
        if(artifact){
            //If we want an artifact, make it move near the robot.
            par->ya +=1;
            par->xa += 0;//sin(par->xa);
        }
        else{
            //Else put it far away from the action.
            par->ya = -12;
            par->xa = -2;
        }

        if(motion){
            par->yr+=1;
            par->xr+=0.1*sin(0.5*par->yr);
        }

        try{
            xrpos[i] = par->xin;
            yrpos[i] = par->yin;
            areax[i] = par->areax;
            areay[i] = par->areay;
            double box=0.2;
            for (int j=1;j<i+1;j++){
//                R->DrawBox(xrpos[j]-box,xrpos[j]+box,yrpos[j]-box,yrpos[j]+box,QPen(Qt::darkMagenta),QBrush(Qt::NoBrush));
                R->DrawBox(xrpos[j]-areax[j]/2,xrpos[j]+areax[j]/2,yrpos[j]-areay[j]/2,yrpos[j]+areay[j]/2,QPen(Qt::darkMagenta),QBrush(Qt::NoBrush));
            }
        }
        catch(const char* msg){
            double error=1;
        }

        QTime dieTime= QTime::currentTime().addMSecs(500);
        while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        if (par->isinside1==1){
//            cout<<"found1:"<<par->xin<<";"<<par->yin<<endl;
            par->isinside1=0;
        }
        if (par->isinside2==1){
//            cout<<"found2:"<<par->xin<<";"<<par->yin<<endl;
            par->isinside2=0;
        }
        if (par->isinside3==1){
//            cout<<"found3:"<<par->xin<<";"<<par->yin<<endl;
            par->isinside3=0;
        }

    }

}

void MainWindow::on_KpSpinBox_valueChanged(double arg1)
{
    par->kp = arg1;
}

void MainWindow::on_xb1_SpinBox_valueChanged(double arg1)
{
    par->xb1 = arg1;
}

void MainWindow::on_yb1_SpinBox_valueChanged(double arg1)
{
    par->yb1 = arg1;
}

void MainWindow::on_xb2_SpinBox_valueChanged(double arg1)
{
    par->xb2 = arg1;
}

void MainWindow::on_yb2_SpinBox_valueChanged(double arg1)
{
    par->yb2 = arg1;
}

void MainWindow::on_xb3_SpinBox_valueChanged(double arg1)
{
    par->xb3 = arg1;
}

void MainWindow::on_yb3_SpinBox_valueChanged(double arg1)
{
    par->yb3 = arg1;
}

void MainWindow::on_checkBox_toggled(bool checked)
{
    motion = checked;
}

void MainWindow::on_eiSpinBox_valueChanged(double arg1)
{
    par->ei = arg1;
}

void MainWindow::on_artifact_checkBox_toggled(bool checked)
{
    artifact = checked;
}

void MainWindow::on_arc_EpsilonSpinBox_valueChanged(double arg1)
{
    par->sonar_arc = M_PI/arg1;
    par->sonar_speed = M_PI/arg1;
}
