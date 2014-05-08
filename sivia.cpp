#include "sivia.h"
#include "math.h"


float get_angle(double p1x,double p1y,double p2x,double p2y){
    return atan2(p1y - p2y, p1x - p2x);
}

void robot_position_estimator(int ninbox, struct sivia_struct *par){
    double tx[ninbox],ty[ninbox];
    double xin=0,yin=0;
    //cout<<"next"<<ninbox<<endl;
    for(int i=0;i<ninbox;i++){
        IntervalVector cur = (par->vin.back());
        //cout<<cur<<endl;
        Interval xcur=cur[0];
        Interval ycur=cur[1];
        tx[i]=xcur.mid();
        ty[i]=ycur.mid();
        par->vin.pop_back();
    }

    for(int i=0;i<ninbox;i++){
        xin += tx[i];
        yin += ty[i];
    }
    xin/=double(ninbox);
    yin/=double(ninbox);

    par->xin = xin;
    par->yin = yin;

}

void Sivia::contract_and_draw(Ctc& c, IntervalVector& X,IntervalVector& viinside,int inside,struct sivia_struct *par,int& nbox, const QColor & pencolor, const QColor & brushcolor) {
    IntervalVector X0=X;       // get a copy
    try {
        c.contract(X);
        if (X==X0) return;     // nothing contracted.
        IntervalVector* rest;
        int n=X0.diff(X,rest); // calculate the set difference
        for (int i=0; i<n; i++) {     // display the boxes
            if(inside!=0){   
                R.DrawBox(rest[i][0].lb(),rest[i][0].ub(), rest[i][1].lb(),rest[i][1].ub(),QPen(pencolor),QBrush(brushcolor));
                if (inside==1){}
                if (inside==2){}
                if (inside==3){}
                par->isinside = 1;
                viinside = rest[i];
                par->vin.push_back(viinside);
                nbox++;
                par->isinside=1;

            }

        }
        delete[] rest;
    } catch(EmptyBoxException&) {
        if(inside!=0)
        R.DrawBox(X0[0].lb(),X0[0].ub(),X0[1].lb(),X0[1].ub(),QPen(pencolor),QBrush(brushcolor));
    }
}


Sivia::Sivia(repere& R, struct sivia_struct *par) : R(R) {

    // Create the function we want to apply SIVIA on.
    Variable x,y;
    double xb=par->xb1,yb=par->yb1;

    double arc=3.14/2;

    double r = pow(par->sonar_radius,2);
    double th1 = par->th1;
    double th2=th1+arc;
    double th21=th1 -50;
    double th22=th21 + arc;
    double th31=th1 + 70;
    double th32=th31 + arc;
    double e=0.5;
    double epsilon = par->epsilon;

    // First SONAR
    Function f(x,y,sqr(x-xb)+sqr(y-yb));

    NumConstraint c1(x,y,f(x,y)<=r+e);
    NumConstraint c2(x,y,f(x,y)>=0);
    NumConstraint c3(x,y,f(x,y)>r+e);
    NumConstraint c4(x,y,f(x,y)<0);


    double sign1,sign2;
    if(cos(th1)>0) sign1=1;
    else sign1=-1;
    if(cos(th2)<0) sign2=1;
    else sign2=-1;


    NumConstraint cth11(x,y,sign1*(y-yb-((sin(th1))/(cos(th1)))*(x-xb))<0);
    NumConstraint cth12(x,y,sign1*(y-yb-((sin(th1))/(cos(th1)))*(x-xb))>0);
    NumConstraint cth21(x,y,sign2*(y-yb-((sin(th2))/(cos(th2)))*(x-xb))<0);
    NumConstraint cth22(x,y,sign2*(y-yb-((sin(th2))/(cos(th2)))*(x-xb))>0);


//     Create contractors with respect to each
//     of the previous constraints.
    CtcFwdBwd out1(c1);
    CtcFwdBwd out2(c2);
    CtcFwdBwd in1(c3);
    CtcFwdBwd in2(c4);


    CtcFwdBwd outth1(cth12);
    CtcFwdBwd inth1(cth11);
    CtcFwdBwd inth2(cth21);
    CtcFwdBwd outth2(cth22);

//    CtcIn inside(f,Interval(-1,1));
//    CtcNotIn outside(f,Interval(-1,1));
    // Create a contractor that removes all the points
    // that do not satisfy either f(x,y)<=2 or f(x,y)>=0.
    // These points are "outside" of the solution set.
    CtcCompo outside1(out1,out2,outth1,outth2);

    // Create a contractor that removes all the points
    // that do not satisfy both f(x,y)>2 or f(x,y)<0.
    // These points are "inside" the solution set.
    CtcUnion inside11(in1,in2,inth1);
    CtcUnion inside1(inside11,inth2);


    // Second SONAR
    double xb2=par->xb2,yb2=par->yb2;

    Function f2(x,y,sqr(x-xb2)+sqr(y-yb2));
    NumConstraint c21(x,y,f2(x,y)<=r+e);
    NumConstraint c22(x,y,f2(x,y)>=0);
    NumConstraint c23(x,y,f2(x,y)>r+e);
    NumConstraint c24(x,y,f2(x,y)<0);


    double sign21,sign22;
    if(cos(th21)>0) sign21=-1;
    else sign21=1;
    if(cos(th22)<0) sign22=1;
    else sign22=-1;


    NumConstraint cth211(x,y,sign21*(y-yb2-((sin(th21))/(cos(th21)))*(x-xb2))<0);
    NumConstraint cth212(x,y,sign21*(y-yb2-((sin(th21))/(cos(th21)))*(x-xb2))>0);
    NumConstraint cth221(x,y,sign22*(y-yb2-((sin(th22))/(cos(th22)))*(x-xb2))<0);
    NumConstraint cth222(x,y,sign22*(y-yb2-((sin(th22))/(cos(th22)))*(x-xb2))>0);

//     Create contractors with respect to each
//     of the previous constraints.
    CtcFwdBwd out21(c21);
    CtcFwdBwd out22(c22);
    CtcFwdBwd in21(c23);
    CtcFwdBwd in22(c24);


    CtcFwdBwd outth21(cth211);
    CtcFwdBwd inth21(cth212);
    CtcFwdBwd inth22(cth221);
    CtcFwdBwd outth22(cth222);

//    CtcIn inside(f,Interval(-1,1));
//    CtcNotIn outside(f,Interval(-1,1));
    // Create a contractor that removes all the points
    // that do not satisfy either f(x,y)<=2 or f(x,y)>=0.
    // These points are "outside" of the solution set.
    CtcCompo outside2(out21,out22,outth21,outth22);

    // Create a contractor that removes all the points
    // that do not satisfy both f(x,y)>2 or f(x,y)<0.
    // These points are "inside" the solution set.
    CtcUnion inside21(in21,in22,inth21);
    CtcUnion inside2(inside21,inth22);



    //Third SONAR

    double xb3=par->xb3,yb3=par->yb3;

    Function f3(x,y,sqr(x-xb3)+sqr(y-yb3));
    NumConstraint c31(x,y,f3(x,y)<=r+e);
    NumConstraint c32(x,y,f3(x,y)>=0);
    NumConstraint c33(x,y,f3(x,y)>r+e);
    NumConstraint c34(x,y,f3(x,y)<0);


    double sign31,sign32;
    if(cos(th31)>0) sign31=-1;
    else sign31=1;
    if(cos(th32)<0) sign32=1;
    else sign32=-1;


    NumConstraint cth311(x,y,sign31*(y-yb3-((sin(th31))/(cos(th31)))*(x-xb3))<0);
    NumConstraint cth312(x,y,sign31*(y-yb3-((sin(th31))/(cos(th31)))*(x-xb3))>0);
    NumConstraint cth321(x,y,sign32*(y-yb3-((sin(th32))/(cos(th32)))*(x-xb3))<0);
    NumConstraint cth322(x,y,sign32*(y-yb3-((sin(th32))/(cos(th32)))*(x-xb3))>0);

//     Create contractors with respect to each
//     of the previous constraints.
    CtcFwdBwd out31(c31);
    CtcFwdBwd out32(c32);
    CtcFwdBwd in31(c33);
    CtcFwdBwd in32(c34);


    CtcFwdBwd outth31(cth311);
    CtcFwdBwd inth31(cth312);
    CtcFwdBwd inth32(cth321);
    CtcFwdBwd outth32(cth322);

//    CtcIn inside(f,Interval(-1,1));
//    CtcNotIn outside(f,Interval(-1,1));
    // Create a contractor that removes all the points
    // that do not satisfy either f(x,y)<=2 or f(x,y)>=0.
    // These points are "outside" of the solution set.
    CtcCompo outside3(out31,out32,outth31,outth32);

    // Create a contractor that removes all the points
    // that do not satisfy both f(x,y)>2 or f(x,y)<0.
    // These points are "inside" the solution set.
    CtcUnion inside31(in31,in32,inth31);
    CtcUnion inside3(inside31,inth32);

    //CtcQInter inter(inside,1);


    //Robot MODELISATION

    double xr = par->xr; //robot position x
    double yr = par->yr; //robot position y

    double wr = 1; //robot width
    double lr = 4; //robot length

    NumConstraint inrx1(x,y,x>xr+wr/2);
    NumConstraint outrx1(x,y,x<xr+wr/2);
    NumConstraint inrx2(x,y,x<x-wr/2);
    NumConstraint outrx2(x,y,x>xr-wr/2);
    NumConstraint inry1(x,y,y<yr-lr/2);
    NumConstraint outry1(x,y,y>yr-lr/2);
    NumConstraint inry2(x,y,y>yr+lr/2);
    NumConstraint outry2(x,y,y<yr+lr/2);

    CtcFwdBwd incrx1(inrx1);
    CtcFwdBwd incrx2(inrx2);
    CtcFwdBwd incry1(inry1);
    CtcFwdBwd incry2(inry2);

    CtcFwdBwd outcrx1(outrx1);
    CtcFwdBwd outcrx2(outrx2);
    CtcFwdBwd outcry1(outry1);
    CtcFwdBwd outcry2(outry2);

    CtcCompo outy(outcry1,outcry2,outcrx2);
    CtcUnion iny(incry1,incry2,incrx2);

    CtcUnion inrtemp(incrx1,incrx2,incry1);
    CtcUnion inr(inrtemp,incry2);
    CtcCompo outrtemp(outcrx1,outcrx2,outcry1);
    CtcCompo outr(outrtemp,outcry2);

    CtcCompo insideb(inside1,inside2,inr);
    CtcUnion outsideb(outside1,outside2);
    CtcCompo outside(outsideb,outr);
    CtcUnion inside(insideb,inr);

    CtcUnion inside1r(inside1,inr);
    CtcCompo outside1r(outside1,outr);

    CtcUnion inside2r(inside2,inr);
    CtcCompo outside2r(outside2,outr);

    CtcUnion inside3r(inside3,inr);
    CtcCompo outside3r(outside3,outr);

    // Build the initial box.
    IntervalVector box(2);
    box[0]=Interval(-10,10);
    box[1]=Interval(-10,10);
    par->vin.clear();
    // Build the way boxes will be bisected.
    // "LargestFirst" means that the dimension bisected
    // is always the largest one.
    int nbox1=0;
    LargestFirst lf;
    IntervalVector viinside1(2);
    stack<IntervalVector> s;
    s.push(box);
    cout<<"new"<<endl;
    while (!s.empty()) {
        IntervalVector box=s.top();
        s.pop();
            contract_and_draw(inside1r,box,viinside1,1,par,nbox1,Qt::magenta,Qt::red);
            if (box.is_empty()) { continue; }

            contract_and_draw(outside1r,box,viinside1,0,par,nbox1,Qt::darkBlue,Qt::cyan);
            if (box.is_empty()) { continue; }

            if (box.max_diam()<epsilon) {
                R.DrawBox(box[0].lb(),box[0].ub(),box[1].lb(),box[1].ub(),QPen(Qt::yellow),QBrush(Qt::NoBrush));
            } else {
                pair<IntervalVector,IntervalVector> boxes=lf.bisect(box);
                s.push(boxes.first);
                s.push(boxes.second);
            }
    }

    if(par->isinside==1){
        robot_position_estimator(nbox1,par);
        par->isinside1=1;
        par->isinside=0;
    }
    par->vin.clear();

    IntervalVector box2(2);
    box2[0]=Interval(-10,10);
    box2[1]=Interval(-10,10);

    // Build the way boxes will be bisected.
    // "LargestFirst" means that the dimension bisected
    // is always the largest one.
    int nbox2=0;
    LargestFirst lf2;
    IntervalVector viinside2(2);
    stack<IntervalVector> s2;
    s2.push(box2);
    while (!s2.empty()) {
        IntervalVector box2=s2.top();
        s2.pop();
            contract_and_draw(inside2r,box2,viinside2,2,par,nbox2,Qt::magenta,Qt::red);
            if (box2.is_empty()) { continue; }

            contract_and_draw(outside2r,box2,viinside2,0,par,nbox2,Qt::darkBlue,Qt::cyan);
            if (box2.is_empty()) { continue; }

            if (box2.max_diam()<epsilon) {
                R.DrawBox(box2[0].lb(),box2[0].ub(),box2[1].lb(),box2[1].ub(),QPen(Qt::yellow),QBrush(Qt::NoBrush));
            } else {
                pair<IntervalVector,IntervalVector> boxes2=lf2.bisect(box2);
                s2.push(boxes2.first);
                s2.push(boxes2.second);
            }
    }
    if(par->isinside==1){
        robot_position_estimator(nbox2,par);
        par->isinside2=1;
        par->isinside=0;
    }
    par->vin.clear();
    IntervalVector box3(2);
    box3[0]=Interval(-10,10);
    box3[1]=Interval(-10,10);

    // Build the way boxes will be bisected.
    // "LargestFirst" means that the dimension bisected
    // is always the largest one.
    int nbox3=0;
    LargestFirst lf3;
    IntervalVector viinside3(2);
    stack<IntervalVector> s3;
    s3.push(box3);
    while (!s3.empty()) {
        IntervalVector box3=s3.top();
        s3.pop();
            contract_and_draw(inside3r,box3,viinside3,3,par,nbox3,Qt::magenta,Qt::red);
            if (box3.is_empty()) { continue; }

            contract_and_draw(outside3r,box3,viinside3,0,par,nbox3,Qt::darkBlue,Qt::cyan);
            if (box3.is_empty()) { continue; }

            if (box3.max_diam()<epsilon) {
                R.DrawBox(box3[0].lb(),box3[0].ub(),box3[1].lb(),box3[1].ub(),QPen(Qt::yellow),QBrush(Qt::NoBrush));
            } else {
                pair<IntervalVector,IntervalVector> boxes3=lf3.bisect(box3);
                s3.push(boxes3.first);
                s3.push(boxes3.second);
            }
    }
    if(par->isinside==1){
        robot_position_estimator(nbox3,par);
        par->isinside3=1;
        par->isinside=0;
    }
    par->vin.clear();

    if (par->isinside1 ==1 || par->isinside2 ==1 || par->isinside3 ==1){
        double aimth1 = get_angle(xb,yb,par->xin,par->yin)+M_PI;
        double aimth2 = get_angle(xb2,yb2,par->xin,par->yin)+M_PI;
        double aimth3 = get_angle(xb3,yb3,par->xin,par->yin)+M_PI;

        R.DrawLine(xb,yb,xb+r*cos(aimth1),yb+r*sin(aimth1),QPen(Qt::red));
        R.DrawLine(xb2,yb2,xb2+r*cos(aimth2),yb2+r*sin(aimth2),QPen(Qt::red));
        R.DrawLine(xb3,yb3,xb3+r*cos(aimth3),yb3+r*sin(aimth3),QPen(Qt::red));

    }

    r = sqrt(r);
    cout<<"th1"<<th1<<endl;
    R.DrawLine(xb,yb,xb+r*cos(th2),yb+r*sin(th2),QPen(Qt::green));
    R.DrawLine(xb2,yb2,xb2+r*cos(th22),yb2+r*sin(th22),QPen(Qt::green));
    R.DrawLine(xb3,yb3,xb3+r*cos(th32),yb3+r*sin(th32),QPen(Qt::green));

    R.DrawRobot(xr-wr/2,yr+lr/2,-3.14/2,wr,lr);
    R.Save("paving");
}
