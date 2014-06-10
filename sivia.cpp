#include "sivia.h"
#include "math.h"

#include <string>


float get_angle(double p1x,double p1y,double p2x,double p2y){
    return atan2(p1y - p2y, p1x - p2x);
}

void robot_position_estimator(int ninbox, struct sivia_struct *par){
    double tx[ninbox],ty[ninbox];
    double tax[ninbox],tay[ninbox];
    double xin=0,yin=0,area=0,taxs=0,tays=0;
    double xmin=100,xmax=-100,ymin=100,ymax=-100;
    //cout<<"next"<<ninbox<<endl;
    for(int i=0;i<ninbox;i++){
        IntervalVector cur = (par->vin.back());
        //cout<<cur<<endl;
        Interval xcur=cur[0];
        Interval ycur=cur[1];
        tx[i]=xcur.mid();
        ty[i]=ycur.mid();
        tax[i]=xcur.diam();
        tay[i]=ycur.diam();

        if(xcur.lb()<xmin) xmin=xcur.lb();
        if(xcur.ub()>xmax) xmax=xcur.ub();
        if(ycur.lb()<ymin) ymin=ycur.lb();
        if(ycur.ub()>ymax) ymax=ycur.ub();

        par->vin.pop_back();
    }

    for(int i=0;i<ninbox;i++){
        xin += tx[i];
        yin += ty[i];
        area += tax[i]*tay[i];
        taxs += tax[i];
        tays += tay[i];
    }
    xin/=double(ninbox);
    yin/=double(ninbox);
    area/=double(ninbox);

    if (area>par->area) {
        if (xin!=0 & yin!=0){
            par->xin = xin;
            par->yin = yin;
        }
        par->area = area;
        par->areax = (xmax-xmin);
        par->areay = (ymax-ymin);
    }
//    else
//        cout<<"no update"<<endl;
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

    par->area = 0;

    // Create the function we want to apply SIVIA on.
    Variable x,y;
    double ei = par->ei;
    double xb=par->xb1,yb=par->yb1;
    Interval xbi=Interval(par->xb1-ei,par->xb1+ei),ybi=Interval(par->yb1-ei,par->yb1+ei);

    double arc = par->sonar_arc;

    double r = pow(par->sonar_radius,2);
    double th1 = par->th[0];
    double th2=th1+arc;
    double th21= par->th[1];
    double th22=th21 + arc;
    double th31= par->th[2];
    double th32=th31 + arc;
    double e=1;
    double epsilon = par->epsilon;

    double xin,yin;

    // First SONAR
    Function f(x,y,sqr(x-xbi)+sqr(y-ybi));

    NumConstraint c1(x,y,f(x,y)<=r+e);
    NumConstraint c2(x,y,f(x,y)>=e);
    NumConstraint c3(x,y,f(x,y)>r+e);
    NumConstraint c4(x,y,f(x,y)<e);


    double sign1,sign2;
    if(cos(th1)>0) sign1=1;
    else sign1=-1;
    if(cos(th2)<0) sign2=1;
    else sign2=-1;


    NumConstraint cth11(x,y,sign1*(y-ybi-((sin(th1))/(cos(th1)))*(x-xbi))<0);
    NumConstraint cth12(x,y,sign1*(y-ybi-((sin(th1))/(cos(th1)))*(x-xbi))>0);
    NumConstraint cth21(x,y,sign2*(y-ybi-((sin(th2))/(cos(th2)))*(x-xbi))<0);
    NumConstraint cth22(x,y,sign2*(y-ybi-((sin(th2))/(cos(th2)))*(x-xbi))>0);


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
    Interval xb2i=Interval(par->xb2-ei,par->xb2+ei),yb2i=Interval(par->yb2-ei,par->yb2+ei);

    Function f2(x,y,sqr(x-xb2i)+sqr(y-yb2i));
    NumConstraint c21(x,y,f2(x,y)<=r+e);
    NumConstraint c22(x,y,f2(x,y)>=e);
    NumConstraint c23(x,y,f2(x,y)>r+e);
    NumConstraint c24(x,y,f2(x,y)<e);


    double sign21,sign22;
    if(cos(th21)>0) sign21=-1;
    else sign21=1;
    if(cos(th22)<0) sign22=1;
    else sign22=-1;


    NumConstraint cth211(x,y,sign21*(y-yb2i-((sin(th21))/(cos(th21)))*(x-xb2i))<0);
    NumConstraint cth212(x,y,sign21*(y-yb2i-((sin(th21))/(cos(th21)))*(x-xb2i))>0);
    NumConstraint cth221(x,y,sign22*(y-yb2i-((sin(th22))/(cos(th22)))*(x-xb2i))<0);
    NumConstraint cth222(x,y,sign22*(y-yb2i-((sin(th22))/(cos(th22)))*(x-xb2i))>0);

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
    Interval xb3i=Interval(par->xb3-ei,par->xb3+ei),yb3i=Interval(par->yb3-ei,par->yb3+ei);

    Function f3(x,y,sqr(x-xb3i)+sqr(y-yb3i));
    NumConstraint c31(x,y,f3(x,y)<=r+e);
    NumConstraint c32(x,y,f3(x,y)>=e);
    NumConstraint c33(x,y,f3(x,y)>r+e);
    NumConstraint c34(x,y,f3(x,y)<e);


    double sign31,sign32;
    if(cos(th31)>0) sign31=-1;
    else sign31=1;
    if(cos(th32)<0) sign32=1;
    else sign32=-1;


    NumConstraint cth311(x,y,sign31*(y-yb3i-((sin(th31))/(cos(th31)))*(x-xb3i))<0);
    NumConstraint cth312(x,y,sign31*(y-yb3i-((sin(th31))/(cos(th31)))*(x-xb3i))>0);
    NumConstraint cth321(x,y,sign32*(y-yb3i-((sin(th32))/(cos(th32)))*(x-xb3i))<0);
    NumConstraint cth322(x,y,sign32*(y-yb3i-((sin(th32))/(cos(th32)))*(x-xb3i))>0);

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

    //Artifact MODELISATION

    double xa = par->xa;
    double ya = par->ya;

    double ra = par->ra;

    Function f_a(x,y,sqr(x-xa)+sqr(y-ya));

    NumConstraint ca1(x,y,f_a(x,y)<=sqr(ra));
    NumConstraint ca2(x,y,f_a(x,y)>=sqr(ra)-par->thick);
    NumConstraint ca3(x,y,f_a(x,y)>sqr(ra));
    NumConstraint ca4(x,y,f_a(x,y)<sqr(ra)-par->thick);

    CtcFwdBwd aout1(ca1);
    CtcFwdBwd aout2(ca2);
    CtcFwdBwd ain1(ca3);
    CtcFwdBwd ain2(ca4);

    CtcUnion ain(ain1,ain2);
    CtcCompo aout(aout1,aout2);


    //Robot MODELISATION

    double xr = par->xr; //robot position x
    double yr = par->yr; //robot position y

    double wr = par->wr; //robot width
    double lr = par->lr; //robot length
    double ep = par->thick;

    xr = par->xr - wr/2;
    NumConstraint inrx1(x,y,x>xr+ep);
    NumConstraint outrx1(x,y,x<xr+ep);
    NumConstraint inrx2(x,y,x<xr-ep);
    NumConstraint outrx2(x,y,x>xr-ep);
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

    CtcUnion inrtemp(incrx1,incrx2,incry1);
    CtcUnion inr1(inrtemp,incry2);
    CtcCompo outrtemp(outcrx1,outcrx2,outcry1);
    CtcCompo outr1(outrtemp,outcry2);

    //2nd rectangle
    xr = par->xr + wr/2;

    NumConstraint inrx21(x,y,x>xr+ep);
    NumConstraint outrx21(x,y,x<xr+ep);
    NumConstraint inrx22(x,y,x<xr-ep);
    NumConstraint outrx22(x,y,x>xr-ep);
    NumConstraint inry21(x,y,y<yr-lr/2);
    NumConstraint outry21(x,y,y>yr-lr/2);
    NumConstraint inry22(x,y,y>yr+lr/2);
    NumConstraint outry22(x,y,y<yr+lr/2);

    CtcFwdBwd incrx21(inrx21);
    CtcFwdBwd incrx22(inrx22);
    CtcFwdBwd incry21(inry21);
    CtcFwdBwd incry22(inry22);

    CtcFwdBwd outcrx21(outrx21);
    CtcFwdBwd outcrx22(outrx22);
    CtcFwdBwd outcry21(outry21);
    CtcFwdBwd outcry22(outry22);

    CtcUnion inrtemp2(incrx21,incrx22,incry21);
    CtcUnion inr2(inrtemp2,incry22);
    CtcCompo outrtemp2(outcrx21,outcrx22,outcry21);
    CtcCompo outr2(outrtemp2,outcry22);


    //3nd rectangle top rectangle
    yr=par->yr+par->lr/2;
    xr=par->xr;

    NumConstraint inrx31(x,y,x>xr+wr/2+ep);
    NumConstraint outrx31(x,y,x<xr+wr/2+ep);
    NumConstraint inrx32(x,y,x<xr-wr/2-ep);
    NumConstraint outrx32(x,y,x>xr-wr/2-ep);
    NumConstraint inry31(x,y,y<yr-ep);
    NumConstraint outry31(x,y,y>yr-ep);
    NumConstraint inry32(x,y,y>yr+ep);
    NumConstraint outry32(x,y,y<yr+ep);

    CtcFwdBwd incrx31(inrx31);
    CtcFwdBwd incrx32(inrx32);
    CtcFwdBwd incry31(inry31);
    CtcFwdBwd incry32(inry32);

    CtcFwdBwd outcrx31(outrx31);
    CtcFwdBwd outcrx32(outrx32);
    CtcFwdBwd outcry31(outry31);
    CtcFwdBwd outcry32(outry32);

    CtcUnion inrtemp3(incrx31,incrx32,incry31);
    CtcUnion inr3(inrtemp3,incry32);
    CtcCompo outrtemp3(outcrx31,outcrx32,outcry31);
    CtcCompo outr3(outrtemp3,outcry32);

    //4 rectangle bot

    yr=par->yr-par->lr/2;
    xr=par->xr;

    NumConstraint inrx41(x,y,x>xr+wr/2+ep);
    NumConstraint outrx41(x,y,x<xr+wr/2+ep);
    NumConstraint inrx42(x,y,x<xr-wr/2-ep);
    NumConstraint outrx42(x,y,x>xr-wr/2-ep);
    NumConstraint inry41(x,y,y<yr-ep);
    NumConstraint outry41(x,y,y>yr-ep);
    NumConstraint inry42(x,y,y>yr+ep);
    NumConstraint outry42(x,y,y<yr+ep);

    CtcFwdBwd incrx41(inrx41);
    CtcFwdBwd incrx42(inrx42);
    CtcFwdBwd incry41(inry41);
    CtcFwdBwd incry42(inry42);

    CtcFwdBwd outcrx41(outrx41);
    CtcFwdBwd outcrx42(outrx42);
    CtcFwdBwd outcry41(outry41);
    CtcFwdBwd outcry42(outry42);

    CtcUnion inrtemp4(incrx41,incrx42,incry41);
    CtcUnion inr4(inrtemp4,incry42);
    CtcCompo outrtemp4(outcrx41,outcrx42,outcry41);
    CtcCompo outr4(outrtemp4,outcry42);

    CtcCompo inrtp(inr1,inr2,inr3);
    CtcUnion outrtp(outr1,outr2,outr3);

    CtcCompo inr(inrtp,inr4);
    CtcUnion outr(outrtp,outr4);

    yr = par->yr;

    int maxq = 3; //nb of contractors
    int Qinter = 2;
    int ctcq = maxq - Qinter + 1; //nb for q-relaxed function of Ibex


    Array<Ctc> inside1r1(inside1,inr,ain);
    Array<Ctc> outside1r1(outside1,outr,aout);

    Array<Ctc> inside2r1(inside2,inr,ain);
    Array<Ctc> outside2r1(outside2,outr,aout);

    Array<Ctc> inside3r1(inside3,inr,ain);
    Array<Ctc> outside3r1(outside3,outr,aout);

    CtcQInter outside1r(outside1r1,Qinter);
    CtcQInter inside1r(inside1r1,ctcq);

    CtcQInter outside2r(outside2r1,Qinter);
    CtcQInter inside2r(inside2r1,ctcq);

    CtcQInter outside3r(outside3r1,Qinter);
    CtcQInter inside3r(inside3r1,ctcq);


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
        //cout<<"area1: "<<par->area<<endl;
    }

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
        //cout<<"area2: "<<par->area<<endl;
    }
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
        //cout<<"area3: "<<par->area<<endl;
    }
    par->state.clear();
    if (par->isinside1 ==1 || par->isinside2 ==1 || par->isinside3 ==1){
        double *aimth = new double[3];
        aimth[0] = get_angle(xb,yb,par->xin,par->yin)+M_PI ;
        aimth[1] = get_angle(xb2,yb2,par->xin,par->yin)+M_PI;
        aimth[2] = get_angle(xb3,yb3,par->xin,par->yin)+M_PI;

        R.DrawLine(xb,yb,xb+r*cos(aimth[0]),yb+r*sin(aimth[0]),QPen(Qt::red));
        R.DrawLine(xb2,yb2,xb2+r*cos(aimth[1]),yb2+r*sin(aimth[1]),QPen(Qt::red));
        R.DrawLine(xb3,yb3,xb3+r*cos(aimth[2]),yb3+r*sin(aimth[2]),QPen(Qt::red));

        par->state = std::string("found");
        double kp = par->kp;
        double u[3];

        for (int i=0;i<3;i++){
            u[i] =   -kp*atan(tan((par->th[i] - (aimth[i] - arc/2.0 ))/2));
            if(u[i]>par->sonar_speed) par->th[i] += par->sonar_speed;
            if(u[i]<-par->sonar_speed) par->th[i] += -par->sonar_speed;
            else par->th[i] += u[i];
        }
//        for (int i=0;i<3;i++){
//            u[i] =   atan(tan((par->th[i] - (aimth[i] - arc/2.0 ))/2));
//            par->th[i] -=u[i];
//        }
    }

    r = sqrt(r);
    //cout<<"th1"<<th1<<endl;
    R.DrawEllipse(xb,yb,par->ei,QPen(Qt::black),QBrush(Qt::NoBrush));
    R.DrawEllipse(xb2,yb2,par->ei,QPen(Qt::black),QBrush(Qt::NoBrush));
    R.DrawEllipse(xb3,yb3,par->ei,QPen(Qt::black),QBrush(Qt::NoBrush));

    R.DrawLine(xb,yb,xb+r*cos(th2),yb+r*sin(th2),QPen(Qt::green));
    R.DrawLine(xb2,yb2,xb2+r*cos(th22),yb2+r*sin(th22),QPen(Qt::green));
    R.DrawLine(xb3,yb3,xb3+r*cos(th32),yb3+r*sin(th32),QPen(Qt::green));

    R.DrawLine(xb,yb,xb+r*cos(th1),yb+r*sin(th1),QPen(Qt::green));
    R.DrawLine(xb2,yb2,xb2+r*cos(th21),yb2+r*sin(th21),QPen(Qt::green));
    R.DrawLine(xb3,yb3,xb3+r*cos(th31),yb3+r*sin(th31),QPen(Qt::green));

    R.DrawEllipse(par->xa,par->ya,par->ra,QPen(Qt::black),QBrush(Qt::NoBrush));

    R.DrawRobot(xr-wr/2,yr+lr/2,-3.14/2,wr,lr);
    R.Save("paving");

    par->vin.clear();
}
