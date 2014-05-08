#include "sivia.h"
#include "math.h"


void Sivia::contract_and_draw(Ctc& c, IntervalVector& X,int inside, const QColor & pencolor, const QColor & brushcolor) {
    IntervalVector X0=X;       // get a copy
    try {
        c.contract(X);
        if (X==X0) return;     // nothing contracted.
        IntervalVector* rest;
        int n=X0.diff(X,rest); // calculate the set difference
        for (int i=0; i<n; i++) {     // display the boxes
            if(inside!=0)
            {R.DrawBox(rest[i][0].lb(),rest[i][0].ub(), rest[i][1].lb(),rest[i][1].ub(),QPen(pencolor),QBrush(brushcolor));}
            if (inside==1) cout<<"found you  1!"<<endl;
            if (inside==2) cout<<"found you  2!"<<endl;
        }
        delete[] rest;
    } catch(EmptyBoxException&) {
        if(inside!=0)
        R.DrawBox(X0[0].lb(),X0[0].ub(),X0[1].lb(),X0[1].ub(),QPen(pencolor),QBrush(brushcolor));
    }
}


Sivia::Sivia(repere& R, double epsilon,double th1,double ry) : R(R) {

    // Create the function we want to apply SIVIA on.
    Variable x,y;
    double xb=1,yb=3;
    double xr=4,yr=8;

    double arc=3.14/2;

    double r = 30;
    double th2=th1+arc;
    double th21=th1 -50;
    double th22=th21 + arc;
    double e=0.5;

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

    double xb2=-3,yb2=-5;
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


    //CtcQInter inter(inside,1);



    NumConstraint inrx1(x,y,x>2);
    NumConstraint outrx1(x,y,x<2);
    NumConstraint inrx2(x,y,x<1);
    NumConstraint outrx2(x,y,x>1);
    NumConstraint inry1(x,y,y<ry);
    NumConstraint outry1(x,y,y>ry);
    NumConstraint inry2(x,y,y>ry+3);
    NumConstraint outry2(x,y,y<ry+3);

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

    // Build the initial box.
    IntervalVector box(2);
    box[0]=Interval(-10,10);
    box[1]=Interval(-10,10);

    // Build the way boxes will be bisected.
    // "LargestFirst" means that the dimension bisected
    // is always the largest one.
    LargestFirst lf;

    stack<IntervalVector> s;
    s.push(box);
    cout<<"new"<<endl;
    while (!s.empty()) {
        IntervalVector box=s.top();
        s.pop();
            contract_and_draw(inside1r,box,1,Qt::magenta,Qt::red);
            if (box.is_empty()) { continue; }

            contract_and_draw(outside1r,box,0,Qt::darkBlue,Qt::cyan);
            if (box.is_empty()) { continue; }

            if (box.max_diam()<epsilon) {
                R.DrawBox(box[0].lb(),box[0].ub(),box[1].lb(),box[1].ub(),QPen(Qt::yellow),QBrush(Qt::NoBrush));
            } else {
                pair<IntervalVector,IntervalVector> boxes=lf.bisect(box);
                s.push(boxes.first);
                s.push(boxes.second);
            }
    }

    IntervalVector box2(2);
    box2[0]=Interval(-10,10);
    box2[1]=Interval(-10,10);

    // Build the way boxes will be bisected.
    // "LargestFirst" means that the dimension bisected
    // is always the largest one.
    LargestFirst lf2;

    stack<IntervalVector> s2;
    s2.push(box2);
    cout<<"new"<<endl;
    while (!s2.empty()) {
        IntervalVector box2=s2.top();
        s2.pop();
            contract_and_draw(inside2r,box2,2,Qt::magenta,Qt::red);
            if (box2.is_empty()) { continue; }

            contract_and_draw(outside2r,box2,0,Qt::darkBlue,Qt::cyan);
            if (box2.is_empty()) { continue; }

            if (box2.max_diam()<epsilon) {
                R.DrawBox(box2[0].lb(),box2[0].ub(),box2[1].lb(),box2[1].ub(),QPen(Qt::yellow),QBrush(Qt::NoBrush));
            } else {
                pair<IntervalVector,IntervalVector> boxes2=lf2.bisect(box2);
                s2.push(boxes2.first);
                s2.push(boxes2.second);
            }
    }

    R.DrawRobot(1.5,ry+1.5,-3.14/2);
    R.Save("paving");
}
