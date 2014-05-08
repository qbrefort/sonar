#ifndef SIVIA_H
#define SIVIA_H

#include "repere.h"
#include "ibex.h"
#include <string>

using namespace ibex;
using namespace std;

struct sivia_struct{
    double xb1,xb2,xb3,yb1,yb2,yb3;
    double xin,yin;
    double epsilon;
    double th1,th2,th3;
    double xr,yr;
    double sonar_radius;
    int inside;
    int isinside;
    int isinside1,isinside2,isinside3;
    std::string state;
    vector<IntervalVector> vin;
};

class Sivia {
public:

    /*
     * Run the SIVIA algorithm.
     *
     * Parameters:
     * R:   where to draw the boxes.
     * epsilon: precision downto which boxes are bisected.
     */
    Sivia(repere& R, sivia_struct *my_struct);


    /*
     * Contract "box" with "c" and draw the trace (i.e., the difference between box and c(box))
     * with the colors "pencolor" and "brushcolor".
     */
    void contract_and_draw(Ctc& c, IntervalVector& box,IntervalVector& viinside, int inside, sivia_struct *par,int& nbox, const QColor &pencolor, const QColor &brushcolor);

private:
    repere& R;
};

#endif // SIVIA_H
