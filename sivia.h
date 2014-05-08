#ifndef SIVIA_H
#define SIVIA_H

#include "repere.h"
#include "ibex.h"

using namespace ibex;
using namespace std;

class Sivia {
public:

    /*
     * Run the SIVIA algorithm.
     *
     * Parameters:
     * R:   where to draw the boxes.
     * epsilon: precision downto which boxes are bisected.
     */
    Sivia(repere& R, double epsilon, double th1, double ry);

    /*
     * Contract "box" with "c" and draw the trace (i.e., the difference between box and c(box))
     * with the colors "pencolor" and "brushcolor".
     */
    void contract_and_draw(Ctc& c, IntervalVector& box,  int inside, const QColor & pencolor, const QColor &brushcolor);

private:
    repere& R;
};

#endif // SIVIA_H
