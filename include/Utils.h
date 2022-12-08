#ifndef UTILS_H
#define UTILS_H

#include <ostream>
#include <vector>
#include <utility>

// #include <opencv2/opencv.hpp>
// #include "CvPlot/cvplot.h"

namespace robotics_mcl {

inline bool isEqDoubles(double a, double b) {
    return fabs(a - b) < 0.01;
}

struct Coord {
    Coord() = default;
    Coord(double x, double y, double w) : x(x), y(y), w(w) { }
    
    bool operator==(const Coord& c) {
        return isEqDoubles(this->x, c.x) 
                && isEqDoubles(this->y, c.y) 
                    && isEqDoubles(this->w, c.w); 
    }

    double x, y;
    double w;
};

struct Orientation {
    Orientation() = default;
    Orientation(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) { }
    
    double x, y, z, w;
};

struct Scan {
    Scan() : maxLen(17), minRange(0.45), maxRange(8), ranges({}) {
        ranges = std::vector<double>(maxLen);
    }
    
    int maxLen;
    double minRange;
    double maxRange;
    std::vector<double> ranges;
};

struct Map {
    Map() : width(0), height(0), x({}), y({})
            , xmin(0), xmax(850), ymin(0), ymax(850), resolution(0) { }
    
    int xmin, xmax;
    int ymin, ymax;
    int width;
    int height;
    double resolution;
    std::vector<int> x;
    std::vector<int> y;
};

inline std::ostream& operator<<(std::ostream& os, const Coord& p) {
    os << "(" << p.x << ", " << p.y << ", " << p.w << ")";
    return os; 
}

inline std::ostream& operator<<(std::ostream& os, const Orientation& o) {
    os << "(" << o.x << ", " << o.y << ", " << o.z << ", " << o.w << ")";
    return os;
}

}

#endif