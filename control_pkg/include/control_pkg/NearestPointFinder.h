// NearestPointFinder.h
#ifndef NEAREST_POINT_FINDER_H
#define NEAREST_POINT_FINDER_H

#include <vector>
#include <cmath>
#include <limits>

class NearestPointFinder {
public:
    NearestPointFinder(const std::vector<double>& x_list, const std::vector<double>& y_list)
        : x_list_(x_list), y_list_(y_list) {}

    struct Result {
        int index;
        double distance;

        Result(int idx = -1, double dist = std::numeric_limits<double>::max())
            : index(idx), distance(dist) {}
    };

    Result findNearestPoint(double query_x, double query_y) {
        Result result;
        for (size_t i = 0; i < x_list_.size(); ++i) {
            double dx = query_x - x_list_[i];
            double dy = query_y - y_list_[i];
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < result.distance) {
                result.index = i;
                result.distance = distance;
            }
        }
        return result;
    }

private:
    const std::vector<double>& x_list_;
    const std::vector<double>& y_list_;
};

#endif // NEAREST_POINT_FINDER_H
