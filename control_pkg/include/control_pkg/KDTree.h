#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>
#include <numeric>

class KDTree {
public:
    struct Node {
        double x, y;
        int index;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;

        Node(double x, double y, int idx) : x(x), y(y), index(idx) {}
    };

    KDTree(const std::vector<double>& x_list, const std::vector<double>& y_list) {
        std::vector<int> indices(x_list.size());
        std::iota(indices.begin(), indices.end(), 0);
        root_ = buildTree(x_list, y_list, indices, 0);
    }

    std::pair<int, double> findNearest(double query_x, double query_y) {
        double minDistance = std::numeric_limits<double>::max();
        int nearestIndex = -1;
        searchTree(root_.get(), query_x, query_y, 0, nearestIndex, minDistance);
        return {nearestIndex, minDistance};
    }

private:
    std::unique_ptr<Node> root_;

    std::unique_ptr<Node> buildTree(const std::vector<double>& x_list, const std::vector<double>& y_list, std::vector<int>& indices, int depth) {
        if (indices.empty()) return nullptr;

        int axis = depth % 2;
        auto comparator = [&](int a, int b) {
            return axis == 0 ? x_list[a] < x_list[b] : y_list[a] < y_list[b];
        };
        std::nth_element(indices.begin(), indices.begin() + indices.size() / 2, indices.end(), comparator);
        int medianIdx = indices[indices.size() / 2];

        std::unique_ptr<Node> node = std::make_unique<Node>(x_list[medianIdx], y_list[medianIdx], medianIdx);
        auto leftIndices = std::vector<int>(indices.begin(), indices.begin() + indices.size() / 2);
        auto rightIndices = std::vector<int>(indices.begin() + indices.size() / 2 + 1, indices.end());
        node->left = buildTree(x_list, y_list, leftIndices, depth + 1);
        node->right = buildTree(x_list, y_list, rightIndices, depth + 1);

        return node;
    }

    void searchTree(Node* node, double query_x, double query_y, int depth, int& nearestIndex, double& minDistance) {
        if (!node) return;

        double dx = query_x - node->x;
        double dy = query_y - node->y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < minDistance) {
            minDistance = distance;
            nearestIndex = node->index;
        }

        int axis = depth % 2;
        double delta = axis == 0 ? dx : dy;

        Node* firstSide = delta < 0 ? node->left.get() : node->right.get();
        Node* secondSide = delta < 0 ? node->right.get() : node->left.get();

        if (firstSide) searchTree(firstSide, query_x, query_y, depth + 1, nearestIndex, minDistance);
        if (delta * delta < minDistance && secondSide) {
            searchTree(secondSide, query_x, query_y, depth + 1, nearestIndex, minDistance);
        }
    }
};
#endif // KDTREE_H