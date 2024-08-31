#include <vector>
#include <algorithm>
#include <limits>

class ThrottleBrakeLookup {
public:
    typedef std::vector<std::pair<double, double>> LookupTable;

    // 使用查找表初始化
    void Init(const LookupTable& lookupTable) {
        table = lookupTable;
        std::sort(table.begin(), table.end(), 
                  [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                      return a.first < b.first;
                  });
    }

    // 根据加速度查找油门或刹车指令
    double Lookup(double acceleration) const {
        if (table.empty()) return 0.0;

        // 如果加速度小于表中最小值，返回最小值对应的油门/刹车
        if (acceleration <= table.front().first) return table.front().second;
        // 如果加速度大于表中最大值，返回最大值对应的油门/刹车
        if (acceleration >= table.back().first) return table.back().second;

        // 在表中查找第一个大于等于给定加速度的元素
        auto upper = std::upper_bound(table.begin(), table.end(), std::make_pair(acceleration, std::numeric_limits<double>::max()),
                                      [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                                          return a.first < b.first;
                                      });

        // 找到正好匹配或进行线性插值
        if (upper == table.end()) {
            return table.back().second;
        } else if (upper->first == acceleration) {
            return upper->second;
        } else {
            auto lower = upper - 1;
            double fraction = (acceleration - lower->first) / (upper->first - lower->first);
            return lower->second + fraction * (upper->second - lower->second);
        }
    }

private:
    LookupTable table;
};
