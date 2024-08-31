

#include "control_pkg/digital_filter.h"
#include <cmath>
#include <iostream>

namespace
{

const double kDoubleEpsilon = 1.0e-6;

}  // namespace


namespace control
{

DigitalFilter::DigitalFilter(const std::vector<double> &denominators,
                             const std::vector<double> &numerators)
{
    SetCoefficients(denominators, numerators);
}

void DigitalFilter::SetDenominators(const std::vector<double> &denominators)
{
    denominators_ = denominators;
    y_values_.resize(denominators_.size(), 0.0);
}

void DigitalFilter::SetNumerators(const std::vector<double> &numerators)
{
    numerators_ = numerators;
    x_values_.resize(numerators_.size(), 0.0);
}

void DigitalFilter::SetCoefficients(const std::vector<double> &denominators,
                                    const std::vector<double> &numerators)
{
    SetDenominators(denominators);
    SetNumerators(numerators);
}

void DigitalFilter::SetDeadZone(const double deadzone)
{
    dead_zone_ = std::fabs(deadzone);
    std::cout << "Setting digital filter dead zone = " << dead_zone_ << std::endl;
}

double DigitalFilter::Filter(const double x_insert)
{
    if (denominators_.empty() || numerators_.empty()) {
        // std::cout << "Empty denominators or numerators" << std::endl;
        return 0.0;
    }

    x_values_.pop_back();
    x_values_.push_front(x_insert);
    const double kXside =
        Compute(x_values_, numerators_, 0, numerators_.size() - 1);

    y_values_.pop_back();
    const double kYside =
        Compute(y_values_, denominators_, 1, denominators_.size() - 1);

    double y_insert = 0.0;
    if (std::fabs(denominators_.front()) > kDoubleEpsilon) {
        y_insert = (kXside - kYside) / denominators_.front();
    }
    y_values_.push_front(y_insert);

    return UpdateLast(y_insert);
}

void DigitalFilter::ResetValues()
{
    std::fill(x_values_.begin(), x_values_.end(), 0.0);
    std::fill(y_values_.begin(), y_values_.end(), 0.0);
}

double DigitalFilter::UpdateLast(const double input)
{
    const double kDiff = std::fabs(input - last_);
    if (kDiff < dead_zone_) {
        return last_;
    }
    last_ = input;
    return input;
}

double DigitalFilter::Compute(const std::deque<double> &values,
                              const std::vector<double> &coefficients,
                              const std::size_t coeff_start,
                              const std::size_t coeff_end)
{
    if (coeff_start <= coeff_end && coeff_end < coefficients.size()) {
        // std::cout << "coefficients size " << coefficients.size() << std::endl;
    } else {
        // std::cout << "Wrong size" << std::endl;
        return -1;
    }
    if ((coeff_end - coeff_start + 1) == values.size()) {
        // std::cout << "value size " << values.size() << std::endl;
    } else {
        // std::cout << "Empty size" << std::endl;
    }
    double sum = 0.0;
    auto i = coeff_start;
    for (const auto value : values) {
        sum += value * coefficients[i];
        ++i;
    }
    return sum;
}

const std::vector<double> &DigitalFilter::Denominators() const
{
    return denominators_;
}

const std::vector<double> &DigitalFilter::Numerators() const
{
    return numerators_;
}

double DigitalFilter::DeadZone() const { return dead_zone_; }

const std::deque<double> &DigitalFilter::InputsQueue() const
{
    return x_values_;
}

const std::deque<double> &DigitalFilter::OutputsQueue() const
{
    return y_values_;
}

}  // namespace common
