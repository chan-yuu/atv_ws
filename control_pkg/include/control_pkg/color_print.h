#ifndef COLOR_PRINT_H
#define COLOR_PRINT_H

#include <iostream>
#include <sstream>
#include <string>

namespace color_print {

template<typename T>
void print_args(std::ostringstream& oss, const T& arg) {
    oss << arg << " ";
}

template<typename T, typename... Args>
void print_args(std::ostringstream& oss, const T& arg, const Args&... args) {
    oss << arg << " ";
    print_args(oss, args...);
}

// Print in green color
template<typename... Args>
inline void prGreen(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[92m" << oss.str() << "\033[00m";
}

// Print in red color
template<typename... Args>
inline void prRed(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[91m" << oss.str() << "\033[00m";
}

// Print in yellow color
template<typename... Args>
inline void prYellow(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[93m" << oss.str() << "\033[00m";
}

// Print in blue color
template<typename... Args>
inline void prBlue(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[94m" << oss.str() << "\033[00m";
}

// Print in purple color
template<typename... Args>
inline void prPurple(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[95m" << oss.str() << "\033[00m";
}

// Print in cyan color
template<typename... Args>
inline void prCyan(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[96m" << oss.str() << "\033[00m";
}

// Print in orange color
template<typename... Args>
inline void prOrange(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[33m" << oss.str() << "\033[00m";
}

// Print in pink color
template<typename... Args>
inline void prPink(const Args&... args) {
    std::ostringstream oss;
    print_args(oss, args...);
    std::cout << "\033[95m" << oss.str() << "\033[00m";
}

}  // namespace color_print

#endif  // COLOR_PRINT_H