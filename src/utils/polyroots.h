
#include <boost/math/tools/roots.hpp>
#include <boost/math/tools/cubic_roots.hpp>
#include <boost/math/tools/quartic_roots.hpp>

std::array<double, 2> quadraticRoots(double a, double b, double c){
    auto roots = boost::math::tools::quadratic_roots(a, b, c);
    return {roots.first, roots.second};
}

std::array<double, 3> cubicRoots(double a, double b, double c, double d){
    return boost::math::tools::cubic_roots(a, b, c, d);
}

std::array<double, 4> quarticRoots(double a, double b, double c, double d, double e) {
    return boost::math::tools::quartic_roots(a, b, c, d, e);
}