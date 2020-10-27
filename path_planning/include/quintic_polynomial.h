#ifndef QUINTIC_POLYNOMIAL_H
#define QUINTIC_POLYNOMIAL_H

class QuinticPolynomial{
    private:
        double a0, a1, a2, a3, a4, a5;
        
    public:
        QuinticPolynomial() = default;
        QuinticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double t);
        double calc_point(double t);
        double calc_first_derivative(double t);
        double calc_second_derivative(double t);
        double calc_third_derivative(double t);
};

#endif