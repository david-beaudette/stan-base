#ifndef HORNER_H
#define HORNER_H

// ----------------------------------------------------------------------------
//  y = horner(n,c,x) evaluates a polynomial y = f(x) at x.  The polynomial has
//                    degree n-1.  The coefficients of the polynomial are stored
//                    in the 1-D array c, which has n elements.
//
//  NOTE:  The polynomial coefficients are multipliers of monomial terms of
//         decreasing order.  In other words, the polynomial is assumed to be
//         written in the form
//
//            y = c_1*x^n + c_2*x^(n-1) + ... + c_(n-2)*x + c_(n-1)
//
//  Also note that if there are n coefficients, the polynomial is of degree n-1
//  and the largest index in c is n-1

float horner(int n, const float *c, float x);

#endif // HORNER_H