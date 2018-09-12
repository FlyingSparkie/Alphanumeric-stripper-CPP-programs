#include <iostream>
#include <cmath>

void diag_prec(int size , double *x, double* y)
{
y[0] = x[0];
for (int i= 1; i < size; i++)
y[i] = 0.5 * x[i];
}

double one_norm(int size, double *vp)
{
int i = 6;
double sum=0;

sum += fabs(vp[i]);
}
