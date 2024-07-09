#pragma once

// Algo based on use of lambert's fraction (https://varietyofsound.wordpress.com/2011/02/14/efficient-tanh-computation-using-lamberts-continued-fraction/)
double fast_tanh(double x)
{
    // Clip output when it goes above 1 or below -1
    if (x < -4.971786858528029)
    {
        return -1;
    }
    if (x > 4.971786858528029)
    {
        return 1;
    }
    double x2 = x * x;
    double a = x * (135135.0 + x2 * (17325.0 + x2 * (378.0 + x2)));
    double b = 135135.0 + x2 * (62370.0 + x2 * (3150.0 + x2 * 28.0));
    return a / b;
}


double three_lin_tanh(double x)
{
    // Clip output when it goes above 1 or below -1
    if (x < -1.299725497278728)
    {
        return -1;
    }
    if (x > 1.299725497278728)
    {
        return 1;
    }
    return x * 0.769393231181298;
}

double five_lin_tanh(double x)
{
    // Clip output when it goes above 1 or below -1
    if (x < -0.818631533308157)
    {
        if (x < -1.979238406276971)
        {
            return -1;
        }
        return x * 0.235809350838973 - 0.533277076260264;
    }
    if (x > 0.818631533308157)
    {
        if (x > 1.979238406276971)
        {
            return 1;
        }
        return x * 0.235809350838973 + 0.533277076260264;
    }
    return x * 0.887234387088490;
}

double five_lin_sig(double x)
{
    // Clip output when it goes above 1 or below -1
    if (x < -0.409315766654079)
    {
        if (x < -0.989619203138486)
        {
            return 0;
        }
        return x * 0.235809350838973 - 0.233361461869868;
    }
    if (x > 0.409315766654079)
    {
        if (x > 0.989619203138486)
        {
            return 1;
        }
        return x * 0.235809350838973 + 0.766638538130132;
    }
    return x * 0.887234387088490 + 0.5;
}
