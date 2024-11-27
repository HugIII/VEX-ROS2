#pragma once

// Algo based on use of lambert's fraction (https://varietyofsound.wordpress.com/2011/02/14/efficient-tanh-computation-using-lamberts-continued-fraction/)
inline double fast_tanh(double x)
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

inline double three_lin_tanh(double x)
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

inline double five_lin_tanh(double x)
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

double fifteen_lin_tanh(double x)
{
    if (x < -0.349846806360468)
    {
        if (x > -1.288690112231558)
        {
            if (x > -0.652273105339896)
            {
                return 0.784454209160348 * x - 0.067136203627008;
            }
            else
            {
                if (x > -0.952698399354790)
                {
                    return 0.558066602782327 * x - 0.214802750649666;
                }
                else
                {
                    return 0.349394991344478 * x - 0.413603860857289;
                }
            }
        }
        else
        {
            if (x > -2.289691672012657)
            {
                if (x > -1.703950163909747)
                {
                    return 0.184190996012522 * x - 0.626500616142730;
                }
                else
                {
                    return 0.073287597472899 * x - 0.815474480262468;
                }
            }
            else
            {
                if (x > -3.369261331568106)
                {
                    return 0.015487206401210 * x - 0.947819554338388;
                }
                else
                {
                    return -1;
                }
            }
        }
    }
    else
    {
        if (x < 1.288690112231558)
        {
            if (x < 0.652273105339896)
                if (x < 0.349846806360468)
                {
                    return 0.976355928445541 * x;
                }
                else
                {
                    return 0.784454209160348 * x + 0.067136203627008;
                }
            else
            {
                if (x < 0.952698399354790)
                {
                    return 0.558066602782327 * x + 0.214802750649666;
                }
                else
                {
                    return 0.349394991344478 * x + 0.413603860857289;
                }
            }
        }
        else
        {
            if (x < 2.289691672012657)
            {
                if (x < 1.703950163909747)
                {
                    return 0.184190996012522 * x + 0.626500616142730;
                }
                else
                {
                    return 0.073287597472899 * x + 0.815474480262468;
                }
            }
            else
            {
                if (x < 3.369261331568106)
                {
                    return 0.015487206401210 * x + 0.947819554338388;
                }
                else
                {
                    return 1;
                }
            }
        }
    }
}
