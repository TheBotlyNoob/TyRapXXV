package frc.sim;

import java.util.Random;

public class SimUtilities {
    public static Random random = new Random();

    public static float getGuasian(float mean, float sigma)
    {
        return (float)SimUtilities.random.nextGaussian(mean, sigma);
    }

    public static double getGuasian(double mean, double sigma)
    {
        return SimUtilities.random.nextGaussian(mean, sigma);
    }
}
