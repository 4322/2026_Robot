package frc.robot.util;

public class AverageStat {
    private double[] samples;
    private int maxSamples;
    private int numSamples = 0;
    private int nextIndex = 0;
    private double runningTotal = 0;

    AverageStat(int maxSamples) {
        this.maxSamples = maxSamples; 
        samples = new double[maxSamples];

    }

}
