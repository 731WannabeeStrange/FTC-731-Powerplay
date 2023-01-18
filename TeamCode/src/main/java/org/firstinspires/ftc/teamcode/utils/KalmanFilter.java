package org.firstinspires.ftc.teamcode.utils;

public class KalmanFilter {
    protected double Q;
    protected double R;
    protected int numSamples;
    protected double P = 1;
    protected double K;
    protected double x;
    public SizedStack<Double> estimates;
    protected LinearRegression regression;

    public KalmanFilter(double Q, double R, int numSamples) {
        this.Q = Q;
        this.R = R;
        this.numSamples = numSamples;
        this.x = 0;
        this.estimates = new SizedStack<>(numSamples);
        for (int i = 0; i < numSamples; i++) {
            estimates.push(0.0);
        }
        regression = new LinearRegression(stackToArr());
        findK();
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getX() {
        return x;
    }

    public double estimate(double measurement) {
        regression.runLeastSquares();
        x += regression.predictNextValue() - estimates.peek();
        x += K * (measurement - x);
        estimates.push(x);
        regression = new LinearRegression(stackToArr());
        return x;
    }

    public void findK() {
        for (int i = 0; i < 2000; i++) solveDARE();
    }

    public void solveDARE() {
        P = P + Q;
        K = P / (P + R);
        P = (1 - K) * P;
    }

    protected double[] stackToArr() {
        double[] arrVals = new double[numSamples];
        for (int i = 0; i < estimates.size(); i++) {
            arrVals[i] = estimates.get(i);
        }
        return arrVals;
    }
}
