package org.firstinspires.ftc.teamcode;

public class PID_Parameters {
    private double kp, ki, kd;
    private double integralSum;
    private double lastError;
    private double lastTime;

    public PID_Parameters(double Kp, double Ki, double Kd){
        this.kp = Kp;
        this.ki = Ki;
        this.kd = Kd;
    }

    public double calculate(double target, double current){
        double error = target - current;
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = currentTime - lastTime;

        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;
        integralSum += error * deltaTime;

        double output = kp * error + ki * integralSum + kd * derivative;

        lastError = error;
        lastTime = currentTime;

        return output;
    }


    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.nanoTime() / 1e9;
    }

}
