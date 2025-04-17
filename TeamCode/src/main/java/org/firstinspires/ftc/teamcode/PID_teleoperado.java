package org.firstinspires.ftc.teamcode;

public class PID_teleoperado {
    private double Kp, Ki, Kd, Kf;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    private double holdPosition = 0;
    private boolean holding = false;

    public PID_teleoperado(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

    }

    public void setHoldPosition(double position) {
        holdPosition = position;
        reset();
        holding = true;
    }

    public void stopHold() {
        holding = false;
        reset();
    }

    public boolean isHolding() {
        return holding;
    }

    public double calculate(double current) {
        if (!holding) return 0;

        double error = holdPosition - current;
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = currentTime - lastTime;

        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        integralSum += error * deltaTime;

        double output = Kp * error + Ki * integralSum + Kd * derivative;

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
