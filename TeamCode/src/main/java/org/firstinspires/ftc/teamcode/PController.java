package org.firstinspires.ftc.teamcode;

public class PController {
    public double setPoint = 0;
    public double minInput = 0;
    public double maxInput = 0;
    public double minOutput = 0;
    public double maxOutput = 0, thresholdPercent = 0;
    private double currentError = 0;

    private double Kp;

    public PController(double Kp) {
        this.Kp = Kp;
    }

    public void setThresholdValue(double thresholdPercent){
        this.thresholdPercent = thresholdPercent;
    }

    public void setSetPoint(int target){
        this.setPoint = target;
    }

    public void setInputRange(double minInput, double maxInput){
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    public void setOutputRange(double minOutput, double maxOutput){
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public double getComputedOutput(double input){
        currentError = setPoint - input; // sem abs, pra manter sinal

        double output = currentError * Kp;

        // Limita o valor com sinal preservado
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < -maxOutput) {
            output = -maxOutput;
        }

        // Elimina zona morta
        if (Math.abs(output) < minOutput) {
            output = 0;
        }

        return output;
    }

    public boolean hasPControllerReachedTarget(){
        double percentDifferenceFromTarget =
                (Math.abs(currentError)/(maxInput - minInput)) * 100;
        return percentDifferenceFromTarget >= thresholdPercent;
    }
}
