package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Move_PController_Auto extends LinearOpMode {
    DcMotor armMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        int encoderDegreesToAttain = 200;
        double minPower = 0.01;
        double maxPower = 0.5;
        PController pController = new PController(0.03);
        pController.setInputRange(50,500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);

        armMotor = hardwareMap.get(DcMotor.class, "gobilda");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int currentPosition = armMotor.getCurrentPosition();
        double powerS = minPower + pController.getComputedOutput(currentPosition);
        double powerD = minPower -pController.getComputedOutput(currentPosition);

        while (opModeIsActive()){
            if(currentPosition < encoderDegreesToAttain){
                armMotor.setPower(powerS);
            }else{
                armMotor.setPower(powerD);
            }
        }

        armMotor.setPower(0);
    }
}
