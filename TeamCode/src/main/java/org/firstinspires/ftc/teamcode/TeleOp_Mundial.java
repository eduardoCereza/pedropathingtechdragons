package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;


/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado Mundial")
public class TeleOp_Mundial extends OpMode {
    private Follower follower;
    DcMotorEx slide, armMotorL, armMotorR;
    double powerRU, powerRD, powerLU, powerLD;
    Servo servo1, servo2, garra;
    int estado;
    boolean holdingPosition = false, modeBase = false;
    private final Pose startPose = new Pose(0, 0, 0);
    PController pidL, pidR;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        garra = hardwareMap.get(Servo.class, "garra");

        armMotorL = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        armMotorR = hardwareMap.get(DcMotorEx.class, "armmotorright");
        armMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        moveServo();
        moveSlide();
        armBase();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }

    //TODO: Mover Slide
    public void moveSlide() {

        int current = slide.getCurrentPosition();
        int limit = -3500;

        double joystickInput = gamepad2.left_stick_y; // Captura a entrada do joystick

        // Se o joystick for movido para cima e a posição for menor que 0, move o motor
        if (joystickInput > 0 && current < 0) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move o motor
        else if (joystickInput < 0 && current > limit+10) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick estiver parado e o motor ainda não estiver segurando a posição
        else if (!holdingPosition) { // O operador ! (negação) verifica se holdingPosition é false
            slide.setTargetPosition(current); // Define a posição atual como alvo
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            slide.setPower(0.1); // Aplica uma pequena potência para segurar a posição
            holdingPosition = true; // Marca que o motor está segurando a posição
        }

        if(gamepad2.x){
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Posição Slide:", current);
    }


    //TODO: Mover base do atuador
    public void armBase() {
        if(gamepad2.dpad_down){
            int target = 0;
            double minPower = 0.5;
            double maxPower = 1;
            PController pid = new PController(1);
            pid.setInputRange(0, 900);
            pid.setSetPoint(target);
            pid.setOutputRange(minPower, maxPower);

            armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            powerLU = minPower + pid.getComputedOutput(armMotorL.getCurrentPosition());
            powerLD = minPower - pid.getComputedOutput(armMotorL.getCurrentPosition());

            powerRU = minPower + pid.getComputedOutput(armMotorR.getCurrentPosition());
            powerRD = minPower - pid.getComputedOutput(armMotorR.getCurrentPosition());

            if(armMotorR.getCurrentPosition() < target){
                armMotorR.setPower(powerRU);
                armMotorL.setPower(powerLU);}
            else {
                armMotorR.setPower(powerRD);
                armMotorL.setPower(powerLD);
            }

        } else if (gamepad2.dpad_up) {
            int target = 600;
            double minPower = 0.5;
            double maxPower = 1;
            PController pid = new PController(1);
            pid.setInputRange(0, 900);
            pid.setSetPoint(target);
            pid.setOutputRange(minPower, maxPower);

            armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            powerLU = minPower + pid.getComputedOutput(armMotorL.getCurrentPosition());
            powerLD = minPower - pid.getComputedOutput(armMotorL.getCurrentPosition());

            powerRU = minPower + pid.getComputedOutput(armMotorR.getCurrentPosition());
            powerRD = minPower - pid.getComputedOutput(armMotorR.getCurrentPosition());

            if(armMotorR.getCurrentPosition() < target){
                armMotorR.setPower(powerRU);
                armMotorL.setPower(powerLU);}
            else {
                armMotorR.setPower(powerRD);
                armMotorL.setPower(powerLD);
            }

        }

    }

    //Todo: Mover servo
    public void moveServo(){

        if(gamepad2.a){
            servo1.setPosition(1);
            servo2.setPosition(0);
            telemetry.addLine("Pick");
        }else if(gamepad2.y){
            servo1.setPosition(0);
            servo2.setPosition(1);
            telemetry.addLine("Clip");
        }else if(gamepad2.b){
            servo1.setPosition(0.6);
            servo2.setPosition(0.4);
            telemetry.addLine("90");
        }

        if(gamepad2.right_bumper){
            garra.setPosition(0.6);
        }else{
            garra.setPosition(0);
        }
    }
}