package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;


/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado teste")
public class TeleOp_teste extends OpMode {
    private Follower follower;
    DcMotorEx slide, armMotorL, armMotorR;
    double powerR, powerL;
    Servo servo1, servo2, garra;
    boolean holdingPosition = false, modeBase = false;
    private final Pose startPose = new Pose(0, 0, 0);
    PController pidL, pidR;

    int targetR, targetL;

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

        double joystickInput = gamepad2.right_stick_y;

        /*
        armMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double min = 0.05;
        double max = 0.5;
        pidL = new PController(0.5);
        pidL.setInputRange(0, 1000);
        pidL.setSetPoint(targetL);
        pidL.setOutputRange(min, max);

        pidR = new PController(0.5);
        pidR.setInputRange(0, 1000);
        pidR.setSetPoint(targetR);
        pidR.setOutputRange(min, max);

        powerL = min + pidL.getComputedOutput(armMotorL.getCurrentPosition());
        powerR = min + pidR.getComputedOutput(armMotorR.getCurrentPosition());

        double powerL2 = min - pidL.getComputedOutput(armMotorL.getCurrentPosition());
        double powerR2 = min - pidR.getComputedOutput(armMotorR.getCurrentPosition());

        telemetry.addData("Posição Left: ", armMotorL.getCurrentPosition());
        telemetry.addData("Posição Right: ", armMotorR.getCurrentPosition());

         */

        if (joystickInput > 0) {
            armMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorL.setPower(joystickInput);
            armMotorR.setPower(joystickInput);
            modeBase = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move o motor
        else if (joystickInput < 0) {
            armMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorL.setPower(-joystickInput);
            armMotorR.setPower(-joystickInput);
            modeBase = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick estiver parado e o motor ainda não estiver segurando a posição
        else if (!modeBase) {
            armMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // O operador ! (negação) verifica se holdingPosition é false
            armMotorL.setTargetPosition(armMotorL.getCurrentPosition());
            armMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotorL.setPower(1);

            armMotorR.setTargetPosition(armMotorR.getCurrentPosition());
            armMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotorR.setPower(1);
            modeBase = true;
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