package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Inspecion")
public class Inspecion extends OpMode {
    private Follower follower;
    private FollowerConstants followerConstants;
    DcMotorEx slide, armMotorL, armMotorR;
    Servo servo1, garra;
    boolean holdingPosition = false, modeBase = false;
    private final Pose startPose = new Pose(0, 0, 0);
    int limit0, estado;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        garra = hardwareMap.get(Servo.class, "garra");

        armMotorL = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        armMotorR = hardwareMap.get(DcMotorEx.class, "armmotorright");
        armMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    public void move_Slide(){

        int giroMotor = 200;
        int limitregra = -2000;

        int posRight = armMotorR.getCurrentPosition();
        int MAX_POSITION = -3225; //-2125

        int maximoAtual = limitregra;
        int currentPosition = slide.getCurrentPosition(); // Obtém a posição atual do motor
        double joystickInput = gamepad2.left_stick_y; // Captura a entrant do joystick

        if(posRight >= giroMotor){
            //TODO: ÁREA SUPERIOR
            maximoAtual = MAX_POSITION;
        }else{
            //TODO: ÁREA INFERIOR
            maximoAtual = limitregra;
        }
        if (currentPosition < maximoAtual) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(1);
        } else if (currentPosition <= 200) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (currentPosition > maximoAtual + 100) {
                slide.setPower(joystickInput);
            } else {
                if (joystickInput <= 0) {
                    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.setTargetPosition(currentPosition);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(0.25);
                } else {
                    slide.setPower(joystickInput);
                }
            }

        }
        telemetry.addData("JOYSTICK POS:", joystickInput);
        telemetry.addData("SLIDE POS: ", slide.getCurrentPosition());


    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            estado = 2;
        } else if (gamepad1.b) {
            estado = 1;
        }

        if (estado == 1) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addLine("Normal Chassi");
        } else if (estado == 2) {
            follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addLine("Inverso Chassi");

        }

        moveServo();
        //moveSlide();
        move_Slide();
        armBase();

        telemetry.addData("Pos Left: ", armMotorL.getCurrentPosition());
        telemetry.addData("Pos Right: ", armMotorR.getCurrentPosition());
    }

    //TODO: Mover Slide
    public void moveSlide() {

        int current = slide.getCurrentPosition();
        int limit = -2125;
        double joystickInput = gamepad2.left_stick_y; // Captura a entrada do joystick

        // Se o joystick for movido para cima e a posição for menor que 0, move o motor
        if (joystickInput > 0 && current < 0) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move o motor
        else if (joystickInput < 0 && current > limit) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(joystickInput);
            holdingPosition = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick estiver parado e o motor ainda não estiver segurando a posição
        else if (!holdingPosition) { // O operador ! (negação) verifica se holdingPosition é false
            slide.setTargetPosition(current); // Define a posição atual como alvo
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            slide.setPower(0.3); // Aplica uma pequena potência para segurar a posição
            holdingPosition = true; // Marca que o motor está segurando a posição
        }

        if(gamepad2.x){
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Posição Slide:", current);
    }

    //TODO: Mover base do atuador
    public void armBase() {

        double j = -gamepad2.right_stick_y;
        int currentL = armMotorL.getCurrentPosition();
        int currentR = armMotorR.getCurrentPosition();

        // Se o joystick for movido para cima e a posição for menor que 0, move o motor
        if (j > 0) {
            armMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorL.setPower(0.35);

            armMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorR.setPower(0.35);

            modeBase = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick for movido para baixo e ainda não atingiu o limite, move o motor
        else if (j < 0) {
            armMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorL.setPower(-0.22);

            armMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorR.setPower(-0.22);
            modeBase = false; // O motor está se movendo, então não está segurando posição
        }
        // Se o joystick estiver parado e o motor ainda não estiver segurando a posição
        else if (!modeBase) {

            armMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // O operador ! (negação) verifica se holdingPosition é false
            armMotorL.setTargetPosition(currentL); // Define a posição atual como alvo
            armMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            armMotorL.setPower(1); // Aplica uma pequena potência para segurar a posição

            armMotorR.setTargetPosition(currentR); // Define a posição atual como alvo
            armMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            armMotorR.setPower(1);

            modeBase = true; // Marca que o motor está segurando a posição
        }

        if(gamepad2.dpad_up){
            armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("POS LEFT:", armMotorL.getCurrentPosition());
        telemetry.addData("POS RIGHT: ", armMotorR.getCurrentPosition());
        telemetry.addData("Power Left: ", armMotorR.getPower());
        telemetry.addData("Joystick Atuador: ", j);
    }

    //Todo: Mover servo1
    public void moveServo(){

        if(gamepad2.y){
            //posição pick
            servo1.setPosition(0);
            telemetry.addLine("Pick");
        }else if(gamepad2.a){
            servo1.setPosition(1);
            telemetry.addLine("Clip");
        }else if(gamepad2.b){
            servo1.setPosition(0.65);
            telemetry.addLine("90");
        }

        if(gamepad2.right_bumper){
            garra.setPosition(0.6);

        }else{
            garra.setPosition(0);
        }


    }
}