package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado teste")
public class TeleOp_teste extends OpMode {

    double kP = 1;
    double kI = 0;
    double kD = 0;

    double integralL = 0;
    double integralR = 0;
    double lastErrorL = 0;
    double lastErrorR = 0;

    private Follower follower;
    DcMotorEx slide, armMotorL, armMotorR;
    Servo servo1, servo2, garra;
    boolean holdingPosition = false, modeBase = false;
    private final Pose startPose = new Pose(0, 0, 0);

    int targetPosition;

    long lastTime;

    @Override
    public void init() {
        lastTime = System.nanoTime();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        garra = hardwareMap.get(Servo.class, "garra");

        servo1.setDirection(Servo.Direction.REVERSE);

        armMotorL = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        armMotorR = hardwareMap.get(DcMotorEx.class, "armmotorright");
        armMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        double j = gamepad2.right_stick_y;

        // Parâmetros do PID

        long lastTime = System.nanoTime();

        if (j > 0 || j < 0) {
            // Controle manual quando o joystick está sendo movido
            armMotorL.setPower(j);
            armMotorR.setPower(j);

            // Salva a posição como o "targetPosition" quando o movimento começa
            targetPosition = (int) ((armMotorL.getCurrentPosition() + armMotorR.getCurrentPosition()) / 2.0);
            integralL = 0;  // Zera o integral ao iniciar o movimento
            integralR = 0;
            modeBase = false;
        } else if (!modeBase) {
            // Controle PID para segurar a posição quando o joystick está solto

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            // Posição atual dos motores
            double currentPosL = armMotorL.getCurrentPosition();
            double currentPosR = armMotorR.getCurrentPosition();

            // Erro para o motor esquerdo (L)
            double errorL = targetPosition - currentPosL;
            integralL += errorL * deltaTime;
            double derivativeL = (errorL - lastErrorL) / deltaTime;
            double outputL = kP * errorL + kI * integralL + kD * derivativeL;

            // Erro para o motor direito (R)
            double errorR = targetPosition - currentPosR;
            integralR += errorR * deltaTime;
            double derivativeR = (errorR - lastErrorR) / deltaTime;
            double outputR = kP * errorR + kI * integralR + kD * derivativeR;

            // Aplica a potência com limites de -0.5 a 0.5
            armMotorL.setPower(Range.clip(outputL, -0.5, 0.5));
            armMotorR.setPower(Range.clip(outputR, -0.5, 0.5));

            // Atualiza os erros para a próxima iteração
            lastErrorL = errorL;
            lastErrorR = errorR;
            modeBase = true;
        }

        if(gamepad2.dpad_down){
            armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Atualiza o telemetria para debugar
        telemetry.addData("Left Pos", armMotorL.getCurrentPosition());
        telemetry.addData("Right Pos", armMotorR.getCurrentPosition());
        telemetry.update();
    }



    //Todo: Mover servo
    public void moveServo(){

        if(gamepad2.a){
            servo1.setPosition(0.85);
            servo2.setPosition(0.85);
            telemetry.addLine("Pick");
        }else if(gamepad2.y){
            servo1.setPosition(0.0);
            servo2.setPosition(0.0);
            telemetry.addLine("Clip");
        }else if(gamepad2.b){
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
            telemetry.addLine("90");
        }

        if(gamepad2.right_bumper){
            garra.setPosition(0.6);
        }else{
            garra.setPosition(0);
        }
    }
}