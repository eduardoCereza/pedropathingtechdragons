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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado Teste Atuador")
public class teste_atuado extends OpMode {

    // Definir constantes do PIDF
    double kP = 0.1, kI = 0.01, kD = 0.05, kF = 0.0;

    // Variáveis para controle
    double setPoint = 0;
    double integralMotor1 = 0, integralMotor2 = 0;
    double lastErrorMotor1 = 0, lastErrorMotor2 = 0;

    private Follower follower;
    DcMotorEx slide, armMotorL, armMotorR;
    double powerR, powerL;
    Servo servo1, servo2, garra;
    boolean holdingPosition = false, modeBase = false;
    private final Pose startPose = new Pose(0, 0, 0);
    int targetR, targetL, estado;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo2.setDirection(Servo.Direction.REVERSE);
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

        if(gamepad1.a) {
            estado = 2;
        }else if(gamepad1.b){
            estado = 1;
        }

        if(estado == 1){
            follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addLine("Normal Chassi");
        }else if(estado ==2){
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addLine("Inverso Chassi");

        }

        moveServo();
        moveSlide();
        armBase();

        telemetry.addData("Pos Left: ", armMotorL.getCurrentPosition());
        telemetry.addData("Pos Right: ", armMotorR.getCurrentPosition());

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

        // Quando o analógico for solto, registra a posição dos motores
        if (gamepad2.right_stick_y == 0) {
            setPoint = (currentPosMotor1 + currentPosMotor2) / 2; // Média das posições
        }

        // Calcula os erros para ambos os motores
        errorMotor1 = setPoint - currentPosMotor1;
        errorMotor2 = setPoint - currentPosMotor2;

        // Calcula os integrals e derivatives para ambos os motores
        integralMotor1 += errorMotor1;
        integralMotor2 += errorMotor2;
        derivativeMotor1 = errorMotor1 - lastErrorMotor1;
        derivativeMotor2 = errorMotor2 - lastErrorMotor2;

    // Calcula os PIDF para ambos os motores
        double outputMotor1 = kP * errorMotor1 + kI * integralMotor1 + kD * derivativeMotor1 + kF * (setPoint - currentPosMotor1);
        double outputMotor2 = kP * errorMotor2 + kI * integralMotor2 + kD * derivativeMotor2 + kF * (setPoint - currentPosMotor2);

    // Aplica a saída aos motores
        armMotorR.setPower(outputMotor1);
        armMotorL.setPower(outputMotor2);

    // Atualiza os erros anteriores para o próximo ciclo
        lastErrorMotor1 = errorMotor1;
        lastErrorMotor2 = errorMotor2;

        telemetry.addData("POS LEFT:", armMotorL.getCurrentPosition());
        telemetry.addData("POS RIGHT: ", armMotorR.getCurrentPosition());
    }


    //Todo: Mover servo
    public void moveServo(){

        if(gamepad2.y){
            servo1.setPosition(0.85);
            servo2.setPosition(0.85);
            telemetry.addLine("Pick");
        }else if(gamepad2.a){
            servo1.setPosition(0);
            servo2.setPosition(0);
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