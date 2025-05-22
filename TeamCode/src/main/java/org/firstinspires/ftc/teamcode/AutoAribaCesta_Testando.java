package org.firstinspires.ftc.teamcode;

//importações referentes ao pedro pathing

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;

import java.lang.annotation.Target;


@Autonomous(name = "Cesta México Oficial - Testando")
public class AutoAribaCesta_Testando extends OpMode {

    public void clipPos(){
        servo.setPosition(1.0);
        clippos = 1;
        pickpos = 0;
        specimenpickpos = 0;
    }

    public void servo(double target){
        servo.setPosition(target);
    }
    public void pickPos(){
        servo.setPosition(0.0);
        clippos = 0;
        pickpos = 1;
        specimenpickpos = 0;

    }
    public void specimenPickpos(){
        servo.setPosition(0.6);
        clippos = 0;
        pickpos= 0;
        specimenpickpos = 1;
    }
    public void closed(){
        garra.setPosition(0.0);
        isopen = 0;
    }
    public void open(){
        garra.setPosition(0.6);
        isopen = 1;
    }
    public void subir(int target){

        while (Left.getCurrentPosition() <= target){

            Left.setTargetPosition(target);

            Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Left.setPower(1);
            Right.setPower(1);
            holdArm = 0;
        }

        Left.setPower(0);
        Right.setPower(0);
        //holdArm = 1;
    }
    public void descer(int target){

        while (Left.getCurrentPosition() >= target){

            Left.setTargetPosition(target);
            Right.setTargetPosition(-target);

            Left.setPower(-0.4);
            Right.setPower(-0.4);
            holdArm = 0;
        }
        Left.setPower(0.0);
        Right.setPower(0.0);
        //holdArm = 1;
    }

    public void hold(){

        PIDFController controller;

        double minPower = 0.7;
        double maxPower = 1.0;
        controller = new PIDFController(12, 4, 5, 13);
        controller.setInputRange(-4000, 4000);
        controller.setOutputRange(minPower, maxPower);

        double powerM = maxPower + controller.getComputedOutput(Left.getCurrentPosition());
        double powerM1 = maxPower + controller.getComputedOutput(Right.getCurrentPosition());

        Left.setTargetPosition(Left.getCurrentPosition());
        Right.setTargetPosition(Right.getCurrentPosition());

        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Left.setPower(powerM);
        Right.setPower(powerM1);

    }
    public void extender(int target){
        while (slide.getCurrentPosition() >= target){
            slide.setTargetPosition(target);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1.0);
            holdSlide = 0;
        }
        slide.setPower(0.0);
        holdSlide = 1;

    }
    public void recuar(int target){

        while(slide.getCurrentPosition() <= target){
            slide.setTargetPosition(target);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1.0);
            holdSlide = 0;
        }
        slide.setPower(0.0);
        holdSlide = 1;
    }
    public void stay(){
        int currentPosition = slide.getCurrentPosition();

        slide.setTargetPosition(currentPosition); // Define a posição atual como alvo
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
        slide.setPower(0.5); // Aplica uma pequena potência para segurar a posição
    }

    Pose pose;
    int isopen;
    int num;
    int  specimenpickpos, clippos, pickpos;
    int holdSlide;
    int holdArm;
    private DcMotorEx slide, Left, Right;
    private Servo garra; //servo1 da garra/ponta
    private Servo servo;
    private Follower follower; //sla tbm
    private Timer pathTimer, opmodeTimer; //sla ja veio no código
    private int pathState; //variável de controle das trajetórias e ações
    // y = lados (se for maior vai para a direita)
    // x = frente e tras (se for maior vai para frente)
    private final Pose startPose = new Pose(0, 80, Math.toRadians(0));//posição inicial do robô

    private final Pose move0 = new Pose(14.5, 153, Math.toRadians(0));
    private final Pose move01 = new Pose(16, 156, Math.toRadians(0));
    private final Pose move02 = new Pose(17, 160, Math.toRadians(0));

    private final Pose move1 = new Pose(13, 150, Math.toRadians(0));
    private final Pose move2 = new Pose(11, 145, Math.toRadians(0));


    private PathChain traj0, traj1, traj2, traj3, traj4; //conjunto de trajetórias

    public void buildPaths() {

        //sempre acerta

        traj0 = follower.pathBuilder()
                //vai até o a cesta para colocar o specimen amarelo que já esta no robo
                .addPath(new BezierLine(new Point(startPose), new Point(move0)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        //variavel

        traj1 = follower.pathBuilder()
                //vai até o primeiro specimen amarelo
                .addPath(new BezierLine(new Point(move0), new Point(move1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        traj2 = follower.pathBuilder()//vai para trás
                .addPath(new BezierLine(new Point(move1), new Point(move01)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        //mudar
        traj3 = follower.pathBuilder()//vai para o segundo specimen amarelo
                .addPath(new BezierLine(new Point(move01), new Point(move2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        traj4 = follower.pathBuilder()//vai para o segundo specimen amarelo
                .addPath(new BezierLine(new Point(move2), new Point(move02)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

    //dependendo de como funcionar a movimentação do atuador, esses cases vão precisar ser dividos e dividir as trajetórias neles, testar antes
    public void autonomousPathUpdate(){
        switch (pathState) {
            //faz a trajetória

            //TODO: primeiro
            case 0: //certo
                servo(0);
                follower.followPath(traj0, 1, false);
                setPathState(1);
                break;
            case 1: //certo
                if (!follower.isBusy() && pathState == 1) {
                    subir(720);
                    extender(-3100);
                    servo(1);
                    pathTimer.resetTimer();
                    setPathState(101);
                }
                break;
            case 2: //certo
                if (!follower.isBusy() && pathState == 2) {
                    open();
                    open();
                    servo(0);
                    setPathState(3);
                }
                break;

            case 3: //certo
                if (!follower.isBusy() && pathState == 3) {
                    recuar(-10);
                    descer(0);
                    Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setPathState(4);
                }
                break;

                //TODO: segundo
            case 4: //certo
                follower.followPath(traj1, 0.35, false);
                setPathState(5);
                break;
            case 5://certo
                if(!follower.isBusy() && pathState == 5){
                extender(-1850);
                open();
                //pathTimer.resetTimer();
                setPathState(6);
            }
                break;
            case 6://certo
                if(!follower.isBusy() && pathState == 6){
                    closed();
                    closed();
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathState == 7) {
                    recuar(-10);
                    setPathState(8);
                }
                break;
            case 8://certo
                subir(720);
                follower.followPath(traj2, 1, false);
                setPathState(9);
                break;
            case 9://certo
                if (!follower.isBusy() && pathState == 9) {
                    extender(-3300);
                    servo(1);
                    pathTimer.resetTimer();
                    setPathState(103);
                }
                break;
            case 10://certo
                if (!follower.isBusy() && pathState == 10) {
                    open();
                    open();
                    servo(0);
                    setPathState(11);
                }
                break;
            case 11://certo
                if (!follower.isBusy() && pathState == 11) {
                    recuar(-30);
                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    descer(0);
                    setPathState(12);
                }
                break;

                //TODO: terceiro
            case 12:
                follower.followPath(traj3, 0.7, false);
                setPathState(13);
                break;
            case 13:
                if(!follower.isBusy() && pathState == 13){
                    extender(-2330);
                    open();
                    //pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy() && pathState == 14){
                    closed();
                    closed();
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy() && pathState == 15) {
                    recuar(-10);
                    subir(730);
                    setPathState(16);
                }
                break;
            case 16:
                follower.followPath(traj4, 1, false);
                setPathState(17);
                break;
            case 17:
                if (!follower.isBusy() && pathState == 17) {
                    extender(-3300);
                    servo(1);
                    pathTimer.resetTimer();
                    setPathState(107);
                }
                break;
            case 18:
                if (!follower.isBusy() && pathState == 18) {
                    open();
                    open();
                    servo(0);
                    setPathState(19);
                }
                break;
            case 19:
                break;
            case 101:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    setPathState(2);
                }
                break;
            case 102:
                if(pathTimer.getElapsedTimeSeconds() > 0.5){
                    setPathState(6);
                }
                break;
            case 103:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    setPathState(10);
                }
                break;
            case 104:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    setPathState(14);
                }
                break;
            case 107:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    setPathState(18);
                }
                break;
        }
    }
    //controle das trajetórias
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    //loop
    @Override
    public void loop() {

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("pos", slide.getCurrentPosition());
        telemetry.addData("state", holdSlide);
        telemetry.addData("state arm", holdArm);
        telemetry.update();

        pose = follower.getPose();

        //talvez precise mudar
        if (holdSlide == 1){
            stay();
        }

        if (holdArm == 1){
            hold();
        }

        if (isopen == 0){
            garra.setPosition(0);
        }
        if (clippos == 1){
            servo.setPosition(0.95);
        }
        if (pickpos == 1){
            servo.setPosition(0.0);
        }
        if (specimenpickpos == 1){
            servo.setPosition(0.6);
        }

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("RIGHT POS: ", Right.getCurrentPosition());
        telemetry.addData("LEFT POS: ", Left.getCurrentPosition());
        telemetry.addData("RIGHT MODE: ", Right.getDirection());
        telemetry.addData("LEFT MODE: ", Left.getDirection());
        telemetry.update();




    }

    //se precisar fazer alguma ação no init tem que por aq
    @Override
    public void init() {

        /*
        holdSlide = 0;

        holdArm = 1;

        isopen = 0;

        clippos = 1;
        pickpos = 0;
        specimenpickpos = 0;

         */
        holdArm = 1;
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        servo = hardwareMap.get(Servo.class, "servo1");
        garra = hardwareMap.get(Servo.class, "garra");
        Left = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        Right = hardwareMap.get(DcMotorEx.class, "armmotorright");

        servo.setDirection(Servo.Direction.REVERSE);

        //Right.setDirection(DcMotorEx.Direction.REVERSE);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        garra.setPosition(0);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower =  new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    //só um loop pra quando dar init
    @Override
    public void init_loop() {

    }

    //quando começar ele define a variável de controle como 0 e ja começa as ações
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    //quando mandar parar ele fará oque está aq
    @Override
    public void stop() {
        holdArm = 0;
        holdSlide = 0;
        isopen = 1;
        clippos = 0;
        pickpos = 0;
        specimenpickpos = 0;
    }
}