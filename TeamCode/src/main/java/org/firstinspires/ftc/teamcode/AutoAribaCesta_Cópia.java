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


@Autonomous(name = "Cesta México Oficial - Cópia")
public class AutoAribaCesta_Cópia extends OpMode {

    public void clipPos(){
        servo.setPosition(0.95);
        clippos = 1;
        pickpos = 0;
        specimenpickpos = 0;
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
        garra.setPosition(0.5);
        isopen = 1;
    }
    public void subir(int target){

        while (Right.getCurrentPosition() <= target){

            Right.setTargetPosition(target);

            Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Left.setPower(0.5);
            Right.setPower(0.5);
            holdArm = 0;
        }
        Left.setTargetPosition(Left.getCurrentPosition());
        Right.setTargetPosition(Right.getCurrentPosition());

        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Left.setPower(1);
        Right.setPower(1);
        holdArm =1;
    }
    public void descer2(int target){

        while (Right.getCurrentPosition() <= target){

            Right.setTargetPosition(target);

            Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Left.setPower(0.5);
            Right.setPower(0.5);
            holdArm = 0;
        }
        Left.setTargetPosition(Left.getCurrentPosition());
        Right.setTargetPosition(Right.getCurrentPosition());

        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Left.setPower(1);
        Right.setPower(1);
        holdArm =1;
    }

    public void descer(int target){

        while (Right.getCurrentPosition() <= target){

            Left.setTargetPosition(target);
            Right.setTargetPosition(-target);

            Left.setPower(-0.4);
            Right.setPower(-0.4);
            holdArm = 0;
        }
        Left.setPower(0.0);
        Right.setPower(0.0);
        holdArm = 1;
    }
    public void hold(){

        PIDFController controller;

        double minPower = 0.4;
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
        slide.setPower(0.1); // Aplica uma pequena potência para segurar a posição
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

    private final Pose move0 = new Pose(38, 132, Math.toRadians(-90.00));
    private final Pose move1 = new Pose(30, 110, Math.toRadians(0));
    private final Pose move2 = new Pose(-5, 20, Math.toRadians(180));
    private final Pose move3 = new Pose(31.22, 125.5, Math.toRadians(180));
    private final Pose move4 = new Pose(22.969, 131.289, Math.toRadians(180));
    private final Pose move5 = new Pose(31.220, 133.742, Math.toRadians(180));
    private final Pose move6 = new Pose(93.438, 81.783, Math.toRadians(180));


    private PathChain traj0, traj1, traj2, traj3, traj4, traj5, traj6, traj7; //conjunto de trajetórias

    public void buildPaths() {

        traj0 = follower.pathBuilder()
                //vai até o a cesta para colocar o specimen amarelo que já esta no robo
                .addPath(new BezierLine(new Point(startPose), new Point(move0)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-90.00))
                .build();


        traj1 = follower.pathBuilder()
                //vai até o primeiro specimen amarelo
                .addPath(new BezierLine(new Point(move0), new Point(move1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        traj2 = follower.pathBuilder()//vai para trás
                .addPath(new BezierLine(new Point(move1), new Point(move2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        traj3 = follower.pathBuilder()//vai para o segundo specimen amarelo
                .addPath(new BezierLine(new Point(move2), new Point(move3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

                //vai para tras
        traj4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(move3), new Point(move4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        traj5 = follower.pathBuilder()//vai para o último specimen amarelo
                .addPath(new BezierLine(new Point(move4), new Point(move5)))
                .setTangentHeadingInterpolation()
                .build();

                //vai para tras
        traj6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(move5), new Point(move6)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        traj7 = follower.pathBuilder()//vai para a area de pontuacao
                .addPath(new BezierLine(new Point(move6), new Point(move6)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    //dependendo de como funcionar a movimentação do atuador, esses cases vão precisar ser dividos e dividir as trajetórias neles, testar antes
    public void autonomousPathUpdate() {
        switch (pathState) {
            //faz a trajetória
            case 0:
                subir(650);
                follower.followPath(traj0, 1,true);
                setPathState(1);
                break;
            case 1:


                if (!follower.isBusy() && pathState == 1){
                    specimenPickpos();
                    extender(-3600);
                    num = 1;

                }
                if (num == 1){
                    open();
                    recuar(0);
                    descer2(0);
                    setPathState(2);
                }
                break;
                /*
            case 2:
                //frente
                follower.followPath(traj1, 1, true);
                pathState = 3;
                break;
            case 3:
                //pegar
                extender(-1500);
                closed();
                recuar(0);
                pathState = 4;
                break;
            case 4:
                //tras
                follower.followPath(traj2, 1, true);
                pathState = 5;
                break;
            case 5:
                //colocar
                clipPos();
                subir(-650);
                extender(-1800);
                open();
                specimenPickpos();
                recuar(0);
                descer(0);
                pathState = 6;
                break;
            case 6:
                //frente
                follower.followPath(traj3, 1, true);
                pathState = 7;
                break;
            case 7:
                //pegar
                extender(-1500);
                closed();
                recuar(0);
                pathState = 8;
            case 8:
                //tras
                follower.followPath(traj4, 1,  true);
                pathState = 9;
                break;
            case 9:
                //colocar
                clipPos();
                subir(-650);
                extender(-1800);
                open();
                specimenPickpos();
                recuar(0);
                descer(0);
                pathState = 10;
                break;
            case 10:
                //frente
                follower.followPath(traj5, 1, true);
                pathState = 11;
                break;
            case 11:
                //pegar
                extender(-1500);
                closed();
                recuar(0);
                pathState = 12;
                break;
            case 12:
                //tras
                follower.followPath(traj6, 1, true);
                pathState = 13;
                break;
            case 13:
                //colocar
                clipPos();
                subir(-650);
                extender(-1800);
                open();
                specimenPickpos();
                recuar(0);
                descer(0);
                pathState = 14;
                break;
            case 14:
                //end
                follower.followPath(traj7, 1, true);
                break;

                 */
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

    }

    //se precisar fazer alguma ação no init tem que por aq
    @Override
    public void init() {

        holdSlide = 0;

        holdArm = 1;

        isopen = 0;

        clippos = 1;
        pickpos = 0;
        specimenpickpos = 0;

        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        servo = hardwareMap.get(Servo.class, "servo1");
        garra = hardwareMap.get(Servo.class, "garra");
        Left = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        Right = hardwareMap.get(DcMotorEx.class, "armmotorright");

        servo.setDirection(Servo.Direction.REVERSE);

        Left.setDirection(DcMotorEx.Direction.REVERSE);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        garra.setPosition(0);

        servo.setPosition(0.5);

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