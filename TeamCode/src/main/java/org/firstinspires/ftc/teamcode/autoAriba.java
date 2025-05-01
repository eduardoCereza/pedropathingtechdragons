package org.firstinspires.ftc.teamcode;

//importações referentes ao pedro pathing

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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


@Autonomous(name = "Ariba mexico mundial oficial")
public class autoAriba extends OpMode {

    public void clipPos(){
        leftS.setPosition(0.0);
        rightS.setPosition(0.0);
        clippos = 1;
        pickpos = 0;
        specimenpickpos = 0;
    }
    public void pickPos(){
        leftS.setPosition(1.0);
        rightS.setPosition(1.0);
        clippos = 0;
        pickpos = 1;
        specimenpickpos = 0;

    }
    public void specimenPickpos(){
        leftS.setPosition(0.5);
        rightS.setPosition(0.5);
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

        while (Right.getCurrentPosition() <= target){

            Left.setTargetPosition(-target);
            Right.setTargetPosition(target);

            Left.setPower(0.8);
            Right.setPower(0.8);
            holdArm = 0;
        }
        Left.setPower(0.0);
        Right.setPower(0.0);
        holdArm =1;
    }
    public void descer(int target){

        while (Right.getCurrentPosition() >= target){

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
    public void extender( int target){

        while (slide.getCurrentPosition() >= target){
            slide.setTargetPosition(target);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(-1.0);
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
    int isopen;
    int num;
    int  specimenpickpos, clippos, pickpos;
    int holdSlide;
    int holdArm;
    Pose pose;
    private DcMotorEx slide, Left, Right;
    private Servo garra; //servo da garra/ponta
    private Servo leftS, rightS;
    private Follower follower; //sla tbm
    private Timer pathTimer, opmodeTimer; //sla ja veio no código
    private int pathState; //variável de controle das trajetórias e ações
    // y = lados (se for maior vai para a direita)
    // x = frente e tras (se for maior vai para frente)
    private final Pose startPose = new Pose(0, 71, Math.toRadians(180)); //posição inicial do robô
    private final Pose ClipPose = new Pose(24.3, 71, Math.toRadians(180));
    private final Pose Control1 = new Pose(6, 20, Math.toRadians(180));
    private final Pose move2 = new Pose(45, 33, Math.toRadians(180)); //vai para frente
    private final Pose move3 = new Pose(45, 15, Math.toRadians(180));
    private final Pose move4 = new Pose(6, 15, Math.toRadians(180)); //empurra o sample para o jogador humano
    private final Pose move5 = new Pose(45,-5, Math.toRadians(180));// vai para a direita na frente do segundo sample
    private final Pose move6 = new Pose(6, -5, Math.toRadians(180)); //empurra o segundo sample para a área do jogador humano
    private final Pose move8 = new Pose(50, -10, Math.toRadians(180)); //foi para frente
    private final Pose move9 = new Pose(50, -30, Math.toRadians(180)); //foi para a direita na frente do terceiro sample
    private final Pose move10 = new Pose(5, -30, Math.toRadians(180)); //empurrou o terceiro sample para a area do jogador humano
    private final Pose move11 = new Pose(30, -30, Math.toRadians(180)); // voltou para frente
    private PathChain traj1, traj2, traj3, traj4; //conjunto de trajetórias

    public void buildPaths() {

        traj1 = follower.pathBuilder()
                //vai para frente para clipar
                .addPath(new BezierLine(new Point(startPose), new Point(ClipPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        traj2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(ClipPose), new Point(Control1), new Point(move2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(move2), new Point(move3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(move3), new Point(move4)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(move4), new Point(move3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        traj3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(move3), new Point(move5)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(move5), new Point(move6)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        traj4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(move6), new Point(ClipPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    //dependendo de como funcionar a movimentação do atuador, esses cases vão precisar ser dividos e dividir as trajetórias neles, testar antes
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                subir(390);
                extender(-700);
                closed();
                follower.followPath(traj1, 0.5, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && pathState == 1){
                    extender(-1900);
                    open();
                    num = 1;
                }
                if (num == 1){
                    //mudar
                    recuar(-250);
                    descer(0);
                    follower.followPath(traj2, 0.95, false);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathState ==2){
                    follower.followPath(traj3, 1.0, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy() && pathState == 3){
                    specimenPickpos();
                    closed();
                    clipPos();
                    follower.followPath(traj4, 1.0, true);

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
            leftS.setPosition(0.0);
            rightS.setPosition(0.0);
        }
        if (pickpos == 1){
            leftS.setPosition(1.0);
            rightS.setPosition(1.0);
        }
        if (specimenpickpos == 1){
            leftS.setPosition(0.5);
            rightS.setPosition(0.5);
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
        leftS = hardwareMap.get(Servo.class, "servo2");
        rightS = hardwareMap.get(Servo.class, "servo1");
        garra = hardwareMap.get(Servo.class, "garra");
        Left = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        Right = hardwareMap.get(DcMotorEx.class, "armmotorright");

        leftS.setDirection(Servo.Direction.REVERSE);

        Left.setDirection(DcMotorEx.Direction.REVERSE);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        garra.setPosition(0);

        rightS.setPosition(0);
        leftS.setPosition(0);

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