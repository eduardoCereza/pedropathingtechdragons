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


@Autonomous(name = "Estratégia 2 - Mundial", group = "Examples")
public class Estrategia_2 extends OpMode {

    public void clipPos(){
        leftS.setPosition(0.0);
        rightS.setPosition(1.0);
        clippos = 1;
        pickpos = 0;
        specimenpickpos = 0;
    }
    public void pickPos(){
        leftS.setPosition(1.0);
        rightS.setPosition(0.0);
        clippos = 0;
        pickpos = 1;
        specimenpickpos = 0;

    }
    public void specimenPickpos(){
        leftS.setPosition(0.6);
        rightS.setPosition(0.4);
        clippos = 0;
        pickpos= 0;
        specimenpickpos = 0;
    }
    public void closed(){
        garra.setPosition(0.0);
        isopen = 0;
    }
    public void open(){
        garra.setPosition(0.6);
        isopen = 1;
    }
    public void subir(int targetL, int targetR){

        while (Left.getCurrentPosition() <= -targetL && Right.getCurrentPosition() <= targetR){
            Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Left.setTargetPosition(targetL);
            Right.setTargetPosition(targetR);

            Left.setPower(1);
            Right.setPower(1);
        }
    }
    public void descer(int target){

        while (Left.getCurrentPosition() >= -target && Right.getCurrentPosition() >= target){
            Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Left.setTargetPosition(target);
            Right.setTargetPosition(-target);

            Left.setPower(1);
            Right.setPower(-1);}
    }
    public void hold(){

        PIDFController controller;

        double minPower = 0.2;
        double maxPower = 0.5;
        controller = new PIDFController(10, 3, 4, 12);
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

        while (slide.getCurrentPosition() <= target){
            slide.setTargetPosition(-target);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(-1.0);
        }
        slide.setPower(0.0);
    }
    public void recuar(int target){

        while(slide.getCurrentPosition() >= target){
            slide.setTargetPosition(target);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1.0);
        }
        slide.setPower(0.0);
    }
    int isopen;
    int  specimenpickpos, clippos, pickpos;
    private DcMotorEx slide, Left, Right;
    private Servo garra; //servo da garra/ponta
    private Servo leftS, rightS;
    private Follower follower; //sla tbm
    private Timer pathTimer, opmodeTimer; //sla ja veio no código
    private int pathState; //variável de controle das trajetórias e ações
    // y = lados (se for maior vai para a direita)
    // x = frente e tras (se for maior vai para frente)
    private final Pose startPose = new Pose(0, 70, Math.toRadians(180)); //posição inicial do robô
    private final Pose move1 = new Pose(-10, 20, Math.toRadians(180));
    private final Pose move2 = new Pose(-5, 20, Math.toRadians(180));
    private final Pose move3 = new Pose(31.22, 125.5, Math.toRadians(180));
    private final Pose move4 = new Pose(22.969, 131.289, Math.toRadians(180));
    private final Pose move5 = new Pose(31.220, 133.742, Math.toRadians(180));
    private final Pose move6 = new Pose(93.438, 81.783, Math.toRadians(180));


    private PathChain traj1, traj2, traj3, traj4; //conjunto de trajetórias

    public void buildPaths() {

        traj1 = follower.pathBuilder()
                //vai até o primeiro specimen amarelo
                .addPath(new BezierLine(new Point(startPose), new Point(move1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para trás
                .addPath(new BezierLine(new Point(move1), new Point(move2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
        .build();

        traj2 = follower.pathBuilder()//vai para o segundo specimen amarelo
        .addPath(new BezierLine(new Point(move2), new Point(move3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                //vai para tras
                .addPath(new BezierLine(new Point(move3), new Point(move4)))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        traj3 = follower.pathBuilder()//vai para tras//vai para o último specimen amarelo
                .addPath(new BezierLine(new Point(move4), new Point(move5)))
                .setTangentHeadingInterpolation()

                //vai para tras
                .addPath(new BezierLine(new Point(move5), new Point(move6)))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();

        traj4 = follower.pathBuilder()//vai para tras//vai para a area de pontuacao
                .addPath(new BezierLine(new Point(move6), new Point(move6)))
                .setConstantHeadingInterpolation(Math.toRadians(180)).build();
    }

    //dependendo de como funcionar a movimentação do atuador, esses cases vão precisar ser dividos e dividir as trajetórias neles, testar antes
    public void autonomousPathUpdate() {
        switch (pathState) {
            //faz a trajetória
            case 0:
                //inicia a trajetória
                follower.followPath(traj1, 0.6,true);

                pathState = 1;


            case 1:
                if(!follower.isBusy()){
                    //extender(-900);
                    //open();
                    //closed();
                    //recuar(0);

                }
                follower.followPath(traj2, true);
                pathState = 2;
                /*
            case 2:
                if(!follower.isBusy()){

                }
                follower.followPath(traj3, true);
                pathState = 3;
            case 3:
                if(!follower.isBusy()){

                }
                follower.followPath(traj4, true);

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

        if (follower.isBusy() && slide.getPower() < 0.3){
            int currentPosition = slide.getCurrentPosition();

            slide.setTargetPosition(currentPosition); // Define a posição atual como alvo
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Mantém o motor na posição
            slide.setPower(0.1); // Aplica uma pequena potência para segurar a posição

        }
        if (follower.isBusy() && Left.getPower() < 0.3 && Right.getPower() < 0.3){
            hold();
        }
        if (isopen == 0){
            garra.setPosition(0.0);
        }
        if (clippos == 1){
            leftS.setPosition(0.0);
            rightS.setPosition(1.0);
        }
        if (pickpos == 1){
            leftS.setPosition(1.0);
            rightS.setPosition(0.0);
        }
        if (specimenpickpos == 1){
            leftS.setPosition(0);
            rightS.setPosition(0);
        }

        follower.update();
        autonomousPathUpdate();
    }

    //se precisar fazer alguma ação no init tem que por aq
    @Override
    public void init() {

        isopen = 0;

        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        leftS = hardwareMap.get(Servo.class, "servo2");
        rightS = hardwareMap.get(Servo.class, "servo1");
        garra = hardwareMap.get(Servo.class, "garra");
        Left = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        Right = hardwareMap.get(DcMotorEx.class, "armmotorright");

        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    }

}
