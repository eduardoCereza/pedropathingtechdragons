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


@Autonomous(name = "Ariba mexico mundial oficial")
public class autoAriba_cópia extends OpMode {

    //left =
    //right =
    public void clipPos(){
        leftS.setPosition(1.0);
        rightS.setPosition(0.0);
        clippos = 1;
        pickpos = 0;
        specimenpickpos = 0;
    }
    public void pickPos(){
        leftS.setPosition(0.0);
        rightS.setPosition(1.0);
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
    public void subir(int target){

        while (Left.getCurrentPosition() <= -target && Right.getCurrentPosition() <= target){
            PController controllerR, controllerL;

            double minPower = 0.01;
            double maxPower = 0.5;
            controllerR = new PController(1);
            controllerR.setInputRange(0, 700);
            controllerR.setSetPoint(450);
            controllerR.setOutputRange(minPower, maxPower);

            controllerL = new PController(1);
            controllerL.setInputRange(0, 700);
            controllerL.setSetPoint(450);
            controllerL.setOutputRange(minPower, maxPower);

            double powerL = minPower + controllerL.getComputedOutput(Left.getCurrentPosition());
            double powerR = minPower + controllerR.getComputedOutput(Right.getCurrentPosition());

            double powerLD = minPower - controllerL.getComputedOutput(Left.getCurrentPosition());
            double powerRD = minPower - controllerR.getComputedOutput(Right.getCurrentPosition());

            Left.setTargetPosition(Left.getCurrentPosition());
            Right.setTargetPosition(Right.getCurrentPosition());

            if (Right.getCurrentPosition() < target) {
                Left.setPower(powerL);
                Right.setPower(powerR);
            }else{
                Left.setPower(powerLD);
                Right.setPower(powerRD);            }
        }
        Left.setPower(0.0);
        Right.setPower(0.0);
    }
    public void descer(int target){

        while (Left.getCurrentPosition() >= -target && Right.getCurrentPosition() >= target){
            Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Left.setTargetPosition(target);
            Right.setTargetPosition(-target);

            Left.setPower(1);
            Right.setPower(-1);}
        Left.setPower(0.0);
        Right.setPower(0.0);
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
            holdSlide = 0;
        }
        slide.setPower(0.0);
        holdSlide = 1;
    }
    public void recuar(int target){

        while(slide.getCurrentPosition() >= target){
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
    int  specimenpickpos, clippos, pickpos;
    int holdSlide;
    int holdArm;
    private DcMotorEx slide, Left, Right;
    private Servo garra; //servo da garra/ponta
    private Servo leftS, rightS;
    private Follower follower; //sla tbm
    private Timer pathTimer, opmodeTimer; //sla ja veio no código
    private int pathState; //variável de controle das trajetórias e ações
    // y = lados (se for maior vai para a direita)
    // x = frente e tras (se for maior vai para frente)
    private final Pose startPose = new Pose(0, 71, Math.toRadians(180)); //posição inicial do robô
    private final Pose ClipPose = new Pose(10, 71, Math.toRadians(180)); //clipa
    private final Pose move1 = new Pose(10, 30, Math.toRadians(180)); //após clipar vai para a direita
    private final Pose move2 = new Pose(50, 30, Math.toRadians(180)); //vai para frente
    private final Pose move3 = new Pose(50, 15, Math.toRadians(180));// vai para direita na frente do primeiro sample
    private final Pose move4 = new Pose(5, 15, Math.toRadians(180)); //empurra o sample para o jogador humano
    private final Pose move5 = new Pose(50, 15, Math.toRadians(180)); //vai para frente
    private final Pose move6 = new Pose(50,-10, Math.toRadians(180));// vai para a direita na frente do segundo sample
    private final Pose move7 = new Pose(5, -10, Math.toRadians(180)); //empurra o segundo sample para a área do jogador humano
    private final Pose move8 = new Pose(50, -10, Math.toRadians(180)); //foi para frente
    private final Pose move9 = new Pose(50, -30, Math.toRadians(180)); //foi para a direita na frente do terceiro sample
    private final Pose move10 = new Pose(5, -30, Math.toRadians(180)); //empurrou o terceiro sample para a area do jogador humano
    private final Pose move11 = new Pose(30, -30, Math.toRadians(180)); // voltou para frente
    private PathChain traj1; //conjunto de trajetórias

    public void buildPaths() {

        traj1 = follower.pathBuilder()
                //vai para frente para clipar
                .addPath(new BezierLine(new Point(startPose), new Point(ClipPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para o lado
                .addPath(new BezierLine(new Point(ClipPose), new Point(move1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para frente próximo do primeiro sample
                .addPath(new BezierLine(new Point(move1), new Point(move2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para a direita na frente do primeiro sample
                .addPath(new BezierLine(new Point(move2), new Point(move3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para tras empurrando o primeiro sample para o jogador
                .addPath(new BezierLine(new Point(move3), new Point(move4)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para frente para perto do segundo sample
                .addPath(new BezierLine(new Point(move4), new Point(move5)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para a direita na frente do segundo sample
                .addPath(new BezierLine(new Point(move5), new Point(move6)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //empurra o segundo sample para o jogador humano
                .addPath(new BezierLine(new Point(move6), new Point(move7)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para a frente próximo o terceiro sample
                .addPath(new BezierLine(new Point(move7), new Point(move8)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para a direita na frente do terceiro sample
                .addPath(new BezierLine(new Point(move8), new Point(move9)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //empurra o terceiro sample para o jogador humano
                .addPath(new BezierLine(new Point(move9), new Point(move10)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                //vai para frente um pouco para sair da área do jogador humano (mudará de acordo com a estratégia
                .addPath(new BezierLine(new Point(move10), new Point(move11)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    //dependendo de como funcionar a movimentação do atuador, esses cases vão precisar ser dividos e dividir as trajetórias neles, testar antes
    public void autonomousPathUpdate() {
        switch (pathState) {
            //faz a trajetória
            case 0:

                //inicia a trajetória
                // troca para fazer nada


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

        //talvez precise mudar
        if (holdSlide == 1){
            stay();
        }


        if (isopen == 0){
            garra.setPosition(0.0);
        }
        if (clippos == 1){
            leftS.setPosition(1.0);
            rightS.setPosition(0.0);
        }
        if (pickpos == 1){
            leftS.setPosition(0.0);
            rightS.setPosition(1.0);
        }
        if (specimenpickpos == 1){
            leftS.setPosition(0.4);
            rightS.setPosition(0.6);
        }

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    //se precisar fazer alguma ação no init tem que por aq
    @Override
    public void init() {

        isopen = 0;
        holdSlide = 1;

        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        leftS = hardwareMap.get(Servo.class, "servo2");
        rightS = hardwareMap.get(Servo.class, "servo1");
        garra = hardwareMap.get(Servo.class, "garra");
        Left = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        Right = hardwareMap.get(DcMotorEx.class, "armmotorright");

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
