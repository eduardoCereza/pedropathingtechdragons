package org.firstinspires.ftc.teamcode;

//importações referentes ao pedro pathing
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;


//importações não referentes ao pedro pathing
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Ariba mexico", group = "Examples")
public class autoAriba extends OpMode {

    private Servo servo1, servo2, garra;

    private DcMotor slide, armMotorL, armMotorR;//servo da garra/ponta

    private Follower follower; //sla tbm

    private Timer pathTimer, actionTimer, opmodeTimer; //sla ja veio no código

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
                follower.followPath(traj1, true);

                //troca para fazer nada
                setPathState(1);
                break;

            //faz nada
            case 1:
                if(!follower.isBusy()) {

                    setPathState(-1);
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

        Pose pose = follower.getPose();
        if (pose.getX() == ClipPose.getX() && pose.getY() == ClipPose.getY()){


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
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        garra = hardwareMap.get(Servo.class, "garra");

        armMotorL = hardwareMap.get(DcMotorEx.class, "armmotorleft");
        armMotorR = hardwareMap.get(DcMotorEx.class, "armmotorright");
        armMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower =  new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    //só um loop pra quando dar init
    @Override
    public void init_loop() {}

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