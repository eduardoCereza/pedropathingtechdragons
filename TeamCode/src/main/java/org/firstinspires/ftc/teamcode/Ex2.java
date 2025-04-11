package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;

@Autonomous(name = "Example 2", group = "Examples")
public class Ex2 extends OpMode {
    private Follower follower;
    private Timer opmodeTimer, pathTimer, actionTimer;
    private PathChain pathChain, pathChain2;
    private final Pose startPose = new Pose(0, 71, Math.toRadians(0));

    public void buildPaths() {

         pathChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(0, 71, Point.CARTESIAN), new Point(20, 71, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                 .build();

         pathChain2 = follower.pathBuilder()
                 .setPathEndTimeoutConstraint(3.0)
                 .build();
    }


    public void state(int estado){
        if(estado == 1){
            pathTimer.resetTimer();
            follower.followPath(pathChain, true);

        }
        if(estado == 2){
            pathTimer.resetTimer();
            follower.followPath(pathChain2, true);
        } else if (estado == 3) {

        }

    }



    @Override
    public void loop() {

        follower.update();
        state(1);

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();

    }

    @Override
    public void stop() {
    }
}