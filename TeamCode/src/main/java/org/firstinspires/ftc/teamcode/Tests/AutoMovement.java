package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoThings.PathFollowerWrapper;
import org.firstinspires.ftc.teamcode.Board;

import java.util.Arrays;

import Wheelie.Path;
import Wheelie.Pose2D;

@Autonomous (group = "Test")
public class AutoMovement extends LinearOpMode {

    //TODO add necessary rotations
    private PathFollowerWrapper followerWrapper;
    private Board board;

    private Pose2D start = new Pose2D(0,0,0);

    private Pose2D[] toSub = new Pose2D[] {
            new Pose2D(0,0,0),
            new Pose2D(24, 0, 0),
            new Pose2D(24, 24, 0)
    };
    @Override
    public void runOpMode() throws InterruptedException {
        board = new Board(hardwareMap);
        followerWrapper = new PathFollowerWrapper(hardwareMap, start, 8);

        waitForStart();

        followLoop(toSub, 5);
        //followLoop(toObser, 5);
        //followLoop(backUp, 5);

    }

    private void followLoop(Pose2D[] a, double waitTime){
        followerWrapper.setPath(followerWrapper.getPose(), new Path(followerWrapper.getPose(), a));

        while(followerWrapper.getFollower() != null && opModeIsActive()){
            followerWrapper.updatePose(
                    board.getAngle());
            double[] vectorCom = followerWrapper.followPath();
            board.drive(vectorCom[0], vectorCom[1], vectorCom[2]);

            telemetry.addData("Position",
                    followerWrapper.getPose().x + ", " +
                            followerWrapper.getPose().y + ", " +
                            followerWrapper.getPose().h);
            telemetry.addData("Vector", Arrays.toString(vectorCom));

            telemetry.addLine();
            if(followerWrapper.getFollower() != null) {
                Pose2D goal = a[followerWrapper.getCurrentWayPoint() + 1];
                telemetry.addData("Goal",
                        goal.x + ", " +
                                goal.y + ", " +
                                goal.h);
            }

            telemetry.update();
        }
        //TODO move this timer out of the function;
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(time.time() < waitTime && opModeIsActive());

    }
}
