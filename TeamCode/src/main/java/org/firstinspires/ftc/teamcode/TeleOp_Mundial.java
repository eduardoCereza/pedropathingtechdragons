package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;


/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado")
public class TeleOp_Mundial extends OpMode {
    private Follower follower;
    DcMotorEx slide;

    PID_Parameters pid;

    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PID_Parameters(1, 0, 0);

    }


    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        moveSlide();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    //TODO: Mover Slide
    public void moveSlide(){

        double input = gamepad2.left_stick_y;
        int holdPos = 0;
        int currentPos = slide.getCurrentPosition();

        if (Math.abs(input) > 0.05){
            slide.setPower(input);

            holdPos = currentPos;
            pid.reset();
        }else {
            double holdPower = pid.calculate(holdPos, currentPos);
            slide.setPower(holdPower);
        }



    }

    //TODO: Mover base do atuador
    public void moveBase(){

    }

    //Todo: Mover servo
    public void moveServo(){

    }
}