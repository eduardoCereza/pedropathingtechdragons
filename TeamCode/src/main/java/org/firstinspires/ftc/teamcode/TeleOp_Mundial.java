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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;


/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado Mundial")
public class TeleOp_Mundial extends OpMode {
    private Follower follower;
    DcMotorEx slide;

    Servo servo1, servo2;

    boolean wasMoving = false;
    int holdPos = 0;

    PID_Parameters pid;

    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower =  new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        slide = hardwareMap.get(DcMotorEx.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");


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
        moveServo();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    //TODO: Mover Slide
    public void moveSlide(){
        pid = new PID_Parameters(1, 0, 0);

        double input = gamepad2.left_stick_y;
        int currentPos = slide.getCurrentPosition();

        if (Math.abs(input) > 0.05){
            wasMoving = true;
            slide.setPower(input);
        }else {
            if(wasMoving) {
                holdPos = slide.getCurrentPosition();
                wasMoving = false;
                pid.reset();

            }
            double power = pid.calculate(holdPos, currentPos);
            slide.setPower(power);
        }
    }

    //TODO: Mover base do atuador
    public void moveBase(){

    }

    //Todo: Mover servo
    public void moveServo(){

        if(gamepad2.right_bumper){
            servo1.setPosition(1);
            servo2.setPosition(1);
        }else {
            servo1.setPosition(0);
            servo2.setPosition(0);
        }
    }
}