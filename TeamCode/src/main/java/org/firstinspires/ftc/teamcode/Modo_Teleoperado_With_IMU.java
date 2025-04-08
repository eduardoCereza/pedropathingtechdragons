package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class Modo_Teleoperado_With_IMU extends OpMode {

    //Variáveis
    double x, y, turn;
    //Importando componentes de inicializacao
    HardwareMap h;
    DcMotor leftF, leftB, rightF, rightB, slide;
    IMU imu;
    @Override
    public void init(){

        initialization();


    }


    public void loop(){

        if (gamepad2.options){
            imu.resetYaw();
        }
        chassi_Move();
        move_Slide();

    }

    public void initialization(){
        leftF = hardwareMap.get(DcMotor.class, "leftf");
        leftB = hardwareMap.get(DcMotor.class, "leftb");
        rightF = hardwareMap.get(DcMotor.class, "rightf");
        rightB = hardwareMap.get(DcMotor.class, "rightb");

        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftF.setDirection(DcMotorSimple.Direction.REVERSE);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);

        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = hardwareMap.get(DcMotor.class, "gobilda");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(parameters);
    }

    public void chassi_Move(){

        double botHeading = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);

        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x;

        double rotx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double roty = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotx = rotx * 1.1;

        double denominator = Math.max(Math.abs(roty) + Math.abs(rotx) + Math.abs(turn), 1);
        double powerLF = (roty + rotx + turn) / denominator;
        double powerLB = (roty - rotx + turn) / denominator;
        double powerRF = (roty - rotx - turn) / denominator;
        double powerRB = (roty + rotx - turn) / denominator;


        leftF.setPower(powerLF);
        rightF.setPower(powerRF);
        leftB.setPower(powerLB);
        rightB.setPower(powerRB);
    }

    public void move_Slide(){
        int limitSlide = 3100;
        double j = gamepad2.left_stick_y;
        double minPower = 0.01;
        double maxPower = 0.5;
        PController pController = new PController(0.1);
        pController.setInputRange(50,3600);
        pController.setOutputRange(minPower, maxPower);
        int currentPosition = -(slide.getCurrentPosition());
        double powerS = maxPower + pController.getComputedOutput(currentPosition);
        double powerD = maxPower - pController.getComputedOutput(currentPosition);

        if (currentPosition > limitSlide) {
            if (j > 0.1) {
                slide.setPower(powerS);
            } else if (j < 0) {
                slide.setPower(-powerS);
            } else{
                slide.setPower(powerD);
            }
        }else {
            if (j < 0) {
                slide.setPower(-powerS);
            } else {
                slide.setPower(powerD);
            }
        }

        telemetry.addData("Pos:", currentPosition);
        telemetry.update();

    }
}
