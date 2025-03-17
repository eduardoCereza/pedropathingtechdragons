package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp
public class Modo_Teleoperado_With_IMU extends OpMode {

    //Variáveis
    double sin, cos, theta, power, max;
    double x, y, turn;
    double leftf,leftb, rightf, rightb;

    //Importando componentes de inicializacao
    HardwareMap h;
    DcMotor leftF, leftB, rightF, rightB;
    IMU imu;
    @Override
    public void init(){

        initialization();

    }

    public void loop(){

        chassi_Move();

    }

    public void initialization(){
        leftF = h.get(DcMotor.class, "leftf");
        leftB = h.get(DcMotor.class, "leftb");
        rightF = h.get(DcMotor.class, "rightf");
        rightB = h.get(DcMotor.class, "rightb");

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

        imu = h.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(parameters);
    }

    public void chassi_Move(){

        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x;

        theta = Math.atan2(y, x);
        power = Math.hypot(x, y);

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));


        double powerLF = power * cos/max + turn;
        double powerLB = power * sin/max - turn;
        double powerRF = power * sin/max + turn;
        double powerRB = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1){
            powerLF /= power + turn;
            powerLB /= power + turn;
            powerRF /= power + turn;
            powerRB /= power + turn;
        }

        leftF.setPower(powerLF);
        rightF.setPower(powerRF);
        leftB.setPower(powerLB);
        rightB.setPower(powerRB);
    }
}
