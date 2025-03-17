package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp
public class Modo_Teleoperado extends OpMode {

    //Variáveis
    double sen, cos, theta, power, max;
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
    }
}
