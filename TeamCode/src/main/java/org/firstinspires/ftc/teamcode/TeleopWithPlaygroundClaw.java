package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp

public class TeleopWithPlaygroundClaw extends LinearOpMode {


    private final double ARM_DOWN = 0.30;
    private final double ARM_OUT = 0;//0.025
    private final double ARM_TUCKED = .45;
    private Servo lcs = null;
    private Servo rcs = null;
    private Servo ars = null;
    private CRServo intake = null;
    private DcMotor lift = null;
    private Claw lc;
    private Claw rc;
    private Claw ar;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    BNO055IMU imu;
    Orientation angles = new Orientation();

    double initYaw;
    double adjustedYaw;

    @Override
    public void runOpMode() {

        lcs = hardwareMap.get(Servo.class, "left claw");
        rcs = hardwareMap.get(Servo.class, "right claw");
        ars = hardwareMap.get(Servo.class, "arm rotate");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.get(DcMotor.class, "lift");

        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        initYaw = angles.firstAngle;


        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lc = new Claw(lcs, .25, 1);
        rc = new Claw(rcs, .75, 0);
        //ar = new Claw(ars, .30, 0.025);
        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.a)
            {
                lc.open();
                rc.open();
            }
            else if (gamepad1.b)
            {
                rc.close();
                lc.close();
            }

            if (lift.getCurrentPosition() > 650)
            {
                ars.setPosition(ARM_OUT);
            }
            /*else if (lift.getCurrentPosition() > 160)
            {
                ars.setPosition(ARM_TUCKED);
            }*/
            else
            {
                ars.setPosition(ARM_DOWN);
            }

            if (gamepad1.dpad_up)
            {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(.3);
            }
            else if(gamepad1.dpad_down)
            {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(-.3);
            }
            else
            {
                //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //lift.setTargetPosition(lift.getCurrentPosition());
                //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //lift.setPower(1);
                lift.setPower(0);
            }

            if(gamepad1.x)
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
            }

            telemetry.addData("LiftPosition", lift.getCurrentPosition());
            telemetry.update();

            updateDrive();
        }

    }

    private void updateDrive()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        adjustedYaw = angles.firstAngle-initYaw;
        double zerodYaw = - initYaw + angles.firstAngle;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x) * 180/Math.PI; //angle of gamepad
        double realTheta;
        realTheta = (360 - zerodYaw) + theta;
        double power = Math.hypot(x, y);

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPwr = (power * cos / maxSinCos + turn);
        double rightFrontPwr = (power * sin / maxSinCos - turn);
        double leftBackPwr = (power * sin / maxSinCos + turn);
        double rightBackPwr = (power * cos / maxSinCos - turn);

        if ( (power + Math.abs(turn)) > 1 )
        {
            leftFrontPwr /= power + turn;
            rightBackPwr /= power - turn;
            leftBackPwr /= power + turn;
            leftFrontPwr /= power - turn;
        }

        leftFront.setPower(leftFrontPwr);
        rightFront.setPower(rightFrontPwr);
        leftBack.setPower(leftBackPwr);
        rightBack.setPower(rightBackPwr);
    }
}
