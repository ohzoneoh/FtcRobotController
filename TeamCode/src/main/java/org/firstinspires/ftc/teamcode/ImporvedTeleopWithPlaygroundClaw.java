package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "woof woof this is the teleop")
public class ImporvedTeleopWithPlaygroundClaw extends LinearOpMode {


    private final double ARM_DOWN = 0.30;
    private final double ARM_OUT = 0;//0.025
    private DcMotor lift;
    private DcMotor leftfront;
    private DcMotor leftback;
    private CRServo intake;
    private DcMotor rightback;
    private DcMotor rightfront;
    private Servo plane;
    private Servo leftclaw;
    private Servo rightclaw;
    private Servo armrotate;
    private ColorSensor rightcolor_REV_ColorRangeSensor;

    BNO055IMU imu;
    Orientation angles = new Orientation();

    double initYaw;
    double adjustedYaw;

    int liftPosition;

    /**
     * Describe this function...
     */
    private void goLift(int liftAmount) {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(liftAmount);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int liftAmount;
        int claw;
        boolean leftClawUnclicked;
        boolean leftClawToggle;
        boolean rightClawUnclicked;
        boolean rightClawToggle;
        double Vertical;
        double Horizontal;
        double Pivot;

        lift = hardwareMap.get(DcMotor.class, "lift");
        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        intake = hardwareMap.get(CRServo.class, "intake");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        plane = hardwareMap.get(Servo.class, "plane");
        leftclaw = hardwareMap.get(Servo.class, "left claw");
        rightclaw = hardwareMap.get(Servo.class, "right claw");
        armrotate = hardwareMap.get(Servo.class, "arm rotate");
        rightcolor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "rightcolor");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        initYaw = angles.firstAngle;


        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(CRServo.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set servo to Open position
        claw = 0;
        liftAmount = 0;
        liftPosition = 0;
        leftClawUnclicked = false;
        leftClawToggle = false;
        rightClawUnclicked = false;
        rightClawToggle = false;
        waitForStart();
        if (opModeIsActive()) {
            initClaw();
            while (opModeIsActive()) {
                // arm
                // claw servos
                if (gamepad2.dpad_down) {
                    if (liftPosition + -10 >= 0) {
                        liftPosition += -10;
                    }
                    if (liftPosition + -10 >= 0) {
                        liftPosition += -10;
                    }
                } else if (gamepad2.dpad_up) {
                    if (liftPosition + 10 < 2490) {
                        liftPosition += 10;
                    }
                    if (liftPosition + 10 < 2490) {
                        liftPosition += 10;
                    }
                }
                lift.setTargetPosition(liftPosition);
                lift.setPower(0.333);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (gamepad2.x) {
                    plane.setPosition(0);
                } else if (gamepad2.b) {
                    plane.setPosition(90);
                }
                if (gamepad1.x) {
                    plane.setPosition(0);
                } else if (gamepad1.b) {
                    plane.setPosition(90);
                }
                if (gamepad2.left_bumper) {
                    if (leftClawUnclicked) {
                        if (leftClawToggle) {
                            leftclaw.setPosition(1);
                            leftClawToggle = false;
                        } else {
                            leftclaw.setPosition(0.3);
                            leftClawToggle = true;
                            gamepad2.rumble(500);
                        }
                        leftClawUnclicked = false;
                    }
                } else {
                    leftClawUnclicked = true;
                }

                if (gamepad1.left_bumper)
                {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    initYaw = angles.firstAngle;
                }

                if (gamepad2.right_bumper) {
                    if (rightClawUnclicked) {
                        if (rightClawToggle) {
                            rightclaw.setPosition(0);
                            rightClawToggle = false;
                        } else {
                            rightclaw.setPosition(0.8);
                            rightClawToggle = true;
                            gamepad2.rumble(500);
                        }
                        rightClawUnclicked = false;
                    }
                } else {
                    rightClawUnclicked = true;
                }
                if (leftClawToggle) {
                    telemetry.addData("Left Claw", "CLOSED     --------------------");
                } else {
                    telemetry.addData("Left Claw", "OPEN     OOOOOOOOOO");
                }
                if (rightClawToggle) {
                    telemetry.addData("Right Claw", "CLOSED     --------------------");
                } else {
                    telemetry.addData("Right Claw", "OPEN     OOOOOOOOOO");
                }
                /*if (gamepad2.dpad_left) {
                    armrotate.setPosition(0.3);
                } else if (gamepad2.dpad_right) {
                    armrotate.setPosition(-0.3);
                }*/

                if (lift.getCurrentPosition() > 650)
                {
                    armrotate.setPosition(ARM_OUT);
                }
                /*else if (lift.getCurrentPosition() > 160)
                {
                ars.setPosition(ARM_TUCKED);
                }*/
                else
                {
                    armrotate.setPosition(ARM_DOWN);
                }

                updateDrive();

                if (gamepad1.a) {
                    lift.setPower(0);
                    //armrotate.setPosition(-0.3);
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lift.setPower(-0.5);
                    sleep(1000);
                    lift.setPower(0);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initClaw();
                    rightClawToggle = false;
                    leftClawToggle = false;
                }
                if (gamepad2.x) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
                // Use gamepad Bumpers to open close servo
                // Keep Servo position in valid range
                // Keep Servo position in valid range
                telemetry.update();
                if (((DistanceSensor) rightcolor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 5.5) {
                    rightclaw.setPosition(0.8);
                    rightClawToggle = true;
                } else {
                }
            }
        }
    }

    /**
     * Describe this function...
     */
    private void initClaw() {
        leftclaw.setPosition(0.3);
        rightclaw.setPosition(0.8);
        sleep(500);
        armrotate.setPosition(0.3);
        liftPosition = 0;
        lift.setTargetPosition(liftPosition);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            idle();
        }
        armrotate.setPosition(0.3);
        liftPosition = 0;
        lift.setTargetPosition(liftPosition);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {
            idle();
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

        leftfront.setPower(leftFrontPwr);
        rightfront.setPower(rightFrontPwr);
        leftback.setPower(leftBackPwr);
        rightback.setPower(rightBackPwr);

        if (gamepad1.left_bumper)
        {
            leftfront.setPower(leftFrontPwr/2);
            rightfront.setPower(rightFrontPwr/2);
            leftback.setPower(leftBackPwr/2);
            rightback.setPower(rightBackPwr/2);
        }
    }
}
