package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MechRobot {

    //Declare Drive Motors Here
    private DcMotor lfDrive = null;
    private DcMotor lrDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor rrDrive = null;

    //Declare Sensors Here
    BNO055IMU imu = null;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters;


    //Blank Constructor
    public MechRobot() {

    }

    public  void init(HardwareMap h) {
        //Drive Motors
        lfDrive = h.get(DcMotor.class, "lfDrive");
        lrDrive = h.get(DcMotor.class, "lrDrive");
        rfDrive = h.get(DcMotor.class, "rfDrive");
        rrDrive = h.get(DcMotor.class, "rrDrive");

        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        lrDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        rrDrive.setDirection(DcMotor.Direction.REVERSE);

        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void initGyro(HardwareMap h) {
        imu = h.get(BNO055IMU.class, "imu");

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //gravity  = imu.getGravity();

    }
    public float gyroAngle3() {
        Orientation pitch = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return pitch.thirdAngle;
    }
    public float gyroAngle2() {
        Orientation pitch = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return pitch.secondAngle;
    }
    public float gyroAngle1() {
        Orientation pitch = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return pitch.firstAngle;
    }

    public void drive(double leftSpeed, double rightSpeed) {
        lfDrive.setPower(leftSpeed);
        lrDrive.setPower(leftSpeed);
        rfDrive.setPower(rightSpeed);
        rrDrive.setPower(rightSpeed);
    }

    public void turn(double speed) {
        lfDrive.setPower(speed);
        lrDrive.setPower(speed);
        rfDrive.setPower(-speed);
        rrDrive.setPower(-speed);
    }

    public void strafe(double speed) {
        lfDrive.setPower(speed);
        lrDrive.setPower(-speed);
        rfDrive.setPower(-speed);
        rrDrive.setPower(speed);
    }

    public void strafeLeft() {
        lfDrive.setPower(-1);
        lrDrive.setPower(1);
        rfDrive.setPower(1);
        rrDrive.setPower(-1);
    }

    public void strafeRight() {
        lfDrive.setPower(1);
        lrDrive.setPower(-1);
        rfDrive.setPower(-1);
        rrDrive.setPower(1);
    }

    public void diagonalLeft(double speed) {
        lfDrive.setPower(speed);
        lrDrive.setPower(0);
        rfDrive.setPower(0);
        rrDrive.setPower(speed);
    }

    public void diagonalRight(double speed) {
        lfDrive.setPower(0);
        lrDrive.setPower(speed);
        rfDrive.setPower(speed);
        rrDrive.setPower(0);
    }



}
