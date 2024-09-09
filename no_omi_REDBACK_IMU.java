package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "REDBACK_IMU (Blocks to Java)")
public class REDBACK_IMU extends LinearOpMode {

  private BNO055IMU imu;
  private DcMotor leftbackmotor;
  private DcMotor leftfrontmotor;
  private DcMotor rightbackmotor;
  private DcMotor rightfrontmotor;

  float z_orientation;

  /**
   * Describe this function...
   */
  private float get_Z_axis_orientation() {
    Orientation angles;

    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return angles.firstAngle;
  }

  /**
   * Describe this function...
   */
  private void init_IMU() {
    BNO055IMU.Parameters IMU_Parameters;

    IMU_Parameters = new BNO055IMU.Parameters();
    IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
    IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    IMU_Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    telemetry.addData("status: ", "init imu....");
    telemetry.update();
    imu.initialize(IMU_Parameters);
    telemetry.addData("status: ", "imu done");
    telemetry.update();
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    leftbackmotor = hardwareMap.get(DcMotor.class, "left back motor");
    leftfrontmotor = hardwareMap.get(DcMotor.class, "left front motor");
    rightbackmotor = hardwareMap.get(DcMotor.class, "right back motor");
    rightfrontmotor = hardwareMap.get(DcMotor.class, "right front motor");

    leftbackmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftfrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftbackmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftfrontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightbackmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightfrontmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    init_IMU();
    waitForStart();
    if (opModeIsActive()) {
      ENCODER(1.9 * 1440);
      rotateCCW(80);
      ENCODER(1.6 * 1440);
      rotateCW(10);
      ENCODER(2.9 * 1440);
    }
  }

  /**
   * Describe this function...
   */
  private void ENCODER(double move_to_postion) {
    leftbackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftfrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightbackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightfrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftbackmotor.setPower(0.5);
    leftfrontmotor.setPower(0.5);
    rightbackmotor.setPower(0.5);
    rightfrontmotor.setPower(0.5);
    while (leftbackmotor.getCurrentPosition() < move_to_postion) {
    }
    leftbackmotor.setPower(0);
    leftfrontmotor.setPower(0);
    rightbackmotor.setPower(0);
    rightfrontmotor.setPower(0);
    sleep(250);
  }

  /**
   * Describe this function...
   */
  private void rotateCCW(int target_orientation_angles) {
    z_orientation = get_Z_axis_orientation();
    leftbackmotor.setPower(-0.3);
    leftfrontmotor.setPower(-0.3);
    rightbackmotor.setPower(0.3);
    rightfrontmotor.setPower(0.3);
    while (z_orientation < target_orientation_angles) {
      z_orientation = get_Z_axis_orientation();
    }
    leftbackmotor.setPower(0);
    leftfrontmotor.setPower(0);
    rightbackmotor.setPower(0);
    rightfrontmotor.setPower(0);
    sleep(250);
  }

  /**
   * Describe this function...
   */
  private void rotateCW(int target_orientation_angles) {
    z_orientation = get_Z_axis_orientation();
    leftbackmotor.setPower(0.3);
    leftfrontmotor.setPower(0.3);
    rightbackmotor.setPower(-0.3);
    rightfrontmotor.setPower(-0.3);
    while (z_orientation > target_orientation_angles) {
      z_orientation = get_Z_axis_orientation();
    }
    rightbackmotor.setPower(0);
    rightfrontmotor.setPower(0);
    leftbackmotor.setPower(0);
    leftfrontmotor.setPower(0);
    sleep(250);
  }
}
