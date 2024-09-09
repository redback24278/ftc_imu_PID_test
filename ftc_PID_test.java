public class SelfBalancingOpMode extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        waitForStart();

        double targetAngle = 0;
        double kP = 0.5;
        double kI = 0.1;
        double kD = 0.05;

        double previousError = 0;
        double integral = 0;

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;

            double error = targetAngle - currentAngle;
            integral += error;
            double derivative = error - previousError;

            double correction = kP * error + kI * integral + kD * derivative;

            leftMotor.setPower(correction);
            rightMotor.setPower(-correction);

            previousError = error;
        }
    }
}
