package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * gamepad1:
 *   left stick  = drive / strafe
 *   right stick = turn
 *   a           = intake on/off
 *   b           = outtake on/off
 *   y           = flywheel on/off
 */
@TeleOp
public class SimpleTeleOp extends OpMode {
    // Hardware names -- must match the Driver Station configuration.
    public static String INTAKE_STAGE1_NAME = "intake1";
    public static String INTAKE_STAGE23_NAME = "intake2";
    public static String FLYWHEEL_NAME = "launcher";

    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -0.6;

    public static double TARGET_RPM = 3600.0;
    public static double TICKS_PER_REV = 28.0; // goBILDA 6000 RPM (1:1) motor: 28 counts/rev at the output shaft

    // Flywheel PIDF gains, in motor power per (encoder tick / second).
    public static double KF = 1.0 / (6000.0 * 28.0 / 60.0); // 1 / free speed at 12 V
    public static double KP = 0.0008;
    public static double KI = 0.0005;
    public static double KD = 0.0;
    public static double INTEGRAL_ZONE = 0.10;      // only integrate within 10% of target
    public static double MAX_INTEGRAL_POWER = 0.15; // cap on the I term's contribution

    private Follower follower;
    private VoltageSensor battery;
    private DcMotor intakeStage1;
    private DcMotor intakeStage23;
    private DcMotorEx flywheel;
    private Servo wedge;
    private Servo gate;

    private enum IntakeState { OFF, IN, OUT }
    int wedgeFlag = 0;
    private IntakeState intakeState = IntakeState.OFF;
    private boolean flywheelOn = false;
    private double integral = 0;
    private double lastError = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.create(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();

        intakeStage1 = hardwareMap.get(DcMotor.class, INTAKE_STAGE1_NAME);
        intakeStage23 = hardwareMap.get(DcMotor.class, INTAKE_STAGE23_NAME);
        intakeStage1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeStage23.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeStage1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeStage23.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wedge = hardwareMap.get(Servo.class, "wedge");
        gate = hardwareMap.get(Servo.class,"gate");

        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Our own PIDF below; the encoder still reads in this mode.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready");
    }

    @Override
    public void start() {
        loopTimer.reset();
        wedge.setPosition(0);
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Pedro convention: +strafe is left, +turn is counter-clockwise, so the x axes are negated.
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();


        if (gamepad1.aWasPressed()) {
            intakeState = intakeState == IntakeState.IN ? IntakeState.OFF : IntakeState.IN;
        }
        if (gamepad1.bWasPressed()) {
            intakeState = intakeState == IntakeState.OUT ? IntakeState.OFF : IntakeState.OUT;
        }
        if (gamepad1.yWasPressed()) flywheelOn = !flywheelOn;

        double intakePower = 0.0;
        if (intakeState == IntakeState.IN) intakePower = INTAKE_POWER;
        if (intakeState == IntakeState.OUT) intakePower = OUTTAKE_POWER;
        intakeStage1.setPower(intakePower);
        intakeStage23.setPower(intakePower);

        double targetTicksPerSecond = TARGET_RPM * TICKS_PER_REV / 60.0;
        double velocity = flywheel.getVelocity();
        double error = targetTicksPerSecond - velocity;

        if (flywheelOn) {
            if (Math.abs(error) < targetTicksPerSecond * INTEGRAL_ZONE) {
                integral += error * dt;
            }
            double iTerm = clamp(KI * integral, -MAX_INTEGRAL_POWER, MAX_INTEGRAL_POWER);
            integral = KI == 0 ? 0 : iTerm / KI; // stop the integral winding past its cap

            double derivative = dt > 0 ? (error - lastError) / dt : 0.0;
            double feedforward = KF * targetTicksPerSecond * (12.0 / battery.getVoltage());
            double power = feedforward + KP * error + iTerm + KD * derivative;
            flywheel.setPower(clamp(power, 0.0, 1.0));
        } else {
            integral = 0;
            flywheel.setPower(0);
        }
        lastError = error;

        Pose pose = follower.pose();
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "off");
        telemetry.addData("RPM target / actual", "%.0f / %.0f", TARGET_RPM, velocity * 60.0 / TICKS_PER_REV);
        telemetry.addData("Intake", intakeState);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Battery", "%.2f V", battery.getVoltage());
        telemetry.addData("flag", wedgeFlag);
        telemetry.addData("servo", wedge.getPosition());


        if (gamepad1.xWasPressed()) {
            if (wedgeFlag == 0) {
                wedge.setPosition(0.35);
                wedgeFlag = 1;
            } else {
                wedge.setPosition(0);
                wedgeFlag = 0;
            }
        }

        if (gamepad1.dpadDownWasPressed()) {
            gate.setPosition(0.0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            gate.setPosition(0.2);
        }
    }





    @Override
    public void stop() {
        flywheel.setPower(0);
        intakeStage1.setPower(0);
        intakeStage23.setPower(0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
