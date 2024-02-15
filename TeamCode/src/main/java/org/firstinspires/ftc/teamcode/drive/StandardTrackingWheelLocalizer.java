package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = .945; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

//    public static double LATERAL_DISTANCE = 8.711887328332374; // in; distance between the left and right wheels
    public static double LATERAL_DISTANCE = 8.728947588220594; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -7.5; // in; offset of the lateral wheel
    public static double X_MULIPLIER = 1.0016741771330908219836848347776;
    public static double Y_MULIPLIER = 1.0024873816442744484608265187964;

    private Encoder leftEncoder, rightEncoder, backEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        backEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int backPos = backEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(backPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) * X_MULIPLIER,
                encoderTicksToInches(rightPos) * X_MULIPLIER,
                encoderTicksToInches(backPos * Y_MULIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int backVel = (int) backEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(backVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULIPLIER,
                encoderTicksToInches(rightVel) * X_MULIPLIER,
                encoderTicksToInches(backVel * Y_MULIPLIER)
        );
    }
}
