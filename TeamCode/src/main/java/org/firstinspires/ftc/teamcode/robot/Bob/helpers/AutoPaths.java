package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoPaths {

    //red blue mirroring
    public static final double FIELD_SIZE = 144.0; // full field in inches

    public static Pose mirror(Pose red) {
        double x = red.getX();
        double y = red.getY();
        double h = red.getHeading();

        double newX = x;
        double newY = FIELD_SIZE - y;
        double newH = -h;

        if (newH > Math.PI) newH -= 2 * Math.PI;
        if (newH <= -Math.PI) newH += 2 * Math.PI;

        return new Pose(newX, newY, newH);
    }



    public static final Pose START_POSE =
            new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90));

    // Path1
    public static final Pose P1_START = new Pose(86.646, 9.354);
    public static final Pose P1_END   = new Pose(72.123, 71.877);

    // Path2
    public static final Pose P2_A = new Pose(72.123, 71.877);
    public static final Pose P2_B = new Pose(83.938, 82.708);
    public static final Pose P2_C = new Pose(95, 83.692);

    // Path3
    public static final Pose P3_START = new Pose(95, 83.692);
    public static final Pose P3_END   = new Pose(107, 83.692);

    // Path3_5
    public static final Pose P3_5_START = new Pose(107, 83.692);
    public static final Pose P3_5_END   = new Pose(112, 83.692);

    // Path3_5_5
    public static final Pose P3_5_5_START = new Pose(112, 83.692);
    public static final Pose P3_5_5_END   = new Pose(117, 83.692);

    // Path4
    public static final Pose P4_A = new Pose(117, 83.692);
    public static final Pose P4_B = new Pose(89.600, 83.446);
    public static final Pose P4_C = new Pose(72.123, 72.123);

    // SpikeMark2
    public static final Pose SM2_START = new Pose(72.123, 72.123);
    public static final Pose SM2_END   = new Pose(95.000, 60.000);

    // SpikeMark21
    public static final Pose SM21_START = new Pose(95.000, 60.000);
    public static final Pose SM21_END   = new Pose(107.000, 60.000);

    // SpikeMark22
    public static final Pose SM22_START = new Pose(107.000, 60.000);
    public static final Pose SM22_END   = new Pose(112.000, 60.000);

    // SpikeMark23
    public static final Pose SM23_START = new Pose(112.000, 60.000);
    public static final Pose SM23_END   = new Pose(117.000, 60.000);

    // SpikeMark24
    public static final Pose SM24_START = new Pose(117.000, 60.000);
    public static final Pose SM24_END   = new Pose(72.000, 72.000);

    // Park
    public static final Pose PARK_START = new Pose(72, 72);
    public static final Pose PARK_END   = new Pose(95, 37);



    public static PathChain buildPath1(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(P1_START, P1_END)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(45)
                )
                .build();
    }

    public static PathChain buildPath2(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                P2_A,
                                P2_B,
                                P2_C
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(45),
                        Math.toRadians(0)
                )
                .build();
    }

    public static PathChain buildPath3(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                P3_START,
                                P3_END
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    public static PathChain buildPath3_5(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                P3_5_START,
                                P3_5_END
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    public static PathChain buildPath3_5_5(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                P3_5_5_START,
                                P3_5_5_END
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    public static PathChain buildPath4(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                P4_A,
                                P4_B,
                                P4_C
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(45)
                )
                .build();
    }

    public static PathChain buildSpikeMark2(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SM2_START, SM2_END)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(45),
                        Math.toRadians(0)
                )
                .build();
    }

    public static PathChain buildSpikeMark21(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SM21_START, SM21_END)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();
    }

    public static PathChain buildSpikeMark22(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SM22_START, SM22_END)
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildSpikeMark23(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SM23_START, SM23_END)
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildSpikeMark24(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(SM24_START, SM24_END)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(45)
                )
                .build();
    }

    public static PathChain buildPark(Follower follower) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(PARK_START, PARK_END)
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(45),
                        Math.toRadians(0)
                )
                .build();
    }
}
