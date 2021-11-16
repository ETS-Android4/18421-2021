package com.example.test;

import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.gui.GUI;
import com.amarcolini.joos.trajectory.Trajectory;
import com.amarcolini.joos.trajectory.TrajectoryBuilder;
import com.amarcolini.joos.trajectory.config.GenericConstraints;
import com.amarcolini.joos.util.DoubleProgression;

public class MyClass {
    public static void main(String[] args) {
        Trajectory test = new TrajectoryBuilder(
                new Pose2d(), 0.0,
                new GenericConstraints(
                        30.0, 30.0,
                        Math.toRadians(180.0),
                        Math.toRadians(70.0),
                        0.0
                )
        )
                .splineTo(new Vector2d(30.0, 0.0), Math.toRadians(160))
                .build();

        DoubleProgression progression = new DoubleProgression(0.0, 0.1, (int) (test.duration() / 0.1));

        double max = 0.0;
        for(double t : progression) {
            double headingAccel = test.secondDeriv(t).getHeading();
            if (headingAccel > max)
                max = headingAccel;
//            System.out.println(test.secondDeriv(t).getHeading());
        }
        System.out.println(Math.toRadians(70.0));
        System.out.println(max);
    }
}