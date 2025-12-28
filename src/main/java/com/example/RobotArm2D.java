package com.example;

public class RobotArm2D {
    private double L1 = 1.0; // Length of first link
    private double L2 = 1.0; // Length of second link

    // Compute end-effector position for given joint angles
    public double[] forwardKinematics(double theta1, double theta2) {
        double x = L1 * Math.cos(theta1) + L2 * Math.cos(theta1 + theta2);
        double y = L1 * Math.sin(theta1) + L2 * Math.sin(theta1 + theta2);
        return new double[]{x, y};
    }
}
