package com.example;

import java.util.Random;

public class Individual {
    public double theta1; // Joint angle 1
    public double theta2; // Joint angle 2
    public double fitness; // Distance to target

    private static final Random rand = new Random();

    // Random initialization of joint angles
    public Individual() {
        theta1 = -Math.PI + 2 * Math.PI * rand.nextDouble();
        theta2 = -Math.PI + 2 * Math.PI * rand.nextDouble();
    }

    // Create individual with given angles (used for crossover)
    public Individual(double t1, double t2) {
        this.theta1 = t1;
        this.theta2 = t2;
    }
}