package com.example;
import java.util.*;

public class GeneticAlgorithm {
    private int populationSize = 50;
    private int generations = 200;
    private double mutationRate = 0.1;
    private double crossoverRate = 0.8;

    private RobotArm2D arm;
    private double targetX, targetY;
    private List<Individual> population;
    private Random rand = new Random();

    public GeneticAlgorithm(double x, double y) {
        this.targetX = x;
        this.targetY = y;
        this.arm = new RobotArm2D();
        population = new ArrayList<>();
        for (int i = 0; i < populationSize; i++) {
            population.add(new Individual());
        }
    }

    private double calculateFitness(Individual ind) {
        double[] pos = arm.forwardKinematics(ind.theta1, ind.theta2);
        double dx = pos[0] - targetX;
        double dy = pos[1] - targetY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private Individual tournamentSelection() {
        Individual a = population.get(rand.nextInt(populationSize));
        Individual b = population.get(rand.nextInt(populationSize));
        return a.fitness < b.fitness ? a : b;
    }

    private Individual crossover(Individual p1, Individual p2) {
        if (rand.nextDouble() < crossoverRate) {
            return new Individual(p1.theta1, p2.theta2);
        }
        return new Individual(p1.theta1, p1.theta2);
    }

    private void mutate(Individual ind) {
        if (rand.nextDouble() < mutationRate) {
            ind.theta1 += rand.nextGaussian() * 0.1;
        }
        if (rand.nextDouble() < mutationRate) {
            ind.theta2 += rand.nextGaussian() * 0.1;
        }
    }

    public Individual run() {
        Individual best = null;

        for (int gen = 0; gen < generations; gen++) {
            for (Individual ind : population) {
                ind.fitness = calculateFitness(ind);
            }

            population.sort(Comparator.comparingDouble(i -> i.fitness));
            best = population.get(0);

            double[] pos = arm.forwardKinematics(best.theta1, best.theta2);
            System.out.printf(
                "Gen %d | θ1=%.4f θ2=%.4f | x=%.4f y=%.4f | error=%.6f%n",
                gen, best.theta1, best.theta2, pos[0], pos[1], best.fitness
            );

            if (best.fitness < 0.01) break;

            List<Individual> newPop = new ArrayList<>();
            newPop.add(best); // elitism

            while (newPop.size() < populationSize) {
                Individual p1 = tournamentSelection();
                Individual p2 = tournamentSelection();
                Individual child = crossover(p1, p2);
                mutate(child);
                newPop.add(child);
            }
            population = newPop;
        }
        return best;
    }
}
