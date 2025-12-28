package com.example;
public class MainApp {
    public static void main(String[] args) {
        double[][] targets = {
            {1.2, 0.5},
            {0.5, 1.5},
            {1.5, 0.2}
        };

        for (double[] t : targets) {
            System.out.println("\nTarget: (" + t[0] + ", " + t[1] + ")");
            GeneticAlgorithm ga = new GeneticAlgorithm(t[0], t[1]);
            ga.run();
        }
    }
}
