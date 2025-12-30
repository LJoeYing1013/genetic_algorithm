package lab1a;

import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TabPane.TabClosingPolicy;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.stage.Stage;

public class RoboticAssignmentPartA extends Application {

    private SCARARobot robot = new SCARARobot();
    private Canvas canvas;
    private TextField txtA1, txtA2;
    private Slider sliderQ1, sliderQ2;
    private Label lblRobotPos;
    private TextArea consoleOutput;

    private TextField txtXB = new TextField("10");
    private TextField txtYB = new TextField("5");
    private TextField txtTheta = new TextField("30");
    private TextField txtTx = new TextField("50");
    private TextField txtTy = new TextField("20");
    
    private Label lblXA = new Label("0.00");
    private Label lblYA = new Label("0.00");
    
    private Label lblR00 = new Label("1.00"), lblR01 = new Label("0.00"), lblTx = new Label("0.00");
    private Label lblR10 = new Label("0.00"), lblR11 = new Label("1.00"), lblTy = new Label("0.00");

    @Override
    public void start(Stage primaryStage) {
        TabPane tabPane = new TabPane();
        tabPane.setTabClosingPolicy(TabClosingPolicy.UNAVAILABLE);

        // --- Tab 1: Robot Simulation ---
        Tab tab1 = new Tab("SCARA Robot");
        tab1.setContent(createRobotView());

        // --- Tab 2: Transformation Calculator ---
        Tab tab2 = new Tab("Frame Transformation");
        tab2.setContent(createCalculatorView());

        tabPane.getTabs().addAll(tab1, tab2);

        Scene scene = new Scene(tabPane, 750, 700);
        primaryStage.setTitle("ROBOTIC ASSIGNMENT 1: FORWARD AND INVERSE KINEMATICS");
        primaryStage.setScene(scene);
        primaryStage.show();
    }
    
    private BorderPane createRobotView() {
        BorderPane root = new BorderPane();
        root.setPadding(new Insets(10));

        // Drawing Canvas
        canvas = new Canvas(600, 350);
        // Wrapper to center the canvas
        VBox canvasContainer = new VBox(canvas);
        canvasContainer.setAlignment(Pos.CENTER);
        canvasContainer.setStyle("-fx-background-color: white; -fx-border-color: lightgray;");
        root.setCenter(canvasContainer);

        // Controls Area
        VBox controls = new VBox(10);
        controls.setPadding(new Insets(10));
        controls.setStyle("-fx-background-color: #f0f0f0; -fx-border-color: #ccc;");

        // Length Inputs
        HBox lengthBox = new HBox(10);
        lengthBox.setAlignment(Pos.CENTER_LEFT);
        txtA1 = new TextField("150"); txtA1.setPrefWidth(60);
        txtA2 = new TextField("100"); txtA2.setPrefWidth(60);
        Button btnUpdateLengths = new Button("Set Lengths");
        lengthBox.getChildren().addAll(new Label("Length a1:"), txtA1, new Label("Length a2:"), txtA2, btnUpdateLengths);

        // Sliders
        sliderQ1 = new Slider(-180, 180, 45);
        sliderQ1.setShowTickMarks(true); sliderQ1.setShowTickLabels(true);
        sliderQ2 = new Slider(-180, 180, -30);
        sliderQ2.setShowTickMarks(true); sliderQ2.setShowTickLabels(true);

        // Reverse Motion Button
        Button btnReverse = new Button("Calculate Reverse Motion (Matrix Inverse)");
        lblRobotPos = new Label("Pos: ");

        // Output Area
        consoleOutput = new TextArea();
        consoleOutput.setPrefHeight(150);
        consoleOutput.setEditable(false);
        consoleOutput.setText("Move sliders and click 'Calculate Reverse Motion' to see b_T_a.");

        controls.getChildren().addAll(
            lengthBox, 
            new Label("Angle q1:"), sliderQ1, 
            new Label("Angle q2:"), sliderQ2,
            btnReverse,
            lblRobotPos,
            consoleOutput
        );
        root.setBottom(controls);

        // Event Handling
        sliderQ1.valueProperty().addListener((obs, oldVal, newVal) -> updateRobot());
        sliderQ2.valueProperty().addListener((obs, oldVal, newVal) -> updateRobot());

        btnUpdateLengths.setOnAction(e -> {
            try {
                robot.a1 = Double.parseDouble(txtA1.getText());
                robot.a2 = Double.parseDouble(txtA2.getText());
                updateRobot();
            } catch (Exception ex) {}
        });

        btnReverse.setOnAction(e -> performReverseCalculation());

        // Initial Draw
        updateRobot();
        
        return root;
    }

    private void updateRobot() {
        robot.q1 = sliderQ1.getValue();
        robot.q2 = sliderQ2.getValue();
        double ex = robot.getEndX();
        double ey = robot.getEndY();
        lblRobotPos.setText(String.format("End-Effector Global Pos: (%.2f, %.2f)", ex, ey));
        drawRobotArm(robot.getJ2X(), robot.getJ2Y(), ex, ey);
    }

    private void performReverseCalculation() {
        SCARARobot.Matrix3x3 forwardT = robot.getForwardMatrix();
        SCARARobot.Matrix3x3 reverseT = forwardT.inverse();

        double globalX = forwardT.m[0][2];
        double globalY = forwardT.m[1][2];

        // Validation
        double backX = reverseT.m[0][0]*globalX + reverseT.m[0][1]*globalY + reverseT.m[0][2];
        double backY = reverseT.m[1][0]*globalX + reverseT.m[1][1]*globalY + reverseT.m[1][2];

        StringBuilder sb = new StringBuilder();
        sb.append("--- Forward Transformation (a_T_b) ---\n");
        sb.append(forwardT.toString()).append("\n\n");
        sb.append("--- Reverse Transformation (b_T_a) ---\n");
        sb.append(reverseT.toString()).append("\n\n");
        sb.append("--- VERIFICATION ---\n");
        sb.append(String.format("End Pos transformed back to Base: (%.2f, %.2f)\n", backX, backY));
        
        if (Math.abs(backX) < 0.01 && Math.abs(backY) < 0.01) {
            sb.append("RESULT: SUCCESS (Returned to Origin)");
        } else {
            sb.append("RESULT: FAILED");
        }
        consoleOutput.setText(sb.toString());
    }

    private void drawRobotArm(double j2x, double j2y, double ex, double ey) {
        GraphicsContext gc = canvas.getGraphicsContext2D();
        double w = canvas.getWidth();
        double h = canvas.getHeight();
        gc.clearRect(0, 0, w, h);
        
        double originX = w / 4; 
        double originY = h / 2;

        gc.save();
        gc.translate(originX, originY);
        gc.scale(1, -1); 

        gc.setStroke(Color.LIGHTGRAY);
        gc.setLineWidth(1);
        gc.strokeLine(-50, 0, 400, 0); 
        gc.strokeLine(0, -150, 0, 150); 

        gc.setStroke(Color.RED);
        gc.setLineWidth(5);
        gc.strokeLine(0, 0, j2x, j2y);

        gc.setStroke(Color.BLUE);
        gc.strokeLine(j2x, j2y, ex, ey);

        gc.setFill(Color.BLACK);
        gc.fillOval(-5, -5, 10, 10);
        gc.fillOval(j2x - 5, j2y - 5, 10, 10);
        gc.setFill(Color.GREEN);
        gc.fillOval(ex - 5, ey - 5, 10, 10);

        gc.restore();
    }

    private VBox createCalculatorView() {
        VBox root = new VBox(20);
        root.setPadding(new Insets(20));
        root.setAlignment(Pos.CENTER);
        root.setStyle("-fx-font-family: 'Arial'; -fx-background-color: #f4f4f4;");

        Label title = new Label("Coordinate Frame Transformation Calculator");
        title.setFont(Font.font("Arial", FontWeight.BOLD, 18));
        Label formula = new Label("P_A = T_B_A  * P_B");
        formula.setFont(Font.font("Courier New", FontWeight.BOLD, 16));

        GridPane inputGrid = new GridPane();
        inputGrid.setHgap(15); inputGrid.setVgap(10);
        inputGrid.setAlignment(Pos.CENTER);
        inputGrid.setStyle("-fx-background-color: white; -fx-padding: 15; -fx-background-radius: 10; -fx-border-color: #ccc;");

        // Row 0
        inputGrid.add(new Label("Point B (x):"), 0, 0); inputGrid.add(txtXB, 1, 0);
        inputGrid.add(new Label("Point B (y):"), 0, 1); inputGrid.add(txtYB, 1, 1);

        // Row 1
        inputGrid.add(new Label("Rotation θ (°):"), 2, 0); inputGrid.add(txtTheta, 3, 0);
        inputGrid.add(new Label("Trans X (tx):"), 2, 1);   inputGrid.add(txtTx, 3, 1);
        inputGrid.add(new Label("Trans Y (ty):"), 2, 2);   inputGrid.add(txtTy, 3, 2);

        HBox matrixView = new HBox(10);
        matrixView.setAlignment(Pos.CENTER);
        matrixView.setStyle("-fx-padding: 20;");

        VBox vecA = createVectorBox("P^A", lblXA, lblYA, new Label("1"));
        Label eq = new Label(" = "); eq.setFont(Font.font(24));
        
        GridPane matrixGrid = new GridPane();
        matrixGrid.setHgap(10); matrixGrid.setVgap(5);
        matrixGrid.setStyle("-fx-border-color: black; -fx-border-width: 0 2 0 2; -fx-padding: 5;");
        matrixGrid.setAlignment(Pos.CENTER);
        
        addMatLabel(matrixGrid, lblR00, 0, 0); addMatLabel(matrixGrid, lblR01, 1, 0); addMatLabel(matrixGrid, lblTx, 2, 0);
        addMatLabel(matrixGrid, lblR10, 0, 1); addMatLabel(matrixGrid, lblR11, 1, 1); addMatLabel(matrixGrid, lblTy, 2, 1);
        addMatLabel(matrixGrid, new Label("0"), 0, 2); addMatLabel(matrixGrid, new Label("0"), 1, 2); 
        addMatLabel(matrixGrid, new Label("1"), 2, 2);

        Label visXB = new Label("0"); Label visYB = new Label("0");
        VBox vecB = createVectorBox("P^B", visXB, visYB, new Label("1"));

        matrixView.getChildren().addAll(vecA, eq, matrixGrid, new Label(" * "), vecB);

        // Logic
        Runnable calculate = () -> {
            try {
                double xb = Double.parseDouble(txtXB.getText());
                double yb = Double.parseDouble(txtYB.getText());
                double theta = Double.parseDouble(txtTheta.getText());
                double tx = Double.parseDouble(txtTx.getText());
                double ty = Double.parseDouble(txtTy.getText());

                double rad = Math.toRadians(theta);
                double cos = Math.cos(rad);
                double sin = Math.sin(rad);

                lblR00.setText(String.format("%.3f", cos));
                lblR01.setText(String.format("%.3f", -sin));
                lblTx.setText(String.format("%.1f", tx));
                lblR10.setText(String.format("%.3f", sin));
                lblR11.setText(String.format("%.3f", cos));
                lblTy.setText(String.format("%.1f", ty));

                visXB.setText(String.format("%.1f", xb));
                visYB.setText(String.format("%.1f", yb));

                double xa = (cos * xb) + (-sin * yb) + tx;
                double ya = (sin * xb) + (cos * yb) + ty;

                lblXA.setText(String.format("%.2f", xa));
                lblYA.setText(String.format("%.2f", ya));
                lblXA.setStyle("-fx-text-fill: blue; -fx-font-weight: bold;");
                lblYA.setStyle("-fx-text-fill: blue; -fx-font-weight: bold;");
            } catch (NumberFormatException e) {
                lblXA.setText("Err"); lblYA.setText("Err");
            }
        };

        txtXB.textProperty().addListener(e -> calculate.run());
        txtYB.textProperty().addListener(e -> calculate.run());
        txtTheta.textProperty().addListener(e -> calculate.run());
        txtTx.textProperty().addListener(e -> calculate.run());
        txtTy.textProperty().addListener(e -> calculate.run());

        calculate.run();
        root.getChildren().addAll(title, formula, inputGrid, matrixView);
        return root;
    }

    private VBox createVectorBox(String title, Label x, Label y, Label z) {
        VBox v = new VBox(5);
        v.setAlignment(Pos.CENTER);
        v.setStyle("-fx-border-color: black; -fx-border-width: 0 2 0 2; -fx-padding: 10; -fx-min-width: 60;");
        x.setFont(Font.font(16)); y.setFont(Font.font(16)); z.setFont(Font.font(16));
        v.getChildren().addAll(x, y, z);
        VBox container = new VBox(5);
        container.setAlignment(Pos.CENTER);
        Label lblTitle = new Label(title);
        lblTitle.setStyle("-fx-font-weight: bold;");
        container.getChildren().addAll(lblTitle, v);
        return container;
    }

    private void addMatLabel(GridPane grid, Label l, int c, int r) {
        l.setFont(Font.font(14));
        l.setMinWidth(60);
        l.setAlignment(Pos.CENTER);
        grid.add(l, c, r);
    }

    public static class SCARARobot {
        private double a1 = 150; 
        private double a2 = 100; 
        private double q1 = 45;  
        private double q2 = -30; 

        static class Matrix3x3 {
            double[][] m = new double[3][3];
            public Matrix3x3() { for (int i = 0; i < 3; i++) m[i][i] = 1; }

            public static Matrix3x3 createLinkTransform(double angleDegrees, double length) {
                Matrix3x3 mat = new Matrix3x3();
                double rad = Math.toRadians(angleDegrees);
                double c = Math.cos(rad);
                double s = Math.sin(rad);
                mat.m[0][0] = c; mat.m[0][1] = -s;
                mat.m[1][0] = s; mat.m[1][1] = c;
                mat.m[0][2] = length * c;
                mat.m[1][2] = length * s;
                return mat;
            }

            public Matrix3x3 multiply(Matrix3x3 other) {
                Matrix3x3 result = new Matrix3x3();
                for (int i = 0; i < 3; i++) { 
                    for (int j = 0; j < 3; j++) { 
                        result.m[i][j] = 0;
                        for (int k = 0; k < 3; k++) result.m[i][j] += this.m[i][k] * other.m[k][j];
                    }
                }
                return result;
            }

            public Matrix3x3 inverse() {
                Matrix3x3 inv = new Matrix3x3();
                inv.m[0][0] = this.m[0][0]; inv.m[0][1] = this.m[1][0];
                inv.m[1][0] = this.m[0][1]; inv.m[1][1] = this.m[1][1];
                double tx = this.m[0][2]; double ty = this.m[1][2];
                inv.m[0][2] = -(inv.m[0][0] * tx + inv.m[0][1] * ty);
                inv.m[1][2] = -(inv.m[1][0] * tx + inv.m[1][1] * ty);
                return inv;
            }

            @Override
            public String toString() {
                return String.format("[ %6.2f  %6.2f  %6.2f ]\n[ %6.2f  %6.2f  %6.2f ]\n[ %6.2f  %6.2f  %6.2f ]",
                    m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]);
            }
        }

        public Matrix3x3 getForwardMatrix() {
            return Matrix3x3.createLinkTransform(q1, a1).multiply(Matrix3x3.createLinkTransform(q2, a2));
        }
        public double getEndX() { return getForwardMatrix().m[0][2]; }
        public double getEndY() { return getForwardMatrix().m[1][2]; }
        public double getJ2X() { return Matrix3x3.createLinkTransform(q1, a1).m[0][2]; }
        public double getJ2Y() { return Matrix3x3.createLinkTransform(q1, a1).m[1][2]; }
    }

    public static void main(String[] args) {
        launch(args);
    }
}