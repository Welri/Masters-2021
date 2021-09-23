package darp_pack;

import java.awt.*;
import java.util.*;

/**
 * Created by atkap on 5/16/2016.
 */
public class DARP {
    /* USER NOTES ON DARP
        Enew = Eold * CriterionMatrix * RandoMatrix * ConnectivityMatrix

        Criterion matrix - restores fairness between robots
        Connectivity matrix - ensures connectedness amongst cells
        Random matrix - to resolve issues with sharing

    */
    private double variateWeight, randomLevel;
    private int rows, cols, nr, ob, maxIter;
    private int[][] GridEnv;
    private ArrayList<Integer[]> RobotsInit; // Coordinates of where robots are initially
    private ArrayList<int[][]> BWlist;
    private int[][] A;
    private boolean[][] robotBinary;
    private int[] ArrayOfElements;
    private boolean[] ConnectedRobotRegions;
    private boolean success;
    private ArrayList<boolean[][]> BinrayRobotRegions;
    private int maxCellsAss, minCellsAss;
    private double elapsedTime;
    private int discr;
    private boolean canceled;
    private boolean UseImportance;

    // USER CODER START
    private int final_iterations;
    private int[][][] Ilabel_final;
    private boolean[] connected_bool;
    private int[][] obstacle_locations;
    private int distance_measure;
    // USER CODE END

    // Constructor
    public DARP(int r, int c, int[][] src, int iters, double vWeight, double rLevel, int discr, boolean imp, int dm) { // USER edit point - added String dm
        this.rows = r; // input - rows
        this.cols = c; // input - cols
        this.GridEnv = deepCopyMatrix(src);
        this.nr = 0; // initialization - nr counts the robots
        this.ob = 0; // initialization - ob counts the obstacles
        this.maxIter = iters; // input - maximum iterations
        this.RobotsInit = new ArrayList<>();
        this.A = new int[rows][cols];
        this.robotBinary = new boolean[rows][cols]; // initialization - RIP when True
        this.variateWeight = vWeight; // input
        this.randomLevel = rLevel; // input
        this.discr = discr; // input
        this.canceled = false;
        this.UseImportance = imp; // input
        // USER code start
        this.distance_measure = dm;
        // USER code end
        defineRobotsObstacles(); // function call - defineRobotObstacle
    }

    public void constructAssignmentM() {
        // USER CODE START
        connected_bool = new boolean[nr];
        Ilabel_final = new int[nr][rows][cols];
        this.final_iterations = 0;
        // USER CODE END

        long startTime = System.nanoTime();
        // Constant Initializations
        int NoTiles = rows * cols;

        int termThr;
        double fairDivision = 1.0 / nr;
        int effectiveSize = NoTiles - nr - ob;
        if (effectiveSize % nr != 0) {
            termThr = 1;
        } else {
            termThr = 0;
        }

        ArrayList<double[][]> AllDistances = new ArrayList<>();
        ArrayList<double[][]> TilesImportance = new ArrayList<>();

        for (int r = 0; r < nr; r++) {
            AllDistances.add(new double[rows][cols]);
            TilesImportance.add(new double[rows][cols]);
        }

        double[] MaximumDist, MaximumImportance, MinimumImportance;
        MaximumDist = new double[nr];
        MaximumImportance = new double[nr];
        MinimumImportance = new double[nr];
        for (int r = 0; r < nr; r++) {
            MinimumImportance[r] = Double.MAX_VALUE;
        }

        float[][] ONES2D = new float[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                double tempSum = 0.0;
                for (int r = 0; r < nr; r++) {
                    // USER edit start - used to just be Euclidean
                    if (this.distance_measure == 0){
                        AllDistances.get(r)[i][j] = EuclideanDis(RobotsInit.get(r), new Integer[] { i, j });
                    } else if (this.distance_measure == 1){
                        AllDistances.get(r)[i][j] = ManhattanDis(RobotsInit.get(r), new Integer[] { i, j });
                    } else if (this.distance_measure == 2){
                        AllDistances.get(r)[i][j] = GeodesicManhattanDis(RobotsInit.get(r), new Integer[] { i, j });
                    }
                    // USER edit end
                    if (AllDistances.get(r)[i][j] > MaximumDist[r]) {
                        MaximumDist[r] = AllDistances.get(r)[i][j];
                    }
                    tempSum += AllDistances.get(r)[i][j];
                }
                for (int r = 0; r < nr; r++) {
                    TilesImportance.get(r)[i][j] = 1.0 / (tempSum - AllDistances.get(r)[i][j]);
                    if (TilesImportance.get(r)[i][j] > MaximumImportance[r]) {
                        MaximumImportance[r] = TilesImportance.get(r)[i][j];
                    }
                    if (TilesImportance.get(r)[i][j] < MinimumImportance[r]) {
                        MinimumImportance[r] = TilesImportance.get(r)[i][j];
                    }
                }
                ONES2D[i][j] = 1;
            }
        }

        success = false;

        ArrayList<double[][]> MetricMatrix = deepCopyListMatrix(AllDistances);

        double[][] criterionMatrix = new double[rows][cols];

        while (termThr <= discr && !success && !canceled) {
            // Initializations
            double downThres = ((double) NoTiles - (double) termThr * (nr - 1)) / (double) (NoTiles * nr);
            double upperThres = ((double) NoTiles + termThr) / (double) (NoTiles * nr);

            success = true;
            // Main optimization loop
            int iter = 0;
            while (iter <= maxIter && !canceled) {
                assign(MetricMatrix);

                ArrayList<float[][]> ConnectedMultiplierList = new ArrayList<>();
                double[] plainErrors = new double[nr];
                double[] divFairError = new double[nr];

                // Connected Areas Component
                for (int r = 0; r < nr; r++) {
                    float[][] ConnectedMultiplier = deepCopyMatrix(ONES2D);
                    ConnectedRobotRegions[r] = true;

                    ConnectComponent cc = new ConnectComponent();
                    int[][] Ilabel = cc.compactLabeling(BWlist.get(r), new Dimension(cols, rows), true);

                    // USER CODE START
                    Ilabel_final[r] = Ilabel;
                    connected_bool[r] = (cc.getMaxLabel() <= 1);
                    // USER CODE END

                    if (cc.getMaxLabel() > 1) { // At least one unconnected regions among r-robot's regions is found
                        ConnectedRobotRegions[r] = false;

                        // Find robot's sub-region and construct Robot and Non-Robot binary regions
                        cc.constructBinaryImages(Ilabel[RobotsInit.get(r)[0]][RobotsInit.get(r)[1]]);

                        // Construct the final Connected Component Multiplier
                        ConnectedMultiplier = CalcConnectedMultiplier(cc.NormalizedEuclideanDistanceBinary(true),
                                cc.NormalizedEuclideanDistanceBinary(false));

                    }
                    ConnectedMultiplierList.add(r, ConnectedMultiplier);

                    // Calculate the deviation from the the Optimal Assignment
                    plainErrors[r] = ArrayOfElements[r] / (double) effectiveSize;
                    // System.out.print(ArrayOfElements[r]+ ", ");
                    // divFairError[r] = fairDivision - plainErrors[r];
                    if (plainErrors[r] < downThres) {
                        divFairError[r] = downThres - plainErrors[r];
                    } else if (plainErrors[r] > upperThres) {
                        divFairError[r] = upperThres - plainErrors[r];
                    }
                }

                // System.out.print("Iteration: "+iter+", ");

                // Exit conditions
                if (isThisAGoalState(termThr)) {
                    break;
                }

                double TotalNegPerc = 0.0, totalNegPlainErrors = 0.0;
                double[] correctionMult = new double[nr];

                for (int r = 0; r < nr; r++) {
                    if (divFairError[r] < 0) {
                        TotalNegPerc += Math.abs(divFairError[r]);
                        totalNegPlainErrors += plainErrors[r];
                    }
                    correctionMult[r] = 1.0;
                }

                // Restore Fairness among the different partitions
                for (int r = 0; r < nr; r++) {
                    if (totalNegPlainErrors != 0.0) {

                        // correctionMult[r]=1.0
                        // -(divFairError[r]/(TotalNegPerc*nr))*(totalNegPlainErrors/2.0);

                        if (divFairError[r] < 0.0) {
                            correctionMult[r] = 1.0 + (plainErrors[r] / totalNegPlainErrors) * (TotalNegPerc / 2.0);
                        } else {
                            correctionMult[r] = 1.0 - (plainErrors[r] / totalNegPlainErrors) * (TotalNegPerc / 2.0);
                        }

                        criterionMatrix = calculateCriterionMatrix(TilesImportance.get(r), MinimumImportance[r],
                                MaximumImportance[r], correctionMult[r], (divFairError[r] < 0));
                    }
                    MetricMatrix.set(r, FinalUpdateOnMetricMatrix(criterionMatrix, generateRandomMatrix(),
                            MetricMatrix.get(r), ConnectedMultiplierList.get(r)));
                }

                iter++;
            }
            // USER CODE START
            this.final_iterations = this.final_iterations + iter;
            // USER CODE END

            // USER CODE DELETION (changed from >= to >)
            // Can't remember why this was a problem, but it was (think it increased iterations or made it not exit or something - like it reaches roughly 0 maxIter but just keeps going)
            if (iter > maxIter) {
                maxIter = maxIter / 2;
                success = false;
                termThr++;
            }
        }

        elapsedTime = (double) (System.nanoTime() - startTime) / Math.pow(10, 9);
        calculateRobotBinaryArrays();

    }

    private void calculateRobotBinaryArrays() {
        BinrayRobotRegions = new ArrayList<>();
        for (int r = 0; r < nr; r++) {
            BinrayRobotRegions.add(new boolean[rows][cols]);
        }
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (A[i][j] < nr) {
                    BinrayRobotRegions.get(A[i][j])[i][j] = true;
                }
            }
        }
    }

    private double[][] FinalUpdateOnMetricMatrix(double[][] CM, double[][] RM, double[][] curentONe, float[][] CC) {
        double[][] MMnew = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                MMnew[i][j] = curentONe[i][j] * CM[i][j] * RM[i][j] * CC[i][j];
            }
        }

        return MMnew;
    }

    private double[][] generateRandomMatrix() {

        double[][] RandomMa = new double[rows][cols];
        Random randomno = new Random();

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                // USER CODE START 
                // RandomMa[i][j] = 1;
                // USER CODE END

                // The above was to figure out if random matrix is important - it is very important

                // USER Edit start
                if (randomLevel == 0){
                    RandomMa[i][j] = 1.0; // So that if the user doesn't want a random element they can just set the randomLevel to 0 
                }else{
                    RandomMa[i][j] = 2.0 * randomLevel * randomno.nextDouble() + 1.0 - randomLevel; // Only this line is theirs
                }
                // USER Edit end
                
            }
        }

        return RandomMa;
    }

    private double[][] calculateCriterionMatrix(double[][] TilesImp, double minImp, double maxImp, double corMult,
            boolean SmallerThan0) {
        double[][] retrunCriter = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (UseImportance) {
                    if (SmallerThan0) {
                        retrunCriter[i][j] = (TilesImp[i][j] - minImp) * ((corMult - 1) / (maxImp - minImp)) + 1;
                    } else {
                        retrunCriter[i][j] = (TilesImp[i][j] - minImp) * ((1 - corMult) / (maxImp - minImp)) + corMult;
                    }
                } else {
                    retrunCriter[i][j] = corMult;
                }
            }
        }

        return retrunCriter;
    }

    private boolean isThisAGoalState(int thres) {
        maxCellsAss = 0;
        minCellsAss = Integer.MAX_VALUE;

        for (int r = 0; r < nr; r++) {
            if (maxCellsAss < ArrayOfElements[r]) {
                maxCellsAss = ArrayOfElements[r];
            }
            if (minCellsAss > ArrayOfElements[r]) {
                minCellsAss = ArrayOfElements[r];
            }
        }
        for (int r = 0; r < nr; r++) {
            if (!ConnectedRobotRegions[r]) {
                return false;
            }
        }

        /*
         * System.out.println("Discrepancey: "+(maxCellsAss-minCellsAss)+"("+thres+")");
         * for (int r=0;r<nr;r++) { if (!ConnectedRobotRegions[r]) {return false;}
         * 
         * }
         */

        return (maxCellsAss - minCellsAss) <= thres;

    }

    private float[][] CalcConnectedMultiplier(float[][] dist1, float[][] dist2) {
        float[][] returnM = new float[rows][cols];
        float MaxV = 0;
        float MinV = Float.MAX_VALUE;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                returnM[i][j] = dist1[i][j] - dist2[i][j];
                if (MaxV < returnM[i][j]) {
                    MaxV = returnM[i][j];
                }
                if (MinV > returnM[i][j]) {
                    MinV = returnM[i][j];
                }
            }
        }

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                returnM[i][j] = (returnM[i][j] - MinV) * ((2 * (float) variateWeight) / (MaxV - MinV))
                        + (1 - (float) variateWeight);
            }
        }

        return returnM;
    }

    private void assign(ArrayList<double[][]> Q) {
        // Construct BWList - Similar to Assignment matrix except it is multiple matrices with 1 where assigned to that robot
        // Construct A - Assignment matrix - Different number depending on which cell is assigned to that robot
        BWlist = new ArrayList<>();
        for (int r = 0; r < nr; r++) {
            BWlist.add(new int[rows][cols]);
            BWlist.get(r)[RobotsInit.get(r)[0]][RobotsInit.get(r)[1]] = 1;
        }

        ArrayOfElements = new int[nr];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (GridEnv[i][j] == -1) {
                    double minV = Q.get(0)[i][j];
                    int indMin = 0;
                    for (int r = 1; r < nr; r++) {
                        if (Q.get(r)[i][j] < minV) {
                            minV = Q.get(r)[i][j];
                            indMin = r;
                        }
                    }
                    A[i][j] = indMin;
                    BWlist.get(indMin)[i][j] = 1;
                    ArrayOfElements[indMin]++;
                } else if (GridEnv[i][j] == -2) {
                    A[i][j] = nr;
                }
            }
        }

    }

    private double EuclideanDis(double[] a, double[] b) {
        int vecSize = a.length;
        double d = 0.0;
        for (int i = 0; i < vecSize; i++) {
            d += Math.pow(a[i] - b[i], 2.0);
        }
        return Math.sqrt(d);
    }

    private double EuclideanDis(Integer[] a, Integer[] b) {
        int vecSize = a.length;
        double d = 0.0;
        for (int i = 0; i < vecSize; i++) {
            d += Math.pow(a[i] - b[i], 2.0);
        }
        return Math.sqrt(d);
    }

    private int[][] deepCopyMatrix(int[][] input) {
        if (input == null)
            return null;
        int[][] result = new int[input.length][];
        for (int r = 0; r < input.length; r++) {
            result[r] = input[r].clone();
        }
        return result;
    }

    private ArrayList<double[][]> deepCopyListMatrix(ArrayList<double[][]> input) {
        if (input == null)
            return null;
        ArrayList<double[][]> result = new ArrayList<>();
        for (int r = 0; r < input.size(); r++) {
            result.add(deepCopyMatrix(input.get(r)));
        }
        return result;
    }

    private double[][] deepCopyMatrix(double[][] input) {
        if (input == null)
            return null;
        double[][] result = new double[input.length][];
        for (int r = 0; r < input.length; r++) {
            result[r] = input[r].clone();
        }
        return result;
    }

    private int[] deepCopyMatrix(int[] input) {
        if (input == null)
            return null;
        int[] result = input.clone();
        return result;
    }

    private double[] deepCopyMatrix(double[] input) {
        if (input == null)
            return null;
        double[] result = input.clone();
        return result;
    }

    private float[][] deepCopyMatrix(float[][] input) {
        if (input == null)
            return null;
        float[][] result = new float[input.length][];
        for (int r = 0; r < input.length; r++) {
            result[r] = input[r].clone();
        }
        return result;
    }

    private void defineRobotsObstacles() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (GridEnv[i][j] == 2) {
                    robotBinary[i][j] = true;
                    RobotsInit.add(new Integer[] { i, j });
                    GridEnv[i][j] = nr; // GridEnv gets overwritten with that robot index
                    A[i][j] = nr; // Assignment matrix at the robot IP should be assigned to that robot
                    nr++; // Increment robot
                } else if (GridEnv[i][j] == 1) {
                    ob++; // Increment number of obstacles
                    GridEnv[i][j] = -2; // Set GridEnv to -2 at that point
                } else {
                    GridEnv[i][j] = -1;
                } // Every slot that is not a RIP or a Obstacle gets assigned -1
            }
        }
        ConnectedRobotRegions = new boolean[nr];

        // USER code start
        int counter = 0;
        obstacle_locations = new int[ob][2];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (GridEnv[i][j] == -2){
                    obstacle_locations[counter][0] = i;
                    obstacle_locations[counter][1] = j;
                    counter++;
                }
            }
        }
        // USER code end
    }

    private void printMatrix(int[][] M) {
        int r = M.length;
        int c = M[0].length;

        System.out.println();
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                System.out.print(M[i][j] + " ");
            }
            System.out.println();
        }
    }

    private void printMatrix(float[][] M) {
        int r = M.length;
        int c = M[0].length;

        System.out.println();
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                System.out.print(M[i][j] + " ");
            }
            System.out.println();
        }
    }

    private void printMatrix(double[][] M) {
        int r = M.length;
        int c = M[0].length;

        System.out.println();
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                System.out.print(M[i][j] + " ");
            }
            System.out.println();
        }
    }

    public boolean getSuccess() {
        return success;
    }

    public int getNr() {
        return nr;
    }

    public int getNumOB() {
        return ob;
    }

    public int[][] getAssignmentMatrix() {
        return A;
    }

    public boolean[][] getRobotBinary() {
        return robotBinary;
    }

    public ArrayList<boolean[][]> getBinrayRobotRegions() {
        return BinrayRobotRegions;
    }

    public ArrayList<Integer[]> getRobotsInit() {
        return RobotsInit;
    }

    public int getEffectiveSize() {
        return 4 * (rows * cols - ob);
    }

    public int getMaxCellsAss() {
        return 4 * (maxCellsAss + 1);
    }

    public int getMinCellsAss() {
        return 4 * (minCellsAss + 1);
    }

    public double getElapsedTime() {
        return elapsedTime;
    }

    public int getDiscr() {
        return discr;
    }

    public int getMaxIter() {
        return maxIter;
    }

    public void setCanceled(boolean c) {
        this.canceled = c;
    }

    public int getAchievedDiscr() {
        return maxCellsAss - minCellsAss;
    }

    // USER CODE START
    public int getIterations() {
        return final_iterations;
    }

    public boolean[] getConnectedBool() {
        return connected_bool;
    }

    public int[][][] getIlabel() {
        return Ilabel_final;
    }

    public int[] getArrayOfElements() {
        return ArrayOfElements;
    }

    private double ManhattanDis(Integer[] a, Integer[] b) {
        int vecSize = a.length;
        double d = 0.0;
        for (int i = 0; i < vecSize; i++) {
            d += Math.abs(a[i] - b[i]);
        }
        return d;
    }

    private double GeodesicManhattanDis(Integer[] start, Integer[] end){
        // Only works with one start and endpoint
        int d = 0;
        Node initialNode = new Node(start[0],start[1]);
        Node finalNode = new Node(end[0],end[1]);
        int rows = this.rows;
        int cols = this.cols;
        AStar aStar = new AStar(rows,cols,initialNode,finalNode);
        aStar.setBlocks(this.obstacle_locations); // would be better if not necessary to declare this every time
        java.util.List<Node> path = aStar.findPath();
        for (Node node : path) {
            // System.out.println(node);
            d++;
        }
        return(d);
    }


    // Should find a way to calculate Geodesic Distance (maybe use Astar)
    // USER CODE END
}
