import STC_pack.*;

public class Running_STC {
    public static void main(String[] args) {
        prim prim_problem = new prim();
        int graph[][] = new int[][] {   { 0, 2, 0, 6, 0 },
                                        { 2, 0, 3, 8, 5 },
                                        { 0, 3, 0, 0, 7 },
                                        { 6, 8, 0, 0, 9 },
                                        { 0, 5, 7, 9, 0 } };

		
        prim_problem.setV(5);
		prim_problem.primMST(graph);
        prim_problem.printMST();
    }
}
