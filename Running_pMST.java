import STC_pack.*;
import java.io.*;
import java.util.*;

public class Running_pMST {
    public static void main(String[] args) {
        try{
            // Read Input
            File file_in = new File("MST_Input.txt");
            BufferedReader br = new BufferedReader(new FileReader(file_in));
            String st;
            int vertices = 0;
            
            if ((st = br.readLine()) != null) {
                vertices = Integer.parseInt(st);
            }
            int graph[][] = new int[vertices][vertices];
            
            for(int i=0;i<vertices;i++){
                for(int j=0;j<vertices;j++){
                    if ((st = br.readLine()) != null) {
                        graph[i][j] = Integer.parseInt(st);
                    }         
                }
            }
            br.close();

            prim prim_problem = new prim();            
            prim_problem.setV(vertices);
            int parent[] = new int[vertices];
            parent = prim_problem.primMST(graph);

            File file_out = new File("MST_Output.txt");
            BufferedWriter brw = new BufferedWriter(new FileWriter(file_out));
            for(int v=0;v<vertices;v++){
                brw.write(String.valueOf(parent[v]));
                brw.write('\n');
            }
            brw.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
        
    }
}
