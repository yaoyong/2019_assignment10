package jobinterviewquestions;

import algs4.*;

public class ShortestPaths {
    /*
    Question 1
    Monotonic shortest path.
    */
    private DirectedEdge[] edgeTo;
    private double[] distTo;
    private IndexMinPQ<Double> pq;

    /*
    1. Sort edges of each vertex, ascending or descending
    2. relax edges using Dijkstra algorithm, check monotonic condition before check distTo
     */
     
     public class DijkstraSP
{
 private DirectedEdge[] edgeTo;
 private double[] distTo;
 private IndexMinPQ<Double> pq;
 public DijkstraSP(EdgeWeightedDigraph G, int s)
 {
 edgeTo = new DirectedEdge[G.V()];
 distTo = new double[G.V()];
 pq = new IndexMinPQ<Double>(G.V());
 for (int v = 0; v < G.V(); v++)
 distTo[v] = Double.POSITIVE_INFINITY;
 distTo[s] = 0.0;
 pq.insert(s, 0.0);
 while (!pq.isEmpty())
 {
 int v = pq.delMin();
 for (DirectedEdge e : G.adj(v))
 relax(e);
 }
 }
 }
 private void relax(DirectedEdge e)
 {
 int v = e.from(), w = e.to();
 if (distTo[w] > distTo[v] + e.weight())
 {
 distTo[w] = distTo[v] + e.weight();
 edgeTo[w] = e;
 if (pq.contains(w)) pq.decreaseKey(w, distTo[w]);
 else pq.insert (w, distTo[w]);
 }
 }

    /*
    Question 2
    Second shortest path.
    */

 public class ShortestPathAnd2ndShortestDijkstras {
	static final int NO_PARENT = -1;	
	static Set<Integer> edges = new LinkedHashSet<>(); //nodes in the shortest path
	static Set<Integer> dists = new TreeSet<>(); //list of distances of paths from src to dest, sorted 
	
	//use Dijkstra’s Shortest Path Algorithm, O(n^2) Space O(n)
    static void shortestPath(int[][] adjacencyMatrix,  int src, int dest) { 
        int n = adjacencyMatrix[0].length; 
        int[] shortest = new int[n]; 
        boolean[] added = new boolean[n]; 
        for (int v = 0; v < n;v++)  { 
            shortest[v] = Integer.MAX_VALUE; 
            added[v] = false; 
        } 
        shortest[src] = 0; 
        int[] parents = new int[n]; 
        parents[src] = NO_PARENT; 
 
        for (int i = 1; i < n; i++)  { 
            int v1 = -1; //store temp data
            int min = Integer.MAX_VALUE; 
            for (int v = 0;  v < n;  v++) { 
                if (!added[v] &&  shortest[v] < min) { 
                    v1 = v; 
                    min = shortest[v]; 
                } 
            } 
            added[v1] = true; 
            for (int v = 0; v < n; v++)  { 
                int dist = adjacencyMatrix[v1][v];                  
                if (dist > 0 && ((min + dist) <shortest[v])){ 
                    parents[v] = v1; 
                    shortest[v] = min + dist; 
                } 
            } 
        }  
        dists.add(shortest[dest]);
        visitUtil(dest, parents); 
    } 
   
    static void visitUtil(int i,int[] parents)  { 	
        if (i == NO_PARENT)        	
            return; 
   	
        visitUtil(parents[i], parents);             
        edges.add(i);
    } 
 
    public static void main(String[] args) { 
		/*
		 *     0
		 *  10/ \3
		 *   /   \
		 *  1--1--4
		 * 5|  8/ |2
		 *  | /   |
		 *  2--7--3
		 */   	
    	//the value of the node to itself is 0, 
    	//if not direction connection, value is 0 
        int[][] adjacencyMatrix = new int[][] {
            { 0,10, 0, 0, 3 },
            {10, 0, 5, 0, 1 },
            { 0, 5, 0, 7, 8 },
            { 0, 0, 7, 0, 2 },
            { 3, 1, 8, 2, 0 }
        };
        
        //get shortest path
        int src = 2, dest = 4;
        shortestPath(adjacencyMatrix,src,dest); 
 
        //get 2nd shortest by removing each edge in shortest and compare  
        int tmp = -1, s1 = -1, d1 = -1; //store temp data
        List<Integer> list = new ArrayList<Integer>(edges);        
        for (int i = 0; i < list.size()-1 ; i++) {
        	int s = list.get(i);
        	int d = list.get(i + 1);
        	if (tmp != -1) {
        		adjacencyMatrix[s1][d1] = tmp;
        		adjacencyMatrix[d1][s1] = tmp;
        	}
        	tmp = adjacencyMatrix[s][d];
        	s1 = s;
        	d1 = d;
	        adjacencyMatrix[s][d] = 0;
	        adjacencyMatrix[d][s] = 0;
	        shortestPath(adjacencyMatrix, src , dest);
        }
        
        list = new ArrayList<Integer>(dists); 
        System.out.println("Shortest:" + list.get(0));
        System.out.println("2nd shortest:" + list.get(1));       
    } 
}



    /*
    Question 3
    Shortest path with one skippable edge.
     */
     
    public Iterable<DirectedEdge> skippablePath(EdgeWeightedDigraph G,int s, int t) {
        DijkstraSP spaths = new DijkstraSP(G, s);
        DijkstraSP tpaths = new DijkstraSP(G.reverse(), t);

        double min = Double.MAX_VALUE;
        DirectedEdge skippable = null;

        for (DirectedEdge e : G.edges()) {
            int v = e.from();
            int w = e.to();
            if (spaths.distTo(v) + tpaths.distTo(w) < min) {
                skippable = e;
                min = spaths.distTo(v) + tpaths.distTo(w);
            }
        }

        Stack<DirectedEdge> skippablepath = new Stack<DirectedEdge>();
        Stack<DirectedEdge> tmp = new Stack<DirectedEdge>();

        for (DirectedEdge e : tpaths.pathTo(skippable.to())) skippablepath.push(e);
        skippablepath.push(skippable);
        for (DirectedEdge e : spaths.pathTo(skippable.from())) tmp.push(e);
        for (DirectedEdge e : tmp) skippablepath.push(e);
        return skippablepath;
    }
}
