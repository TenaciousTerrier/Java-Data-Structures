/**
 * 
 */
package graph;
import java.util.*;
import java.io.*;
import java.util.concurrent.*;
import util.GraphLoader;

//import org.graphstream.graph.*;
//import org.graphstream.graph.implementations.*;
import java.util.*;

/**
 * @author Thomas Hepner
 * 
 * For the warm up assignment, you must implement your Graph in a class
 * named CapGraph.  Here is the stub file.
 *
 */
public class CapGraph implements Graph {
	private int numNodes;
	private int numEdges;
	private Map<Integer, CapNode> map;
	private Map<Integer, CapNode> mapTranspose;
	
	public CapGraph()
	{
		numNodes = 0;
		numEdges = 0;
		map = new HashMap<Integer, CapNode>();
		mapTranspose = new HashMap<Integer, CapNode>();
	}
	
	public int getNumNodes() {
		return numNodes;
	}
	
	
	public int getNumEdges() {
		return numEdges;
	}
	
	/* (non-Javadoc)
	 * @see graph.Graph#addVertex(int)
	 */
	@Override
	public void addVertex(int num) {
		if(!map.keySet().contains(num)) {
			CapNode node = new CapNode(num);
			numNodes += 1;
			map.put(num, node);
		}
	}

	/* (non-Javadoc)
	 * @see graph.Graph#addEdge(int, int)
	 */
	@Override
	public void addEdge(int from, int to) throws IndexOutOfBoundsException {
		if(map.keySet().contains(from) && map.keySet().contains(to)) {
			CapEdge edge = new CapEdge(from, to);
			numEdges += 1;
			CapNode node = map.get(from);
			node.addEdge(edge);
		} 
		else {
			throw new IndexOutOfBoundsException("Input integers do not exist as nodes in graph!"); 
		}
	}

	/* (non-Javadoc)
	 * @see graph.Graph#getEgonet(int)
	 */
	@Override
	public Graph getEgonet(int center) {
		Graph Egonet = new CapGraph();
		Egonet.addVertex(center);
		HashSet<CapEdge> centerEdges = map.get(center).getEdges();		
		Iterator<CapEdge> iterator = centerEdges.iterator();

		// Map center node and connect to adjacent nodes
		for(CapEdge nextEdge : centerEdges) {
			Egonet.addVertex(nextEdge.getEndNode());
			Egonet.addEdge(center,  nextEdge.getEndNode());
		}		
		
		// Map edges between non-central nodes
		while(iterator.hasNext()) {
			CapEdge nextEdge = iterator.next();
			Egonet.addEdge(nextEdge.getEndNode(), center);
			
			// Find adjacent edges to the current node
			HashSet<Integer> adjEdges = new HashSet<Integer>();
			for(CapEdge j : map.get(nextEdge.getEndNode()).getEdges()) {
				adjEdges.add(j.getEndNode());
			}
			
			// Connect adjacent edges where appropriate
			for(CapEdge edge : centerEdges) {				
				if(adjEdges.contains(edge.getEndNode())) {
					Egonet.addVertex(edge.getEndNode());
					Egonet.addEdge(nextEdge.getEndNode(), edge.getEndNode());
				}
			}	
		}
		return Egonet;
	}
	
	/* (non-Javadoc)
	 * @see graph.Graph#getSCCs()
	 */
	@Override
	public List<Graph> getSCCs() {
		List<Graph> SCCs = null;
		
		// Create stack of vertices in graph, smallest integer value should be at top of stack
		List<Integer> verticesList = new ArrayList(map.keySet());
		Collections.sort(verticesList, Collections.reverseOrder());
		Stack<Integer> vertices = new Stack<Integer>();	
		for(Integer vertex : verticesList) {
			vertices.push(vertex);
		}	
		
		// Visit all nodes in the graph and keep track of order of exploration				
		Stack<Integer> finished = DFS(map, vertices, null);	
		
		// Compute transpose of Graph
		CapGraph graphTranspose = transposeGraph(map);
		mapTranspose = graphTranspose.map;

		// Visit all nodes in the graph in reverse order
		SCCs = new ArrayList<Graph>();
		Stack<Integer> finishedTranspose = DFS(mapTranspose, finished, SCCs);	
		return SCCs;
	}
	
	// Reverses edges in CapGraph
	private CapGraph transposeGraph(Map<Integer, CapNode> map) {
		CapGraph GraphTranspose = new CapGraph();
		HashSet<CapNode> vertices = new HashSet<CapNode>();
		for(Integer vertex : map.keySet()) {
			vertices.add(map.get(vertex));
		}

		for(CapNode vertex : vertices) {
			HashSet<CapEdge> edges = vertex.getEdges();	
			GraphTranspose.addVertex(vertex.getNode());
			for(CapEdge edge : edges) {
				GraphTranspose.addVertex(edge.getEndNode());
				GraphTranspose.addEdge(edge.getEndNode(), vertex.getNode());
			}
		}
		return GraphTranspose;
	}

	// Depth First Search Algorithm: Iterates over all nodes in the graph
	private Stack<Integer> DFS(Map<Integer, CapNode> map, Stack<Integer> vertices, List<Graph> SCCs) { 
		
		HashSet<Integer> visited = new HashSet<Integer>(); 
		Stack<Integer> finished = new Stack<Integer>(); 
		
		while(!vertices.isEmpty()) {
			Integer v = vertices.pop();
			CapGraph subgraph = new CapGraph();		
			HashSet<CapEdge> edges = map.get(v).getEdges();

			if(!visited.contains(v)) {
				subgraph.addVertex(v);
				subgraph = DFS_VISIT(map, v, visited, finished, subgraph);
			}
			if(SCCs != null & subgraph.getNumNodes() > 0) {
				SCCs.add(subgraph);
			}
		}
		return finished;
	}
	
	// Visits node if it hasn't been visited yet
	private CapGraph DFS_VISIT(Map<Integer, CapNode> map, Integer v, HashSet<Integer> visited, Stack<Integer> finished, CapGraph subgraph) throws IndexOutOfBoundsException {
		visited.add(v);
		//System.out.println("2");
		HashSet<CapEdge> edges = map.get(v).getEdges();
		for(CapEdge edge : edges) { 
			Integer n = edge.getEndNode();
			if(!visited.contains(n)) {
				subgraph.addVertex(n);
				subgraph.addEdge(v, n);
				DFS_VISIT(map, n, visited, finished, subgraph);
			}
		}
		finished.push(v);
		return subgraph;
	}
	
	/* (non-Javadoc)
	 * @see graph.Graph#exportGraph()
	 */
	@Override
	public HashMap<Integer, HashSet<Integer>> exportGraph() {
		HashMap<Integer, HashSet<Integer>> export = new HashMap<Integer, HashSet<Integer>>();
		for(Integer key : map.keySet()) {
			HashSet<CapEdge> edges = map.get(key).getEdges();
			HashSet<Integer> adjEdges = new HashSet<Integer>();
			for(CapEdge j : edges) {
				adjEdges.add(j.getEndNode());
			}
			export.put(key, adjEdges);
		}		
		return export;
	}
	
	public HashSet<Integer> getMinimumDominatingSet() {
		// Mark all vertices as uncovered:
		HashSet<Integer> dominantNodes = new HashSet<Integer>();
		HashSet<Integer> vertices = new HashSet(map.keySet());
		HashSet<Integer> coveredVertices = new HashSet<Integer>();
		
		// While there are uncovered vertices:
		while(!vertices.isEmpty()) {
			// Find the vertex, v, which would cover the most uncovered vertices
			int maxCoverage = 0;
			int maxNode = 0;
			for(Integer vertex : vertices) {
				HashSet<CapEdge> edges = map.get(vertex).getEdges();
				if(edges.size() > maxCoverage) {
					maxCoverage = edges.size();
					maxNode = vertex;
				}
			}
			
			// Add v to the dominant set
			dominantNodes.add(maxNode);
			vertices.remove(maxNode);

			// Mark that vertex and all of its neighbors as covered
			coveredVertices.add(maxNode);
			HashSet<CapEdge> edges = map.get(maxNode).getEdges();
			for(CapEdge edge : edges) { 
				Integer node = edge.getEndNode();
				coveredVertices.add(node);
				vertices.remove(node);
			}
		}

		return dominantNodes;
	}
	
	public List<Graph> getCommunities() {
		List<Graph> communities = null; 
		double time = 0;
		double dtime = 0;
		communities = getSCCs();

		while(communities.size() < 20) {
			// Compute betweenness of all edges
			// For each node v: 
			CapEdge highestBetweenness = null;
			for(Integer start : map.keySet()) {	
				final long startTime = System.nanoTime();
				for(Integer goal : map.keySet()) {

					if(goal != start) {
						// 1. BFS of graph starting at v	
						final long checkTime = System.nanoTime();
						Map<Integer, HashSet<Integer>> parent = dijkstra(start, goal);
						dtime += (double) (System.nanoTime() - checkTime) / 1000000000.0;
												
						// 2. compute # of shortest paths from v to each other node
						// 3. Distribute flow to edges along these paths			
						Stack<Integer> shortestPaths = new Stack<Integer>();
						shortestPaths.add(goal);
						while(!shortestPaths.isEmpty()) {
							Integer top = shortestPaths.pop();
							if(!top.equals(start) && parent != null) {
								HashSet<Integer> nextNodes = parent.get(top);
								for(Integer next : nextNodes) {
									shortestPaths.push(next);
									
									// Add +1 to betweenness for top and next edges
									HashSet<CapEdge> topEdges = map.get(top).getEdges();
									HashSet<CapEdge> nextEdges = map.get(next).getEdges();
									
									CapEdge edgeBetweenness = null;
									for(CapEdge edge : topEdges) {
										if(edge.getStartNode() == top && edge.getEndNode() == next) {
											edge.setBetweenness(edge.getBetweenness() + 1.0);
										}
										if(edgeBetweenness == null || edge.getBetweenness() > edgeBetweenness.getBetweenness() ) {
											edgeBetweenness = edge;
										}
									}
									
									for(CapEdge edge : nextEdges) {									
										if(edge.getStartNode() == next && edge.getEndNode() == top) {
											edge.setBetweenness(edge.getBetweenness() + 1.0);
										}
										if(edgeBetweenness == null || edge.getBetweenness() > edgeBetweenness.getBetweenness() ) {
											edgeBetweenness = edge;
										}
									}
									
									// Determine edge with highest betweenness
									if(highestBetweenness == null || edgeBetweenness.getBetweenness() > highestBetweenness.getBetweenness() ) {
										highestBetweenness = edgeBetweenness;
									}
								}
							}
						}
					}
				}
				long duration = System.nanoTime() - startTime;
				double seconds = (double) duration / 1000000000.0;
				time += seconds;
			}
			// Remove edges of highest betweenness
			System.out.println("Edge Cut: " + highestBetweenness.getStartNode() + ", " + highestBetweenness.getEndNode());
			System.out.println("Betweenness: " + highestBetweenness.getBetweenness());
			
			Integer startNode = highestBetweenness.getStartNode(); 
			Integer endNode = highestBetweenness.getEndNode();
			
			map.get(startNode).getEdges().remove(highestBetweenness);
			CapEdge compEdge = findEdge(map.get(endNode).getEdges(), endNode, startNode);
			map.get(endNode).getEdges().remove(compEdge);
			
			// Reset edge-betweeenness to 0
			resetBetweenness();
			resetNodeDistances();	
			communities = getSCCs();
			System.out.println("# of Communities: " + communities.size());
			System.out.println();
		}
		System.out.println("Edge Running Time: " + time);	
		System.out.println("BFS Running Time: " + dtime);	
		return communities;
	}
	
	private CapEdge findEdge(HashSet<CapEdge> edges, int start, int end) {
		for(CapEdge e : edges) {
			if(e.getStartNode() == start && e.getEndNode() == end) {
				return e; 
			}
		}
		return null;
	}
		
	public Map<Integer, HashSet<Integer>> dijkstra(Integer start, Integer goal) {
		// Initialize PriorityQueue, parent map, and visited HashSet, and distances to infinity
		PriorityQueue<CapNode> myQueue = new PriorityQueue<CapNode>();
		Set<CapNode> visited = new HashSet<CapNode>();
		Map<Integer, HashSet<Integer>> parent = new HashMap<Integer, HashSet<Integer>>(); 
		CapNode curr = map.get(start);
		curr.setDistance(0);
		myQueue.add(curr);

		while(!myQueue.isEmpty()) {		
			curr = myQueue.remove();
			
			if(!visited.contains(curr)) {
				visited.add(curr);
				
				// Return parent map if curr location equals goal location
				if(curr.getNode() == goal) {
					return parent;
				}		
				
				// Add nodes to parent map, visited, and PriorityQueue
				for(CapEdge edge : curr.getEdges()) {
					Integer end = edge.getEndNode();
					CapNode node = map.get(end);	
		
					if(!visited.contains(node)) {
						double newDist = curr.getDistance() + 1;	
						double priorDist = node.getDistance();
						
						if(newDist <= priorDist) { 
							node.setDistance(newDist);	
							myQueue.add(node);
							HashSet<Integer> nodes = new HashSet<Integer>();
							if(parent.get(node.getNode()) != null) {
								nodes = parent.get(node.getNode());
							}
							nodes.add(curr.getNode());
							parent.put(node.getNode(), nodes); 
						}
					}
				} 			
			}
		}
		return null;
	}

	// Reset CapNode distances' to INFINITY
	
	public void resetNodeDistances() {
		for(Integer key : map.keySet()) {
			CapNode node = map.get(key);
			node.setDistance(Double.POSITIVE_INFINITY);
		}
	}
			
	public void resetBetweenness() {
		for(Integer key : map.keySet()) {
			CapNode node = map.get(key);
			for(CapEdge edge : node.getEdges()) {
				edge.setBetweenness(0);
			}
		}
	}
	
	public static void main(String[] args) {		
		// Use this code to test results
		CapGraph theMap = new CapGraph();
		System.out.print("DONE. \nLoading the map..."); 
		GraphLoader.loadGraph(theMap, "data/facebook_1000.txt"); // data/communities/case_10.txt facebook_1000
		System.out.println("DONE.");	
		System.out.println();
		
		// Find # of Nodes and Edges in the Graph
		System.out.println("Nodes: " + theMap.getNumNodes() + " , Edges: " + theMap.getNumEdges());
		
		// Store map in different format for grading purposes
		/*
		HashMap<Integer, HashSet<Integer>> exportMap = theMap.exportGraph();
		for(Integer key : exportMap.keySet()) {
			System.out.println("Node: " + key + " , Edges: " + exportMap.get(key));
		}
		*/
		
		// Test EgoNet
		/*
		Graph ego = theMap.getEgonet(22);
		HashMap<Integer, HashSet<Integer>> exportEgo = ego.exportGraph();
		for(Integer key : exportEgo.keySet()) {
			System.out.println("Node: " + key + " , Edges: " + exportEgo.get(key));
		}
		*/
		
		// Test getSCCs
		//final long startTime = System.nanoTime();
		List<Graph> SCCs = theMap.getSCCs();
		System.out.println("Strongly Connected Components: ");
		for(Graph SCC : SCCs) {
			HashMap<Integer, HashSet<Integer>> exportNextSCC = SCC.exportGraph();
			Set<Integer> graphNodes = exportNextSCC.keySet();
			System.out.print(graphNodes + " : ");
			for(Integer node : graphNodes) {
				System.out.print(node + " ");
			}
			System.out.println();
		}
		System.out.println();
		System.out.println("Size: " + SCCs.size());
		//final long duration = System.nanoTime() - startTime;
		//double seconds = (double) duration / 1000000000.0;
		//System.out.println("Duration: " + seconds + " seconds"); // duration / 1000000000
		System.out.println();
		
		
		// Test getMinimumDominatingSet
		// 1st Test: small_test_graph --> figured out solution by hand --> check algorithm answer
		// 2nd Test: small_test_graph_2 --> should return [2, 7]
		// 3rd Test: small_test_graph_3 --> fails to find minimum dominating set should rerun [4, 1, 8]
		
		HashSet<Integer> dominantSet = theMap.getMinimumDominatingSet();
		System.out.println("Minimum Dominating Set: ");
		System.out.println(dominantSet);
		System.out.println();
		
		// Test BFS Method
		/*
		int start = 2;
		int goal = 12;
		final long startTime = System.nanoTime();
		Map<Integer, ArrayList<Integer>> route = theMap.dijkstra(start, goal);
		final long duration = System.nanoTime() - startTime;
		double seconds = (double) duration / 1000000000.0;
		System.out.println("Duration: " + seconds + " seconds"); // duration / 1000000000
		System.out.println();
		System.out.println();
		*/
		
		
		// Get Communities
		final long startTime = System.nanoTime();
		List<Graph> communities = theMap.getCommunities();
		final long duration = System.nanoTime() - startTime;
		System.out.println();
		System.out.println("Communities Size: " + communities.size());
		for(Graph community : communities) {
			HashMap<Integer, HashSet<Integer>> exportNextCommunity = community.exportGraph();
			Set<Integer> graphNodes = exportNextCommunity.keySet();
			// System.out.print(graphNodes + " : ");
			for(Integer node : graphNodes) {
				System.out.print(node + " ");
			}
			System.out.println();
		}
		double seconds = (double) duration / 1000000000.0;
		System.out.println("Duration: " + seconds + " seconds"); // duration / 1000000000
		
	}
}
