/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 * <br><br> 
 * @author UCSD MOOC development team
 * @author Lane Chasteen
 */
public class MapGraph implements Graph {
	//TODO: Add your member variables here in WEEK 3
	//-- properties --//
	private final Map<Node, Set<Edge>> graph;
	private int numVertices;
	private int numEdges;


	//-- constructors --//
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {	
		graph = new HashMap<>();
		numVertices = 0;
		numEdges = 0;
	}
	
	//-- MapGraph methods --//
	/**
	 * Returns a copy {@link Set} of {@link Node}(s) which represent the neighbors of the {@code vertex}.
	 * @param vertex - {@link Node} to get the neighbors of.
	 * @return {@link Set} of {@link Node}(s) which represent the neighbors of the {@code vertex}.
	 */
	private Set<Node> getNeighbors(Node vertex) {
		Set<Node> result = new HashSet<>();
		for (Edge e : graph.get(vertex)) {
			result.add(e.getVertex().getNode());
		}
		return result;
	}
	
	/**
	 * Returns the actual {@link Set} of {@link Edge}(s) which represent the neighbors of the {@code vertex}.
	 * @param vertex - {@link Node} to get the neighbors of.
	 * @return {@link Set} of {@link Edge}(s) which represent the neighbors of the {@code vertex}.
	 */
	private Set<Edge> getEdgesNotVisited(Node vertex) {
		return graph.get(vertex);
	}
	
	/**
	 * Returns a {@link List} of {@link GeographicPoint}(s) from start to end based
	 * on the {@code parentMap}. Precondition: {@code parentMap} must be ordered from
	 * the breath-first search with the child vertex as the key and the parent vertex
	 * is the value. 
	 * @param parentMap - {@link Map} of {@link Node}(s) from the BFS.
	 * @param start - {@link Node} starting point.
	 * @param goal - {@link Node} ending point.
	 * @return {@link List} of {@link GeographicPoint}(s) from start to end based
	 * on the {@code parentMap} or an empty list.
	 */
	private List<GeographicPoint> getPathFrom(
			Map<Node, Node> parentMap, 
			Node start, 
			Node goal) {
		List<GeographicPoint> result = new ArrayList<>();
		// Start in reverse.
		Node curr = goal;
		do {
			result.add(curr.getGeographicPoint());
		} 
		while ((curr = parentMap.get(curr)) != null);
		// add the start point
		
		if (!result.isEmpty()) {
			// Reverse so start is first and goal is last.
			Collections.reverse(result);
		}
		return result;
	}

	//-- Graph methods --//
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	@Override
	public int getNumVertices() {	
		return numVertices;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	@Override
	public Set<GeographicPoint> getVertices() {
		Set<GeographicPoint> vertices = new HashSet<>();
		for (Node n : graph.keySet()) {
			vertices.add(n.getGeographicPoint());
		}
		return vertices;
	}

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	@Override
	public int getNumEdges() {
		return numEdges;
	}

	/** 
	 * Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	@Override
	public boolean addVertex(GeographicPoint location) {
		if (location == null) {
			return false;
		}
		// Vertex nodes must be unique in the graph.
		Node n = new Node(location);
		if (graph.containsKey(n)) {
			return false;
		}
		graph.put(n, new HashSet<>());
		numVertices++;
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The lengtimport roadgraph.Node.PriorityNode;
h of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	@Override
	public void addEdge(GeographicPoint from, 
			GeographicPoint to, 
			String roadName,
			String roadType, 
			double length) throws IllegalArgumentException {

		if (from == null || to == null) {
			throw new IllegalArgumentException("Unable to add edge to graph with a null starting or ending point.");
		}
		if (length < 0) {
			throw new IllegalArgumentException("Edge length is invalid. Must be greater than or equal to zero.");
		}
		if (roadName == null || roadType == null) {
			throw new IllegalArgumentException("Road name and or road type must not be null.");
		}
		Node nFrom = new Node(from);
		Node nTo = new Node(to);
		if (!this.graph.containsKey(nFrom) || !this.graph.containsKey(nTo)) {
			throw new IllegalArgumentException("Unable to find starting or ending refrence point.");
		}
		
		Edge e = new Edge(new PriorityNode(nTo, 0, length), roadName, roadType);
		
		graph.get(nFrom).add(e);
		numEdges++;
	}

	/** 
	 * Find the path from start to goal using breadth first search
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	@Override
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	/** 
	 * Find the path from start to goal using breadth first search
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal) or {@code null}
	 *   if no path is found.
	 */
	@Override
	public List<GeographicPoint> bfs(
			GeographicPoint start, 
			GeographicPoint goal, 
			Consumer<GeographicPoint> nodeSearched) {
		
		if (start == null || goal == null) {
			return null;
		}
		Node nStart = new Node(start);
		Node nGoal = new Node(goal);
		Set<Node> visited = new HashSet<>();
		Map<Node, Node> parentMap = new HashMap<>();
		Queue<Node> pointQueue = new LinkedList<>();
		// Add the first element
		pointQueue.add(nStart);
		visited.add(nStart);
		// Loop until empty.
		while (!pointQueue.isEmpty()) {
			Node curr = pointQueue.remove();
			if (curr.equals(nGoal)) {
				return getPathFrom(parentMap, nStart, nGoal);
			}
			nodeSearched.accept(curr.getGeographicPoint());
			for (Node n : getNeighbors(curr)) {
				if (n == null || visited.contains(n)) {
					continue;
				}
				visited.add(n);
				parentMap.put(n, curr);
				pointQueue.add(n);
			}	
		}
		

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return null;
	}

	/** 
	 * Find the path from start to goal using Dijkstra's algorithm
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	@Override
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return dijkstra(start, goal, temp);
	}

	/** 
	 * Find the path from start to goal using Dijkstra's algorithm
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	@Override
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			return null;
		}
		Node nStart = new Node(start);
		Node nGoal = new Node(goal);
		Set<Node> visited = new HashSet<>();
		Map<Node, Node> parentMap = new HashMap<>();
		PriorityQueue<PriorityNode> pointQueue = new PriorityQueue<PriorityNode>(10, (o1, o2) -> {
			//return o1 == null ? (o2 == null ? 0 : -1) : (o2 == null ? 1 : Double.compare(o1.getWeight(), o2.getWeight()));
			return Double.compare(o1.getWeight(), o2.getWeight());
		});

		// Add the first element
		pointQueue.add(new PriorityNode(nStart,0, 0));
		
		// Loop until empty.
		while (!pointQueue.isEmpty()) {
			PriorityNode pn = pointQueue.remove();
			System.out.println("---- From queue: " + pn.getNode() + " weight: " + pn.getWeight());
			Node curr = pn.getNode();
			if (!visited.contains(curr)) {
				visited.add(curr);
				if (curr.equals(nGoal)) {
					return getPathFrom(parentMap, nStart, nGoal);
				}
				nodeSearched.accept(curr.getGeographicPoint());
				for (Edge n : graph.get(curr)) {
					PriorityNode en = n.getVertex();
					if (visited.contains(en.getNode())) {
						continue;
					}
					//System.out.println("Current length to next: " + Node.distance(pn.getNode(), en.getNode()) + " node N distance: " + en.getLength());
//					if (en == null) {
//						continue;
//					}
					
					double v = pn.getWeight() + en.getLength();
					System.out.println(en.getNode() + " pn.getWeight(): " + pn.getWeight() + " v: " + v + " en.getWeight(): " + en.getWeight());
					if (v < en.getWeight() || en.getWeight() == 0) {
						en.setWeight(v);
						parentMap.put(en.getNode(), curr);
						pointQueue.add(en);
					}
					else {
						System.out.println("Otherwise: " + en.getWeight());
					}
				}
			}
		}
		

		return null;
	}

	/** 
	 * Find the path from start to goal using A-Star search
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	@Override
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return aStarSearch(start, goal, temp);
	}

	/** 
	 * Find the path from start to goal using A-Star search
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	@Override
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return null;
	}


	//-- Main method --//
	  /** Compare the user's result with the right answer.
     * @param i The graph number
     * @param result The user's graph
     * @param corr The correct answer
     * @param start The point to start from
     * @param end The point to end at
     */
    public static String judge(int i, MapGraph result, CorrectAnswer corr, GeographicPoint start, GeographicPoint end) {
        // Correct if paths are same length and have the same elements
        String feedback = DijkstraGrader.appendFeedback(i, "Running Dijkstra's algorithm from (" + start.getX() + ", " + start.getY() + ") to (" + end.getX() + ", " + end.getY() + ")");
        int correct = 0;
        List<GeographicPoint> path = result.dijkstra(start, end);
        if (path == null) {
            if (corr.path == null) {
                feedback += "PASSED.";
                correct++;
            } else {
                feedback += "FAILED. Your implementation returned null; expected \n" + DijkstraGrader.printPath(corr.path) + ".";
            }
        } else if (path.size() != corr.path.size() || !corr.path.containsAll(path)) {
            feedback += "FAILED. Expected: \n" + DijkstraGrader.printPath(corr.path) + "Got: \n" + DijkstraGrader.printPath(path);
            if (path.size() != corr.path.size()) {
                feedback += "Your result has size " + path.size() + "; expected " + corr.path.size() + ".";
            } else {
                feedback += "Correct size, but incorrect path.";
            }
        } else {
            feedback += "PASSED.";
            correct++;
        }
        return feedback;
    }
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		MapGraph graph = new MapGraph();



		GraphLoader.loadRoadMap("data/graders/mod3/map3.txt", graph);
		CorrectAnswer corr = new CorrectAnswer("data/graders/mod3/map3.txt.answer", false);

		String result = judge(3, graph, corr, new GeographicPoint(0, 0),new GeographicPoint(0, 4) );
		System.out.println(result);
		// You can use this method for testing.  


		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		 */


		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		 */

	}
}
