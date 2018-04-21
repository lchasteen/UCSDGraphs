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
	private final Map<GeographicPoint, Set<Edge>> graph;
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
	 * Returns a {@link Set} of {@link GeographicPoint}(s) which represent the neighbors of the {@code vertex}.
	 * @param vertex - {@link GeographicPoint} to get the neighbors of.
	 * @return {@link Set} of {@link GeographicPoint}(s) which represent the neighbors of the {@code vertex}.
	 */
	private Set<GeographicPoint> getNeighbors(GeographicPoint vertex) {
		Set<GeographicPoint> result = new HashSet<>();
		for (Edge e : graph.get(vertex)) {
			result.add(e.getVertex());
		}
		return result;
	}
	
	/**
	 * Returns a {@link List} of {@link GeographicPoint}(s) from start to end based
	 * on the {@code parentMap}. Precondition: {@code parentMap} must be ordered from
	 * the breath-first search with the child vertex as the key and the parent vertex
	 * is the value. 
	 * @param parentMap - {@link Map} of {@link GeographicPoints(s) from the BFS.
	 * @param start - {@link GeographicPoint} starting point.
	 * @param goal - {@link GeographicPoint} ending point.
	 * @return {@link List} of {@link GeographicPoint}(s) from start to end based
	 * on the {@code parentMap} or an empty list.
	 */
	private List<GeographicPoint> getPathFrom(
			Map<GeographicPoint, GeographicPoint> parentMap, 
			GeographicPoint start, 
			GeographicPoint goal) {
		
		List<GeographicPoint> result = new ArrayList<>();
		// Start in reverse.
		GeographicPoint curr = goal;
		do {
			result.add(curr);
		} 
		while ((curr = parentMap.get(curr)) != null); //&& curr != start);
		// add the start point
		
		// Reverse so start is first and goal is last.
		Collections.reverse(result);
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
		vertices.addAll(graph.keySet());
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
		if (location == null || graph.containsKey(location)) {
			return false;
		}
		graph.put(location, new HashSet<>());
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
	 * @param length The length of the road, in km
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
		if (!this.graph.containsKey(from) || !this.graph.containsKey(to)) {
			throw new IllegalArgumentException("Unable to find starting or ending refrence point.");
		}
		if (length < 0) {
			throw new IllegalArgumentException("Edge length is invalid. Must be greater than or equal to zero.");
		}
		if (roadName == null || roadType == null) {
			throw new IllegalArgumentException("Road name and or road type must not be null.");
		}
		
		Edge e = new Edge(to, roadName, roadType, length);
		this.graph.get(from).add(e);
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
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		Queue<GeographicPoint> pointQueue = new LinkedList<>();
		// Add the first element
		pointQueue.add(start);
		visited.add(start);
		// Loop until empty.
		while (!pointQueue.isEmpty()) {
			GeographicPoint curr = pointQueue.remove();
			if (curr.equals(goal)) {
				return getPathFrom(parentMap, start, goal);
			}
			nodeSearched.accept(curr);
			for (GeographicPoint n : getNeighbors(curr)) {
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
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

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

	/**
	 * Identified an edge in a {@link GeographicPoint} point graph.
	 * Contains a geographic point which is the ending location from
	 * the point of reference which contains this object.
	 * <br><br>
	 * @author Lane Chasteen
	 */
	public final class Edge {

		//-- properties --//
		private final GeographicPoint vertex;
		private final String roadName;
		private final String roadType;
		private final double length;

		//-- constructors --//
		/**
		 * Creates a new Edge.
		 * @param vertex - The ending reference point.
		 * @param roadName - The name of the road.
		 * @param roadType - The road type.
		 * @param length - The length from the starting point to the internal vertex.
		 * @throws IllegalArgumentException if vertex, roadName, or roadType is {@code null}.
		 */
		public Edge(GeographicPoint vertex, String roadName, String roadType, double length) throws IllegalArgumentException {
			if (vertex == null) {
				throw new IllegalArgumentException("Unable to create a new " 
						+ Edge.class.getSimpleName() 
						+ " with a null " 
						+ GeographicPoint.class.getSimpleName());
			}
			if (roadName == null || roadType == null) {
				throw new IllegalArgumentException("Unable to create a new " 
						+ Edge.class.getSimpleName() 
						+ " with a null road name or road type.");
			}
			this.vertex = vertex;
			this.roadName = roadName;
			this.roadType = roadType;
			this.length = length;
		}

		//-- Edge methods --//
		/**
		 * Returns the {@link GeographicPoint} as the vertex reference.
		 * @return {@link GeographicPoint} as the vertex reference.
		 */
		public GeographicPoint getVertex() {
			return vertex;
		}

		/**
		 * Returns the road name.
		 * @return The road name.
		 */
		public String getRoadName() {
			return roadName;
		}

		/**
		 * Returns the road type.
		 * @return The road type.
		 */
		public String getRoadType() {
			return roadType;
		}

		/**
		 * Returns the distance in KM between the {@code vertex} and
		 * the parent.
		 * @return The distance in KM between the {@code vertex} and 
		 * the parent.
		 */
		public double getLength() {
			return length;
		}

		//-- Object methods --//
		@Override
		public boolean equals(Object o) {
			if (o == this) {
				return true;
			}
			if (!(o instanceof Edge)) {
				return false;
			}
			Edge e = (Edge) o;

			return e.vertex.equals(vertex)
					&& e.roadName.equals(roadName) 
					&& e.roadType.equals(roadType);
		}

		@Override
		public int hashCode() {
			int result = 37;
			result = 31 * result + vertex.hashCode();
			result = 31 * result + roadName.hashCode();
			result = 31 * result + roadType.hashCode();
			return result;
		}
	}
}
