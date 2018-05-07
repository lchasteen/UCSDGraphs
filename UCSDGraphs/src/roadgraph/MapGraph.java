/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
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
	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private final HashMap<GeographicPoint,MapNode> pointNodeMap;
	private final HashSet<MapEdge> edges;


	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		pointNodeMap = new HashMap<>();
		edges = new HashSet<>();
	}

	/**
	 * Returns a {@link List} of {@link GeographicPoint}(s) from start to end based
	 * on the {@code parentMap}. Precondition: {@code parentMap} must be ordered from
	 * the search with the child vertex as the key and the parent vertex
	 * is the value. 
	 * @param parentMap - {@link Map} of {@link MapNode}(s) from the search.
	 * @param start - {@link MapNode} starting point.
	 * @param goal - {@link MapNode} ending point.
	 * @return {@link List} of {@link GeographicPoint}(s) from start to end based
	 * on the {@code parentMap} or an empty list.
	 */
	private List<GeographicPoint> reconstructPath(
			Map<MapNode,MapNode> parentMap,
			MapNode start, 
			MapNode goal) {

		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}

	//	private Map<MapNode, Double> initializeAllToInfinity() {
	//		Map<MapNode,Double> distances = new HashMap<>();
	//
	//		Iterator<MapNode> iter = pointNodeMap.values().iterator();
	//		while (iter.hasNext()) {
	//			MapNode node = iter.next();
	//			distances.put(node, Double.POSITIVE_INFINITY);
	//		}
	//		return distances;
	//	}

	/** 
	 * Get a set of neighbor nodes from a mapNode
	 * @param node  The node to get the neighbors from
	 * @return A set containing the MapNode objects that are the neighbors 
	 * 	of node
	 */
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}

	@Override
	public int getNumVertices()	{
		return pointNodeMap.values().size();
	}

	@Override
	public Set<GeographicPoint> getVertices() {
		return pointNodeMap.keySet();
	}

	@Override
	public int getNumEdges() {
		return edges.size();
	}

	@Override
	public boolean addVertex(GeographicPoint location) {
		if (location == null) {
			return false;
		}
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
			return true;
		}

		return false;
	}

	@Override
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		MapNode n1 = pointNodeMap.get(from);
		MapNode n2 = pointNodeMap.get(to);


		// check nodes are valid
		if (from == null || to == null) {
			throw new IllegalArgumentException("Unable to add edge to graph with a null starting or ending point.");
		}
		if (length < 0) {
			throw new IllegalArgumentException("Edge length is invalid. Must be greater than or equal to zero.");
		}
		if (roadName == null || roadType == null) {
			throw new IllegalArgumentException("Road name and or road type must not be null.");
		}

		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);

	}	

	@Override
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	@Override
	public List<GeographicPoint> bfs(
			GeographicPoint start, 
			GeographicPoint goal, 
			Consumer<GeographicPoint> nodeSearched) {
		/* Note that this method is a little long and we might think
		 * about refactoring it to break it into shorter methods as we 
		 * did in the Maze search code in week 2 */

		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.remove();

			// hook for visualization
			nodeSearched.accept(next.getLocation());

			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);
		return path;
	}

	@Override
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return dijkstra(start, goal, temp);
	}

	@Override
	public List<GeographicPoint> dijkstra(
			GeographicPoint start, 
			GeographicPoint goal, 
			Consumer<GeographicPoint> nodeSearched)	{
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);		

		Queue<MapNode> pq = new PriorityQueue<>(10, new Comparator<MapNode>() {
			public int compare(MapNode x, MapNode y) {
				return Double.compare(x.getStartDistance(), y.getStartDistance());
			};
		});


		Map<MapNode, Double> nodeDistances = new HashMap<>();
		// Initialize the distance map with infinity values.
		for (MapNode n : pointNodeMap.values()) {
			nodeDistances.put(n, Double.POSITIVE_INFINITY);
		}
		Map<MapNode,MapNode> parentMap = new HashMap<>();
		Set<MapNode> visited = new HashSet<>();

		// Add the first element
		startNode.setStartDistance(0d);
		nodeDistances.put(startNode, Double.valueOf(0d));
		pq.add(startNode);
		MapNode curr = null;
		int accum = 1;
		// Keep looping until empty.
		while (!pq.isEmpty()) {
			// Remove from queue.
			curr = pq.remove();
			System.out.println("------------dijkstra [" + accum + "}" + curr.toString());
			accum++;
			// Only search non-visited nodes
			if (!visited.contains(curr)){
				visited.add(curr);
				nodeSearched.accept(curr.getLocation());
				// Are we done?
				if (curr.equals(endNode)) {
					return reconstructPath(parentMap, startNode, endNode);
				}
			
				for (MapNode neighbor : getNeighbors(curr)) {
					if (!visited.contains(neighbor) ){  

						// Distance of the current node to the neighbor.
						double currentToNeighbor = curr.getLocation().distance(neighbor.getLocation());
						// Total
						double total = curr.getStartDistance() + currentToNeighbor;

						// Check out the parent map for any key matches which equal the next neighbor.
						// Check out the weight of any neighbors which have already been tracked but
						// are not marked as visited. Only replace the weight if the current calculated
						// weight is lower.
						if(total < nodeDistances.get(neighbor)){
							// update distance
							nodeDistances.put(neighbor, total);
							neighbor.setStartDistance(total);
							// set parent
							parentMap.put(neighbor, curr);
							pq.add(neighbor);
						}
					}
				}
			}
		}
		return null;
	}
	
	/**
	 * Returns a {@link List} of {@link GeographicPoint}(s) with best route order considering
	 * distances. Implementation of 2 opt algorithm.
	 * @param vertices - {@link List} of {@link Geographic Point}(s) in route order with identical
	 * start {@link GeographicPoint} as the first element and the start point as the last element.
	 * @return {@link List} of {@link GeographicPoint}(s) with best route order considering
	 * distances.
	 */
	public List<GeographicPoint> twoOpt(List<GeographicPoint> route) {
		
		List<GeographicPoint> vertices = new ArrayList<>(route);
		// Get the current projected distance.
		double totDist = getLength(vertices);
		
		List<GeographicPoint> bestRoute;
		
		double cDist;
		int routeSwap = 1;
		
		while (routeSwap != 0) { //loop until no improvements are made.
			routeSwap = 0;

			// Outer Loop: 1 to size - 2 avoid start and end point
			// Inner Loop: outer loop + 1 start to last vertex - 1.
			for (int i = 1; i < vertices.size() - 2; i++) {
				for (int j = i + 1; j < vertices.size() - 1; j++) {
					double ab = vertices.get(i).distance(vertices.get(i - 1));
					double cd = vertices.get(j + 1).distance(vertices.get(j));
					double ac = vertices.get(i).distance(vertices.get(j + 1));
					double bd = vertices.get(i - 1).distance(vertices.get(j));
					// Check to see if route ac + bd is better than ab + cd
					if ((ab + cd) >= (ac + bd)) {
						// Reverse all from i to j
						bestRoute = swap(i, j, vertices); 
						
						cDist = getLength(bestRoute);
						// Is it better? If so then set the best route.
						if (cDist < totDist) {
							vertices = bestRoute;
							totDist = cDist;
							routeSwap++;
						}
					}
				}
			}
		}
		
		return vertices;
	}

	/**
	 * Swaps the vertices between {@code i} and {@code j}. Returns a {@link List}
	 * of the results.
	 * @param i - Start vertex.
	 * @param j - End vertex.
	 * @param vertices - {@link List} of {@link GeographicPoint}(s) to modify.
	 * @return {@link List} of vertices with values swaped from i to j.
	 */
	private static List<GeographicPoint> swap( int i, int j, List<GeographicPoint> vertices) {
		// Invert the order from i to j.
		List<GeographicPoint> resultList = new ArrayList<>();

		//**************** Start
		// Add from start to i to the result list.
		int size = vertices.size();
		for (int start = 0; start <= i - 1; start++) {
			resultList.add(vertices.get(start));
		}
		
		//**************** Middle
		// Invert
		int counter = 0;
		for (int mid = i; mid <= j; mid++) {
			// reverse order j - counter
			resultList.add(vertices.get(j - counter));
			counter++;
		}

		//**************** End
		// Add from j + 1 to the end
		for (int end = j + 1; end < size; end++) {
			resultList.add(vertices.get(end));
		}

		return resultList;
	}

	/**
	 * Returns the projected total distance in Cartesian space from each vertex.
	 * @param vertices - {@link GeographicPoint} vertex.
	 * @return The projected total distance.
	 */
	private double getLength(List<GeographicPoint> vertices) {
		double result = 0;
		// Track the distance from last to first.
		// Get the last point as the previous.
		GeographicPoint p = vertices.get(vertices.size() - 1);
		for (GeographicPoint gp : vertices) {
			// distance of current to previous.
			result += gp.distance(p);
			p = gp;
		}
		return result;
	}

	@Override
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {};
		return aStarSearch(start, goal, temp);
	}

	@Override
	public List<GeographicPoint> aStarSearch(
			GeographicPoint start, 
			GeographicPoint goal, 
			Consumer<GeographicPoint> nodeSearched) {

		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);		

		Queue<MapNode> pq = new PriorityQueue<>(10, new Comparator<MapNode>() {
			public int compare(MapNode x, MapNode y) {
				return Double.compare(x.getStartDistance(), y.getStartDistance());
			};
		});


		Map<MapNode, Double> nodeDistances = new HashMap<>();
		// Initialize the distance map with infinity values.
		for (MapNode n : pointNodeMap.values()) {
			nodeDistances.put(n, Double.POSITIVE_INFINITY);
		}
		Map<MapNode,MapNode> parentMap = new HashMap<>();
		Set<MapNode> visited = new HashSet<>();

		// Add the first element
		startNode.setStartDistance(0d);
		nodeDistances.put(startNode, Double.valueOf(0d));
		pq.add(startNode);
		MapNode curr = null;
		int accum = 1;
		// Keep looping until empty.
		while (!pq.isEmpty()) {
			// Remove from queue.
			curr = pq.remove();
			System.out.println("------------A* [" + accum + "}" + curr.toString());
			accum++;
			// Only search non-visited nodes
			if (!visited.contains(curr)){
				visited.add(curr);
				nodeSearched.accept(curr.getLocation());
				// Are we done?
				if (curr.equals(endNode)) {
					return reconstructPath(parentMap, startNode, endNode);
				}
				for (MapNode neighbor :  getNeighbors(curr)) {
					if (!visited.contains(neighbor) ){  

						// Distance of neighbor to the end point.
						double neighborToEnd = neighbor.getLocation().distance(endNode.getLocation());
						// Distance of the current node to the neighbor.
						double currentToNeighbor = curr.getLocation().distance(neighbor.getLocation());
						// Total
						double total = curr.getStartDistance() + currentToNeighbor + neighborToEnd;

						// Check out the parent map for any key matches which equal the next neighbor.
						// Check out the weight of any neighbors which have already been tracked but
						// are not marked as visited. Only replace the weight if the current calculated
						// weight is lower.
						if(total < nodeDistances.get(neighbor)){
							// update distance
							nodeDistances.put(neighbor, total);
							neighbor.setStartDistance(total);
							neighbor.setPossibleDistanceToEnd(neighborToEnd);
							// set parent
							parentMap.put(neighbor, curr);
							pq.add(neighbor);
						}
					}
				}
			}
		}
		return null;
	}

	public static void main(String[] args) {
		//		System.out.print("Making a new map...");
		//		MapGraph theMap = new MapGraph();
		//		System.out.print("DONE. \nLoading the map...");
		//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		//		System.out.println("DONE.");

		// You can use this method for testing.  

		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);


		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		 */


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

	}

}