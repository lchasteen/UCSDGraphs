/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;


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
				if (x.getStartDistance() < y.getStartDistance()) {
					return -1;              
				}               
				if (x.getStartDistance() > y.getStartDistance()) {
					return 1;
				}
				return 0;
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
		
		// Keep looping until empty.
		while (!pq.isEmpty()) {
			// Remove from queue.
			curr = pq.remove();
			System.out.println(curr.toString());
			// Only search non-visited nodes
			if (!visited.contains(curr)){
				visited.add(curr);
				nodeSearched.accept(curr.getLocation());
				// Are we done?
				if (curr.equals(endNode)) {
					return reconstructPath(parentMap, startNode, endNode);
				}

				Set<MapNode> neighbors = getNeighbors(curr);
				for (MapNode neighbor : neighbors) {
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
							// enqueue
							pq.add(neighbor);
						}
					}
				}
			}
		}
		return null;
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
				if (x.getStartDistance() < y.getStartDistance()) {
					return -1;              
				}               
				if (x.getStartDistance() > y.getStartDistance()) {
					return 1;
				}
				return 0;
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
		
		// Keep looping until empty.
		while (!pq.isEmpty()) {
			// Remove from queue.
			curr = pq.remove();
			System.out.println(curr.toString());
			// Only search non-visited nodes
			if (!visited.contains(curr)){
				visited.add(curr);
				nodeSearched.accept(curr.getLocation());
				// Are we done?
				if (curr.equals(endNode)) {
					return reconstructPath(parentMap, startNode, endNode);
				}

				Set<MapNode> neighbors = getNeighbors(curr);
				for (MapNode neighbor : neighbors) {
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
							// enqueue
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

	}

}