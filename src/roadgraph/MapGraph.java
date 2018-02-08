/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private Map<GeographicPoint, MapNode> map;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
	    map = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
	    return map.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return map.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int num = 0;
		for(Map.Entry<GeographicPoint, MapNode> entry: map.entrySet()) {
			num += entry.getValue().getEdgesNumber();
		}
		return num;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
	    if(location == null) {
	    	return false;
		}
		if(map.containsKey(location)) {
	    	return false;
		}
		map.put(location, new MapNode(location));
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if(!map.containsKey(from) || !map.containsKey(to) || from == null ||
				to == null || roadName == null || roadType == null || length < 0) {
			throw new IllegalArgumentException();
		}
		map.get(from).getEdges().add(new MapEdge(map.get(from), map.get(to), roadName, roadType, length));
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if(start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		Map<MapNode, MapNode> parentMap = new HashMap<>();
		boolean found = bfsSearch(start, goal, parentMap, nodeSearched);

		if(!found) {
			System.out.println("No path exists");
			return null;
		}

		return constructPath(start, goal, parentMap);

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}

	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
							  Map<MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		Set<MapNode> visited = new HashSet<>();
		Queue<MapNode> toExplore = new LinkedList<>();
		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		visited.add(startNode);
		toExplore.add(startNode);
		boolean found = false;
		while(!toExplore.isEmpty()) {
			MapNode cur = toExplore.remove();
			nodeSearched.accept(cur.getLocation());
			if(cur == goalNode) {
				found = true;
				break;
			}
			List<MapNode> neighbors = cur.getNeighbors();
			for(MapNode neighborNode: neighbors) {
				if(!visited.contains(neighborNode)) {
					visited.add(neighborNode);
					parentMap.put(neighborNode, cur);
					toExplore.add(neighborNode);
				}
			}
		}
		return found;
	}

	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
												Map<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<>();
		MapNode startNode = map.get(start);
		MapNode curNode = map.get(goal);
		while(curNode != startNode) {
			path.addFirst(curNode.getLocation());
			curNode = parentMap.get(curNode);
		}
		path.addFirst(startNode.getLocation());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
        if(start == null || goal == null) {
            System.out.println("Start or goal node is null!  No path exists.");
            return null;
        }
        Map<MapNode, MapNode> parentMap = new HashMap<>();
        boolean found = dijkstraSearch(start, goal, parentMap, nodeSearched);

        if(!found) {
            System.out.println("No path exists");
            return null;
        }

        return constructPath(start, goal, parentMap);
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}

	private boolean dijkstraSearch(GeographicPoint start, GeographicPoint goal,
                                   Map<MapNode, MapNode> parentmap,
                                   Consumer<GeographicPoint> nodeSearched) {
        reset();
        PriorityQueue<MapNode> toExplore = new PriorityQueue<>(new DijkstraComparator());
        return shortestSearch(start, goal, parentmap, toExplore, nodeSearched);
    }

    private boolean shortestSearch(GeographicPoint start, GeographicPoint goal, Map<MapNode, MapNode> parentMap,
                                   PriorityQueue<MapNode> toExplore, Consumer<GeographicPoint> nodeSearched) {
	    Set<MapNode> visited = new HashSet<>();
	    MapNode startNode = map.get(start);
	    startNode.setShortDistance(0);
	    MapNode goalNode = map.get(goal);
	    toExplore.offer(startNode);
	    boolean found = false;
	    while(!toExplore.isEmpty()) {
	        MapNode currentNode = toExplore.poll();
	        nodeSearched.accept(currentNode.getLocation());
	        if(!visited.contains(currentNode)) {
	            visited.add(currentNode);
	            if(currentNode == goalNode) {
	                found = true;
	                break;
                }
                List<MapNode> neighbors = currentNode.getNeighbors();
	            for(MapNode neighborNode: neighbors) {
	                if(!visited.contains(neighborNode)) {
	                    double newShortDistance = currentNode.getShortDistance() +
                                currentNode.getDistance(neighborNode);
                        if(newShortDistance < neighborNode.getShortDistance()) {
                            neighborNode.setShortDistance(newShortDistance);
                            parentMap.put(neighborNode, currentNode);
                            toExplore.offer(neighborNode);
                        }
                    }
                }
            }
        }
	    return found;
    }

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
        if(start == null || goal == null) {
            System.out.println("Start or goal node is null!  No path exists.");
            return null;
        }
        Map<MapNode, MapNode> parentMap = new HashMap<>();
        boolean found = aStarSearch(start, goal, parentMap, nodeSearched);

        if(!found) {
            System.out.println("No path exists");
            return null;
        }

        return constructPath(start, goal, parentMap);
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}

	private boolean aStarSearch(GeographicPoint start, GeographicPoint goal,
                                Map<MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
        reset();
	    PriorityQueue<MapNode> toExplore = new PriorityQueue<>(new AStarComparator(map.get(goal)));
	    return shortestSearch(start, goal, parentMap, toExplore, nodeSearched);
    }

	private void reset() {
	    for(MapNode node: map.values()) {
	        node.setShortDistance(Double.POSITIVE_INFINITY);
        }
    }


	public static void main(String[] args)
	{
		MapGraph firstMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
        System.out.println("bfs: " + firstMap.bfs(testStart, testEnd).size());
		System.out.println("dijkstra: " + firstMap.dijkstra(testStart, testEnd).size());
		System.out.println("astar: " + firstMap.aStarSearch(testStart, testEnd).size());
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		testStart = new GeographicPoint(1.0, 1.0);
		testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
        System.out.println("bfs: " + simpleTestMap.bfs(testStart,testEnd).size());
        System.out.println("dijkstra: " + simpleTestMap.dijkstra(testStart,testEnd).size());
        System.out.println("astar: " + simpleTestMap.aStarSearch(testStart,testEnd).size());


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data
        testStart = new GeographicPoint(32.869423, -117.220917);
        testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
        System.out.println("bfs: " + testMap.bfs(testStart,testEnd).size());
        System.out.println("dijkstra: " + testMap.dijkstra(testStart,testEnd).size());
        System.out.println("astar: " + testMap.aStarSearch(testStart,testEnd).size());


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
        System.out.println("bfs: " + testMap.bfs(testStart,testEnd).size());
		System.out.println("dijkstra: " + testMap.dijkstra(testStart,testEnd).size());
		System.out.println("astar: " + testMap.aStarSearch(testStart,testEnd).size());


		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);

		testStart = new GeographicPoint(32.8648772, -117.2254046);
		testEnd = new GeographicPoint(32.8660691, -117.217393);

        System.out.println("bfs: " + theMap.bfs(testStart,testEnd).size());
		System.out.println("dijkstra: " + theMap.dijkstra(testStart,testEnd).size());
		System.out.println("astar: " + theMap.aStarSearch(testStart,testEnd).size());
	}
	
}
