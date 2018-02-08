package roadgraph;

import geography.GeographicPoint;

import java.util.*;

/**
 * Created by JeffZhang on 1/15/18.
 */
public class MapNode {
    private GeographicPoint location;
    private Set<MapEdge> edges;
    private double shortDistance = Double.POSITIVE_INFINITY;

    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new HashSet<MapEdge>();
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public Set<MapEdge> getEdges() {
        return edges;
    }

    public int getEdgesNumber() {
        return edges.size();
    }

    public double getShortDistance() {
        return shortDistance;
    }

    public void setShortDistance(double shortDistance) {
        this.shortDistance = shortDistance;
    }

    public List<MapNode> getNeighbors() {
        List<MapNode> neighbors = new LinkedList<>();
        for(MapEdge edge: edges) {
            neighbors.add(edge.getOtherNode(this));
        }
        return neighbors;
    }

    public double getDistance(MapNode other){
        return this.location.distance(other.getLocation());
//        for(MapEdge edge: edges) {
//            if(edge.getStartNode() == this && edge.getEndNode() == other) {
//                return edge.getLength();
//            }
//        }
//        return Double.POSITIVE_INFINITY;
    }

    public int hashCode() {
        return location.hashCode();
//        final int prime = 31;
//        int result = 1;
//        result = prime * result + ((id == null) ? 0 : id.hashCode());
//        return result;
    }

    public boolean equals(Object object) {
        if(!(object instanceof MapNode) || object == null) {
            return false;
        }
        return ((MapNode) object).location.equals(location);
    }
}
