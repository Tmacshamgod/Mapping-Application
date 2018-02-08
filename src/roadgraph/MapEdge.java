package roadgraph;

import geography.GeographicPoint;

import java.util.List;

/**
 * Created by JeffZhang on 1/15/18.
 */
public class MapEdge {
    private MapNode start;
    private MapNode end;
    private String roadName;
    private String roadType;
    private double length;

    public MapEdge(MapNode start, MapNode end, String roadName,
                   String roadType, double length) {
        this.start = start;
        this.end = end;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public MapNode getStartNode() {
        return start;
    }

    public MapNode getEndNode() {
        return end;
    }

    public double getLength() {
        return length;
    }

    public MapNode getOtherNode(MapNode node) {
        if(node == start) {
            return end;
        }
        if(node == end) {
            return start;
        }
        return null;
    }
}
