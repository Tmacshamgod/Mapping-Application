package roadgraph;

import java.util.Comparator;

/**
 * Created by JeffZhang on 1/17/18.
 */
public class AStarComparator implements Comparator<MapNode> {
    private MapNode destination;

    public AStarComparator(MapNode destination) {
        this.destination = destination;
    }

    @Override
    public int compare(MapNode node1, MapNode node2) {
        return Double.compare(node1.getShortDistance() + node1.getDistance(destination),
                node2.getShortDistance() + node2.getDistance(destination));
    }
}