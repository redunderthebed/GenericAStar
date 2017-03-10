using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace generic_astar
{

    /**
        Simple point class for demo purposes, obviously could be swapped out with Unity vectors
    */
    class Point
    {
        public float X { get; }
        public float Y { get; }

        public Point(float x, float y)
        {
            X = x;
            Y = y;
        }
        public float Distance(Point other)
        {
            return (float)Math.Sqrt(Math.Pow(this.X - other.X, 2) + Math.Pow(this.Y - other.Y, 2));
        }
    }

    /**
       Connects AStar nodes to states
    */
    public abstract class AStarMap
    {
        public abstract IEnumerable<AStarNode> GetNeighbours(AStarNode node);
        public abstract float GetHeuristic(AStarNode node);
    }

    /**
        Compares two AStarNodes by their FScore
    */
    class FScoreComparer : IComparer<AStarNode>
    {
        public int Compare(AStarNode x, AStarNode y)
        {
            return (int)(x.FScore - y.FScore);
        }
    }

    /**
        Empty interface for nav/planning state object.
    */
    public interface IAStarState : ICloneable
    {
        bool EqualState(IAStarState other);
        
    }

    /**
        Translates between nav mesh and AStarEngine
    */
    class NavNode : IAStarState
    {
        static float error = 0.03f;
        public Point Position;
        public Dictionary<NavNode, float> Edges;

        public NavNode (Point position)
        {
            Position = position;
            Edges = new Dictionary<NavNode, float>();
        }

        /**
            Checks if two State objects represent the same state
        **/
        public bool EqualState(IAStarState other)
        {
            
            if(other is NavNode)
            {
                return this.Position.Distance(((NavNode)other).Position) < error;
            }
            else
            {
                return false;
            }
        }

        public override int GetHashCode()
        {
            return (int)(Position.X + Position.Y);
        }

        public object Clone()
        {
            return new NavNode(new Point(this.Position.X, this.Position.Y));
        }
    }

    class NavMap : AStarMap
    {
        public List<NavNode> Nodes;

        public NavMap()
        {
            this.Nodes = new List<NavNode>();
        }
        public override float GetHeuristic(AStarNode node)
        {
            if(node.Value is NavNode)
            {
                NavNode current = ((NavNode)node.Value);
                NavNode goal = ((NavNode)node.Goal);
                return current.Position.Distance(goal.Position);
            }
            throw new Exception("You're comparing the wrong types of nodes");
        }

        public override IEnumerable<AStarNode> GetNeighbours(AStarNode node)
        {
            NavNode current = ((NavNode)node.Value);
            foreach(KeyValuePair<NavNode, float> edge in current.Edges)
            {
                //Construct an A*Node based on the edge
                AStarNode neighbour = new AStarNode(edge.Key, node.Goal, edge.Value, this);
                yield return neighbour;
            }
        }
    }


    class AStarEngine
    {
        SortedSet<AStarNode> openSet;
        HashSet<AStarNode> closedSet;

        public AStarEngine()
        {
            openSet = new SortedSet<AStarNode>(new FScoreComparer());
            closedSet = new HashSet<AStarNode>();
        }

        private IEnumerable <IAStarState> reconstructPath(AStarNode goal)
        {
            AStarNode current = goal;
            LinkedList<IAStarState> path = new LinkedList<IAStarState>();
            while(current != null)
            {
                path.AddFirst(current.Value);
                current = current.CameFrom;
            }
            return path;
        }

        public IEnumerable <IAStarState> FindSolution(IAStarState start, IAStarState goal, AStarMap map)
        {
            //It should add to the goal state the preconditions of the action
            closedSet.Clear();
            openSet.Clear();
            AStarNode startNode = new AStarNode(start, goal, 0, map);
            startNode.Goal = goal;
            openSet.Add(startNode);
            while(openSet.Count > 0)
            {
                AStarNode current = openSet.First();
                openSet.Remove(current);
                if (current.Value.EqualState(current.Goal))
                {
                    return reconstructPath(current);
                }
                closedSet.Add(current);
                foreach(AStarNode neighbour in map.GetNeighbours(current))
                {
                    if (closedSet.Contains(neighbour))
                    {
                        continue;
                    }
                    float tentativeGScore = current.GScore + neighbour.Cost;
                    if (openSet.Contains(neighbour) == false)
                    {
                        openSet.Add(neighbour);
                    }
                    else if (tentativeGScore >= neighbour.GScore)
                    {
                        continue;
                    }
                    neighbour.CameFrom = current;
                    neighbour.GScore = tentativeGScore;
                    //FScore updates automatically
                }
            }
            return null;
        }

    }

    public class AStarNode
    {
        public float Cost { get; set; } //Cost of action for this node
        public float Heuristic { get; set; } //Distance of this state to goal
        public float GScore { get; set; } //Path goodness
        public float FScore { get { return GScore + Heuristic; } } //AStar goodness
        public AStarNode CameFrom;
        public IAStarState Goal;
        public IAStarState Value { get; }

        public AStarNode(IAStarState value, IAStarState goal, float cost, AStarMap map)
        {
            this.Cost = cost;
            this.GScore = float.MaxValue;
            this.Heuristic = float.MaxValue;
            this.Value = value;
            this.CameFrom = null;
            this.Goal = goal;
            this.Heuristic = map.GetHeuristic(this);
        }

        public override bool Equals(object obj)
        {
            if(obj is AStarNode)
            {
                AStarNode other = ((AStarNode)obj);
                if(other.Value.GetHashCode() == this.Value.GetHashCode())
                {
                    return this.Value.Equals(other);
                }
                return false;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return Value.GetHashCode();
        }

        public bool SameValue(AStarNode other)
        {
            return Object.ReferenceEquals(this.Value, other.Value);
        }
    }

    

    

    

    class Program
    {
        static void Main(string[] args)
        {
            NavMap map = new NavMap();
            NavNode a = new NavNode(new Point(0, 0));
            NavNode b = new NavNode(new Point(3, 4));
            NavNode c = new NavNode(new Point(0, 4));
            NavNode d = new NavNode(new Point(6, 2));
            NavNode e = new NavNode(new Point(6, 4));
            NavNode f = new NavNode(new Point(2, 1));
            NavNode g = new NavNode(new Point(3, 2));

            a.Edges.Add(c, 4);
            a.Edges.Add(f, 3);
            b.Edges.Add(d, 5);
            b.Edges.Add(g, 2);
            c.Edges.Add(b, 3);
            d.Edges.Add(e, 2);
            f.Edges.Add(g, 2);
            g.Edges.Add(c, 5);

            AStarEngine engine = new AStarEngine();
            IEnumerable<NavNode> plan = engine.FindSolution(a, e, map).Select(x => ((NavNode)x));
            foreach(NavNode navNode in plan)
            {
                Console.Out.WriteLine("({0}, {1})", navNode.Position.X, navNode.Position.Y);
            }
            

            GOAPMap goapMap = new GOAPMap();
            Action buildFire = new Action("build_fire", new List<WorldStateToken>() { new WorldStateToken("has_wood", true) }, new List<WorldStateToken>() { new WorldStateToken("has_fire", true) }, 10);
            Action cookFood = new Action("cook_food", new List<WorldStateToken>() { new WorldStateToken("has_fire", true), new WorldStateToken("has_raw_food", true) }, new List<WorldStateToken>() { new WorldStateToken("has_cooked_food", true) }, 10);
            Action huntFood = new Action("hunt_food", new List<WorldStateToken>() { }, new List<WorldStateToken> {new WorldStateToken("has_raw_food", true) }, 10);
            Action chopWood = new Action("chop_wood", new List<WorldStateToken>() { }, new List<WorldStateToken> { new WorldStateToken("has_wood", true) }, 10);
            goapMap.AddAction(buildFire);
            goapMap.AddAction(cookFood);
            goapMap.AddAction(huntFood);
            goapMap.AddAction(chopWood);
            GOAPNode start = new GOAPNode(new WorldState(), null);
            
            WorldState goalWorldState = new WorldState();
            goalWorldState.SetToken("has_cooked_food", true);
            GOAPNode goal = new GOAPNode(goalWorldState, null);


            

            IEnumerable<GOAPNode> actionPlan = engine.FindSolution(start, goal, goapMap).Select(x => ((GOAPNode)x));
            foreach(GOAPNode node in actionPlan)
            {
                if (node.LastAction != null)
                {
                    Console.Out.WriteLine(node.LastAction.Name);
                }
                else
                {
                    Console.Out.WriteLine("no action");
                }
            }
            Console.ReadKey();
        }
    }
}
