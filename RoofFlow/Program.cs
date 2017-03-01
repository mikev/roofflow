using QuickGraph;
using QuickGraph.Algorithms.ShortestPath;
using QuickGraph.Algorithms.Observers;
using QuickGraph.Algorithms.TopologicalSort;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.Contracts;
using Newtonsoft.Json;

namespace Roofflow
{
    public enum TaskType
    {
        Assign,
        Accept,
        Choose,
        Sign,
        Wire,
        Fund,
        CloseOut
    }

    public enum NodeType
    {
        Start,
        End,
        Link,
        PassFail,
        AllPass
    }

    public static class Extensions
    {

        private static Random rng = new Random();

        public static void Shuffle<T>(this IList<T> list)
        {
            int n = list.Count;
            while (n > 1)
            {
                n--;
                int k = rng.Next(n + 1);
                T value = list[k];
                list[k] = list[n];
                list[n] = value;
            }
        }
    }

    public enum NodeState
    {
        Init,
        Pass,
        Fail
    }

    public class Vertex
    {
        public string Name { get; set; }
        public NodeType Type { get; set; }
        public NodeState State { get; set; }

        public Vertex(string name, NodeType type)
        {
            this.Name = name;
            this.Type = type;
            this.State = NodeState.Init;
        }

        public override string ToString()
        {
            // return this.name + "->" + this.type;
            return this.Name;
        }

    }

    // 'a1': { 'Id':'a1', 'Src': 'A', 'Dst':'B', 'TaskType':'Assign' },
    public class Task
    {
        public string Id { get; set; }
        public string Src { get; set; }
        public string Dst { get; set; }
        public int Est { get; set; }
        public TaskType Type { get; set; }
    }

    public class Workflow
    {
        public class WorkflowJson
        {
            public IDictionary<string, Vertex> NodeList { get; set; }
            public IDictionary<string, Task> TaskList { get; set; }
        }

        private BidirectionalGraph<Vertex, Edge<Vertex>> graph;
        public Dictionary<Edge<Vertex>, string> edgeList;
        public Dictionary<string, Edge<Vertex>> edgeMap;
        public Dictionary<string, Vertex> nodeMap;

        public Dictionary<Edge<Vertex>, double> edgeCostList;

        public Dictionary<string, bool> edgeState;
        public Dictionary<string, bool> nodeState;

        private IList<Vertex> sortedVertexList;

        public Workflow()
        {
            this.graph = new BidirectionalGraph<Vertex, Edge<Vertex>>(true);
            this.edgeList = new Dictionary<Edge<Vertex>, string>();
            this.edgeMap = new Dictionary<string, Edge<Vertex>>();
            this.nodeMap = new Dictionary<string, Vertex>();

            // Define some weights to the edges
            this.edgeCostList = new Dictionary<Edge<Vertex>, double>();

            this.edgeState = new Dictionary<string, bool>();
            this.nodeState = new Dictionary<string, bool>();

            string json = @"{
                'NodeList': {
                    'A': { 'Name':'A', 'NodeType':'Pass' },
                    'B': { 'Name':'B', 'NodeType':'Pass' },
                    'C': { 'Name':'C', 'NodeType':'Pass' },
                    'D': { 'Name':'D', 'NodeType':'Pass' },
                    'E': { 'Name':'E', 'NodeType':'Pass' },
                    'Z': { 'Name':'Z', 'NodeType':'Pass' }
                },
                'TaskList': {
                    'a1': { 'Id':'a1', 'Src': 'A', 'Dst':'B', 'TaskType':'Assign', 'Est':24 },
                    'b1': { 'Id':'b1', 'Src': 'B', 'Dst':'C', 'TaskType':'Choose', 'Est':4 },
                    'b2': { 'Id':'b2', 'Src': 'B', 'Dst':'C', 'TaskType':'Accept', 'Est':24 },
                    'c1': { 'Id':'c1', 'Src': 'C', 'Dst':'D', 'TaskType':'Approve', 'Est':24 },
                    'c2': { 'Id':'c2', 'Src': 'C', 'Dst':'D', 'TaskType':'Wire', 'Est':48 },
                    'c3': { 'Id':'c3', 'Src': 'C', 'Dst':'D', 'TaskType':'Sign', 'Est':1 },
                    'd1': { 'Id':'d1', 'Src': 'D', 'Dst':'E', 'TaskType':'CloseOut', 'Est':24 },
                    'd2': { 'Id':'d2', 'Src': 'D', 'Dst':'E', 'TaskType':'Sign', 'Est':24 },
                    'd3': { 'Id':'d3', 'Src': 'D', 'Dst':'Z', 'TaskType':'Sign', 'Est':24 },
                    'e1': { 'Id':'e1', 'Src': 'E', 'Dst':'Z', 'TaskType':'Sign', 'Est':24 }
                }
            }";

            WorkflowJson wf = JsonConvert.DeserializeObject<WorkflowJson>(json);

            foreach (KeyValuePair<string, Vertex> kvp in wf.NodeList)
            {
                Vertex v = kvp.Value;
                AddVertex(v);
            }

            foreach (KeyValuePair<string, Task> kvp in wf.TaskList)
            {
                Task t = kvp.Value;
                AddTask(t);
            }

            // Add some vertices to the graph
            //var a = AddVertex("A", NodeType.Start);
            //var b = AddVertex("B", NodeType.Link);
            //var c = AddVertex("C", NodeType.Link);
            //var d = AddVertex("D", NodeType.Link);
            //var e = AddVertex("E", NodeType.Link);
            //var z = AddVertex("Z", NodeType.End);

            // Add the edges
            //AddEdge("a1", a, b);
            //AddEdge("b1", b, c);
            //AddEdge("b2", b, c);
            //AddEdge("c1", c, d);
            //AddEdge("c2", c, d);
            //AddEdge("c3", c, d);
            //AddEdge("d1", d, e);
            //AddEdge("d2", d, e);
            //AddEdge("d3", d, z);
            //AddEdge("e1", e, z);

            // Perform a topological sort of the directed acycle graph
            // This gives us the tasks and nodes in a time and dependent order.
            var sort = new QuickGraph.Algorithms.TopologicalSort.TopologicalSortAlgorithm<Vertex, Edge<Vertex>>(this.graph);
            sort.Compute();
            this.sortedVertexList = sort.SortedVertices;

            // Initialize our starting node
            nodeState["A"] = true;
        }

        public Vertex AddVertex(string name, NodeType type)
        {
            var v = new Vertex(name, type);
            nodeState.Add(name, false);
            nodeMap.Add(name, v);
            graph.AddVertex(v);
            return v;
        }

        public Vertex AddVertex(Vertex v)
        {
            nodeState.Add(v.Name, false);
            nodeMap.Add(v.Name, v);
            graph.AddVertex(v);
            return v;
        }

        public Edge<Vertex> AddEdge(string name, Vertex source, Vertex target)
        {
            var edge = new Edge<Vertex>(source, target);
            graph.AddEdge(edge);
            edgeList.Add(edge, name);
            edgeMap.Add(name, edge);
            edgeState.Add(name, false);
            return edge;
        }

        public void AddTask(Task task)
        {
            var source = nodeMap[task.Src];
            var target = nodeMap[task.Dst];
            var name = task.Id;
            var edge = AddEdge(name, source, target);
            // include the time duration cost in hours.  It is a negative number!
            edgeCostList.Add(edge, 0 - task.Est);
        }

        public BidirectionalGraph<Vertex, Edge<Vertex>> Graph
        {
            get { return this.graph; }
        }

        public IEnumerable<Vertex> GetUnsortedVertices
        {
            get { return this.graph.Vertices; }
        }

        public IEnumerable<Edge<Vertex>> GetUnsortedEdges
        {
            get { return this.graph.Edges; }
        }

        public IEnumerable<Edge<Vertex>> GetInEdges(Vertex node)
        {
            return graph.InEdges(node);
        }

        public void ProcessNode(Vertex node)
        {
            switch (node.State)
            {
                case NodeState.Init:
                    break;
            }
        }

        public void MarkCompleted(string taskName)
        {
            edgeState[taskName] = true;
        }

        public void UpdateEdge(Edge<Vertex> e)
        {
            var node = e.Target;
            UpdateNode(node);
        }

        public void UpdateNode(Vertex node)
        {
            var inEdges = graph.InEdges(node);
            bool allCompleted = true;
            foreach (var e in inEdges)
            {
                var name = this.edgeList[e];
                var state = this.edgeState[name];
                if (state == false)
                {
                    allCompleted = false;
                }
            }
            this.nodeState[node.Name] = allCompleted;
        }

        public void UpdateAllNodes()
        {
            var allNodes = this.GetSortedNodes();
            foreach (var node in allNodes)
            {
                this.UpdateNode(node);
            }
        }

        public IEnumerable<string> GetToDoList()
        {
            var sortedNodes = GetSortedNodes();
            foreach (var node in sortedNodes)
            {
                if (nodeState[node.Name] == false)
                    continue;
                var outTasks = graph.OutEdges(node);
                foreach (var task in outTasks)
                {
                    var name = edgeList[task];
                    if (edgeState[name] == false)
                        yield return name;
                }
            }
        }

        //public void FindPath(string @from = "START", string @to = "END")
        //{
        //    Func<Edge<string>, double> edgeCost = AlgorithmExtensions.GetIndexer(EdgeCost);
        //    // Positive or negative weights
        //    TryFunc<string, System.Collections.Generic.IEnumerable<Edge<string>>> tryGetPath = Graph.ShortestPathsBellmanFord(edgeCost, @from);

        //    IEnumerable<Edge<string>> path;
        //    if (tryGetPath(@to, out path))
        //    {
        //        Console.Write("Path found from {0} to {1}: {0}", @from, @to);
        //        foreach (var e in path) { Console.Write(" > {0}", e.Target); }
        //        Console.WriteLine();
        //    }
        //    else { Console.WriteLine("No path found from {0} to {1}."); }
        //}

        public double GetETA(string start = "A")
        {
            Func<Edge<Vertex>, double> edgeCost = e => this.edgeCostList[e];

            // Bellman algorithm works with either positive or negative weights
            BellmanFordShortestPathAlgorithm<Vertex, Edge<Vertex>> pathCalc = new BellmanFordShortestPathAlgorithm<Vertex, Edge<Vertex>>(this.graph, edgeCost);

            Vertex source = nodeMap[start];
            Vertex target = nodeMap["Z"];
            pathCalc.Compute(source);

            var distances = pathCalc.Distances;

            var distanceToTarget = 0 - distances[target];

            return distanceToTarget;
        }

        public IEnumerable<string> GetSortedEdges()
        {
            foreach (var vertex in this.sortedVertexList)
            {
                var edges = graph.OutEdges(vertex);
                foreach (var e in edges)
                {
                    string s = "null";
                    edgeList.TryGetValue(e, out s);
                    yield return s;
                }
            }
        }

        public IEnumerable<Vertex> GetSortedNodes()
        {
            foreach (var vertex in this.sortedVertexList)
            {
                yield return vertex;
            }
        }

        public string GetDynamicStartNode()
        {
            Vertex start = null;
            foreach (var foo in GetSortedNodes())
            {
                var visited = nodeState[foo.Name];
                if (!visited)
                    break;
                start = foo;
            }
            return start.Name;
        }
        public void PrintAllNodes()
        {
            Console.WriteLine("All Vertices");
            foreach (var node in this.graph.Vertices)
            {
                Console.WriteLine(node);
            }
        }
        public void PrintETA()
        {
            string startName = GetDynamicStartNode();
            double eta = GetETA(startName);
            Console.WriteLine("ETA to Z: {0}", eta);
        }

        public void PrintStartNode()
        {
            Console.Write("Starting Node: ");
            string startName = GetDynamicStartNode();
            Console.WriteLine(startName);
        }

        public void PrintSortedNodes()
        {
            Console.WriteLine("Sorted Nodes");
            foreach (var foo in GetSortedNodes())
            {
                Console.WriteLine(foo);
            }
        }

        public void PrintSortedEdges()
        {
            Console.WriteLine("Sorted Edges");
            foreach (var foo in GetSortedEdges())
            {
                Console.WriteLine(foo);
            }
            PrintStartNode();
            PrintETA();
        }

        public void PrintEdgeStates()
        {
            Console.Write("EdgeStates ");
            Console.Write("True: ");
            foreach (KeyValuePair<string, bool> kvp in edgeState)
            {
                if (kvp.Value == true)
                    Console.Write("{0} ", kvp.Key);
            }
            Console.Write("False: ");
            foreach (KeyValuePair<string, bool> kvp in edgeState)
            {
                if (kvp.Value == false)
                    Console.Write("{0} ", kvp.Key);
            }
            Console.WriteLine("");
        }

        public void PrintToDoList()
        {
            Console.Write("ToDo List: ");
            var list = this.GetToDoList();
            foreach (var name in list)
            {
                Console.Write("{0} ", name);
            }
            Console.WriteLine("");
        }

        public void ProcessSortedNodes()
        {
            var sort = new QuickGraph.Algorithms.TopologicalSort.TopologicalSortAlgorithm<Vertex, Edge<Vertex>>(this.graph);
            sort.Compute();
            var r = sort.SortedVertices;

            Console.WriteLine("TopologicalSort");
            foreach (Vertex v in r)
            {
                Console.Write("Vertex: ");
                Console.WriteLine(v);
                var inEdges = graph.InEdges(v);
                foreach (var edge in inEdges)
                {
                    Console.Write("Edge: ");
                    Console.WriteLine(edge);
                }
            }
        }
    }

    public static class Program
    {
        static int[] getActive()
        {
            int[] list = { 1, 2, 3, 4, 5 };
            return list;
        }

        static int[] getToDoList()
        {
            int[] list = { 1, 2, 3, 4, 5 };
            return list;
        }

        static int[] getMilestones()
        {
            int[] list = { 1, 2, 3, 4, 5 };
            return list;
        }

        static int[] getDependencyList()
        {
            int[] list = { 1, 2, 3, 4, 5 };
            return list;
        }

        public static void foo()
        {
            AdjacencyGraph<string, Edge<string>> graph = new AdjacencyGraph<string, Edge<string>>(true);

            // Add some vertices to the graph
            graph.AddVertex("A");
            graph.AddVertex("B");
            graph.AddVertex("C");
            graph.AddVertex("D");
            graph.AddVertex("E");
            graph.AddVertex("F");
            graph.AddVertex("G");
            graph.AddVertex("H");
            graph.AddVertex("I");
            graph.AddVertex("J");

            var e = graph.Edges;
            // Create the edges
            Edge<string> a_b = new Edge<string>("A", "B");
            Edge<string> a_d = new Edge<string>("A", "D");
            Edge<string> b_a = new Edge<string>("B", "A");
            Edge<string> b_c = new Edge<string>("B", "C");
            Edge<string> b_e = new Edge<string>("B", "E");
            Edge<string> c_b = new Edge<string>("C", "B");
            Edge<string> c_f = new Edge<string>("C", "F");
            Edge<string> c_j = new Edge<string>("C", "J");
            Edge<string> d_e = new Edge<string>("D", "E");
            Edge<string> d_g = new Edge<string>("D", "G");
            Edge<string> e_d = new Edge<string>("E", "D");
            Edge<string> e_f = new Edge<string>("E", "F");
            Edge<string> e_h = new Edge<string>("E", "H");
            Edge<string> f_i = new Edge<string>("F", "I");
            Edge<string> f_j = new Edge<string>("F", "J");
            Edge<string> g_d = new Edge<string>("G", "D");
            Edge<string> g_h = new Edge<string>("G", "H");
            Edge<string> h_g = new Edge<string>("H", "G");
            Edge<string> h_i = new Edge<string>("H", "I");
            Edge<string> i_f = new Edge<string>("I", "F");
            Edge<string> i_j = new Edge<string>("I", "J");
            Edge<string> i_h = new Edge<string>("I", "H");
            Edge<string> j_f = new Edge<string>("J", "F");

            // Add the edges
            graph.AddEdge(a_b);
            graph.AddEdge(a_d);
            graph.AddEdge(b_a);
            graph.AddEdge(b_c);
            graph.AddEdge(b_e);
            graph.AddEdge(c_b);
            graph.AddEdge(c_f);
            graph.AddEdge(c_j);
            graph.AddEdge(d_e);
            graph.AddEdge(d_g);
            graph.AddEdge(e_d);
            graph.AddEdge(e_f);
            graph.AddEdge(e_h);
            graph.AddEdge(f_i);
            graph.AddEdge(f_j);
            graph.AddEdge(g_d);
            graph.AddEdge(g_h);
            graph.AddEdge(h_g);
            graph.AddEdge(h_i);
            graph.AddEdge(i_f);
            graph.AddEdge(i_h);
            graph.AddEdge(i_j);
            graph.AddEdge(j_f);

            // Define some weights to the edges
            Dictionary<Edge<string>, double> edgeCost = new Dictionary<Edge<string>, double>(graph.EdgeCount);
            edgeCost.Add(a_b, 4);
            edgeCost.Add(a_d, 1);
            edgeCost.Add(b_a, 74);
            edgeCost.Add(b_c, 2);
            edgeCost.Add(b_e, 12);
            edgeCost.Add(c_b, 12);
            edgeCost.Add(c_f, 74);
            edgeCost.Add(c_j, 12);
            edgeCost.Add(d_e, 32);
            edgeCost.Add(d_g, 22);
            edgeCost.Add(e_d, 66);
            edgeCost.Add(e_f, 76);
            edgeCost.Add(e_h, 33);
            edgeCost.Add(f_i, 11);
            edgeCost.Add(f_j, 21);
            edgeCost.Add(g_d, 12);
            edgeCost.Add(g_h, 10);
            edgeCost.Add(h_g, 2);
            edgeCost.Add(h_i, 72);
            edgeCost.Add(i_f, 31);
            edgeCost.Add(i_h, 18);
            edgeCost.Add(i_j, 7);
            edgeCost.Add(j_f, 8);

            Func<Edge<string>, double> edgeCost2 = e1 => 1; // constant cost
            Dictionary<Edge<string>, double> edgeCost3 = new Dictionary<Edge<string>, double>(graph.EdgeCount);

            // We want to use Dijkstra on this graph
            DijkstraShortestPathAlgorithm<string, Edge<string>> dijkstra = new DijkstraShortestPathAlgorithm<string, Edge<string>>(graph, edgeCost2);

            // attach a distance observer to give us the shortest path distances
            VertexDistanceRecorderObserver<string, Edge<string>> distObserver = new VertexDistanceRecorderObserver<string, Edge<string>>(edgeCost2);
            distObserver.Attach(dijkstra);

            // Attach a Vertex Predecessor Recorder Observer to give us the paths
            VertexPredecessorRecorderObserver<string, Edge<string>> predecessorObserver = new VertexPredecessorRecorderObserver<string, Edge<string>>();
            predecessorObserver.Attach(dijkstra);

            // Run the algorithm with A set to be the source
            dijkstra.Compute("A");

            foreach (KeyValuePair<string, double> kvp in distObserver.Distances)
                Console.WriteLine("Distance from root to node {0} is {1}", kvp.Key, kvp.Value);

            foreach (KeyValuePair<string, Edge<string>> kvp in predecessorObserver.VertexPredecessors)
                Console.WriteLine("If you want to get to {0} you have to enter through the edge {1}", kvp.Key, kvp.Value);

            // Remember to detach the observers
            // distObserver.Detach(dijkstra);
            // predecessorObserver.Detach(dijkstra);
        }

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            if (!Console.IsOutputRedirected) Console.Clear();

            var wf = new Workflow();

            string line;
            Console.WriteLine("Enter tasks to complete (press CTRL+Z to exit):");
            Console.WriteLine();
            do
            {
                wf.PrintEdgeStates();
                wf.UpdateAllNodes();
                wf.PrintStartNode();
                wf.PrintToDoList();
                wf.PrintETA();
                line = Console.ReadLine();
                if (line != null)
                    Console.WriteLine("task: " + line);
                var taskState = wf.edgeState;
                taskState[line] = true;
            } while (line != null);
        }
    }
}

