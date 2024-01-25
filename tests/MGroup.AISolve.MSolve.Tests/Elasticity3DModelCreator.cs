using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Continuum;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Meshes.Generation;
using System;
using System.Collections.Generic;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution;
using MGroup.Solvers.LinearSystem;

namespace MGroup.AISolve.MSolve.Tests
{
    internal class Elasticity3DModelCreator : IModelCreator
    {
        private const double cubeSide = 1.0;
        private const int numElementsPerSide = 16; // must be even

        public Model GetModel(double[] parameterValues)
        {
            // Mesh
            double minX = 0, minY = 0, minZ = 0;
            double maxX = cubeSide, maxY = cubeSide, maxZ = cubeSide;
            var meshGenerator = new UniformMeshGenerator3D<Node>(minX, minY, minZ, maxX, maxY, maxZ,
                numElementsPerSide, numElementsPerSide, numElementsPerSide);
            (IReadOnlyList<Node> nodes, IReadOnlyList<CellConnectivity<Node>> elements)
                = meshGenerator.CreateMesh((id, x, y, z) => new Node(id, x, y, z));

            // Create model
            var model = new Model();
            int s = 0;
			model.SubdomainsDictionary[s] = new Subdomain(s);

            // Materials
            double v = 0.3;
            var material = new ElasticMaterial3D(parameterValues[0], v);

            // Nodes
            foreach (Node node in nodes)
            {
                model.NodesDictionary[node.ID] = node;
            }

            // Elements
            var elementFactory = new ContinuumElement3DFactory(material, null);
            for (int e = 0; e < elements.Count; e++)
            {
                ContinuumElement3D element = elementFactory.CreateElement(elements[e].CellType, elements[e].Vertices);
                element.ID = e;
                model.ElementsDictionary[element.ID] = element;
                model.SubdomainsDictionary[s].Elements.Add(element);
            }

            // Constraints
            var constraints = new List<INodalDisplacementBoundaryCondition>();
            List<Node> constrainedNodes = FindNodesWith(IsNodeConstrained, model);
            foreach (Node node in constrainedNodes)
            {
                constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationX, amount: 0));
                constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationY, amount: 0));
                constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationZ, amount: 0));
            }

            // Loads
            var loads = new List<INodalLoadBoundaryCondition>();
            List<Node> loadedNodes = FindNodesWith(IsNodeLoaded, model);
            double totalLoad = parameterValues[1] / (cubeSide * cubeSide / 4);
            double loadPerNode = totalLoad / loadedNodes.Count;
            foreach (Node node in loadedNodes)
            {
                loads.Add(new NodalLoad(node, StructuralDof.TranslationZ, loadPerNode));
            }
            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

            return model;
        }

        public static List<Node> FindNodesWith(Func<Node, bool> predicate, Model model)
        {
            var result = new List<Node>();
            foreach (Node node in model.NodesDictionary.Values)
            {
                if (predicate(node))
                {
                    result.Add(node);
                }
            }
            return result;
        }

        private static bool IsNodeConstrained(Node node)
        {
            double dx = cubeSide / numElementsPerSide;
            double tol = dx / 4.0;
            return Math.Abs(node.Z) < tol;
        }

        private static bool IsNodeLoaded(Node node)
        {
            double dx = cubeSide / numElementsPerSide;
            double tol = dx / 4.0;
            bool isLoaded = Math.Abs(node.Z - 1.0) < tol;
            isLoaded &= node.X > cubeSide / 4.0 - tol;
            isLoaded &= node.X < 3.0 / 4.0 * cubeSide + tol;
            isLoaded &= node.Y > cubeSide / 4.0 - tol;
            isLoaded &= node.Y < 3.0 / 4.0 * cubeSide + tol;
            return isLoaded;
        }

        public static double GetMeanValueOfMonitoredNode(Node monitorNode, IAlgebraicModel algebraicModel, ISolverLogger solverLogger, double[] solutionValues)
        {
            //int numIterations = solverLogger.GetNumIterationsOfIterativeAlgorithm(0);
            var solutionVector = (GlobalVector)algebraicModel.CreateZeroVector();
            solutionVector.SingleVector = Vector.CreateFromArray(solutionValues);
            return algebraicModel.ExtractSingleValue(solutionVector, monitorNode, StructuralDof.TranslationZ);
        }

        public static Node FindMonitoredNode(Model model)
        {
            if (numElementsPerSide % 2 == 1)
            {
                throw new Exception($"The number of elements along each side of the domain is {numElementsPerSide}," +
                    $" but it must be even, in order to define a central 'monitor' node");
            }

            var result = new List<Node>();
            foreach (Node node in model.NodesDictionary.Values)
            {
                double dx = cubeSide / numElementsPerSide;
                double tol = dx / 4.0;
                bool isMonitored = Math.Abs(node.Z - 1.0) < tol;
                isMonitored &= Math.Abs(node.X - cubeSide / 2.0) < tol;
                isMonitored &= Math.Abs(node.Y - cubeSide / 2.0) < tol;
                if (isMonitored)
                {
                    result.Add(node);
                }
            }

            if (result.Count != 1)
            {
                throw new Exception($"Found {result.Count} monitor nodes, but only 1 was expected");
            }
            else
            {
                return result[0];
            }
        }
    }
}
