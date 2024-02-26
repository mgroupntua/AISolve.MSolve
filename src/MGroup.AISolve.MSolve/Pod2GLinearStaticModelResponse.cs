using MGroup.AISolve.Core;
using MGroup.AISolve.MachineLearning;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.MachineLearning.Surrogates;
using MGroup.LinearAlgebra.Iterative.AlgebraicMultiGrid;
using MGroup.LinearAlgebra.Iterative.AlgebraicMultiGrid.PodAmg;
using MGroup.LinearAlgebra.Iterative.AlgebraicMultiGrid.Smoothing;
using MGroup.LinearAlgebra.Iterative.GaussSeidel;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Termination.Iterations;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MachineLearning.Utilities;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Iterative;
using MGroup.Solvers.LinearSystem;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace MGroup.AISolve.MSolve
{
    /// <summary>
    /// Implements the <see href="https://arxiv.org/abs/2207.02543">POD2G algorithm</see> for the solution of 
    /// parameterized linear steady state computational mechanics problems.
    /// </summary>
    public class Pod2GLinearStaticModelResponse : Pod2GResponse, IMSolveModelResponse
    {
		/// <inheritdoc/>
		public IModelCreator ModelCreator { get; }

        /// <summary>
        /// The last constructed <see cref="IAlgebraicModel"/> constructed from <see cref="InitializeProblem"/>.
        /// </summary>
        public IAlgebraicModel CurrentAlgebraicModel { get; private set; }

        /// <summary>
        /// The last constructed <see cref="ISolverLogger"/> constructed from <see cref="InitializeProblem"/>.
        /// </summary>
        public ISolverLogger CurrentSolverLogger { get; private set; }

        /// <summary>
        /// Constructs a <see cref="Pod2GLinearStaticModelResponse"/> object that implements the
        /// <see href="https://arxiv.org/abs/2207.02543">POD2G algorithm</see> for the solution of 
        /// parameterized linear steady state computational mechanics problems.
        /// </summary>
        /// <param name="podPrincipalComponents">The number of components that the set of raw solutions will be decomposed when POD is applied.</param>
        /// <param name="modelCreator">
        /// The object implementing <see cref="IModelCreator"/> that creates a <see cref="MGroup.MSolve.Discretization.Entities.Model"/>
        /// based on the parameters as obtained from <see cref="IModelResponse.GetModelResponse"/>.
        /// </param>
        public Pod2GLinearStaticModelResponse(int podPrincipalComponents, IModelCreator modelCreator)
			: base(podPrincipalComponents)
        {
            this.ModelCreator = modelCreator;
        }

		/// <inheritdoc/>
		public override double[] GetModelResponse(double[] parameterValues, bool isAIEnhanced)
		{
			if (ModelCreator == null)
			{
				throw new InvalidOperationException("ModelCreator is null");
			}

			var analyzer = InitializeProblem(parameterValues, isAIEnhanced);
			analyzer.Initialize(true);
			analyzer.Solve();

			int numPcgIterations = CurrentSolverLogger.GetNumIterationsOfIterativeAlgorithm(0);
			int solutionLength = CurrentAlgebraicModel.LinearSystem.Solution.Length();
			Debug.WriteLine($"Number of PCG iterations = {numPcgIterations}. Dofs = {solutionLength}.");

			return ((GlobalVector)analyzer.ChildAnalyzer.CurrentAnalysisResult).SingleVector.RawData;
		}

        /// <inheritdoc/>
        public IParentAnalyzer InitializeProblem(double[] parameterValues) => InitializeProblem(parameterValues, false);

        /// <inheritdoc cref="IMSolveModelResponse.InitializeProblem" />
        /// <param name="useAmgPreconditioner">When true, the iterative solver used is equipped with the <see cref="PodAmgPreconditioner"/>.</param>
        private IParentAnalyzer InitializeProblem(double[] parameterValues, bool useAmgPreconditioner)
        {
            var model = ModelCreator.GetModel(parameterValues);

            var pcgBuilder = new PcgAlgorithm.Builder
            {
                ResidualTolerance = 1E-6,
                MaxIterationsProvider = new PercentageMaxIterationsProvider(maxIterationsOverMatrixOrder: 0.5)
            };

            var solverFactory = new PcgSolver.Factory()
            {
                PcgAlgorithm = pcgBuilder.Build(),
            };

            if (useAmgPreconditioner) 
            {
                solverFactory.PreconditionerFactory = amgPreconditionerFactory;
            }

            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            CurrentAlgebraicModel = algebraicModel;
            CurrentSolverLogger = solver.Logger;

            var problem = new ProblemStructural(model, algebraicModel);
            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, linearAnalyzer);

            return staticAnalyzer;
        }
	}
}
