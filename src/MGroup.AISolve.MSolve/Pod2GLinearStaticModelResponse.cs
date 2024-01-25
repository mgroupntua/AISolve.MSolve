using MGroup.AISolve.Core;
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
    public class Pod2GLinearStaticModelResponse : MSolveModelResponse, IAIResponse
    {
        private IList<double[]> trainingModelParameters = new List<double[]>();
        private IList<Vector> trainingSolutionVectors = new List<Vector>();
        private PodAmgPreconditioner.Factory amgPreconditionerFactory;

        /// <summary>
        /// The number of components that the set of raw solutions will be decomposed when POD is applied.
        /// </summary>
        public int PodPrincipalComponents { get; }

        /// <summary>
        /// The <see cref="CaeFffnSurrogate"/> that will be used by the <see cref="PodAmgPreconditioner"/> of the POD2G algorithn.
        /// </summary>
        public CaeFffnSurrogate CaeFffnSurrogate { get; }

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
        {
            var datasetSplitter = new DatasetSplitter()
            {
                MinTestSetPercentage = 0.2,
                MinValidationSetPercentage = 0,
            };
            datasetSplitter.SetOrderToContiguous(DataSubsetType.Training, DataSubsetType.Test);

            var surrogateBuilder = new CaeFffnSurrogate.Builder()
            {
                CaeBatchSize = 20,
                CaeLearningRate = 0.001f,
                CaeNumEpochs = 50, //paper used 500
                DecoderFiltersWithoutOutput = new int[] { 64, 128, 256 },
                EncoderFilters = new int[] { 256, 128, 64, 32 },
                FfnnBatchSize = 20,
                FfnnHiddenLayerSize = 64,
                FfnnLearningRate = 0.0001f,
                FfnnNumEpochs = 300, // paper used 3000
                FfnnNumHiddenLayers = 6,
                Splitter = datasetSplitter,
            };

            this.PodPrincipalComponents = podPrincipalComponents;
            this.ModelCreator = modelCreator;
            this.CaeFffnSurrogate = surrogateBuilder.BuildSurrogate();
            this.amgPreconditionerFactory = new PodAmgPreconditioner.Factory()
            {
                NumIterations = 1,
                SmootherBuilder = new GaussSeidelSmoother.Builder(new GaussSeidelIterationCsrSerial.Builder(), GaussSeidelSweepDirection.Symmetric, numIterations: 1),
                KeepOnlyNonZeroPrincipalComponents = true,
            };
        }

        /// <inheritdoc/>
        public override double[] GetModelResponse(double[] parameterValues)
        {
            var modelResponse = base.GetModelResponse(parameterValues);
            int numPcgIterations = CurrentSolverLogger.GetNumIterationsOfIterativeAlgorithm(0);
            int solutionLength = CurrentAlgebraicModel.LinearSystem.Solution.Length();
            Debug.WriteLine($"Number of PCG iterations = {numPcgIterations}. Dofs = {solutionLength}.");

            return modelResponse;
        }

        /// <inheritdoc/>
        double[] IAIResponse.GetModelResponse(double[] parameterValues)
        {
            if (ModelCreator == null)
            {
                throw new InvalidOperationException("ModelCreator is null");
            }

            var analyzer = InitializeProblem(parameterValues, true);
            analyzer.Initialize(true);
            analyzer.Solve();

            int numPcgIterations = CurrentSolverLogger.GetNumIterationsOfIterativeAlgorithm(0);
            int solutionLength = CurrentAlgebraicModel.LinearSystem.Solution.Length();
            Debug.WriteLine($"Number of PCG iterations = {numPcgIterations}. Dofs = {solutionLength}.");

            return ((GlobalVector)analyzer.ChildAnalyzer.CurrentAnalysisResult).SingleVector.RawData;
        }

        /// <summary>
        /// Stores the <paramref name="response"/> of a model having the specific <paramref name="parameterValues"/>.
        /// This information will be used when <see cref="TrainWithRegisteredModelResponses"/> is called by <see cref="AISolver"/>.
        /// </summary>
        /// <param name="parameterValues">A double array containing arbitrary parameters ordered as in <see cref="IModelResponse.GetModelResponse"/>.</param>
        /// <param name="response">A double array with the raw values of the solution of the linear system constructed by <see cref="InitializeProblem"/>.</param>
        public void RegisterModelResponse(double[] parameterValues, double[] response)
        {
            trainingModelParameters.Add(parameterValues);
            trainingSolutionVectors.Add(Vector.CreateFromArray(response));
        }

        /// <summary>
        /// Performs POD on the solution vectors and trains a <see cref="CaeFffnSurrogate"/> using the data
        /// obtained from <see cref="RegisterModelResponse"/>.
        /// </summary>
        public void TrainWithRegisteredModelResponses()
        {
            // Gather all previous solution vectors as columns of a matrix
            int numSamples = trainingSolutionVectors.Count;
            int numDofs = trainingSolutionVectors[0].Length;
            Matrix solutionVectors = Matrix.CreateZero(numDofs, numSamples);
            for (int j = 0; j < numSamples; ++j)
            {
                solutionVectors.SetSubcolumn(j, trainingSolutionVectors[j]);
            }

            // Free up some memory by deleting the stored solution vectors
            trainingSolutionVectors.Clear();

            // AMG-POD training
            amgPreconditionerFactory.Initialize(solutionVectors, PodPrincipalComponents);

            // Gather all previous model parameters
            if (trainingModelParameters.Count != numSamples)
            {
                throw new Exception($"Have gathered {trainingModelParameters.Count} sets of model parameters, " +
                    $"but {numSamples} solution vectors, while using initial preconditioner.");
            }

            int numParameters = trainingModelParameters[0].Length;
            var parametersAsArray = new double[numSamples, numParameters];
            for (int i = 0; i < numSamples; ++i)
            {
                if (trainingModelParameters[i].Length != numParameters)
                {
                    throw new Exception("The model parameter sets do not all have the same size");
                }

                for (int j = 0; j < numParameters; ++j)
                {
                    parametersAsArray[i, j] = trainingModelParameters[i][j];
                }
            }

            // CAE-FFNN training. Dimension 0 must be the number of samples.
            double[,] solutionsAsArray = solutionVectors.Transpose().CopytoArray2D();
            CaeFffnSurrogate.TrainAndEvaluate(parametersAsArray, solutionsAsArray, null);
        }

        /// <inheritdoc/>
        protected override IParentAnalyzer InitializeProblem(double[] parameterValues) => InitializeProblem(parameterValues, false);

        /// <inheritdoc cref="MSolveModelResponse.InitializeProblem" />
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
