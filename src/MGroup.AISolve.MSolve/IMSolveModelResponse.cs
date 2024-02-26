using MGroup.AISolve.Core;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.Solvers.LinearSystem;
using System;

namespace MGroup.AISolve.MSolve
{
    /// <summary>
    /// Provides a framework for getting the response of standard MSolve problems. The creation of <see cref="MGroup.MSolve.Discretization.Entities.Model"/>
    /// instances is abstracted through the <see cref="IModelCreator"/> interface.
    /// </summary>
    public interface IMSolveModelResponse : IModelResponse
    {
        /// <summary>
        /// The object implementing <see cref="IModelCreator"/> that creates a <see cref="MGroup.MSolve.Discretization.Entities.Model"/>
        /// based on the parameters as obtained from <see cref="IModelResponse.GetModelResponse"/>.
        /// </summary>
        IModelCreator ModelCreator { get; }

        /// <summary>
        /// Sets up the <see cref="IParentAnalyzer"/> that will be used for obtaining the response of a <see cref="MGroup.MSolve.Discretization.Entities.Model"/>
        /// that takes into account the <paramref name="parameterValues"/> provided, as obtained from <see cref="ModelCreator"/>.
        /// </summary>
        /// <param name="parameterValues">A double array containing arbitrary parameters ordered as in <see cref="IModelResponse.GetModelResponse"/>.</param>
        /// <returns>
        /// A <see cref="IParentAnalyzer"/> instance for the solution of the <see cref="MGroup.MSolve.Discretization.Entities.Model"/> 
        /// made, as obtained from <see cref="ModelCreator"/>.
        /// </returns>
        IParentAnalyzer InitializeProblem(double[] parameterValues);

        /// <summary>
        /// Evaluates the response of the <see cref="MGroup.MSolve.Discretization.Entities.Model"/> obtained from <see cref="ModelCreator"/>,
        /// as evaluated from the analysis problem defined in <see cref="InitializeProblem"/>.
        /// </summary>
        /// <param name="parameterValues">A double array containing arbitrary parameters ordered as in <see cref="IModelResponse.GetModelResponse"/>.</param>
        /// <returns>A double array with the raw values of the solution of the linear system constructed by <see cref="InitializeProblem"/>.</returns>
        /// <exception cref="InvalidOperationException"></exception>
        double[] GetModelResponse(double[] parameterValues)
        {
            if (ModelCreator == null) 
            {
                throw new InvalidOperationException("ModelCreator is null");
            }
            
            var analyzer = InitializeProblem(parameterValues);
            analyzer.Initialize(true);
            analyzer.Solve();

            return ((GlobalVector)analyzer.ChildAnalyzer.CurrentAnalysisResult).SingleVector.RawData;
        }
    }
}
