using MGroup.MSolve.Discretization.Entities;

namespace MGroup.AISolve.MSolve
{
    /// <summary>
    /// Defines the generation of a <see cref="Model"/> with specific parameters.
    /// </summary>
    public interface IModelCreator
    {
        /// <summary>
        /// Generates a <see cref="Model"/> that takes into account the <paramref name="parameterValues"/> provided.
        /// </summary>
        /// <param name="parameterValues">A double array containing arbitrary parameters ordered as in <see cref="IModelResponse.GetModelResponse"/>.</param>
        /// <returns>A <see cref="Model"/> instance made using the <paramref name="parameterValues"/> provided.</returns>
        Model GetModel(double[] parameterValues);
    }
}
