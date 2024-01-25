using MGroup.AISolve.Core;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Xunit;

namespace MGroup.AISolve.MSolve.Tests
{
    public class StructuralTests
    {
        private const int numAnalysesTotal = 300;
        private const int numSolutionsForTraining = 50; // paper used 300
        private const int numPrincipalComponents = 8;
        private const int seed = 13;

        [Fact]
        public static void Elasticity3DTest()
        {
            // Do NOT execute on Azure DevOps
            if (Environment.GetEnvironmentVariable("SYSTEM_DEFINITIONID") != null)
            { 
                return; 
            }

            var modelParameters = GenerateParameterValues(numAnalysesTotal);
            var modelCreator = new Elasticity3DModelCreator();
            var modelResponse = new Pod2GLinearStaticModelResponse(numPrincipalComponents, modelCreator);
            var solver = new AISolver(numSolutionsForTraining, modelParameters, modelResponse, modelResponse);
            var responses = new List<double[]>(solver);

            var model = modelCreator.GetModel(new[] { 0d, 0d });
            var monitorNode = Elasticity3DModelCreator.FindMonitoredNode(model);
            var responsesOfMonitoredNode = responses.Select(x => Elasticity3DModelCreator.GetMeanValueOfMonitoredNode(monitorNode, modelResponse.CurrentAlgebraicModel, modelResponse.CurrentSolverLogger, x)).ToArray();
            var mean = responsesOfMonitoredNode.Average();
            Debug.WriteLine($"Total analyses: {numAnalysesTotal}. Training analyses: {numSolutionsForTraining}. " +
                $"Mean uTop={mean}");
        }

        private static IList<double[]> GenerateParameterValues(int numAnalysesTotal)
        {
            double meanE = 2000; //MPa
            double stdevE = 600; //MPa
            double meanP = -10; //MPa
            double stdevP = 3; //MPa
            var rng = new Random(seed);
            double[] paramsE = GenerateSamplesNormal(numAnalysesTotal, meanE, stdevE, rng).ToArray();
            double[] paramsP = GenerateSamplesNormal(numAnalysesTotal, meanP, stdevP, rng).ToArray();

            var samples = new double[numAnalysesTotal][];
            for (int i = 0; i < numAnalysesTotal; i++)
            {
                samples[i] = new[] { paramsE[i], paramsP[i] };
            }

            return samples;
        }

        private static IEnumerable<double> GenerateSamplesNormal(int count, double mean, double stddev, Random rng)
        {
            double[] samples = new double[count];
            MathNet.Numerics.Distributions.Normal.Samples(rng, samples, mean, stddev);
            return samples;
            //return Normal.Samples(rng, mean, stddev);
        }

    }
}
