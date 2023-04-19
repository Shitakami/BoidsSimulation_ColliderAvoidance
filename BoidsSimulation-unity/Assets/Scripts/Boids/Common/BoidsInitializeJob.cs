using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Boids.Job
{
    [BurstCompile]
    internal struct BoidsInitializeJob : IJobParallelFor
    {
        [WriteOnly] public NativeArray<BoidsData> BoidsDatasWrite;
        [ReadOnly] public float3 SimulationAreaCenter;
        [ReadOnly] public float3 SimulationAreaScale;
        [ReadOnly] public float InitializeVelocity;
        [ReadOnly] public Random Random;

        public void Execute(int index)
        {
            BoidsDatasWrite[index] = new BoidsData
            {
                Position = Random.NextFloat3() * SimulationAreaScale + SimulationAreaCenter,
                Velocity = Random.NextFloat3Direction() * InitializeVelocity
            };
        }
    }
}
