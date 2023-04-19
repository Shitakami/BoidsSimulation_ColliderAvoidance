using Boids.Utility;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Boids.Job
{
    [BurstCompile]
    internal struct RegisterInstanceToGridJob : IJobParallelFor
    {
        [WriteOnly] private NativeMultiHashMap<int3, int>.ParallelWriter _gridWriter;
        [ReadOnly] private readonly NativeArray<BoidsData> _boidsDatasRead;
        [ReadOnly] private readonly float _gridScale;
        
        public RegisterInstanceToGridJob(
            NativeMultiHashMap<int3, int>.ParallelWriter gridWriter,
            NativeArray<BoidsData> boidsDatasRead,
            float gridScale)
        {
            _gridWriter = gridWriter;
            _boidsDatasRead = boidsDatasRead;
            _gridScale = gridScale;
        }

        public void Execute(int index)
        {
            var boidsDataPosition = _boidsDatasRead[index].Position;

            var gridIndex = MathematicsUtility.CalculateGridIndex(boidsDataPosition, _gridScale);

            _gridWriter.Add(gridIndex, index);
        }
    }
}