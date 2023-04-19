using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boids.Job
{
    [BurstCompile]
    internal struct RaycastCommandInitializeJob : IJobParallelFor
    {
        [WriteOnly] private NativeArray<RaycastCommand> _raycastCommandsWrite;
        [ReadOnly] private readonly NativeArray<BoidsData> _boidsDatasRead;
        [ReadOnly] private readonly float _rayDistance;

        public RaycastCommandInitializeJob(
            NativeArray<RaycastCommand> raycastCommandsWrite,
            NativeArray<BoidsData> boidsDatasRead,
            float rayDistance)
        {
            _raycastCommandsWrite = raycastCommandsWrite;
            _boidsDatasRead = boidsDatasRead;
            _rayDistance = rayDistance;
        }

        public void Execute(int index)
        {
            _raycastCommandsWrite[index] = new RaycastCommand(
                from: _boidsDatasRead[index].Position,
                direction: math.normalize(_boidsDatasRead[index].Velocity),
                distance: _rayDistance
            );
        }
    }
}
