using Boids.Utility;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boids.Job
{
    [BurstCompile]
    internal struct ApplySteerForceJob : IJobParallelFor
    {
        private NativeArray<BoidsData> _boidsDatasWrite;
        [ReadOnly] private readonly NativeArray<float3> _boidsForceRead;
        [WriteOnly] private NativeArray<Matrix4x4> _boidsTransformMatrices;

        [ReadOnly] private readonly float3 _simulationAreaCenter;
        [ReadOnly] private readonly float3 _simulationAreaScaleHalf;
        [ReadOnly] private readonly float _avoidWallWeight;

        [ReadOnly] private readonly float _deltaTime;
        [ReadOnly] private readonly float _maxSpeed;
        [ReadOnly] private readonly float3 _instanceScale;

        public ApplySteerForceJob(
            NativeArray<BoidsData> boidsDatasWrite,
            NativeArray<float3> boidsForceRead,
            NativeArray<Matrix4x4> boidsTransformMatrices,
            float3 simulationAreaCenter,
            float3 simulationAreaScaleHalf,
            float avoidWallWeight,
            float deltaTime,
            float maxSpeed,
            float3 instanceScale
        )
        {
            _boidsDatasWrite = boidsDatasWrite;
            _boidsForceRead = boidsForceRead;
            _boidsTransformMatrices = boidsTransformMatrices;
            _simulationAreaCenter = simulationAreaCenter;
            _simulationAreaScaleHalf = simulationAreaScaleHalf;
            _avoidWallWeight = avoidWallWeight;
            _deltaTime = deltaTime;
            _maxSpeed = maxSpeed;
            _instanceScale = instanceScale;
        }

        public void Execute(int ownIndex)
        {
            var boidsData = _boidsDatasWrite[ownIndex];
            var force = _boidsForceRead[ownIndex];

            force += CalculateBoundsForce(boidsData.Position, _simulationAreaCenter, _simulationAreaScaleHalf) * _avoidWallWeight;

            var velocity = boidsData.Velocity + (force * _deltaTime);
            boidsData.Velocity = MathematicsUtility.Limit(velocity, _maxSpeed);
            boidsData.Position += velocity * _deltaTime;

            _boidsDatasWrite[ownIndex] = boidsData;

            var rotationY = math.atan2(boidsData.Velocity.x, boidsData.Velocity.z);
            var rotationX = (float) -math.asin(boidsData.Velocity.y / (math.length(boidsData.Velocity.xyz) + 1e-8));
            var rotation = quaternion.Euler(rotationX, rotationY, 0);
            _boidsTransformMatrices[ownIndex] = float4x4.TRS(boidsData.Position, rotation, _instanceScale);
        }

        private static float3 CalculateBoundsForce(float3 position, float3 simulationAreaCenter, float3 simulationAreaScale)
        {
            var acc = new float3();

            acc.x = position.x < simulationAreaCenter.x - simulationAreaScale.x
                ? acc.x + 1.0f
                : acc.x;

            acc.x = position.x > simulationAreaCenter.x + simulationAreaScale.x
                ? acc.x - 1.0f
                : acc.x;

            acc.y = position.y < simulationAreaCenter.y - simulationAreaScale.y
                ? acc.y + 1.0f
                : acc.y;

            acc.y = position.y > simulationAreaCenter.y + simulationAreaScale.y
                ? acc.y - 1.0f
                : acc.y;

            acc.z = position.z < simulationAreaCenter.z - simulationAreaScale.z
                ? acc.z + 1.0f
                : acc.z;

            acc.z = position.z > simulationAreaCenter.z + simulationAreaScale.z
                ? acc.z - 1.0f
                : acc.z;

            return acc;
        }
    }
}