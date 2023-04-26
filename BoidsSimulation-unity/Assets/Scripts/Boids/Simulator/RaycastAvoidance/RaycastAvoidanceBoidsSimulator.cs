using System;
using Boids.Job;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boids
{
    public class RaycastAvoidanceBoidsSimulator : IDisposable
    {
        private readonly RaycastAvoidanceBoidsSetting _raycastAvoidanceBoidsSetting;
        private readonly int _instanceCount;
        private NativeArray<BoidsData> _boidsDatas;
        private NativeMultiHashMap<int3, int> _gridHashMap;
        private NativeArray<RaycastCommand> _raycastCommands;
        private NativeArray<RaycastHit> _raycastHits;
        private NativeArray<float3> _boidsSteers;
        private NativeArray<Matrix4x4> _boidsTransformMatrices;

        public NativeArray<Matrix4x4> BoidsTransformMatrices => _boidsTransformMatrices;

        private JobHandle _jobHandle;

        public RaycastAvoidanceBoidsSimulator(
            RaycastAvoidanceBoidsSetting raycastAvoidanceBoidsSetting,
            int instanceCount
        )
        {
            _raycastAvoidanceBoidsSetting = raycastAvoidanceBoidsSetting;
            _instanceCount = instanceCount;

            _boidsDatas = new NativeArray<BoidsData>(instanceCount, Allocator.Persistent);
            _gridHashMap = new NativeMultiHashMap<int3, int>(instanceCount, Allocator.Persistent);
            _raycastCommands = new NativeArray<RaycastCommand>(instanceCount, Allocator.Persistent);
            _raycastHits = new NativeArray<RaycastHit>(instanceCount, Allocator.Persistent);
            _boidsSteers = new NativeArray<float3>(instanceCount, Allocator.Persistent);
            _boidsTransformMatrices = new NativeArray<Matrix4x4>(instanceCount, Allocator.Persistent);
        }

        public void InitializeBoidsPositionAndRotation(float3 simulationAreaCenter, float3 simulationAreaScale)
        {
            BoidsUtility.InitializeBoidsData(
                _boidsDatas,
                simulationAreaCenter,
                simulationAreaScale / 2,
                _raycastAvoidanceBoidsSetting.InitializedSpeed
            );
        }

        public void ExecuteJob(float3 simulationAreaCenter, float3 simulationAreaScale)
        {
            _gridHashMap.Clear();

            var registerInstanceToGridJob = new RegisterInstanceToGridJob
            (
                _gridHashMap.AsParallelWriter(),
                _boidsDatas,
                _raycastAvoidanceBoidsSetting.NeighborSearchGridScale
            );

            var registerInstanceToGridHandle = registerInstanceToGridJob.Schedule(_instanceCount, 100);

            var boidsJob = new CalculateBoidsSteerForceJob
            (
                _raycastAvoidanceBoidsSetting.CohesionWeight,
                _raycastAvoidanceBoidsSetting.CohesionAffectedRadiusSqr,
                _raycastAvoidanceBoidsSetting.CohesionViewDot,
                _raycastAvoidanceBoidsSetting.SeparateWeight,
                _raycastAvoidanceBoidsSetting.SeparateAffectedRadiusSqr,
                _raycastAvoidanceBoidsSetting.SeparateViewDot,
                _raycastAvoidanceBoidsSetting.AlignmentWeight,
                _raycastAvoidanceBoidsSetting.AlignmentAffectedRadiusSqr,
                _raycastAvoidanceBoidsSetting.AlignmentViewDot,
                _raycastAvoidanceBoidsSetting.MaxSpeed,
                _raycastAvoidanceBoidsSetting.MaxSteerForce,
                _gridHashMap,
                _raycastAvoidanceBoidsSetting.NeighborSearchGridScale,
                _boidsDatas,
                _boidsSteers
            );

            var boidsJobHandler = boidsJob.Schedule(_instanceCount, 100, registerInstanceToGridHandle);
            
            var initializeRaycastCommandJob = new RaycastCommandInitializeJob(
                _raycastCommands,
                _boidsDatas,
                _raycastAvoidanceBoidsSetting.RayDistance
            );

            var initializeRaycastCommandJobHandle = initializeRaycastCommandJob.Schedule(_instanceCount, 100);
            var raycastHandle = RaycastCommand.ScheduleBatch(_raycastCommands, _raycastHits, 100, initializeRaycastCommandJobHandle);

            var applySteerForce = new ApplySteerForceWithAvoidanceJob
            (
                _boidsDatas,
                _boidsSteers,
                _boidsTransformMatrices,
                simulationAreaCenter,
                simulationAreaScale/2,
                _raycastAvoidanceBoidsSetting.AvoidSimulationAreaWeight,
                Time.deltaTime,
                _raycastAvoidanceBoidsSetting.MaxSpeed,
                _raycastAvoidanceBoidsSetting.InstanceScale,
                _raycastAvoidanceBoidsSetting.AvoidRotationVelocity,
                _raycastHits
            );

            _jobHandle = applySteerForce.Schedule(
                _instanceCount, 
                100,
                JobHandle.CombineDependencies(boidsJobHandler, raycastHandle));
            
            JobHandle.ScheduleBatchedJobs();
        }

        public void Complete()
        {
            _jobHandle.Complete();
        }

        public void Dispose()
        {
            // MEMO: Job実行中は NativeArray.Dispose が出来ないので、Jobが完了するまで待つ
            Complete();
            
            _boidsDatas.Dispose();
            _gridHashMap.Dispose();
            _raycastCommands.Dispose();
            _raycastHits.Dispose();
            _boidsSteers.Dispose();
            _boidsTransformMatrices.Dispose();
        }
    }
}
