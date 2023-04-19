using System;
using Boids.Job;
using Boids.Settings;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boids
{
    public class NeighborSearchBoidsSimulator : IDisposable
    {
        private readonly NeighborSearchBoidsSetting _neighborSearchBoidsSetting;
        private readonly int _instanceCount;
        private NativeArray<BoidsData> _boidsDatas;
        private NativeMultiHashMap<int3, int> _gridHashMap;
        private NativeArray<float3> _boidsSteers;
        private NativeArray<Matrix4x4> _boidsTransformMatrices;
        
        public NativeArray<Matrix4x4> BoidsTransformMatrices => _boidsTransformMatrices;

        private JobHandle _jobHandle;
        
        public NeighborSearchBoidsSimulator(
            NeighborSearchBoidsSetting boidsSetting,
            int instanceCount
        )
        {
            _neighborSearchBoidsSetting = boidsSetting;
            _instanceCount = instanceCount;
            
            _boidsDatas = new NativeArray<BoidsData>(instanceCount, Allocator.Persistent);
            _gridHashMap = new NativeMultiHashMap<int3, int>(instanceCount, Allocator.Persistent);
            _boidsSteers = new NativeArray<float3>(instanceCount, Allocator.Persistent);
            _boidsTransformMatrices = new NativeArray<Matrix4x4>(instanceCount, Allocator.Persistent);
        }
        
        public void InitializeBoidsPositionAndRotation(float3 simulationAreaCenter, float3 simulationAreaScale)
        {
            BoidsUtility.InitializeBoidsData(
                _boidsDatas,
                simulationAreaCenter,
                simulationAreaScale / 2,
                _neighborSearchBoidsSetting.InitializedSpeed
            );
        }

        public void ExecuteJob(float3 simulationAreaCenter, float3 simulationAreaScale)
        {
            _gridHashMap.Clear();
            
            var registerInstanceToGridJob = new RegisterInstanceToGridJob
            (
                _gridHashMap.AsParallelWriter(),
                _boidsDatas,
                _neighborSearchBoidsSetting.NeighborSearchGridScale
            );

            var registerInstanceToGridHandle = registerInstanceToGridJob.Schedule(_instanceCount, 0);

            var boidsJob = new CalculateBoidsSteerForceByNeighborSearchJob
            (
                _neighborSearchBoidsSetting.CohesionWeight,
                _neighborSearchBoidsSetting.CohesionAffectedRadiusSqr,
                _neighborSearchBoidsSetting.CohesionViewDot,
                _neighborSearchBoidsSetting.SeparateWeight,
                _neighborSearchBoidsSetting.SeparateAffectedRadiusSqr,
                _neighborSearchBoidsSetting.SeparateViewDot,
                _neighborSearchBoidsSetting.AlignmentWeight,
                _neighborSearchBoidsSetting.AlignmentAffectedRadiusSqr,
                _neighborSearchBoidsSetting.AlignmentViewDot,
                _neighborSearchBoidsSetting.MaxSpeed,
                _neighborSearchBoidsSetting.MaxSteerForce,
                _gridHashMap,
                _neighborSearchBoidsSetting.NeighborSearchGridScale,
                _boidsDatas,
                _boidsSteers
            );

            var boidsJobHandle = boidsJob.Schedule(_instanceCount, 0, registerInstanceToGridHandle);

            var applySteerForce = new ApplySteerForceJob
            (
                _boidsDatas,
                _boidsSteers,
                _boidsTransformMatrices,
                simulationAreaCenter,
                simulationAreaScale/2,
                _neighborSearchBoidsSetting.AvoidSimulationAreaWeight,
                Time.deltaTime,
                _neighborSearchBoidsSetting.MaxSpeed,
                _neighborSearchBoidsSetting.InstanceScale
            );

            _jobHandle = applySteerForce.Schedule(_instanceCount, 0, boidsJobHandle);
            
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
            _boidsSteers.Dispose();
            _boidsTransformMatrices.Dispose();
        }
    }
}