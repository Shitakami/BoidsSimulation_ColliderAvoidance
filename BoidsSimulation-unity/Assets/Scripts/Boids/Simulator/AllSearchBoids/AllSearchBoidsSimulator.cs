using System;
using Boids.Job;
using Boids.Settings;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Boids
{
    public class AllSearchBoidsSimulator : IDisposable
    {
        private readonly AllSearchBoidsSetting _allSearchBoidsSetting;
        private readonly int _instanceCount;
        private NativeArray<BoidsData> _boidsDatas;
        private NativeArray<float3> _boidsDataSteer;
        private NativeArray<Matrix4x4> _boidsTransformMatrices;
        
        public NativeArray<Matrix4x4> BoidsTransformMatrices => _boidsTransformMatrices;

        private JobHandle _jobHandle;
        
        public AllSearchBoidsSimulator(
            AllSearchBoidsSetting boidsSetting,
            int instanceCount
        )
        {
            _allSearchBoidsSetting = boidsSetting;
            _instanceCount = instanceCount;

            _boidsDatas = new NativeArray<BoidsData>(instanceCount, Allocator.Persistent);
            _boidsDataSteer = new NativeArray<float3>(instanceCount, Allocator.Persistent);
            _boidsTransformMatrices = new NativeArray<Matrix4x4>(instanceCount, Allocator.Persistent);
        }
        
        public void InitializeBoidsPositionAndRotation(float3 simulationAreaCenter, float3 simulationAreaScale)
        {
            BoidsUtility.InitializeBoidsData(
                _boidsDatas,
                simulationAreaCenter,
                simulationAreaScale / 2,
                _allSearchBoidsSetting.InitializedSpeed
            );
        }

        public void ExecuteJob(float3 simulationAreaCenter, float3 simulationAreaScale)
        {
            var boidsJob = new CalculateBoidsSteerForceByAllSearchJob
            (
                _allSearchBoidsSetting.CohesionWeight,
                _allSearchBoidsSetting.CohesionAffectedRadiusSqr,
                _allSearchBoidsSetting.CohesionViewDot,
                _allSearchBoidsSetting.SeparateWeight,
                _allSearchBoidsSetting.SeparateAffectedRadiusSqr,
                _allSearchBoidsSetting.SeparateViewDot,
                _allSearchBoidsSetting.AlignmentWeight,
                _allSearchBoidsSetting.AlignmentAffectedRadiusSqr,
                _allSearchBoidsSetting.AlignmentViewDot,
                _allSearchBoidsSetting.MaxSpeed,
                _allSearchBoidsSetting.MaxSteerForce,
                _boidsDatas,
                _boidsDataSteer
            );

            var boidsJobHandle = boidsJob.Schedule(_instanceCount, 0);

            var applySteerForce = new ApplySteerForceJob
            (
                _boidsDatas,
                _boidsDataSteer,
                _boidsTransformMatrices,
                simulationAreaCenter,
                simulationAreaScale/2,
                _allSearchBoidsSetting.AvoidSimulationAreaWeight,
                Time.deltaTime,
                _allSearchBoidsSetting.MaxSpeed,
                _allSearchBoidsSetting.InstanceScale
            );

            var applySteerForceHandle = applySteerForce.Schedule(_instanceCount, 0, boidsJobHandle);
            applySteerForceHandle.Complete();
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
            _boidsDataSteer.Dispose();
            _boidsTransformMatrices.Dispose();
        }
    }
}