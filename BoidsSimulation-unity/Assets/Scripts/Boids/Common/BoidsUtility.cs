using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Boids
{
    public static class BoidsUtility
    {
        public static void InitializeBoidsData(BoidsData[] boidsDatas, Vector3 simulationAreaCenter, Vector3 simulationAreaScale, float initializeVelocity)
        {
            for (var i = 0; i < boidsDatas.Length; ++i)
            {
                var randPosition = Random.insideUnitSphere;
                boidsDatas[i].Position = new float3(
                    simulationAreaScale.x * randPosition.x + simulationAreaCenter.x,
                    simulationAreaScale.y * randPosition.y + simulationAreaCenter.y,
                    simulationAreaScale.z * randPosition.z + simulationAreaCenter.z
                );
                boidsDatas[i].Velocity = Random.insideUnitSphere * initializeVelocity;
            }
        }
        
        public static void InitializeBoidsData(NativeArray<BoidsData> boidsDatas, float3 simulationAreaCenter, float3 simulationAreaScaleHalf, float initializeVelocity)
        {
            for (var i = 0; i < boidsDatas.Length; ++i)
            {
                boidsDatas[i] = new BoidsData
                {
                    Position = simulationAreaScaleHalf * Random.insideUnitSphere + simulationAreaCenter,
                    Velocity = Random.insideUnitSphere * initializeVelocity
                };
            }
        }
    }
}