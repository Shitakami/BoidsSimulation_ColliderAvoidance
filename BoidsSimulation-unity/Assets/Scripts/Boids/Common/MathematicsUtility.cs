using Unity.Mathematics;

namespace Boids.Utility
{
    internal static class MathematicsUtility
    {
        internal static float3 Limit(float3 vec, float max)
        {
            var lenght = math.sqrt(math.dot(vec, vec));
            return lenght > max
                ? vec * (max / lenght)
                : vec;
        }

        internal static int3 CalculateGridIndex(float3 position, float gridScale)
        {
            return new int3(
                (int)math.floor(position.x / gridScale),
                (int)math.floor(position.y / gridScale),
                (int)math.floor(position.z / gridScale)
            );
        }
        
        internal static bool IsCloseToGrid(float3 targetPosition, int3 gridIndex, float gridScale)
        {
            var gridMin = new float3(gridIndex.x * gridScale, gridIndex.y * gridScale, gridIndex.z * gridScale);;
            var gridMax = gridMin + new float3(gridScale, gridScale, gridScale);

            var closestGridPoint = new float3(
                math.clamp(targetPosition.x, gridMin.x, gridMax.x),
                math.clamp(targetPosition.y, gridMin.y, gridMax.y),
                math.clamp(targetPosition.z, gridMin.z, gridMax.z)
            );

            var distanceSqrFromTargetToGrid = math.distancesq(targetPosition, closestGridPoint);

            return distanceSqrFromTargetToGrid <= gridScale*gridScale;
        }
    }
}