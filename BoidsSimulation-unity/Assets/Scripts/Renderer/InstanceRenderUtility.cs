using Unity.Collections;
using UnityEngine;

namespace RendererUtility
{
    public static class InstanceRenderUtility
    {
        public static void DrawAll(Mesh mesh, RenderParams renderParams, NativeArray<Matrix4x4> matricesArray)
        {
            const int instanceCountPerDraw = 1023;
            var instanceCount = matricesArray.Length;

            for (int i = 0; i < instanceCount; i += instanceCountPerDraw)
            {
                var length = Mathf.Min(instanceCountPerDraw, instanceCount - i);
                Graphics.RenderMeshInstanced(renderParams, mesh, 0, matricesArray, length, i);
            }
        }
    }
}