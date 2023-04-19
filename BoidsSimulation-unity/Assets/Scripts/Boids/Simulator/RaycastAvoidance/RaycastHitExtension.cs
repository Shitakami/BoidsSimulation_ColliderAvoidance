using UnityEngine;

namespace Boids.Extension
{
    internal static class RaycastHitExtension
    {
        internal static bool IsHit(this RaycastHit raycastHit)
        {
            // MEMO: JobSystem処理内では MainThread での実行でないため、raycastHit.collider は動作しないので instanceId の値で判別
            // ObjectのInstanceIDは0が設定されないので、これで問題ないはず
            // https://docs.unity3d.com/2023.1/Documentation/ScriptReference/Object.GetInstanceID.html
            // > It is always unique, and never has the value 0.
            return raycastHit.colliderInstanceID != 0;
        }
    }
}
