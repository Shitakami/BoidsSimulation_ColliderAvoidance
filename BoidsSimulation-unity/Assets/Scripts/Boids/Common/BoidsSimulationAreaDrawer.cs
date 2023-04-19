using UnityEngine;

namespace Boids.Debugger
{
    internal class BoidsSimulationAreaDrawer : MonoBehaviour
    {
        [SerializeField] private Color _gizmoColor = Color.yellow;
        [SerializeField] private bool _drawArea = true;
        private void OnDrawGizmos()
        {
            if (!_drawArea) return;
            
            Gizmos.color = _gizmoColor;
            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, transform.localScale);
            Gizmos.DrawWireCube(Vector3.zero, Vector3.one);
        }
    }
}
