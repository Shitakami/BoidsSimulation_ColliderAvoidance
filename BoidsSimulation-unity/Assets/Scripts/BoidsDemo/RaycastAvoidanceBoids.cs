using Boids;
using RendererUtility;
using UnityEngine;
using UnityEngine.Rendering;

namespace BoidsDemo
{
    public class RaycastAvoidanceBoids : MonoBehaviour
    {
        [SerializeField] private RaycastAvoidanceBoidsSetting _raycastAvoidanceBoidsSetting;
        [SerializeField] private int _instanceCount;

        [SerializeField] private Mesh _mesh;
        [SerializeField] private Material _material;
        
        private RenderParams _renderParams;

        private RaycastAvoidanceBoidsSimulator _raycastAvoidanceBoidsSimulator;
        private Transform _transform;

        private void Start()
        {
            _renderParams = new RenderParams(_material) { receiveShadows = true, shadowCastingMode = ShadowCastingMode.On };

            _transform = transform;
            _raycastAvoidanceBoidsSimulator = new RaycastAvoidanceBoidsSimulator(_raycastAvoidanceBoidsSetting, _instanceCount);
            _raycastAvoidanceBoidsSimulator.InitializeBoidsPositionAndRotation(_transform.position, _transform.localScale);
        }

        private void LateUpdate()
        {
            _raycastAvoidanceBoidsSimulator.Complete();
            
            InstanceRenderUtility.DrawAll(_mesh, _renderParams, _raycastAvoidanceBoidsSimulator.BoidsTransformMatrices);

            _raycastAvoidanceBoidsSimulator.ExecuteJob(_transform.position, _transform.localScale);
        }

        private void OnDestroy()
        {
            _raycastAvoidanceBoidsSimulator.Dispose();
        }
    }
}