using Boids;
using Boids.Settings;
using RendererUtility;
using UnityEngine;
using UnityEngine.Rendering;

namespace BoidsDemo
{
    public class NeighborSearchBoids : MonoBehaviour
    {
        [SerializeField] private NeighborSearchBoidsSetting _neighborSearchBoidsSetting;
        [SerializeField] private int _instanceCount;

        [SerializeField] private Mesh _mesh;
        [SerializeField] private Material _material;
        
        private RenderParams _renderParams;

        private NeighborSearchBoidsSimulator _neighborSearchBoidsSimulator;
        private Transform _transform;

        private void Start()
        {
            _renderParams = new RenderParams(_material) { receiveShadows = true, shadowCastingMode = ShadowCastingMode.On };

            _transform = transform;
            _neighborSearchBoidsSimulator = new NeighborSearchBoidsSimulator(_neighborSearchBoidsSetting, _instanceCount);
            _neighborSearchBoidsSimulator.InitializeBoidsPositionAndRotation(_transform.position, _transform.localScale);
        }

        private void LateUpdate()
        {
            _neighborSearchBoidsSimulator.Complete();
            
            InstanceRenderUtility.DrawAll(_mesh, _renderParams, _neighborSearchBoidsSimulator.BoidsTransformMatrices);

            _neighborSearchBoidsSimulator.ExecuteJob(_transform.position, _transform.localScale);
        }

        private void OnDestroy()
        {
            _neighborSearchBoidsSimulator.Dispose();
        }
    }
}