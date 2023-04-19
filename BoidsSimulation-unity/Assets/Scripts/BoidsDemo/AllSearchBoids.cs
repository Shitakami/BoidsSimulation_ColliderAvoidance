using Boids;
using Boids.Settings;
using RendererUtility;
using UnityEngine;
using UnityEngine.Rendering;

namespace BoidsDemo
{
    public class AllSearchBoids : MonoBehaviour
    {
        [SerializeField] private AllSearchBoidsSetting _allSearchBoidsSetting;
        [SerializeField] private int _instanceCount;

        [SerializeField] private Mesh _mesh;
        [SerializeField] private Material _material;
        
        private RenderParams _renderParams;

        private AllSearchBoidsSimulator _allSearchBoidsSimulator;
        private Transform _transform;

        private void Start()
        {
            _renderParams = new RenderParams(_material) { receiveShadows = true, shadowCastingMode = ShadowCastingMode.On };

            _transform = transform;
            _allSearchBoidsSimulator = new AllSearchBoidsSimulator(_allSearchBoidsSetting, _instanceCount);
            _allSearchBoidsSimulator.InitializeBoidsPositionAndRotation(_transform.position, _transform.localScale);
        }

        private void LateUpdate()
        {
            _allSearchBoidsSimulator.Complete();
            
            InstanceRenderUtility.DrawAll(_mesh, _renderParams, _allSearchBoidsSimulator.BoidsTransformMatrices);

            _allSearchBoidsSimulator.ExecuteJob(_transform.position, _transform.localScale);
        }

        private void OnDestroy()
        {
            _allSearchBoidsSimulator.Dispose();
        }
    }
}
