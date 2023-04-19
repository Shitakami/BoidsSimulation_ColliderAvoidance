using Unity.Mathematics;
using UnityEngine;

namespace Boids.Settings
{
    [CreateAssetMenu(fileName = "AllSearchBoidsSetting", menuName = "Boids/AllSearchSetting")]
    public class AllSearchBoidsSetting : ScriptableObject
    {
        [Header("結合")]
        [SerializeField] private float _cohesionWeight;
        [SerializeField] private float _cohesionAffectedRadius;
        [SerializeField, Range(0, 360)] private float _cohesionViewAngle;

        [Header("分離")]
        [SerializeField] private float _separationWeight;
        [SerializeField] private float _separationAffectedRadius;
        [SerializeField, Range(0, 360)] private float _separationViewAngle;

        [Header("整列")]
        [SerializeField] private float _alignmentWeight;
        [SerializeField] private float _alignmentAffectedRadius;
        [SerializeField, Range(0, 360)] private float _alignmentViewAngle;

        [Header("シミュレーション空間外から戻る力")]
        [SerializeField] private float _avoidSimulationAreaWeight;

        [Header("個体のスケール")]
        [Space(20)]
        [SerializeField] private float3 _instanceScale;
        [SerializeField] private float _maxSpeed;
        [SerializeField] private float _maxSteerForce;
        [SerializeField] private float _initializedSpeed;

        public float CohesionWeight => _cohesionWeight;
        public float CohesionAffectedRadiusSqr => _cohesionAffectedRadius * _cohesionAffectedRadius;
        public float CohesionViewDot => AngleToDot(_cohesionViewAngle);
        
        public float SeparateWeight => _separationWeight;
        public float SeparateAffectedRadiusSqr => _separationAffectedRadius * _separationAffectedRadius;
        public float SeparateViewDot => AngleToDot(_separationViewAngle);

        public float AlignmentWeight => _alignmentWeight;
        public float AlignmentAffectedRadiusSqr => _alignmentAffectedRadius * _alignmentAffectedRadius;
        public float AlignmentViewDot => AngleToDot(_alignmentViewAngle);
        
        public float AvoidSimulationAreaWeight => _avoidSimulationAreaWeight;

        public float3 InstanceScale => _instanceScale;
        public float MaxSpeed => _maxSpeed;
        public float MaxSteerForce => _maxSteerForce;
        public float InitializedSpeed => _initializedSpeed;
        
        private static float AngleToDot(float angle)
        {
            return Mathf.Cos(angle * Mathf.PI / 360);
        }
    }
}