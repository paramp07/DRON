Shader "DRON/ThermalPointCloud"
{
    Properties
    {
        _PointSize ("Point Size", Float) = 0.03
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        LOD 100
        Cull Off
        ZWrite On

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 5.0
            #include "UnityCG.cginc"

            struct ThermalPointData
            {
                float3 position;
                float temperatureCelsius;
                float confidence;
                uint valid;
                fixed4 displayColor;
                float observationDt;
            };

            StructuredBuffer<ThermalPointData> _PointsBuffer;
            float _PointSize;

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
                float size : PSIZE;
            };

            v2f vert (uint id : SV_VertexID)
            {
                v2f o;
                ThermalPointData pt = _PointsBuffer[id];

                o.pos = UnityObjectToClipPos(float4(pt.position, 1.0));
                o.color = pt.displayColor;
                o.size = _PointSize;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}
