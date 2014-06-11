#version 330

uniform usampler2DRect P0Table0;
uniform usampler2DRect P0Table1;
uniform usampler2DRect P0Table2;
uniform isampler2DRect Lut11to16;
uniform usampler2DRect Data;
uniform sampler2DRect ZTable;

in VertexData {
    vec2 TexCoord;
} FragmentIn;

out layout(location = 0) vec4 Debug;

out layout(location = 1) vec3 A;
out layout(location = 2) vec3 B;
out layout(location = 3) vec3 Norm;
out layout(location = 4) float Infrared;

#define M_PI 3.1415926535897932384626433832795

int data(ivec2 uv)
{
  return int(texelFetch(Data, uv).x);
}

float decode_data(ivec2 uv, int sub)
{
  int row_idx = 424 * sub + (uv.y < 212 ? uv.y + 212 : 423 - uv.y);

  int m = int(0xffffffff);
  int bitmask = (((1 << 2) - 1) << 7) & m;
  int idx = (((uv.x >> 2) + ((uv.x << 7) & bitmask)) * 11) & m;

  int col_idx = idx >> 4;
  int upper_bytes = idx & 15;
  int lower_bytes = 16 - upper_bytes;

  ivec2 data_idx0 = ivec2(col_idx, row_idx);
  ivec2 data_idx1 = ivec2(col_idx + 1, row_idx);

  int lut_idx = (uv.x < 1 || 510 < uv.x || col_idx > 352) ? 0 : ((data(data_idx0) >> upper_bytes) | (data(data_idx1) << lower_bytes)) & 2047;
  
  return float(texelFetch(Lut11to16, ivec2(int(lut_idx), 0)).x);
}

vec2 processMeasurementTriple(in ivec2 uv, in usampler2DRect p0table, in int offset, in float ab_multiplier_per_frq, inout bool invalid)
{
  float p0 = -float(texelFetch(p0table, uv).x) * 0.000031 * M_PI;
  vec3 phase_in_rad = vec3(0.0f, 2.094395f, 4.18879f);
  
  vec3 v = vec3(decode_data(uv, offset + 0), decode_data(uv, offset + 1), decode_data(uv, offset + 2));
  
  invalid = invalid && any(equal(v, vec3(32767.0)));
  
  float a = dot(v, cos( p0 + phase_in_rad)) * ab_multiplier_per_frq;
  float b = dot(v, sin(-p0 - phase_in_rad)) * ab_multiplier_per_frq;
  
  return vec2(a, b);
}

void main(void)
{
  ivec2 uv = ivec2(FragmentIn.TexCoord.x, FragmentIn.TexCoord.y);
  
  const float ab_multiplier          = 0.6666667f;
  const float ab_multiplier_per_frq0 = 1.322581f;
  const float ab_multiplier_per_frq1 = 1.0f;
  const float ab_multiplier_per_frq2 = 1.612903f;
  const float ab_output_multiplier   = 16.0f;
  
  bool invalid_pixel = 0.0 < texelFetch(ZTable, uv).x;
  bvec3 invalid = bvec3(invalid_pixel);
  
  vec2 ab0 = processMeasurementTriple(uv, P0Table0, 0, ab_multiplier_per_frq0, invalid.x);
  vec2 ab1 = processMeasurementTriple(uv, P0Table1, 3, ab_multiplier_per_frq1, invalid.y);
  vec2 ab2 = processMeasurementTriple(uv, P0Table2, 6, ab_multiplier_per_frq2, invalid.z);
  
  A = mix(vec3(ab0.x, ab1.x, ab2.x), vec3(0.0), invalid);
  B = mix(vec3(ab0.y, ab1.y, ab2.y), vec3(0.0), invalid);
  Norm = mix(sqrt(A * A + B * B), vec3(65535.0), invalid);
  Infrared = min(dot(Norm, vec3(0.333333333  * ab_multiplier * ab_output_multiplier)), 65535.0);
  
  Debug = vec4(sqrt(vec3(Infrared / 65535.0)), 1.0);
}