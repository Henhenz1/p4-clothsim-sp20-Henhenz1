#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).x;
}

void main() {
  // YOUR CODE HERE
  float wh = u_texture_2_size.x;
  float ht = u_texture_2_size.y;
  float u = v_uv.x;
  float v = v_uv.y;
  vec3 b = cross(v_normal.xyz, v_tangent.xyz);
  mat3 tbn;
  tbn[0] = v_tangent.xyz;
  tbn[1] = b;
  tbn[2] = v_normal.xyz;
  float du = (h(vec2(u + 1 / wh, v)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dv = (h(vec2(u, v + 1 / ht)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  vec3 no = vec3(-du, -dv, 1);
  vec3 nd = tbn*no;

  // Phong
  float ka = 0.1;
  float kd = 1;
  float ks = 0.8;
  float Ia = 0.8;
  int p = 100;
  float r = length(u_light_pos - v_position.xyz);
  vec3 l = (u_light_pos - v_position.xyz) / r;
  vec3 h = (u_cam_pos + l) / length(u_cam_pos + l);
  out_color = vec4(ka * Ia + kd * u_light_intensity / pow(r, 2) * max(0.0, dot(nd, l)) + ks * u_light_intensity / pow(r, 2) * pow(max(0.0, dot(nd, h)), p), 1) * u_color;
  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  // out_color.a = 1;
}

