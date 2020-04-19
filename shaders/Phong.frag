#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float ka = 0.1;
  float kd = 1;
  float ks = 0.8;
  float Ia = 0.8;
  int p = 100;
  float r = length(u_light_pos - v_position.xyz);
  vec3 l = (u_light_pos - v_position.xyz) / r;
  vec3 h = (u_cam_pos + l) / length(u_cam_pos + l);

  // all components
  out_color = vec4(ka * Ia + kd * u_light_intensity / pow(r, 2) * max(0.0, dot(v_normal.xyz, l)) + ks * u_light_intensity / pow(r, 2) * pow(max(0.0, dot(v_normal.xyz, h)), p), 1) * u_color;
  // ambient component
  // out_color = vec4(ka * Ia, ka * Ia, ka * Ia, 1) * u_color;
  // diffuse component
  // out_color = vec4(kd * u_light_intensity / pow(r, 2) * max(0.0, dot(v_normal.xyz, l)), 1) * u_color;
  // specular component
  //out_color = vec4(ks * u_light_intensity / pow(r, 2) * pow(max(0.0, dot(v_normal.xyz, h)), p), 1) * u_color;

  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  // out_color.a = 1;
}

