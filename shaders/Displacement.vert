#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).x;
}

void main() {
  // YOUR CODE HERE
  //vec3 newpos = v_position.xyz + v_normal.xyz * h(v_uv) * u_height_scaling;
  //v_position = vec4(newpos, 1);
  //gl_Position = u_view_projection * u_model * v_position;
  // (Placeholder code. You will want to replace it.)
  v_position = u_model * in_position;
  v_normal = normalize(u_model * in_normal);
  v_uv = in_uv;
  v_tangent = normalize(u_model * in_tangent);
  gl_Position = u_view_projection * u_model * in_position;

  vec3 newpos = v_position.xyz + in_normal.xyz * h(in_uv) * u_height_scaling;
  v_position = vec4(newpos, 1);
  gl_Position = u_view_projection * v_position;
}
