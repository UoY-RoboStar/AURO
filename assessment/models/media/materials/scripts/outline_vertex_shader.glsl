uniform float scale;

void main()
{
  vec3 scaled_vertex = gl_Vertex.xyz + (scale * gl_Normal);
  gl_Position = gl_ModelViewProjectionMatrix * vec4(scaled_vertex, 1.0);
}