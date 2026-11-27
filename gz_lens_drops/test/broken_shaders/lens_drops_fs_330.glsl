#version 330
// A fragment shader that does not compile, for the camera test: the pass
// must turn itself off and the camera go on publishing the plain image.
out vec4 fragColour;
void main()
{
  fragColour = vec4(this is not glsl);
}
