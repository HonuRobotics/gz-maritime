#version 330

// Drops of water on a camera's lens, over the rendered image. Each drop is
// (x, y, radius, unused): x and y in [0, 1] from the top left, the radius
// as a fraction of the image height.
uniform sampler2D RT;
uniform vec4 drops[64];
uniform int dropCount;
uniform float aspect;
uniform float focus;

in vec2 uv;
out vec4 fragColor;

const vec2 kTaps[8] = vec2[8](
  vec2(-0.326, -0.406), vec2(-0.840, -0.074), vec2(-0.696, 0.457),
  vec2(-0.203, 0.621), vec2(0.962, -0.195), vec2(0.473, -0.480),
  vec2(0.519, 0.767), vec2(0.185, -0.893));

void main()
{
  vec2 at = uv;
  float cover = 0.0;
  float radius = 0.0;
  float rim = 0.0;
  for (int i = 0; i < dropCount; ++i)
  {
    vec2 d = (uv - drops[i].xy) * vec2(aspect, 1.0);
    float r = drops[i].z;
    float q = length(d) / max(r, 1e-6);
    if (q < 1.0)
    {
      // A drop is a small lens: it shows the scene upside down and wide.
      float bulge = sqrt(1.0 - q * q);
      vec2 seen = drops[i].xy - (uv - drops[i].xy) * (1.0 + 2.0 * bulge);
      float soft = mix(0.9, 0.15, focus);
      float c = smoothstep(1.0, 1.0 - soft, q);
      if (c > cover)
      {
        cover = c;
        at = seen;
        radius = r;
        rim = q;
      }
    }
  }
  vec3 clear = texture(RT, uv).rgb;
  if (cover <= 0.0)
  {
    fragColor = vec4(clear, 1.0);
    return;
  }
  // Out of focus: the nearer the lens, the wider the blur.
  float spread = radius * mix(0.6, 0.05, focus);
  vec3 sum = texture(RT, clamp(at, 0.0, 1.0)).rgb;
  for (int k = 0; k < 8; ++k)
  {
    vec2 o = kTaps[k] * spread / vec2(aspect, 1.0);
    sum += texture(RT, clamp(at + o, 0.0, 1.0)).rgb;
  }
  vec3 wet = sum / 9.0;
  // A darker edge, where the drop bends the light away.
  wet *= 1.0 - 0.35 * focus * smoothstep(0.7, 1.0, rim);
  fragColor = vec4(mix(clear, wet, cover), 1.0);
}
