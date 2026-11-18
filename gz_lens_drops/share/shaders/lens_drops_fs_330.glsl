#version 330

// Drops as small lenses, by refraction: the height of water on the lens
// gives a surface normal; the camera ray leaves the water through that
// surface by Snell's law, and the direction it goes out on is projected back
// into the image to find what the drop shows.
uniform sampler2D RT;
uniform sampler2D dropMap;   // r: water height (image heights), g: wet film
uniform float aspect;
uniform float tanHalfH;
uniform float focus;
uniform vec2 texel;

in vec2 uv;
out vec4 fragColor;

const float kIor = 1.333;
const vec2 kTaps[12] = vec2[12](
  vec2(-0.326, -0.406), vec2(-0.840, -0.074), vec2(-0.696, 0.457),
  vec2(-0.203, 0.621), vec2(0.962, -0.195), vec2(0.473, -0.480),
  vec2(0.519, 0.767), vec2(0.185, -0.893), vec2(0.507, 0.064),
  vec2(0.896, 0.412), vec2(-0.322, -0.933), vec2(-0.792, -0.598));

vec3 Blur(vec2 _at, float _radius)
{
  vec3 sum = texture(RT, clamp(_at, 0.0, 1.0)).rgb;
  for (int k = 0; k < 12; ++k)
  {
    vec2 o = kTaps[k] * _radius * vec2(1.0 / aspect, 1.0);
    sum += texture(RT, clamp(_at + o, 0.0, 1.0)).rgb;
  }
  return sum / 13.0;
}

void main()
{
  vec4 m = texture(dropMap, uv);
  float h = m.r;
  float wet = m.g;
  vec3 scene = texture(RT, uv).rgb;
  if (h < 1e-6 && wet < 1e-3)
  {
    fragColor = vec4(scene, 1.0);
    return;
  }

  // A thin film of water: a little blur, a little less contrast.
  vec3 filmed = mix(scene, Blur(uv, 0.004) * 0.92 + 0.04, clamp(wet, 0.0, 1.0));

  // Slope of the water, in image heights per image height.
  float hx1 = texture(dropMap, uv + vec2(texel.x, 0.0)).r;
  float hx0 = texture(dropMap, uv - vec2(texel.x, 0.0)).r;
  float hy1 = texture(dropMap, uv + vec2(0.0, texel.y)).r;
  float hy0 = texture(dropMap, uv - vec2(0.0, texel.y)).r;
  float gx = (hx1 - hx0) / (2.0 * texel.x * aspect);
  float gyDown = (hy1 - hy0) / (2.0 * texel.y);
  vec3 n = normalize(vec3(-gx, gyDown, -1.0));   // outward, camera frame, y up

  vec3 d = normalize(vec3((2.0 * uv.x - 1.0) * tanHalfH,
                          (1.0 - 2.0 * uv.y) * tanHalfH / aspect, -1.0));
  vec3 r = refract(d, -n, kIor);

  // Out of focus: drops on the lens are far nearer than the scene.
  float spread = mix(0.03, 0.004, focus);
  vec3 seen;
  if (dot(r, r) < 1e-6)
  {
    // Total internal reflection at the rim: dark.
    seen = Blur(uv, spread) * 0.15;
  }
  else
  {
    vec2 q = vec2(r.x / -r.z / tanHalfH, r.y / -r.z / (tanHalfH / aspect));
    vec2 at = vec2(q.x * 0.5 + 0.5, 0.5 - q.y * 0.5);
    // Beyond the frame the drop shows what the camera does not see: darken
    // a little rather than smear the border.
    float outside = length(max(abs(at - 0.5) - 0.5, 0.0));
    seen = Blur(at, spread) * (1.0 - clamp(outside * 2.0, 0.0, 0.35));
    // What reflects at the water's surface goes back into the camera's
    // housing, which is dark: Fresnel, from water to air.
    float c = abs(dot(d, n));
    float f0 = pow((kIor - 1.0) / (kIor + 1.0), 2.0);
    float fresnel = f0 + (1.0 - f0) * pow(1.0 - c, 5.0);
    seen *= 1.0 - fresnel;
  }

  float cover = smoothstep(0.0, 0.0015, h);
  fragColor = vec4(mix(filmed, seen, cover), 1.0);
}
