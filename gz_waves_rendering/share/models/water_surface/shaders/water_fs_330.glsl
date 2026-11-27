// Copyright (c) 2016 The UUV Simulator Authors.
// Copyright (C) 2026 Honu Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Baseline water surface shader. Intentionally simple:
//   - One bump-map octave perturbs the surface normal.
//   - Reflection direction samples a cubemap once (no LOD bias).
//   - "Cheap HDR" boosts bright reflection pixels by their own
//     luminance × hdrMultiplier.
//   - Fresnel approximated as pow(1 - cos θ, fresnelPower).
//   - Water body colour blends between shallow and deep by the same
//     angular factor (more deep at grazing angles — artistic choice).
//
// Visual extras (foam mask, Schlick Fresnel, reflection roughness,
// crest colour modulation, multi-octave normals) live in follow-up
// commits so each can be evaluated independently against this
// baseline.

#version 330

uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
// Wave heightmap grid (η, Dx, Dy, foam). Always bound: the vertex shader
// samples it for the surface displacement. Here in the fragment shader it is
// read only for the foam alpha when foamStrength > 0 (the gerstner path leaves
// foamStrength at 0, so this read is skipped, but the texture stays bound).
uniform sampler2D heightMap;

uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;
// Reflection roughness in [0, 1]. Adds a LOD bias to the cubemap
// sample so the reflected sky reads as a soft diffuse colour rather
// than a sharp mirror — closer to how real water reflects light.
uniform float roughness;

// FFT tile extent in metres — needed to convert finite-difference
// texel deltas back to world-space derivatives.
uniform float tileSize;
// Foam controls. foamStrength=0 disables the foam path entirely and
// the heightmap sampler is never touched.
uniform float foamStrength;
uniform float foamThreshold;
// When non-zero, the heightmap's alpha channel already holds an analytic
// folding metric (the displacement Jacobian's minimum eigenvalue, 1 = flat,
// < 1 → folding) computed on the CPU. The foam mask then reads it with a
// single sample instead of finite-differencing the displacement.
uniform int useFoamMap;

// The water between an underwater camera and the surface above it, set when
// the renderer draws the world's water (gz_underwater_rendering): the
// attenuation of the surface's light, beta^D (xyz; w is 1 when set, else 0
// and the surface is left as it is); the attenuation of the veil's
// build-up, beta^B; the veil just below the surface, scaled by the daylight
// (xyz; w is the daylight); and the daylight's attenuation with depth, K_d.
uniform vec4 uwAttenuation;
uniform vec4 uwBackscatter;
uniform vec4 uwVeil;
uniform vec4 uwDiffuse;

in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  vec2 baseXY;
  float shelter;
} inPs;

out vec4 fragColor;

// Tessendorf foam mask. The 2D Jacobian of the chop transform
//   (x, y) → (x + Dx, y + Dy)
// (Dx/Dy already carry the engine-applied choppiness) drops below 1
// in compression zones (where chop is bunching the surface).
// Sustained low J is where foam forms in real ocean —
// we ramp foam in below `foamThreshold` and saturate at full white
// once the Jacobian crosses into negative (folding).
float ComputeFoamMask()
{
  if (foamStrength <= 0.0)
    return 0.0;
  ivec2 texSize = textureSize(heightMap, 0);
  // The texel of a grid point is centred half a texel in, as in the vertex
  // shader.
  vec2 heightUV = fract(inPs.baseXY / tileSize + 0.5 / vec2(texSize));

  // Precomputed analytic folding metric in alpha (Ehukai path): one sample,
  // no finite differencing, resolution-independent. 1 = flat, < 1 → folding.
  if (useFoamMap != 0)
  {
    float J = texture(heightMap, heightUV).a;
    return 1.0 - smoothstep(-foamThreshold, foamThreshold, J);
  }

  // Sample the chop derivatives over a FIXED PHYSICAL stencil
  // (~1 m), not one texel. A per-texel stencil makes the foam mask
  // resolution-dependent: at grid_size=1024 / tile_size=256 the
  // step would be 0.25 m, capturing aggressive high-frequency
  // gradients that drive the Jacobian negative across the entire
  // surface → 100% white foam. A fixed metric step keeps the
  // visual identical across grid resolutions.
  float dStep = max(1.0, tileSize / float(texSize.x));
  vec2 stencil = vec2(dStep / tileSize);  // back into UV space

  vec4 px = texture(heightMap, heightUV + vec2( stencil.x, 0.0));
  vec4 nx = texture(heightMap, heightUV + vec2(-stencil.x, 0.0));
  vec4 py = texture(heightMap, heightUV + vec2(0.0,  stencil.y));
  vec4 ny = texture(heightMap, heightUV + vec2(0.0, -stencil.y));

  float dDxdx = (px.g - nx.g) * 0.5 / dStep;
  float dDydy = (py.b - ny.b) * 0.5 / dStep;
  float dDxdy = (py.g - ny.g) * 0.5 / dStep;
  float dDydx = (px.b - nx.b) * 0.5 / dStep;

  float J = (1.0 + dDxdx) * (1.0 + dDydy) - dDxdy * dDydx;

  // Symmetric smoothstep window centred around J = 0 widens the
  // transition so foam edges don't cut hard.
  return 1.0 - smoothstep(-foamThreshold, foamThreshold, J);
}

// The surface seen from below. Within Snell's window, a cone of half-angle
// asin(1 / n) = 48.6 degrees about the vertical, the sky shows through,
// refracted and weighted by the transmission (1 - Schlick's reflectance);
// outside it the surface is a mirror of the water below (total internal
// reflection), drawn as the deep water colour.
const float kWaterIndex = 1.333;
vec4 FromBelow(vec3 _E, vec3 _N)
{
  // With the water drawn, what the mirror shows is the water itself: its
  // veil; and the sky is as bright as the daylight.
  bool fade = uwAttenuation.w > 0.5;
  vec4 below = vec4(fade ? uwVeil.rgb : deepColor.rgb, 1.0);
  // _E climbs to the surface; the normal facing the camera points down.
  vec3 T = refract(_E, -_N, kWaterIndex);
  if (dot(T, T) < 1e-6)
    return below;  // total internal reflection
  vec3 S = vec3(T.x, T.y, -T.z);  // gz-rendering cubemap Z flip
  vec4 sky = texture(cubeMap, S, roughness * 8.0);
  sky.rgb *= (sky.r + sky.g + sky.b) * hdrMultiplier;
  if (fade)
    sky.rgb *= uwVeil.w;
  float cosAir = abs(T.z);
  float r0 = pow((kWaterIndex - 1.0) / (kWaterIndex + 1.0), 2.0);
  float transmission = 1.0 - (r0 + (1.0 - r0) * pow(1.0 - cosAir, 5.0));
  return vec4(mix(below.rgb, sky.rgb, transmission), 1.0);
}

void main()
{
  vec4 bump = texture(bumpMap, inPs.bumpCoord) * 2.0 - 1.0;
  vec3 N = normalize(inPs.rotMatrix * bump.xyz);

  vec3 E = normalize(inPs.eyeVec);
  // Seen from below: the view ray runs along the normal of the triangle
  // drawn here, pointing up out of the water. That triangle's own normal,
  // from how the position changes across the pixel, not the normal
  // interpolated from its corners, which far away, at grazing angles,
  // disagrees with the triangle drawn and would shade parts of a surface
  // seen from above as seen from below. Not "higher than the camera",
  // which a crest above a camera on a boat also is (it faces the camera);
  // and not gl_FrontFacing, which render targets flip against the window:
  // the cross product's sign flips with them too, so it is taken to point
  // up, as a surface that does not fold over does.
  vec3 face = cross(dFdx(inPs.eyeVec), dFdy(inPs.eyeVec));
  if (face.z < 0.0)
    face = -face;
  if (dot(E, face) > 0.0)
  {
    fragColor = FromBelow(E, N);
    if (uwAttenuation.w > 0.5)
    {
      // Seen through the water, by the same model as every other surface
      // (gz_underwater_rendering): the light fades over the range and the
      // veil of the camera's depth builds up, so far away the surface is
      // the colour of the water.
      float range = length(inPs.eyeVec);
      float depth = max(inPs.eyeVec.z, 0.0);
      vec3 veil = uwVeil.rgb * exp(-uwDiffuse.rgb * depth);
      fragColor.rgb = fragColor.rgb * exp(-uwAttenuation.rgb * range) +
                      veil * (vec3(1.0) - exp(-uwBackscatter.rgb * range));
    }
    return;
  }
  vec3 R = reflect(E, N);
  R = vec3(R.x, R.y, -R.z);  // gz-rendering cubemap Z flip

  // Soft reflection: the LOD bias picks a coarser mip of the
  // cubemap, blurring the sky into a diffuse tint instead of a
  // sharp mirror of clouds.
  float lodBias = roughness * 8.0;
  vec4 envColor = texture(cubeMap, R, lodBias);
  envColor.rgb *= (envColor.r + envColor.g + envColor.b) * hdrMultiplier;

  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

  vec4 waterColor = mix(shallowColor, deepColor, facing);
  vec4 color = mix(waterColor, envColor, waterEnvRatio);

  // Foam overlay: blend toward white where the Tessendorf Jacobian
  // identifies a compression zone.
  // Sheltered water breaks less: foam scales with the waves drawn.
  float foam = ComputeFoamMask() * foamStrength * inPs.shelter;
  color.rgb = mix(color.rgb, vec3(1.0), foam);

  // Opaque surface: alpha 1.0 keeps the water out of the transparent
  // render path (no blend state is configured), avoiding depth-sort artifacts.
  fragColor = vec4(color.xyz, 1.0);
}
