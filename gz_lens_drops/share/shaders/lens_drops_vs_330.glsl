// Copyright (C) 2026 Honu Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");

#version 330

in vec4 vertex;
in vec2 uv0;
uniform mat4 worldViewProj;
out vec2 uv;

void main()
{
  gl_Position = worldViewProj * vertex;
  uv = uv0;
}
