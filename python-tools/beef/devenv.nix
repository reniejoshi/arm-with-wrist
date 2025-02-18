{ pkgs, lib, ... }:

let
  buildInputs = with pkgs; [
    stdenv.cc.cc
    xorg.libX11
    xorg.libX11.dev

    libGL
    glfw
    libglvnd
    glm

    imgui
  ];
in
{
  env = { LD_LIBRARY_PATH = "${lib.makeLibraryPath buildInputs}"; };

  languages.python = {
    enable = true;
    uv = {
      enable = true;
      sync.enable = true;
    };
  };

  enterShell = ''
    python --version
  '';
 }
