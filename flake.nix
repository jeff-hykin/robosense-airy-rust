{
  description = "robosense-airy: RoboSense Airy lidar driver (rs_driver port) and the airy-stats CLI";

  inputs = {
    nix-filter.url = "github:numtide/nix-filter";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nix-filter, nixpkgs, flake-utils, crate2nix }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" "x86_64-darwin" ] (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        name = "robosense-airy";
        src = nix-filter.lib { root = ./.; exclude = [ "target" "result" ]; };
        generated = crate2nix.tools.${system}.generatedCargoNix { inherit name src; };

        # builds for `targetPkgs` (native, or pkgsCross.aarch64-multiplatform for an arm64 board)
        crateFor = targetPkgs: (import generated { pkgs = targetPkgs; }).rootCrate.build;

        native = crateFor pkgs;
      in {
        packages = {
          # the library (`lib` output) and airy-stats (`bin`)
          default = native;
          ${name} = native;
        } // pkgs.lib.optionalAttrs (system == "x86_64-linux") {
          aarch64-linux = crateFor pkgs.pkgsCross.aarch64-multiplatform;
        };
        checks.tests = native.override { runTests = true; };

        devShells.default = pkgs.mkShell { packages = [ pkgs.cargo pkgs.rustc pkgs.clippy pkgs.rustfmt ]; };
      });
}
