{
  inputs.self.submodules = true;
  outputs = inputs@{
    self, nixpkgs, flake-parts,
  }:
    flake-parts.lib.mkFlake { inherit inputs; } {
    systems = [ "x86_64-linux" ];
    perSystem = { self', pkgs, system, ... }: {
      packages.default = pkgs.callPackage ./default.nix {};
      devShells.default = pkgs.mkShell {
        buildInputs = self'.packages.default.rawBuildInputs;
        nativeBuildInputs = self'.packages.default.rawNativeBuildInputs;
      };
    };
  };
}
