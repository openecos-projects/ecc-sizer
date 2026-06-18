{
  lib,
  stdenv,

  # nativeBuildInputs
  bison,
  cmake,
  doxygen,
  flex,
  gitMinimal,
  gtest,
  libsForQt5,
  pkg-config,
  swig,

  # buildInputs
  boost186, # 1.87.0 broken https://github.com/boostorg/asio/issues/442
  cbc, # for clp
  cimg,
  clp, # for or-tools
  cudd,
  gperftools,
  eigen,
  glpk,
  lcov,
  lemon-graph,
  libjpeg,
  or-tools,
  pcre,
  python3,
  re2, # for or-tools
  readline,
  spdlog,
  tcl,
  tclPackages,
  yosys,
  zlib,
  xorg,
  llvmPackages,
  yaml-cpp,
}:

stdenv.mkDerivation {
  pname = "Sizer";
  version = "0.1.0-alpha";

  src = with lib.fileset; toSource {
    root = ./.;
    fileset = unions [
      ./cmake
      ./etc
      ./src
      ./submit
      ./test
      ./thirdparty
      ./CMakeLists.txt
      ./LICENSE
      ./README.md
      ./NOTICE
    ];
  };

  nativeBuildInputs = [
    bison
    cmake
    doxygen
    flex
    gitMinimal
    gtest
    libsForQt5.wrapQtAppsHook
    pkg-config
    swig
  ];

  buildInputs = [
    boost186
    cbc
    cimg
    clp
    cudd
    gperftools
    eigen
    glpk
    lcov
    lemon-graph
    libjpeg
    libsForQt5.qtbase
    libsForQt5.qtcharts
    libsForQt5.qtdeclarative
    libsForQt5.qtsvg
    or-tools
    pcre
    python3
    re2
    readline
    spdlog
    tcl
    tclPackages.tclreadline
    yosys
    zlib
    yaml-cpp
  ]
  ++ lib.optionals stdenv.hostPlatform.isLinux [ xorg.libX11 ]
  ++ lib.optionals stdenv.hostPlatform.isDarwin [ llvmPackages.openmp ];

  postPatch = ''
    patchShebangs --build etc/find_messages.py
    patchShebangs --build etc/file_to_string.py

    # Fix shebangs in thirdparty/OpenROAD
    patchShebangs --build thirdparty/OpenROAD/etc/find_messages.py
    patchShebangs --build thirdparty/OpenROAD/etc/file_to_string.py
    # Disable two tests that are failing curently.
    sed 's/^.*partition_gcd/# \0/g' -i thirdparty/OpenROAD/src/par/test/CMakeLists.txt

    # Fix TCL include path expectations
    sed -i 's,<tcl8.6/,<,g' src/ckt.cpp src/analyze_timing.cpp src/lib_parser.cpp
  '';

  cmakeFlags = [
    (lib.cmakeBool "ENABLE_TESTS" true)
    (lib.cmakeBool "USE_SYSTEM_BOOST" true)
    (lib.cmakeBool "USE_SYSTEM_ABC" false)
    (lib.cmakeBool "ABC_SKIP_TESTS" true) # it attempts to download gtest
    (lib.cmakeBool "USE_SYSTEM_OPENSTA" false)
    (lib.cmakeFeature "OPENROAD_VERSION" "v0.1.0-alpha")
    (lib.cmakeBool "CMAKE_RULE_MESSAGES" false)
    (lib.cmakeFeature "TCL_HEADER" "${tcl}/include/tcl.h")
    (lib.cmakeFeature "TCL_LIBRARY" "${tcl}/lib/libtcl${stdenv.hostPlatform.extensions.sharedLibrary}")
  ]
  ++ lib.optionals stdenv.hostPlatform.isDarwin [
    (lib.cmakeFeature "CMAKE_CXX_FLAGS" "-DBOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED")
  ];

  # Resynthesis needs access to the Yosys binaries.
  qtWrapperArgs = [ "--prefix PATH : ${lib.makeBinPath [ yosys ]}" ];

  enableParallelBuilding = true;

  doCheck = false;

  doInstallCheck = false;

  meta = {
    license = lib.licenses.bsd3;
    platforms = lib.platforms.linux ++ lib.platforms.darwin;
    maintainers = with lib.maintainers; [
      trepetti
      hzeller
      Emin017
    ]; # https://github.com/NixOS/nixpkgs/blob/master/pkgs/by-name/op/openroad/package.nix
  };
}
