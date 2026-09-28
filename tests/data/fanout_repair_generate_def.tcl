read_lef /src/thirdparty/OpenROAD/test/Nangate45/Nangate45.lef
read_liberty /src/thirdparty/OpenROAD/test/Nangate45/Nangate45_typ.lib
read_verilog /src/tests/data/fanout_repair_sta.v
link_design top
initialize_floorplan \
  -die_area {0 0 100 100} \
  -core_area {5 5 95 95} \
  -site FreePDK45_38x28_10R_NP_162NW_34O
write_def /build/fanout_repair_input.def
