file(REMOVE_RECURSE
  "libcalibur_pf.a"
  "libcalibur_pf.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/calibur_pf.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
