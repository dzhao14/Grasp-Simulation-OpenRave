FILE(REMOVE_RECURSE
  "CMakeFiles/myprogram.dir/myprogram.cpp.o"
  "myprogram.pdb"
  "myprogram"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/myprogram.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
