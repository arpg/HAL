FILE(REMOVE_RECURSE
  "CMakeFiles/cleansrc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/cleansrc.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
